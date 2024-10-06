"""
Authors: Liu Zhongtian (midsky@zju.edu.cn); Chen Xuanlin (xuanlinchen@zju.edu.cn)
"""
from .tools import *
import rospy

class Boat_Motion_Control(Vehicle):
    def __init__(self, connect_string, baud_rate, boatv5_wrapper, K1, K2, K3, P1, P2, omiga_d, yita_d, deta):
        # super().__init__()
        self.vehicle = connect(params['connect'], baud=params['baud'], wait_ready=True)
        self.boatv5_wrapper = boatv5_wrapper
        self.deta = params['deta']
        self.yita_d = params['yita']
        self.omiga_d = params['omega']
        self.K1 = params['K1']
        self.K2 = params['K2']
        self.K3 = params['K2']
        self.P1 = params['P1']
        self.P2 = params['P2']
        self.OME = params['OME']
        self.gamma_i = 0.0
        self.proj_params = params['projection_parameters']
        set_log_time = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self.logfile_path = f"{params['log_path']}_{set_log_time}.txt"

        self.cam_k = np.array([[self.proj_params[0], 0, self.proj_params[2]], [0, self.proj_params[1], self.proj_params[3]], [0, 0, 1]])
        self.last_time = rospy.Time.now().to_sec()
        self.pti_hat = np.array([-2,-2]).reshape(2, 1)
        self.vt_hat = np.array([0, 0]).reshape(2, 1)
        self.hat_beta = 0.0
        self.hat_sigma = 0.0
        self.beta_desire = 0.0
        self.last_eta_desire = 0.0
        self.filter_vel_1 = 0.0
        self.filter_pos_1 = 0.0
        self.filter_vel_2 = 0.0
        self.filter_pos_2 = 0.0
        # self.frame = 0

    # 控制船的速度
    def send_local_ned_velocity(self, vx, yaw_rate):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            1479,
            0, 0, 0,
            vx, 0, 0,
            0, 0, 0,
            0, yaw_rate)
        self.vehicle.send_mavlink(msg)

    # 船解锁为引导模式并向前形式move_time*速度的距离
    def arm_and_go(self, dist):
        set_move_time = dist
        while not self.vehicle.is_armable:
            print("waiting for boat to initialise...")
            time.sleep(1)
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("waiting for arming...")
            time.sleep(1)
    
        # 向前走dist 
        while set_move_time > 0:    
            self.send_local_ned_velocity(0.5, 0)
            time.sleep(0.1)
            set_move_time = set_move_time - 1
            
        self.send_local_ned_velocity(0, 0)
        time.sleep(1)


    def get_target_angle(self, image):
        
        image_raw, use_time, boxes, scores = self.boatv5_wrapper.infer_onepic(image)
        # self.frame += 1
        cv2.imshow("result", image_raw)
        cv2.waitKey(1)
        if len(boxes) > 0:
            indices = np.argsort(scores)[::-1]  # [::-1] reverse a list or array
            indice = indices[0]
            box = boxes[indice]
            obj_center_x = (box[2] - box[0]) / 2.0 + box[0]
            obj_center_y = (box[3] - box[1]) / 2.0 + box[1]
            img_center_x = self.cam_k[0, 2]
            img_center_y = self.cam_k[1, 2]
            norm_x = (obj_center_x - img_center_x) / self.cam_k[0, 0]
            norm_y = (obj_center_y - img_center_y) / self.cam_k[1, 1]
            # print("norm_x: ", norm_x)
            # print("self.cam_k[0, 0]: ", self.cam_k[0, 0])

            self.gamma_i = np.arctan(norm_x)

    def estimate_distance(self, dtime):
        fi_i = np.pi / 2 - self.vehicle.attitude.yaw
        beta_i = fi_i + np.pi / 2 - self.gamma_i
        phi = np.array([-np.sin(beta_i), np.cos(beta_i)]).reshape(2, 1)
        dot_pti_hat = -np.matmul(phi*phi.T, self.pti_hat)*self.P1 + self.vt_hat - np.array([self.vehicle.velocity[1], self.vehicle.velocity[0]]).reshape(2, 1)
        dot_vt_hat = -np.matmul(phi*phi.T, self.pti_hat)*self.P2
        self.pti_hat = self.pti_hat + dot_pti_hat*dtime
        self.vt_hat = self.vt_hat + dot_vt_hat*dtime
        yita_i = np.linalg.norm(self.pti_hat)
        return yita_i
    
    def get_surge_speed(self):
        fi_i = np.pi / 2 - self.vehicle.attitude.yaw
        u_i = self.vehicle.velocity[0]*np.sin(fi_i) + self.vehicle.velocity[1]*np.cos(fi_i)
        return u_i
    
    def control_filter_1(self, dtime, param, omega):
        dot_filter_pos = self.filter_vel_1
        dot_filter_vel = (-param**2) * (self.filter_pos_1 - omega) - 2 * param * self.filter_vel_1

        self.filter_pos_1 += dot_filter_pos * dtime
        self.filter_vel_1 += dot_filter_vel * dtime

    def control_filter_2(self, dtime, param, omega):
        dot_filter_pos = self.filter_vel_2
        dot_filter_vel = (-param**2) * (self.filter_pos_2 - omega) - 2 * param * self.filter_vel_2

        self.filter_pos_2 += dot_filter_pos * dtime
        self.filter_vel_2 += dot_filter_vel * dtime

   
    def TEC_controller(self):
        now_time = rospy.Time.now().to_sec()
        dtime = now_time - self.last_time
        self.last_time = now_time

        dist = self.estimate_distance(dtime)
        fi_tmp = np.pi / 2 - self.vehicle.attitude.yaw
        surge_speed = self.get_surge_speed()
        beta_tmp = fi_tmp + np.pi / 2 - self.gamma_i
        beta = angle_limit(beta_tmp)
        fi = angle_limit(fi_tmp)

        Rotation_matrix = np.array([[np.cos(fi), np.sin(fi)], 
                                    [-np.sin(fi), np.cos(fi)]])
        
        tmp_angle_1 = fi - beta + np.pi / 2
        eta = angle_limit(tmp_angle_1)

        if abs(dist) < 0.001 :
            dist += 0.001
        
        # ESO estimation
        k3 = 2 * self.OME
        k4 = - self.OME * self.OME


        dot_hat_beta = -k3 * fal(np.sin(self.hat_beta - beta), 0.6, 0.8) + surge_speed / dist - self.hat_sigma
        dot_hat_sigma = -k4 * fal(np.sin(self.hat_beta - beta), 0.6, 0.8) 

        self.hat_beta += dot_hat_beta * dtime
        self.hat_sigma += dot_hat_sigma * dtime

        self.hat_sigma = max(min(self.hat_sigma, 1.5), -1.5)

        # surge controller design
        dot_beta_desire = self.omiga_d
        self.beta_desire += dot_beta_desire

        tmp_angle_2 = beta - self.beta_desire
        tilde_beta = angle_limit(tmp_angle_2)

        tmp_surge_speed = -self.K1 * dist * tilde_beta + dist * (self.hat_sigma + dot_beta_desire)

        self.control_filter_1(dtime, 6.5, tmp_surge_speed)
        set_surge_speed = self.filter_pos_1
        
        # yawspeed controller design
        tilde_dist = dist - self.yita_d
        eta_desire = np.arctan2(tilde_dist, self.deta)
        tmp_angle_3 = eta - eta_desire
        tilde_eta = angle_limit(tmp_angle_3)
        dot_eta_desire = (eta_desire - self.last_eta_desire) / dtime
        self.last_eta_desire = eta_desire
        hat_sigma2 = -self.hat_sigma + surge_speed / dist + dot_eta_desire

        # print("self.hat_sigma: ", self.hat_sigma)
        # print("hat_sigma2: ", hat_sigma2)
        print("dist: ", dist)

        tmp_yawspeed = -(self.K3 * tilde_dist - self.K2 * tilde_eta + hat_sigma2)
        self.control_filter_2(dtime, 6.5, tmp_yawspeed)
        set_yawspeed = self.filter_pos_2

        # 为防止目标脱离摄像头视野范围
        if self.gamma_i < np.arctan((20.0 - self.cam_k[0, 2]) / self.cam_k[0, 0]) :  # -0.68与相机标定参数相关，目标中心点为20时候的值
            set_surge_speed = 0
            set_yawspeed = -1
        
        if self.gamma_i > np.arctan((620.0 - self.cam_k[1, 2]) / self.cam_k[1, 1]) :  # 0.61与相机标定参数相关，目标中心点为620时候的值
            set_surge_speed = 0
            set_yawspeed = 1

        # save msgs
        with open(self.logfile_path, 'a') as f:
            f.write(str(self.last_time))
            f.write(" ")
            f.write(str(dist))
            f.write(" ")
            f.write(str(self.vt_hat[0,0]))
            f.write(" ")
            f.write(str(self.vt_hat[1,0]))
            f.write(" ")
            f.write(str(self.gamma_i))
            f.write(" ")
            f.write(str(fi_tmp))
            f.write(" ")
            f.write(str(fi))
            f.write(" ")
            f.write(str(beta_tmp))
            f.write(" ")
            f.write(str(beta))
            f.write(" ")
            f.write(str(surge_speed))
            f.write(" ")
            f.write(str(tmp_surge_speed))
            f.write(" ")
            f.write(str(set_surge_speed))
            f.write(" ")
            f.write(str(tmp_yawspeed))
            f.write(" ")
            f.write(str(set_yawspeed))
            f.write(" ")
            f.write(str(self.vehicle.location.global_relative_frame.lon))
            f.write(" ")
            f.write(str(self.vehicle.location.global_relative_frame.lat))
            f.write("\n")

        return [set_surge_speed, set_yawspeed]
    
    def target_enclosing(self):
        time.sleep(0.1)  # 防止now和self.last过近，所以把sleep放在最前面
        set_u_i, set_r_i = self.TEC_controller()
        vx = max(min(set_u_i, 1.5), -0.5)
        yawspeed = max(min(set_r_i, 1), -1)

        print("vx: " ,vx , " ", "yawspeed: ", yawspeed)
        print("self.gamma_i: ",self.gamma_i)
        print("----------------------------------------------------------------")

        self.send_local_ned_velocity(vx, yawspeed)