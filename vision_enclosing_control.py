"""
Authors: Liu Zhongtian (midsky@zju.edu.cn); Chen Xuanlin (xuanlinchen@zju.edu.cn)
"""
import rospy
import cv2
from dronekit import Vehicle, connect, VehicleMode
from pymavlink import mavutil
from utils.parameters import readParameters
from utils.tools import *
from utils.motion_control import Boat_Motion_Control
from utils.boatv5 import *

def main(config):
    # get params
    params = readParameters(config_path)
    
    # load custom plugin and engine
    PLUGIN_LIBRARY = params["PLUGIN_LIBRARY"]
    engine_file_path = params["engine_file_path"]

    if len(sys.argv) > 1:
        engine_file_path = sys.argv[1]
    if len(sys.argv) > 2:
        PLUGIN_LIBRARY = sys.argv[2]

    ctypes.CDLL(PLUGIN_LIBRARY)
    
    rospy.init_node("vision_target_enclosing_control")

    # a boatv5TRT instance
    boatv5_wrapper = Boatv5TRT(engine_file_path)

    for i in range(10):
        thread1 = warmUpThread(boatv5_wrapper)
        thread1.start()
        thread1.join()

    boat_vehicle = Boat_Motion_Control(boatv5_wrapper, params)
    print("connect successful!\n")
    boat_vehicle.arm_and_go(10)

    thread1 = inferonepicThread(boat_vehicle)
    thread1.setDaemon(True)
    thread1.start()


    try:
        while not rospy.is_shutdown():
            boat_vehicle.target_enclosing() 
    
    finally:
        print("execute finally")
        # destroy the instance
        boatv5_wrapper.destroy()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    config_path = '/home/<user_name>/Mono-VTE-USV/config_files/boat_config.yaml'
    main(config_path)
    
