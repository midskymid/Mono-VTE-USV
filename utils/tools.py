"""
Authors: Liu Zhongtian (midsky@zju.edu.cn); Chen Xuanlin (xuanlinchen@zju.edu.cn)
"""
import os
import cv2
import numpy as np
import random
import threading
import time


def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLov5 project.
    param: 
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    return:
        no return

    """
    tl = (
        line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )

class inferonepicThread(threading.Thread):
    def __init__(self, boat_vehicle):
        threading.Thread.__init__(self)
        self.read_capture = cv2.VideoCapture(0)
        self.boatv5_vehicle = boat_vehicle
    def run(self):
        if not self.read_capture.isOpened():
            print("can not check camera device")
            exit(0)
        self.read_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.read_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)                              
        while self.read_capture.isOpened():
            ret, frame = self.read_capture.read()
            if not ret:
                break
            self.boatv5_vehicle.get_target_angle(frame)
        
        self.read_capture.release()
        cv2.destroyAllWindows()

class warmUpThread(threading.Thread):
    def __init__(self, boatv5_wrapper):
        threading.Thread.__init__(self)
        self.boatv5_wrapper = boatv5_wrapper

    def run(self):
        batch_image_raw, use_time  = self.boatv5_wrapper.infer(self.boatv5_wrapper.get_raw_image_zeros())
        print('warm_up->{}, time->{:.2f}ms'.format(batch_image_raw[0].shape, use_time * 1000))

def angle_limit(angle_input):
    angle = angle_input + np.pi
    beta = np.mod(angle, 2 * np.pi)
    angle_output = beta - np.pi
    return angle_output

def fal(input, alpha, sigma):
    if abs(input) > sigma:
        y_out = (abs(input) ** alpha) * np.sign(input)
    else:
        y_out = input / sigma ** (1-alpha)
    
    return y_out

    

if __name__ == "__main__":
    pass
