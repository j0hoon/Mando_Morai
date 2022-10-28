#!/usr/bin/env python

import rospy
from morai_msgs.msg import CtrlCmd,EventInfo,Lamps
from morai_msgs.srv import MoraiEventCmdSrv
from nav_msgs.msg import Odometry
import time
from math import pi, sin, cos
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError


class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=1       
        self.ctrl_cmd_pub = rospy.Publisher("ctrl_cmd", CtrlCmd, queue_size=1)

        x = 320 
        y = 240

        self.crop_pts = np.array(
                                    [[
                                        [int(x * 0.15), int(y * 0.90)],
                                        [int(x * 0.30), int(y * 0.52)],
                                        [int(x * 0.80), int(y * 0.52)],
                                        [int(x * 0.85), int(y * 0.90)]
                                    ]]
                                )

    def callback(self, msg):
        try:

            np_arr = np.fromstring(msg.data, np.uint8)

            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except CvBridgeError as e:
            print(e)

        self.gray = cv2.cvtColor(self.img_bgr, cv2.COLOR_RGB2GRAY)
        self.blur = cv2.GaussianBlur(self.gray, (3, 3), 0)
        canny = cv2.Canny(self.blur, 70, 210)

        self.mask = self.mask_roi(canny)

        self.hough = cv2.HoughLinesP(self.mask, 1, 1 * pi/180, 30, np.array([]), 10, 20)
        line_arr = np.squeeze(self.hough)
        self.slope_degree = (np.arctan2(line_arr[:,1] - line_arr[:,3], line_arr[:,0] - line_arr[:,2]) * 180) / np.pi

        line_arr = line_arr[np.abs(self.slope_degree)<160]
        self.slope_degree = self.slope_degree[np.abs(self.slope_degree)<160]

        line_arr = line_arr[np.abs(self.slope_degree)>95]
        self.slope_degree = self.slope_degree[np.abs(self.slope_degree)>95]

        L_lines, R_lines = line_arr[(self.slope_degree>0),:], line_arr[(self.slope_degree<0),:]
        temp = np.zeros((self.mask.shape[0], self.mask.shape[1], 3), dtype=np.uint8)
        L_lines, R_lines = L_lines[:,None], R_lines[:,None]

        #new_L = np.array(L_lines).flatten
        #new_r = np.array(R_lines).flatten

        #print("L: ", new_L)
        #print("R: ", new_r)

        L=0
        i=0
        R=0

        if L_lines.size != 0:

            for i in range(0,3):

                L = L_lines[0,0,i]
                L+=L
                i+=1


        if R_lines.size != 0:


            for i in range(0,3):

                R = R_lines[0,0,i]
                R+=R
                i+=1

        rrr = 1
        lll = 1
        avg_L_degree = L/4*lll
        avg_R_degree = -180+R/4 * rrr
        print("LL:",avg_L_degree)
        print("RR:",avg_R_degree)

        self.ctrl_cmd_msg.accel = 0.1

        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

        p=0.008

        if avg_R_degree == -180:
            avg =  L/4*lll

            self.ctrl_cmd_msg.steering =  -p*avg

            print("turn right")



        elif avg_L_degree == 0:
            avg = -180+R/4 * rrr

            self.ctrl_cmd_msg.steering = - p*avg

            print("Left")
        
        else:
            avg = (L/4*lll + -180+R/4 * rrr)*0.5

            self.ctrl_cmd_msg.steering = - p*avg 






        #self.line_img = np.zeros((self.mask.shape[0], self.mask.shape[1], 3), dtype=np.uint8)
        #self.draw = self.draw_lines(self.line_img, self.hough)


        #self.test = self.weighted_img(self.draw, self.img_bgr)
        
        cv2.imshow("Image window", self.mask)
        cv2.waitKey(1) 

    def mask_roi(self, img):

        h = img.shape[0]
        w = img.shape[1]
        
        if len(img.shape)==3:

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255, 255, 255)

        else:

            mask = np.zeros((h, w), dtype=np.uint8)

            mask_value = (255)

        cv2.fillPoly(mask, self.crop_pts, mask_value)

        mask = cv2.bitwise_and(mask, img)

        return mask

    def draw_lines(img, lines, color=[0, 0, 255], thickness=2): 
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)

    def weighted_img(img, initial_img, p=1, b=1., r=0.): 
        return cv2.addWeighted(initial_img, p, img, b, r)





if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)


    image_parser = IMGParser()

    rospy.spin() 


