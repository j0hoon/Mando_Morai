#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser():
    def __init__(self):

        x = 320 
        y = 240
        self.image_sub = rospy.Subscriber('/image_jpeg/compressed2', CompressedImage, self.callback)

        # tmp for color bound select
        self.crop_pts = np.array(
                                    [[
                                        [int(x * 0.45), int(y * 0.5)], # left lower
                                        [int(x * 0.45), int(y * 0.35)], # left upper
                                        [int(x * 0.65), int(y * 0.35)], # right upper
                                        [int(x * 0.65), int(y * 0.5)]  # right lower
                                    ]]
                                )

    def mask_roi(self,img):
        h = img.shape[0]
        w = img.shape[1]

        if len(img.shape)==3:
            c = img.shape[2]
            mask = np.zeros((h,w,c), dtype=np.uint8)
            mask_value = (255,255,255)

        else:
            mask = np.zeros((h,w), dtype=np.uint8)
            mask_value = (255)
        
        cv2.fillPoly(mask, self.crop_pts, mask_value)
        mask = cv2.bitwise_and(mask, img)

        return mask

    def callback(self,msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8) # Byte -> uint8 array
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # uint8 array -> [w,h,c] image array
        except CvBridgeError as e:
            print(e)
        
        img_bgr = self.mask_roi(img_bgr)

        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        # for detect red

        lower_red = np.array([160,130,0])
        upper_red = np.array([181,255,255])

        # for detect yellow

        lower_yellow = np.array([30,130,0])
        upper_yellow = np.array([40,255,255])

        # for detect green

        lower_green = np.array([50,120,0])
        upper_green = np.array([70,255,255])


        img_rlane = cv2.inRange(img_hsv, lower_red, upper_red)
        img_ylane = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
        img_glane = cv2.inRange(img_hsv, lower_green, upper_green)   

        img_rlane = cv2.cvtColor(img_rlane, cv2.COLOR_GRAY2BGR)
        img_ylane = cv2.cvtColor(img_ylane, cv2.COLOR_GRAY2BGR)
        img_glane = cv2.cvtColor(img_glane, cv2.COLOR_GRAY2BGR)

        # initialization
        rlane_detect, ylane_detect, glane_detect = 0,0,0
        detect_list = []


        for y in range(self.crop_pts[0][2][1], self.crop_pts[0][3][1]+1):
            for x in range(self.crop_pts[0][0][0], self.crop_pts[0][3][0]+1):

                if np.all(img_rlane[y][x]!=0):
                    rlane_detect+=1
                elif np.all(img_ylane[y][x]!=0):
                    ylane_detect+=1
                elif np.all(img_glane[y][x]!=0):
                    glane_detect+=1
        detect_list.append(rlane_detect)
        detect_list.append(ylane_detect)
        detect_list.append(glane_detect)
    
        print(detect_list)

        img_concat = np.concatenate([img_bgr, img_hsv], axis=1)

        cv2.imshow("Image window", img_concat)        
        cv2.waitKey(1)

if __name__=='__main__':

    rospy.init_node('image_parser',anonymous=True)
    image_parser = IMGParser()
    rospy.spin()
