#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from math import pi, sin, cos
 
import cv2
import numpy as np
import os, rospkg

from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self):

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

    def lane_detection(self,lane_camera_status):

        np_arr = np.fromstring(lane_camera_status, np.uint8)

        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.gray = cv2.cvtColor(self.img_bgr, cv2.COLOR_RGB2GRAY)   # 이미지를 흑백으로 변경
        self.blur = cv2.GaussianBlur(self.gray, (3, 3), 0)           #가우시안 필터 적용
        canny = cv2.Canny(self.blur, 70, 210)                        #캐니 알고리즘 적용

        self.mask = self.mask_roi(canny)                             #ROI 설정

        self.hough = cv2.HoughLinesP(self.mask, 1, 1 * pi/180, 30, np.array([]), 10, 20) # 허프 변환

        if self.hough is not None:

            if len(self.hough)!=1:
                line_arr = np.squeeze(self.hough)
            else:
                line_arr = self.hough[0]                   
            self.slope_degree = (np.arctan2(line_arr[:,1] - line_arr[:,3], line_arr[:,0] - line_arr[:,2]) * 180) / np.pi  #허프 변환으로 인식한 차선들 기울기 구하기

            line_arr = line_arr[np.abs(self.slope_degree)<160]                  
            self.slope_degree = self.slope_degree[np.abs(self.slope_degree)<160]     #수평 기울기 제한하기 

            line_arr = line_arr[np.abs(self.slope_degree)>95]
            self.slope_degree = self.slope_degree[np.abs(self.slope_degree)>95]      #수직 기울기 제한

            L_lines, R_lines = line_arr[(self.slope_degree>0),:], line_arr[(self.slope_degree<0),:]  #제한하고 남은 진또배기 차선들이고 판단한 직선 모임
            temp = np.zeros((self.mask.shape[0], self.mask.shape[1], 3), dtype=np.uint8)
            L_lines, R_lines = L_lines[:,None], R_lines[:,None]

            L=0
            i=0
            R=0 # 위에 변수 3개는 for문 오류 해결하려고 선언

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

    # 우회전해야할 때, 오른쪽 차선은 인식을 못하는 경우이니 R_lines에는 요소가 담겨있지 않다. 
    #그래서 if문에 != 적용.....그래서 왼쪽 차선 정보가 담겨있는 L_lines의 첫 번째 차원, 첫번째 행의 요소 4개만 평균치를 내어 조향각으로 생각 
    # (차선이나 옆에 돌담이나 직선으로 검출시 비슷한 기울기를 가진다고 판단)

            rrr = 1
            lll = 1
            avg_L_degree = L/4*lll
            avg_R_degree = -180+R/4 * rrr
            #print("LL:",avg_L_degree)
            #print("RR:",avg_R_degree)

    #평균내는 과정, lll과 rrr은 처음에 만든 파라미터라 없애도 된다.

            p=0.4 #기울기값은 60도 이렇게 나오지만 조향각은 최대 1만 받을 수 있으니 비율을 맞춰주는 파라미터

            if avg_R_degree == -180:
                avg =  L/4*lll

    #-180도인 경우는 오른쪽 차선이 하나도 인지를 못한 상황>>우화전이 필요한 경우이다.
    #  이떄 -180이란 숫자땜에 평균에 영향을 주어 잘못된 조향각을 가지는 걸 방지하기에 평균 계산에 넣지 않았다.

            elif avg_L_degree == 0:
                avg = -180+R/4 * rrr
        
            else:
                avg = (L/4*lll + -180+R/4 * rrr)*0.5
        
            desired_steering = -p*np.deg2rad(avg)
        
        else:
            desired_steering = 0.0

#0도인 경우는 왼쪽 차선이 하나도 인지를 못한 상황>>좌화전이 필요한 경우이다.
#  이떄 0이란 숫자땜에 평균에 영향을 주어 잘못된 조향각을 가지는 걸 방지하기에 평균 계산에 넣지 않았다.

        return desired_steering

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

