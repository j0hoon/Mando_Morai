#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import tf
import os
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj
from math import pi
import numpy as np
from numpy.linalg import inv
import math

class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)

        self.x, self.y = None, None
        self.is_imu=False
        self.is_gps=False

        #TODO:
        self.proj_UTM = Proj(proj='utm',zone=52, ellps='WGS84', preserve_units=False)

        #TODO:
        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link'

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.is_imu==True and self.is_gps == True:
                self.convertLL2UTM()

                #TODO:
                self.odom_pub.publish(self.odom_msg)

                os.system('clear')
                print(self.odom_msg)

                rate.sleep()

    def translationMtx(self,x,y,z):
        M = np.array([[1,0,0,x],
                    [0,1,0,y],
                    [0,0,1,z],
                    [0,0,0,1],         
                            ])
        return M

    def rotationMtx(self, yaw, pitch, roll): # no rotation

        R   = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1],
        ])
        return R

    def transform_GPS2vehicle(self,params_GPS):
        GPS_pos = [params_GPS.get(i) for i in (["X","Y","Z"])]

        x_rel = -1*GPS_pos[0] 
        y_rel = -1*GPS_pos[1]
        z_rel = -1*GPS_pos[2]

        R_T = np.matmul(self.translationMtx(x_rel,y_rel,z_rel),self.rotationMtx(0.,0.,0.))
        R_T = np.matmul(R_T, self.rotationMtx(0.,0.,0.))

        R_T = np.linalg.inv(R_T)

        return R_T

    def navsat_callback(self, gps_msg):

        parameters_GPS = {
            "X": 1.15, # meter
            "Y": 0,
            "Z": 0,
            "YAW": 0, #- 7.5*math.pi/180.0, # radian
            "PITCH": 0,
            "ROLL": 0
        }        

        self.lat_2 = gps_msg.latitude
        self.lon_2 = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        mat = np.array([self.lon_2, self.lat_2,0,0]).reshape(4,1)


        Tr = self.transform_GPS2vehicle(parameters_GPS)
        res = np.dot(Tr,mat)
        self.lon = res[0][0]
        self.lat = res[1][0]

        self.is_gps=True

    #TODO:
    def convertLL2UTM(self):    

        xy_zone = self.proj_UTM(self.lon, self.lat)

        self.x = xy_zone[0] - self.e_o
        self.y = xy_zone[1] - self.n_o
        
        #TODO:
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.

    def quaternion_to_euler_angle(self,msg):
        x=msg.x
        y=msg.y
        z=msg.z
        w=msg.w

        ysqr=y*y

        t0 = 2.0 * (w*x+y*z)
        t1 = 1.0 - 2.0 * (x*x+ysqr)
        X = math.degrees(math.atan2(t0,t1))

        t2 = 2.0 * (w*y - z*x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = 2.0 * (w*z + x*y)
        t4 = 1.0 - 2.0 * (ysqr + z*z)
        Z = math.degrees(math.atan2(t3,t4))

        return X,Y,Z

    def imu_callback(self, data):

        #TODO:
        X,Y,Z = self.quaternion_to_euler_angle(data.orientation)
        self.odom_msg.pose.pose.orientation.x = X
        self.odom_msg.pose.pose.orientation.y = Y
        self.odom_msg.pose.pose.orientation.z = Z # heading angle
        self.odom_msg.pose.pose.orientation.w = data.orientation.w       

        self.is_imu=True

if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass
