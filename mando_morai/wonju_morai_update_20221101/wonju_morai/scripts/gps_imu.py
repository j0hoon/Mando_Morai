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

    def navsat_callback(self, gps_msg):

        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.is_gps=True

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

    #TODO:
    def convertLL2UTM(self):    
        xy_zone = self.proj_UTM(self.lon, self.lat)

        self.x = xy_zone[0] - self.e_o
        self.y = xy_zone[1] - self.n_o

        self.x = self.x - 1.25 * np.cos(np.deg2rad(self.odom_msg.pose.pose.orientation.z))
        self.y = self.y - 1.25 * np.sin(np.deg2rad(self.odom_msg.pose.pose.orientation.z))

        #TODO:
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.



if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass
