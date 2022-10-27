#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray,Float32
from numpy.linalg import multi_dot

class esti_vel:

    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.velocity_pub = rospy.Publisher('/vel', Float32, queue_size=1)
        self.P = np.array([[0.05,0,0,0],[0,0.05,0,0],[0,0,0.05,0],[0,0,0,0.05]])
        self.dt = 0.02
        self.x_dot=0 # init
        self.y_dot=0        
        self.init_time=0.0   
        self.odom_status=None
        
        while not rospy.is_shutdown():
            if self.odom_status:
                break
            else:
                rospy.loginfo("Waiting for message.")        
        self.main()
        
    def odom_callback(self,msg):

        self.odom_status = msg
    
    def sub_odomstatus(self,msg):
        ego_status_x = msg.pose.pose.position.x
        ego_status_y = msg.pose.pose.position.y
        ego_heading  = msg.pose.pose.orientation.z

        return ego_status_x, ego_status_y, ego_heading
    
    def DvKalman(self,x,P,z,dt):
        
        ### Parameter ###
        A=np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]])
        H=np.array([[1,0,0,0],[0,0,1,0]])
        Q=np.array([[0.5,0,0,0],[0,0.5,0,0],[0,0,0.5,0],[0,0,0,0.5]]) # tmp value
        R = np.array([[0.03,0],[0,0.03]])
        #################

        x_pred=np.dot(A,x)

        P_pred=np.dot(np.dot(A,P),A.T)+Q
        DP_H_Ppred = np.dot(H,P_pred)
        DP_HPH = np.dot(DP_H_Ppred,H.T)
        HPH_R = DP_HPH + R

        DP_P_pred_H = np.dot(P_pred, H.T)
        K = np.dot(DP_P_pred_H,np.linalg.inv(HPH_R))

        a = z - np.dot(H,x_pred)
        x_esti = x_pred+np.dot(K,a)
        
        b = np.dot(np.dot(K,H),P_pred)
        P_esti = P_pred - b

        pos_x,vel_x,pos_y,vel_y=x_esti[0][0],x_esti[1][0],x_esti[2][0],x_esti[3][0]
        return pos_x,vel_x,pos_y,vel_y,P_esti 

    def dvKalman(self,x,P,z,dt):

        ### Parameter ###
        A=np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]])
        H=np.array([[1,0,0,0],[0,0,1,0]])
        Q=np.array([[0.5,0,0,0],[0,0.5,0,0],[0,0,0.5,0],[0,0,0,0.5]]) # tmp value
        R = np.array([[0.03,0],[0,0.03]])
        #################

        x_pred=np.dot(A,x)       
        P_pred=multi_dot([A,P,A.T])+Q
        inv_HPHR=multi_dot([H,P_pred,H.T])+R
        K=multi_dot([P_pred,H.T,np.linalg.inv(inv_HPHR)])
        x_esti = x_pred + np.dot(K,(z - np.dot(H,x_pred)))
        P_esti = P_pred - multi_dot([K,H,P_pred])

        pos_x,vel_x,pos_y,vel_y=x_esti[0][0],x_esti[1][0],x_esti[2][0],x_esti[3][0]
        return pos_x,vel_x,pos_y,vel_y,P_esti        

    def main(self):
        rate = rospy.Rate(50)
        ego_x, ego_y, ego_heading = self.sub_odomstatus(self.odom_status)
        x = np.array([ego_x, self.x_dot, ego_y, self.y_dot]).reshape(4,1)
        P = self.P
        dt = self.dt

        while not rospy.is_shutdown():
            ego_x, ego_y, ego_heading = self.sub_odomstatus(self.odom_status)          
            z = np.array([ego_x, ego_y]).reshape(2,1)
            self.esti_x, self.x_dot, self.esti_y, self.y_dot, P  = self.dvKalman(x,P,z,dt)
            estimate_velocity = np.sqrt((self.x_dot**2 + self.y_dot**2))*3.6
            #print "Ego vehicle velocity(x) : ",estimate_velocity, '(km/h)' # only x vel??
            x = np.array([self.esti_x, self.x_dot, self.esti_y, self.y_dot]).reshape(4,1)

            self.velocity_pub.publish(estimate_velocity)
            rate.sleep()

if __name__=='__main__':
    try:
        rospy.init_node('esti_vel')
        esti_vel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
