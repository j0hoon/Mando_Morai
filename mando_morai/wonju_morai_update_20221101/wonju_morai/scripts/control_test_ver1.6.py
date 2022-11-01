#!/usr/bin/env python2

import rospy
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32
import time
from math import pi, sin, cos, atan2, sqrt
from utils import genWP
from utils.select_mission import select_mission
from utils.genWP import WP_X_1, WP_Y_1, WP_X_2, WP_Y_2, WP_X_3, WP_Y_3 
from utils.SWG import SWG_make
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray,Float32
import numpy as np

WHEEL_BASE = 2.7 # KIA NIRO
K = 0.35 # init = 0.5
LW = WHEEL_BASE

def pure_pursuit(ref_x, ref_y, V_x, ego_x, ego_y, yaw):
    # yaw: (rad/s)
    #
    Lp = K * V_x
    Lp_x = ego_x + Lp * np.cos(yaw)
    Lp_y = ego_y + Lp * np.sin(yaw)

    dis_P2 = np.sqrt((np.array(ref_x) - Lp_x)**2+(np.array(ref_y) - Lp_y)**2)

    min_index = np.argmin(dis_P2)

    Way_x = ref_x[min_index]
    Way_y = ref_y[min_index]

    x_2 = (Way_x - ego_x) * np.cos(yaw) + (Way_y - ego_y) * np.sin(yaw)
    y_2 = - (Way_x - ego_x) * np.sin(yaw) + (Way_y - ego_y) * np.cos(yaw)

    L_bar = np.sqrt(x_2**2 +y_2**2)

    sin_alpha = y_2/L_bar

    k = 2 * sin_alpha/L_bar
    steer_angle = LW * 2 * y_2 / (L_bar)**2

    return steer_angle,min_index

class Control:
    
    def __init__(self):

        self.ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.mission_pub = rospy.Publisher('/mission', Int32, queue_size=1)
        self.target_vel_sub = rospy.Subscriber('/target_vel', Int32, self.TV_callback)
        self.stop_signal = rospy.Publisher('/stop_sign', Int32, queue_size=1)
        self.desired_steer_sub = rospy.Subscriber('/des_steer', Float32, self.steering_callback)
        self.target_velocity = None
        self.ctrl_msg= CtrlCmd()
        self.FrameRate = 50
        self.min_index = 0
        self.min_index_for_LD = 0
        self.init_signal=1
        self.old_WP_index = 0
        self.switch_WP=1
        self.odom_status = None
        self.des_steer = None
        while True:
            if self.odom_status:
                break
            else:
                rospy.loginfo("Waiting for message.")                

        self.main()

    def odom_callback(self,msg): # GPS pose, IMU heading angle data

        self.odom_status = msg

    def sub_odom_status(self,msg):
        self.heading_angle = msg.pose.pose.orientation.z
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y       

        return self.x, self.y, np.deg2rad(self.heading_angle)

    def TV_callback(self,msg):

        self.target_velocity = msg.data

    def steering_callback(self,msg):

        self.des_steer = msg.data

    def main(self):
        
        rate = rospy.Rate(self.FrameRate)
        TURN_GPS_BLACKOUT = 100000 # GPS long, lat = 0,0 -> x,y = -6530936, 15863029

        while not rospy.is_shutdown():

            if self.switch_WP==1:
                WP_X=WP_X_1
                WP_Y=WP_Y_1
            elif self.switch_WP==2:
                WP_X=WP_X_2
                WP_Y=WP_Y_2
            else:
                WP_X=WP_X_3
                WP_Y=WP_Y_3                
            self.swg = SWG_make(WP_X,WP_Y) 

            old_WP_index = self.old_WP_index

            if self.odom_status:
                x_ego, y_ego, ego_head = self.sub_odom_status(self.odom_status)

                WP_index=self.swg.WAYPOINT_INITIALIZING(WP_X,WP_Y,x_ego,y_ego)

                if (old_WP_index!=WP_index or min_index>50 or init_signal==1):
                    WP_x,WP_y=self.swg.trajectory_make(WP_X,WP_Y,WP_index,x_ego,y_ego,ego_head)
                    init_signal=0
                    TG_signal=1
                else:
                    TG_signal=0

                if self.target_velocity == None:
                    target_velocity = 19
            
                else:
                    target_velocity = self.target_velocity

                # pure pursuit

                self.ctrl_msg.steering,min_index = pure_pursuit(WP_x, WP_y, target_velocity, x_ego,y_ego,ego_head) # ego_status_vel_x, ego_status_vel_y
                
                if target_velocity == 0:
                    print 'STOP (target velocity is 0)'

                    self.ctrl_msg.longlCmdType = 1

                    self.ctrl_msg.accel=0
                    self.ctrl_msg.brake=1
                    stop_sign=1
                    self.stop_signal.publish(stop_sign)
                    
                elif target_velocity == 25:

                    self.ctrl_msg.longlCmdType = 1

                    self.ctrl_msg.accel=0.8
                    self.ctrl_msg.brake=0                

                else:
                    self.ctrl_msg.longlCmdType = 2
                    self.ctrl_msg.velocity = target_velocity

                old_WP_index=WP_index

                # switch WP
                if WP_index >=45 and self.switch_WP==1:
                    self.switch_WP=2
                    print 'swtich WP'
                elif WP_index >=51 and self.switch_WP==2:
                    self.switch_WP=3
                else:            
                    print ''

                mission = select_mission(old_WP_index, self.switch_WP)

            #if mission == 6: #  -> else:
            if abs(x_ego) > TURN_GPS_BLACKOUT and abs(y_ego) > TURN_GPS_BLACKOUT:

                self.ctrl_msg.steering = self.des_steer

            self.mission_pub.publish(mission)

            # Control message Publish

            self.ctrl_pub.publish(self.ctrl_msg)
            print 'WP index : ',old_WP_index
            rate.sleep()

if __name__ == '__main__':

    try:
        rospy.init_node('Control')
        Control()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass