#!/usr/bin/env python2

import rospy
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
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
K = 0.4 # init = 0.5
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

class pidController : 
    def __init__(self):
        self.p_gain= 0.1
        self.i_gain=0.0
        self.d_gain=0.0

        self.controlTime=0.033
        self.prev_error=0
        self.i_control=0


    def pid(self,target_vel,current_vel):
        error= target_vel-current_vel
        
        p_control=self.p_gain*error
        self.i_control+=self.i_gain*error*self.controlTime
        d_control=self.d_gain*(error-self.prev_error)/self.controlTime

        output=p_control+self.i_control+d_control
        self.prev_error=error
        
        return output

class Control:
    
    def __init__(self):

        self.ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.mission_pub = rospy.Publisher('/mission', Int32, queue_size=1)
        self.target_vel_sub = rospy.Subscriber('/target_vel', Int32, self.TV_callback)
        #self.vel_sub = rospy.Subscriber('/vel', Float32, self.velo_callback)

        self.pid = pidController()
        self.target_velocity = 19
        #self.target_velocity = None
        self.ctrl_msg= CtrlCmd()
        # CtrlCmd message - longlCmdtype, velocity
        self.ctrl_msg.longlCmdType = 2
        self.ctrl_msg.velocity = self.target_velocity
        self.FrameRate = 20
        self.min_index = 0
        self.init_signal=1
        self.old_WP_index = 0
        self.switch_WP=1
        self.odom_status = None
        #self.TV_status = None
        while True:
            if self.odom_status:
                break

        self.main()

    def sub_ego(self, msg):

        ego_status_vel_x = msg.velocity.x
        ego_status_x     = msg.position.x
        ego_status_y     = msg.position.y
        ego_heading      = np.deg2rad(msg.heading)
        
        return ego_status_vel_x, ego_status_x, ego_status_y, ego_heading

    def callbackego(self,msg):

        self.Ego_status = msg

    def odom_callback(self,msg): # GPS pose, IMU heading angle data

        self.odom_status = msg

    def sub_odom_status(self,msg):
        self.heading_angle = msg.pose.pose.orientation.z
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y       

        return self.x, self.y, np.deg2rad(self.heading_angle)

    def TV_callback(self,msg):
        self.TV_status = msg

    def TV_status(self,msg):
        self.target_velocity = msg.data

    def main(self):
        
        rate = rospy.Rate(self.FrameRate)

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
            x_ego, y_ego, ego_head = self.sub_odom_status(self.odom_status)
            #ego_status_vel_x = self.sub_velo_status(self.velo_status)

            WP_index=self.swg.WAYPOINT_INITIALIZING(WP_X,WP_Y,x_ego,y_ego)
            if (old_WP_index!=WP_index or min_index>50 or init_signal==1):
                WP_x,WP_y=self.swg.trajectory_make(WP_X,WP_Y,WP_index,x_ego,y_ego,ego_head)
                init_signal=0
                TG_signal=1
            else:
                TG_signal=0

            self.ctrl_msg.steering,min_index = pure_pursuit(WP_x, WP_y, self.target_velocity, x_ego,y_ego,ego_head) # ego_status_vel_x, ego_status_vel_y
            #self.control_input=self.pid.pid(self.target_velocity, ego_status_vel_x) 
            '''
            if self.control_input > 0 :
                self.ctrl_msg.accel = self.control_input
                self.ctrl_msg.brake = 0
            else:
                self.ctrl_msg.accel = 0
                self.ctrl_msg.brake = -self.control_input
            self.ctrl_msg.accel=  self.control_input
            '''

            self.ctrl_pub.publish(self.ctrl_msg)   
        
            old_WP_index=WP_index

            # switch WP
            if WP_index >=45 and self.switch_WP==1: # init 109
                self.switch_WP=2
                print 'swtich WP'
            elif WP_index >=51 and self.switch_WP==2:
                self.switch_WP=3
            else:

                print 'old_WP_index : ',old_WP_index
                #print 'min index : ', min_index
                print 'x :', x_ego
                print 'y :', y_ego
                
            mission = select_mission(old_WP_index, self.switch_WP)
            self.mission_pub.publish(mission)
            rate.sleep()

if __name__ == '__main__':

    try:
        rospy.init_node('Control')
        Control()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass