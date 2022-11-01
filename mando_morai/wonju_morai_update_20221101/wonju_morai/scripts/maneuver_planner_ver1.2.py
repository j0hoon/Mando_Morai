#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from morai_msgs.msg import EventInfo, Lamps
from morai_msgs.srv import MoraiEventCmdSrv


# use with velodyne_cluster_2.py and control_test_ver1.3.py

####### Parameter #######
lane_curvature=0.00001 # tmp value -> please change value after vision recognition

#########################

class Maneuver:

    def __init__(self):
        self.cluster_sub=rospy.Subscriber("clusters",Float64MultiArray,self.callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.mission_sub = rospy.Subscriber('/mission', Int32, self.mission_callback)
        self.target_vel_pub = rospy.Publisher('/target_vel', Int32, queue_size=1)
        self.cluster_msg = Float64MultiArray()        
        self.clustering_message=None
        self.odom_status = None
        self.mission_status = None
        self.FrameRate = 40
        self.gear_tirgger = 1 # init

        while not rospy.is_shutdown():
            if self.clustering_message and self.odom_status and self.mission_status: # if self.clustering_message and self.odom_status and self.mission_status:
                break
            else:
                rospy.loginfo("Waiting for message.")
        self.main()

    def callback(self,msg):
        self.clustering_message=msg # [] MultiArray
        #print(self.clustering_message)

    def sub_cluster(self,msg):
        clustering_data=msg.data
        return clustering_data

    def odom_callback(self,msg): # GPS pose, IMU heading angle data

        self.odom_status = msg

    def sub_odom_status(self,msg):
        self.heading_angle = msg.pose.pose.orientation.z
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y       

        return self.x, self.y, np.deg2rad(self.heading_angle)

    def mission_callback(self,msg):
        self.mission_status = msg.data

# Threat Assessment function
# Maximum lane Width = 3.5
    def threatAssessment_and_recognitionObject(self,cluster_array,V_ego,lane_curvature): # How to know the speed of the other vehicle??? for TTC
        surround_vehicle_list=[]
        epsilon=3.5 # lane offset
        laneWidth=2.5
        sideWidth=1.25
        IVT_MIN_TIME=1
        MSD_MIN_DISTANCE=12.5

        for i in range(len(cluster_array)//2):
            threat_x=cluster_array[i*2]
            threat_y=cluster_array[i*2+1]
            D_nearest=np.sqrt(threat_x**2+threat_y**2)
            MSD=D_nearest
            #TTC=D_nearest/(V_ego-V_nearest)
            IVT=D_nearest/(V_ego+0.001)

            # Decision RISK
            if IVT<IVT_MIN_TIME:
                Risk_IVT=1
            else:
                Risk_IVT=0
            
            if MSD<MSD_MIN_DISTANCE:
                Risk_MSD=1
            else:
                Risk_MSD=0
            
            RISK_F=Risk_IVT+Risk_MSD
            if RISK_F>=1:
                RISK=1
            else:
                RISK=0
            
            # Recognition Object(only right next to ego vehicle) -> tmp
            if threat_x>0:
                if threat_y > -1*laneWidth-sideWidth and threat_y < -1*laneWidth+sideWidth:
                    object_state="FVR"
                elif threat_y > laneWidth-sideWidth and threat_y < laneWidth+sideWidth:
                    object_state="FVL"
                elif threat_y > -sideWidth and threat_y < sideWidth:
                    object_state="FVI"
                else:
                    object_state=None
            else:
                if threat_y > -1*laneWidth-sideWidth and threat_y < -1*laneWidth+sideWidth:
                    object_state="RVR"
                elif threat_y > laneWidth-sideWidth and threat_y < laneWidth+sideWidth:
                    object_state="RVL"
                elif threat_y > -sideWidth and threat_y < sideWidth:
                    object_state="RVI"
                else:
                    object_state=None

            if object_state == "FVI":
                surround_vehicle_list.append([object_state,RISK])

        return surround_vehicle_list

    def main(self):      
        rate=rospy.Rate(25) # init rate=1
        rospy.wait_for_service('/Service_MoraiEventCmd')

        # gen message for publish


        #set_Event_control.option = 1 # default?

        while not rospy.is_shutdown():
            lamp_cmd = Lamps()
            set_Event_control = EventInfo()
            set_Event_control.lamps = lamp_cmd


            ros_srv = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)

            x_ego, y_ego, ego_head = self.sub_odom_status(self.odom_status)
            const_ego_velocity = 5/3.6


            cluster_msg=self.sub_cluster(self.clustering_message)
            vehicle_list=self.threatAssessment_and_recognitionObject(cluster_msg,const_ego_velocity,lane_curvature) # recognize FVI, Pedestrian

            rospy.loginfo(vehicle_list)


            # mission behavior

            if self.mission_status==11: # 

                set_Event_control.option = 2                        
                set_Event_control.gear = 4 

            elif self.mission_status==1:

                set_Event_control.option = 4                        
                lamp_cmd.turnSignal = 1                 

            else:
                set_Event_control.option = 4                           
                lamp_cmd.turnSignal = 0                
            
            target_vel = 19

            res = ros_srv(set_Event_control)
            self.target_vel_pub.publish(target_vel)
            print 'current mission : ',self.mission_status
            rate.sleep()



if __name__=='__main__':
    try:
        rospy.init_node('Maneuver') 
        Maneuver()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
