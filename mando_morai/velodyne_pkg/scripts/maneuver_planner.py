#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import Float64MultiArray
from morai_msgs.msg import EgoVehicleStatus

####### Parameter #######
lane_curvature=0.00001 # tmp value -> please change value after vision recognition

#########################

class Maneuver:

    def __init__(self):
        self.cluster_sub=rospy.Subscriber("clusters",Float64MultiArray,self.callback)
        self.sub_ego = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.callback_ego)
        self.cluster_msg = Float64MultiArray()        
        self.clustering_message=None
        self.Ego_status = None
        self.FrameRate = 40

        while not rospy.is_shutdown():
            if self.clustering_message and self.Ego_status:
                break
            else:
                rospy.loginfo("Waiting for message.")
        self.main()

    def sub_egostatus(self, msg):
        
        ego_status_velocity = np.sqrt(msg.velocity.x ** 2 + msg.velocity.y ** 2 + msg.velocity.z ** 2)
        ego_status_x        = msg.position.x
        ego_status_y        = msg.position.y
        ego_heading         = msg.heading
        return ego_status_velocity, ego_status_x, ego_status_y, ego_heading

    def callback(self,msg):
        self.clustering_message=msg # [] MultiArray
        #print(self.clustering_message)

    def callback_ego(self, msg):
        
        self.Ego_status = msg

    def sub_cluster(self,msg):
        clustering_data=msg.data
        return clustering_data

# Threat Assessment function
# Maximum lane Width = 3.5
    def threatAssessment_and_recognitionObject(self,cluster_array,V_ego,lane_curvature): # How to know the speed of the other vehicle??? for TTC
        surround_vehicle_list=[]
        epsilon=3.5 # lane offset
        laneWidth=3.5
        sideWidth=1.75
        IVT_MIN_TIME=3.6
        MSD_MIN_DISTANCE=20

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

            if object_state != None:
                surround_vehicle_list.append([object_state,RISK])

        return surround_vehicle_list


    def main(self):      
        rate=rospy.Rate(5) # init rate=1
        while not rospy.is_shutdown():
            ego_status_vel, ego_status_x, ego_status_y, ego_heading = self.sub_egostatus(self.Ego_status)
            cluster_msg=self.sub_cluster(self.clustering_message)
            vehicle_list=self.threatAssessment_and_recognitionObject(cluster_msg,ego_status_vel,lane_curvature)
            #rospy.loginfo(cluster_msg) clsuter_msg = Object 2D coordinate
            rospy.loginfo(vehicle_list)
            #rospy.loginfo(ego_status_vel)
            rate.sleep()

if __name__=='__main__':
    try:
        rospy.init_node('Maneuver') 
        Maneuver()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

