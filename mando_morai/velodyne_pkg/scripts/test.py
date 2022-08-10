#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,ObjectStatusList
from std_msgs.msg import Float64MultiArray
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

lane_curvature=0.00001

class Maneuver:

    def __init__(self):
        self.cluster_sub=rospy.Subscriber("clusters",Float64MultiArray,self.callback)
        self.cluster_msg = Float64MultiArray()
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.callback_ego) 
        self.clustering_message=None
        self.Ego_status = None
        self.cluster_sub=rospy.Subscriber("clusters",Float64MultiArray,self.callback)
   
        while not rospy.is_shutdown():
        
            if self.clustering_message and self.Ego_status:
                break
            else:
                rospy.loginfo("Waiting for message.")    
          


    def sub_ego(self, msg):
        ego_velocity = np.sqrt(msg.velocity.x ** 2 + msg.velocity.y ** 2 + msg.velocity.z ** 2)
        ego_velocity_x = msg.velocity.x
        ego_velocity_y = msg.velocity.y
        ego_velocity_z = msg.velocity.z
        ego_x        = msg.position.x
        ego_y        = msg.position.y
        ego_heading  = msg.heading

        return ego_velocity, ego_velocity_x, ego_velocity_y, ego_velocity_z, ego_x, ego_y, ego_heading


    def callback(self,msg):
        self.clustering_message=msg 


    def sub_cluster(self,msg):
        clustering_data=msg.data
        
        return clustering_data


    def callback_ego(self, msg):       
        self.Ego_status = msg


    def car_list(self,cluster_array,V_ego,lane_curvature): # How to know the speed of the other vehicle??? for TTC
        surround_vehicle_list=[]
        epsilon=3.5 # lane offset
        laneWidth=3.5
        sideWidth=1.75
        IVT_MIN_TIME=3.6
        MSD_MIN_DISTANCE=20
        self.fvi_distance = 50

        for i in range(len(cluster_array)//2):
            threat_x=cluster_array[i*2]
            threat_y=cluster_array[i*2+1]
            D_nearest=np.sqrt(threat_x**2+threat_y**2)
            MSD=D_nearest
            #TTC=D_nearest/(V_ego-V_nearest)
            IVT=D_nearest/(V_ego+0.001)
            print(D_nearest)

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
                    self.fvi_distance = MSD
                    print("fvi = ", MSD)
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

        


class Adaptive_Cruise_Control:

    def __init__(self):

        self.p_gain = 0.5
        self.p = 0.1
        self.i_gain = 0.06
        self.i = 0.03
        self.d_gain = 0.0
        self.d = 0.0
        self.prev_error = 0
        self.p_error =0
        self.i_control = 0
        self.i_c = 0
        self.controlTime = 0.02
        self.target_velocity = 60  #km/h
        self.default_distance = 13 # m
        self.safe_distance = 0

        
    
    def velocity_control(self, target_velocity, current_velocity):

        print("velocity control on")
        error = target_velocity - current_velocity
        print("error = ", error, "current_velocity =", current_velocity)

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output_v = p_control + self.i_control + d_control
        self.prev_error = error
        print("output_v =", output_v)

        return output_v


    def spacing_control(self, safe_distance, front_distance): 

        print("spacing control on")


        err = front_distance - safe_distance
        print("err =", err, "m")

        p = self.p * err
        self.i_c += self.i * err * self.controlTime

        output_s = p + self.i_c
        self.p_error = err
        print("output_s = ", output_s)

        return output_s


    def driving(self, target_velocity, current_velocity, safe_distance, front_distance):

        if safe_distance > front_distance:
            return  self.spacing_control(safe_distance, front_distance)

        elif safe_distance < front_distance:
            return  self.velocity_control(target_velocity, current_velocity)    


class start:

    def __init__(self):      

        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1
        self.maneuver = Maneuver()
        self.adc = Adaptive_Cruise_Control()

        self.main()    

    def main(self):

        rate=rospy.Rate(5) # init rate=1
        while not rospy.is_shutdown():

            ego_velocity, ego_velocity_x, ego_velocity_y, ego_velocity_z, ego_x, ego_y, ego_heading = self.maneuver.sub_ego(self.maneuver.Ego_status)
            cluster_msg = self.maneuver.sub_cluster(self.maneuver.clustering_message)

            vehicle_list=self.maneuver.car_list(cluster_msg,ego_velocity,lane_curvature)

            self.adc.safe_distance = ego_velocity_x* self.adc.controlTime + self.adc.default_distance

            front_distance = self.maneuver.fvi_distance
            print("safe_ distance = ", self.adc.safe_distance, "m")

            control_input = self.adc.driving(self.adc.target_velocity, 3.6*ego_velocity_x, self.adc.safe_distance, front_distance)

            if control_input > 0:

                print("speed up")
                self.ctrl_cmd_msg.accel = control_input
                self.ctrl_cmd_msg.brake = 0.0

            else:
                print("speed down")
                self.ctrl_cmd_msg.accel = 0.0
                self.ctrl_cmd_msg.brake = -control_input 
                

            print("vel =", 3.6*ego_velocity_x)
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            rate.sleep()


if __name__=='__main__':
    try:

        rospy.init_node('Test') 
     
        start()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass