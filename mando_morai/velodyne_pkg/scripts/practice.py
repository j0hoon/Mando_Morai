#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
#from std_msgs.msg import Float32
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus

class STATUS:

    def __init__(self):

        self.Ego_status = None
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.callback_ego) 

        while not rospy.is_shutdown():
        
            if self.Ego_status:
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

    def callback_ego(self, msg):       
        
        self.Ego_status = msg
        
class Adaptive_Cruise_control:

    def __init__(self):
        self.p_gain = 0.5
        self.i_gain = 0.06
        self.d_gain = 0.0
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02
        self.target_velocity = 30  #km/h
        self.default_distance = 13 # m
        self.safe_distance = 0

    def pid_control(self, target, current):

        error = target - current

        p_control = self.p_gain * error

        self.i_control += self.i_gain * error * self.controlTime

        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output

class Purepursuit:

    def __init__(self):


        self.global_path =rospy.Subscriber('/global_path',Path, self.global_path_callback)
        self.local_path= rospy.Subscriber('/local_path',Path, self.local_path_callback)
        self.odometry = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.forward_point = Point()
        self.current_postion = Point()
        self.is_look_forward_point = False
        self.local = Path()

        while not rospy.is_shutdown():
        
            if self.local:
                break
            else:
                rospy.loginfo("Waiting for LOCAL.")    


    def local_path_callback(self, msg):
        #self.test = True
        self.local= msg
        #rospy.loginfo("check callback") 

    def global_path_callback(self, msg):
        #self.is_path = True
        self.global_path_msg = msg

        rate = rospy.Rate(10) # 10hz

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def calc_pure_pursuit(self, vel, yaw, gain, min, max, length):

        #TODO: (2) 속도 비례 Look Ahead Distance 값 설정
        self.lfd = vel * gain
        
        if self.lfd < min : 
            self.lfd=min
        elif self.lfd > max :
            self.lfd=max
        #rospy.loginfo(self.lfd)
        
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        #TODO: (3) 좌표 변환 행렬 생성
        trans_matrix = np.array([
                [cos(yaw), -sin(yaw),translation[0]],
                [sin(yaw),cos(yaw),translation[1]],
                [0                    ,0                    ,1            ]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        if self.local != None:
            #print("success")
            for num,i in enumerate(self.local.poses) :
                path_point=i.pose.position

                global_path_point = [path_point.x,path_point.y,1]
                local_path_point = det_trans_matrix.dot(global_path_point)    

                if local_path_point[0]>0 :
                    dis = sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                    if dis >= self.lfd :
                        self.forward_point = path_point
                        self.is_look_forward_point = True
                        break
        
                #TODO: (4) Steering 각도 계산
                theta = atan2(local_path_point[1],local_path_point[0])
                steering = atan2((2*length*sin(theta)),self.lfd)
                print("angle = ", steering)
                return steering

        elif self.local == None:
            print("forgot local path")

class START:

    def __init__(self):
        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=1       
        self.ctrl_cmd_pub = rospy.Publisher("ctrl_cmd", CtrlCmd, queue_size=1)
        self.status = STATUS()
        self.acc = Adaptive_Cruise_control()
        self.pure = Purepursuit()

        self.main()
    def main(self):    

        rate=rospy.Rate(5) # init rate=1

        while not rospy.is_shutdown():
            ego_velocity, ego_velocity_x, ego_velocity_y, ego_velocity_z, ego_x, ego_y, ego_heading = self.status.sub_ego(self.status.Ego_status) 

            self.ctrl_cmd_msg.accel = self.acc.pid_control(self.acc.target_velocity,3.6*ego_velocity_x)  

            self.ctrl_cmd_msg.steering = self.pure.calc_pure_pursuit(ego_velocity_x, ego_heading, 0.078 , 5, 30, 2.7)

            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node("DRIVING")
        START()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass