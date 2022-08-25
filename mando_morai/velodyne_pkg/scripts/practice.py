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
from std_msgs.msg import Float64MultiArray,Float32
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
from scipy.optimize import linear_sum_assignment

####### Parameter #######
lane_curvature=0.00001 # tmp value -> please change value after vision recognition
INIT_DIST=0.0
THRESHOLD_ASSIGNMENT_DIST=5
#########################


class STATUS:

    def __init__(self):

        self.Ego_status = None
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.callback_ego) 

        while not rospy.is_shutdown():
        
            if self.Ego_status:
                break
            else:
                rospy.loginfo("Waiting Ego Status")    

    def sub_ego(self, msg):
        ego_velocity = np.sqrt(msg.velocity.x ** 2 + msg.velocity.y ** 2 + msg.velocity.z ** 2)
        ego_velocity_x = msg.velocity.x
        ego_velocity_y = msg.velocity.y
        ego_velocity_z = msg.velocity.z
        ego_x        = msg.position.x
        ego_y        = msg.position.y
        ego_z        = msg.position.z
        ego_heading  = msg.heading

        return ego_velocity, ego_velocity_x, ego_velocity_y, ego_velocity_z, ego_x, ego_y, ego_z, ego_heading

    def callback_ego(self, msg):       
        
        self.Ego_status = msg
        
class Maneuver:

    def __init__(self):
        self.cluster_sub=rospy.Subscriber("clusters",Float64MultiArray,self.callback)
        self.cluster_msg = Float64MultiArray()
        self.cluster_global_msg = Float64MultiArray()       
        self.clustering_message=None
        self.Ego_status = None
        self.P=[[0.05,0,0,0],[0,0.05,0,0],[0,0,0.05,0],[0,0,0,0.05]]
        self.dt=0.1 # tmp
        self.x_dot=0 # init
        self.y_dot=0
        self.data_storage=np.empty((0,8),float) # [x_esti, y_esti, x_dot, y_dot, x_z, y_z, P, 0.0] + ID, cnt...
        self.track=np.empty((0,8),float) # init        


        while not rospy.is_shutdown():
            if self.clustering_message :
                break
            else:
                rospy.loginfo("Waiting cluster message.")


    def callback(self,msg):
        self.clustering_message=msg # [] MultiArray
        #print(self.clustering_message)


    def sub_cluster(self,msg):
        clustering_data=msg.data
        return clustering_data

# Threat Assessment function
# Maximum lane Width = 3.5
    
    def local2global(self,cluster_array,ego_x,ego_y,ego_heading):
        global_ob_list=[]
        for i in range(len(cluster_array)//2):
            threat_x=cluster_array[i*2]
            threat_y=cluster_array[i*2+1]
            global_x=threat_x*np.cos(ego_heading)-threat_y*np.sin(ego_heading)+ego_x
            global_y=threat_x*np.sin(ego_heading)+threat_y*np.cos(ego_heading)+ego_y     
            global_coord=[global_x,global_y]
            global_ob_list+=global_coord

        return global_ob_list

    def global2local(self,global_x,global_y,ego_x,ego_y,ego_heading):
        rot_matrix=np.array([[np.cos(ego_heading),-np.sin(ego_heading),ego_x],[np.sin(ego_heading),np.cos(ego_heading),ego_y],[0,0,1]])
        global_coord=np.array([global_x,global_y,1]).reshape(3,1)
        res=np.linalg.inv(rot_matrix)@global_coord
        local_x,local_y=res[0],res[1]

        return local_x,local_y

    def Dvkalman(self,x,P,z,dt): # only distance
        
        ### Parameter ###
        A=np.array([[1,dt],[0,1]])
        H=np.array([1,0]).reshape(1,2)
        Q=np.array([[0.01,0],[0,0.03]]) # tmp value
        R = 0.03
        #################

        x_pred=np.dot(A,x)
        P_pred=np.dot(np.dot(A,P),A.T)+Q
        K=P_pred@(H.reshape(2,1))@np.linalg.inv(H@P_pred@(H.reshape(2,1))+R)

        x_esti=x_pred+K*(z-H@x_pred) # calculate estimate val, z=sensor input(Distance)
        P_esti=P_pred-K@H@P_pred # error convariance

        pos,vel=x_esti[0][0],x_esti[1][0]
        return pos,vel,P_esti 

    def DvKalman(self,x,P,z,dt):
        
        ### Parameter ###
        A=np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]])
        H=np.array([[1,0,0,0],[0,0,1,0]])
        Q=np.array([[0.1,0,0,0],[0,0.1,0,0],[0,0,0.1,0],[0,0,0,0.1]]) # tmp value
        R = np.array([[0.03,0],[0,0.03]])
        #################

        x_pred=np.dot(A,x)

        P_pred=np.dot(np.dot(A,P),A.T)+Q
        K=P_pred@H.T@np.linalg.inv(H@P_pred@H.T+R)
        x_esti=x_pred+K@(z-H@x_pred) # calculate estimate val, z=sensor input(Distance)
        P_esti=P_pred-K@H@P_pred # error convariance

        pos_x,vel_x,pos_y,vel_y=x_esti[0][0],x_esti[1][0],x_esti[2][0],x_esti[3][0]
        return pos_x,vel_x,pos_y,vel_y,P_esti 

    def threatAssessment_and_recognitionObject(self,cluster_array,V_ego,lane_curvature): # How to know the speed of the other vehicle??? for TTC
        surround_vehicle_list=[]
        epsilon=3.5 # lane offset
        laneWidth=3.5
        sideWidth=1.75
        IVT_MIN_TIME=3.6
        MSD_MIN_DISTANCE=20
        self.dis_f = 50

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
                    self.dis_f = np.sqrt(threat_x**2+threat_y**2)
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

        return surround_vehicle_list, self.dis_f

    def estimator(self, ego_x, ego_y, ego_heading):
        rate=rospy.Rate(10) # init rate=1
        while not rospy.is_shutdown():
            # ego, target status definition
  
            cluster_msg=self.sub_cluster(self.clustering_message)

            # local -> global coordinate
            cluster_global_msg=self.local2global(cluster_msg, ego_x, ego_y,ego_heading)

            #vehicle_list=self.threatAssessment_and_recognitionObject(cluster_msg,ego_status_vel,lane_curvature)
            
            for i in range(len(cluster_global_msg)//2):
                threat_x=cluster_global_msg[i*2]
                threat_y=cluster_global_msg[i*2+1]
                # data storage append
                self.data_storage=np.append(self.data_storage,np.array([[threat_x,self.x_dot,threat_y,self.y_dot,threat_x,threat_y,self.P,0.0]], dtype=object), axis=0)
            # data association
            data_list=[]
            matrix=[]

            for i in range(len(self.data_storage)):
                data_list.append([self.data_storage[i][0],self.data_storage[i][2]]) # global coordinate   

            if len(self.track)!=0 and len(data_list)!=0:
                for i in range(len(self.track)):
                    for j in range(len(data_list)):
                        dist=np.sqrt((self.track[i][0]-data_list[j][0])**2 + (self.track[i][2]-data_list[j][1])**2)
                        matrix.append(dist)
                
                matrix=np.array(matrix)
                matrix=matrix.reshape(len(self.track),len(data_list))

                row_ind, col_ind = linear_sum_assignment(matrix)

                # np.array -> list
                row_ind=row_ind.tolist()
                col_ind=col_ind.tolist()

                tmp_row_ind=[]
                tmp_col_ind=[]

                for i in range(len(row_ind)):                 
                    if matrix[row_ind[i]][col_ind[i]] < THRESHOLD_ASSIGNMENT_DIST: # init=5       
                        tmp_row_ind.append(row_ind[i])
                        tmp_col_ind.append(col_ind[i])

                row_ind, col_ind = tmp_row_ind, tmp_col_ind

                for idx in range(len(col_ind)):
                    self.track[row_ind[idx],4] = self.data_storage[col_ind[idx],0]
                    self.track[row_ind[idx],5] = self.data_storage[col_ind[idx],2]
                    
                if len(self.track)>len(row_ind):
                    del_index=[]
                    for i in range(len(self.track)):
                        if i not in row_ind:
                            del_index.append(i)
                    self.track=np.delete(self.track,del_index,axis=0) # ??
                
                if len(self.data_storage)>len(col_ind):
                    for i in range(len(self.data_storage)):
                        if i not in col_ind:
                            self.track=np.vstack([self.track,self.data_storage[i]])

            elif len(self.track)==0 and len(data_list)!=0:
                for data in self.data_storage:
                    self.track=np.vstack([self.track,data])

            self.data_storage=np.empty((0,8),float)

            # Kalman filter -> x,y estimate(x_esti, y_esti)

            for i in range(len(self.track)):
                z=self.track[i,4:6].reshape(2,1)
                x=self.track[i,0:4].reshape(4,1)
                P=self.track[i,6]
                # DvKalman(self,x,P,z,dt)
                x_esti,x_dot_esti,y_esti,y_dot_esti,P_esti=self.DvKalman(x,P,z,self.dt)

                self.track[i,0],self.track[i,1],self.track[i,2],self.track[i,3],self.track[i,6]=x_esti,x_dot_esti,y_esti,y_dot_esti,P_esti
                track_vel=np.sqrt(self.track[i,1]**2+self.track[i,3]**2)
                local_x,local_y=self.global2local(self.track[i,0],self.track[i,2], ego_x, ego_y,ego_heading)


            rate.sleep()
            return track_vel, local_x



class Adaptive_Cruise_control:

    def __init__(self):
        self.p_gain = 0.03
        self.i_gain = 0.02
        self.d_gain = 0.0
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02
        self.target_velocity = 60  #km/h
        self.default_distance = 13 # m
        self.safe_distance = 0

        self.error_s = 0
        self.p_gain_s = 0.5
        self.i_gain_s = 0.2
        self.d_gain_s = 0.1
        self.prev_error_s = 0
        self.i_control_s = 0

    def velocity_control(self, target_velocity, current_velocity):

        error = target_velocity - current_velocity

        p_control = self.p_gain * error

        self.i_control += self.i_gain * error * self.controlTime

        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output

    def spacing_control(self, safe_distance, distance):

        error_s = distance - safe_distance

        p_control = self.p_gain_s * error_s

        self.i_control_s += self.i_gain_s * error_s * self.controlTime

        d_control = self.d_gain_s * (error_s-self.prev_error_s) / self.controlTime

        output_s = p_control + self.i_control + d_control
        self.prev_error = error_s

        return output_s

    def driving(self, target_velocity, current_velocity, safe_distance, distance):

        print("dis = ", distance, "safe =", safe_distance )

        if safe_distance > distance:
            print("spacing")
            return  self.spacing_control(safe_distance, distance)

        elif safe_distance < distance:
            print("velocity control")
            return  self.velocity_control(target_velocity, current_velocity) 

    def error_sum(self, target_velocity, current_velocity, safe_distance, distance):

        vel_gain = 0.005
        spacing_gain = 0.8

        acc = vel_gain*(target_velocity-current_velocity) - spacing_gain*(safe_distance - distance)
        output = acc 

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
        self.vehicle_yaw = 0
        self.error_x = 0
        self.error_y = 0
        self.lfd = 8

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
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

        #rospy.loginfo(self.vehicle_yaw)

    def calc_pure_pursuit(self, vel,  gain, min, max, length):

        #TODO: (2) 속도 비례 Look Ahead Distance 값 설정
        self.lfd = vel * gain
        
        if self.lfd < min : 
            self.lfd=min
        elif self.lfd > max :
            self.lfd=max
        rospy.loginfo(self.lfd)
        
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        #TODO: (3) 좌표 변환 행렬 생성

        trans_matrix = np.array([
                [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                [0                    ,0                    ,1            ]])

        #rospy.loginfo(trans_matrix)

        det_trans_matrix = np.linalg.inv(trans_matrix)

        if self.local != None:
            #print("success")
            for num,i in enumerate(self.local.poses):
                path_point=i.pose.position

                global_path_point = [path_point.x,path_point.y,1]
                local_path_point = det_trans_matrix.dot(global_path_point)    

                #rospy.loginfo(local_path_point)

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

    def estimate_error(self, x, y):

        for num,i in enumerate(self.local.poses):
            self.error_x = abs((i.pose.position.x - x) / i.pose.position.x * 100)
            self.error_y = abs((i.pose.position.y-y) / i.pose.position.y * 100)

            return self.error_x, self.error_y


        #elif self.local == None:
            #print("forgot local path")

class START:

    def __init__(self):
        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=1       
        self.ctrl_cmd_pub = rospy.Publisher("ctrl_cmd", CtrlCmd, queue_size=1)
        self.status = STATUS()
        self.man = Maneuver()
        self.acc = Adaptive_Cruise_control()
        self.pure = Purepursuit()

        self.main()
    def main(self):    

        rate=rospy.Rate(5) # init rate=1

        while not rospy.is_shutdown():
            ego_velocity, ego_velocity_x, ego_velocity_y, ego_velocity_z, ego_x, ego_y, ego_z, ego_heading = self.status.sub_ego(self.status.Ego_status) 

            cluster_msg = self.man.sub_cluster(self.man.clustering_message)

            est_vel,local_x= self.man.estimator(ego_x, ego_y, ego_heading) 

            if est_vel > 0.1:
                self.acc.target_velocity = est_vel *3.6
                rospy.loginfo(self.acc.target_velocity)
            else:
                self.acc.target_velocity = 30

            _,f_d = self.man.threatAssessment_and_recognitionObject(cluster_msg,ego_velocity,lane_curvature)

            safe_distance = self.acc.default_distance + self.acc.controlTime * ego_velocity_x 

            control_input = self.acc.driving(self.acc.target_velocity, 3.6*ego_velocity_x, safe_distance, f_d)

            print("tv =",est_vel*3.6 )
            print("ego_staus =", ego_velocity_x*3.6, "km/h")

            self.ctrl_cmd_msg.steering = float(self.pure.calc_pure_pursuit(ego_velocity_x, 0.78 , 8, 30, 2.7))

            #control_input = self.acc.error_sum(self.acc.target_velocity, 3.6*ego_velocity_x, safe_distance, f_d)

            error_x, error_y = self.pure.estimate_error(ego_x, ego_y)

            print("error_x =", error_x, "%")
            print("error_y =", error_y, "%")

            print(control_input)

            if control_input > 0:

                #print("speed up")
                self.ctrl_cmd_msg.accel = control_input
                self.ctrl_cmd_msg.brake = 0.0

            else:
                #print("speed down")
                self.ctrl_cmd_msg.accel = 0.0
                self.ctrl_cmd_msg.brake = -control_input 


            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node("DRIVING")
        START()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass