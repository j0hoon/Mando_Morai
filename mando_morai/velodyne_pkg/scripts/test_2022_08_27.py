#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
import math
import time
from std_msgs.msg import Float64MultiArray,Float32
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList

####### Parameter #######
lane_curvature=10000 # tmp value -> please change value after vision recognition
INIT_DIST=0.0
THRESHOLD_ASSIGNMENT_DIST=5
TRACK_CNT_THRESHOLD=10 # tmp
#########################

# 1. Point Cloud Data input, Clustering(velodyne_cluster_2.py) -> local2global func -> data storage input
# 2. Data Association with Hungarian Algorithm -> Track update
# 3. Kalman filter -> x,y estimate(x_esti, y_esti) / track=np.array([x_esti,y_esti,x_dot,y_dot,x_data,y_data,P])
# 4. Kalman filter -> P estimate



class Maneuver:

    def __init__(self):
        self.cluster_sub=rospy.Subscriber("clusters",Float64MultiArray, self.callback)
        self.sub_ego = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.callback_ego)
        self.track_pub = rospy.Publisher('/track',Float64MultiArray, queue_size=10)
        self.cluster_msg = Float64MultiArray()
        self.cluster_global_msg = Float64MultiArray()
        self.track_msg = Float64MultiArray()
        self.clustering_message=None
        self.Ego_status = None
        self.P=[[0.05,0,0,0],[0,0.05,0,0],[0,0,0.05,0],[0,0,0,0.05]]
        self.dt=0.1 # tmp
        self.x_dot=0 # init
        self.y_dot=0
        self.init_cnt=2
        self.data_storage=np.empty((0,9),float) # [x_esti, y_esti, x_dot, y_dot, x_z, y_z, P, 0.0] + ID, cnt...
        self.track=np.empty((0,9),float) # init
        # self.data_storage,np.array([[threat_x,self.x_dot,threat_y,self.y_dot,threat_x,threat_y,self.P,threat_z,self.init_cnt]], dtype=object)
        self.init_time=0.0        

        # another class definition
        self.threat_assessment=ThreatAssessment()
        # def threatAssessment_decision(track_data,ego_vel)
        # def recognition_decision(track_data,lane_curvature)


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
        ego_heading         = np.deg2rad(ego_heading)
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
    
    def local2global(self,cluster_array,ego_x,ego_y,ego_heading):
        global_ob_list=[]
        for i in range(len(cluster_array)//3): # [1,2,3,4,5,6]
            threat_x=cluster_array[i*3]
            threat_y=cluster_array[i*3+1]
            threat_z=cluster_array[i*3+2]
            global_x=threat_x*np.cos(ego_heading)-threat_y*np.sin(ego_heading)+ego_x
            global_y=threat_x*np.sin(ego_heading)+threat_y*np.cos(ego_heading)+ego_y     
            global_coord=[global_x,global_y,threat_z]
            global_ob_list+=global_coord

        return global_ob_list

    def global2local(self,global_x,global_y,ego_x,ego_y,ego_heading):
        rot_matrix=np.array([[np.cos(ego_heading),-np.sin(ego_heading),ego_x],[np.sin(ego_heading),np.cos(ego_heading),ego_y],[0,0,1]])
        global_coord=np.array([global_x,global_y,1]).reshape(3,1)
        res=np.linalg.inv(rot_matrix)@global_coord
        local_x,local_y=res[0],res[1]

        return local_x,local_y

    def DvKalman(self,x,P,z,dt):
        
        ### Parameter ###
        A=np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]])
        H=np.array([[1,0,0,0],[0,0,1,0]])
        Q=np.array([[2.5,0,0,0],[0,2.5,0,0],[0,0,2.5,0],[0,0,0,2.5]]) # tmp value
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
        rate=rospy.Rate(10) # init rate=1
        while not rospy.is_shutdown():
            # ego, target status definition
            ego_status_vel, ego_status_x, ego_status_y, ego_heading = self.sub_egostatus(self.Ego_status)
            cluster_msg=self.sub_cluster(self.clustering_message)
            check_time=time.time()
            dt=check_time-self.init_time
            # local -> global coordinate
            cluster_global_msg=self.local2global(cluster_msg,ego_status_x,ego_status_y,ego_heading)

            #vehicle_list=self.threatAssessment_and_recognitionObject(cluster_msg,ego_status_vel,lane_curvature)
            
            for i in range(len(cluster_global_msg)//3):
                threat_x=cluster_global_msg[i*3]
                threat_y=cluster_global_msg[i*3+1]
                threat_z=cluster_global_msg[i*3+2]
                # data storage append
                self.data_storage=np.append(self.data_storage,np.array([[threat_x,self.x_dot,threat_y,self.y_dot,threat_x,threat_y,self.P,threat_z,self.init_cnt]], dtype=object), axis=0)
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
                    self.track[row_ind[idx],7] = self.data_storage[col_ind[idx],7] # threat_z
                    if self.track[row_ind[idx],8]<TRACK_CNT_THRESHOLD: # 10
                        self.track[row_ind[idx],8]+=1                   
                    
                if len(self.track)>len(row_ind):
                    print("track count down!")
                    print("@@@@@@@@@@@@@@@@")
                    print("@@@@@@@@@@@@@@@@")
                    print("@@@@@@@@@@@@@@@@")
                    print("@@@@@@@@@@@@@@@@")
                    print("@@@@@@@@@@@@@@@@")
                    print("@@@@@@@@@@@@@@@@")
                    del_index=[]
                    for i in range(len(self.track)):
                        if i not in row_ind:
                            del_index.append(i)
                    for idx in del_index:
                        self.track[idx][8]-=1

                    #self.track=np.delete(self.track,del_index,axis=0) # ??

                
                if len(self.data_storage)>len(col_ind):
                    print("data input in track!")
                    print("@@@@@@@@@@@@@@@@")
                    print("@@@@@@@@@@@@@@@@")
                    print("@@@@@@@@@@@@@@@@")
                    print("@@@@@@@@@@@@@@@@")
                    print("@@@@@@@@@@@@@@@@")
                    print("@@@@@@@@@@@@@@@@")
                    for i in range(len(self.data_storage)):
                        if i not in col_ind:
                            self.track=np.vstack([self.track,self.data_storage[i]])

            elif len(self.track)==0 and len(data_list)!=0:
                print("data list input!")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                for data in self.data_storage:
                    self.track=np.vstack([self.track,data])

            else:
                print("all track data count down!")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                for i in range(len(self.track)):
                    self.track[i][8]-=1

            self.data_storage=np.empty((0,9),float)

            # Track management
            track_delete_list=[]
            for i in range(len(self.track)):
                if self.track[i][8]<=0:
                    track_delete_list.append(i)
            self.track=np.delete(self.track,track_delete_list,axis=0)
            if len(track_delete_list)!=0:
                print("TRACK DELETE!!!!!")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")
                print("@@@@@@@@@@@@@@@@")            

            track_TTC=self.threat_assessment.threatAssessment_decision(self.track,ego_status_vel,ego_status_x,ego_status_y,ego_heading)
            track_Object_state=self.threat_assessment.recognition_decision(self.track,lane_curvature,ego_status_x,ego_status_y,ego_heading)

            # Kalman filter -> x,y estimate(x_esti, y_esti)
            print('----------------------------------')
            print('track length : ',len(self.track))
            for i in range(len(self.track)):
                z=self.track[i,4:6].reshape(2,1)
                x=self.track[i,0:4].reshape(4,1)
                P=self.track[i,6]
                # DvKalman(self,x,P,z,dt)
                x_esti,x_dot_esti,y_esti,y_dot_esti,P_esti=self.DvKalman(x,P,z,self.dt)

                self.track[i,0],self.track[i,1],self.track[i,2],self.track[i,3],self.track[i,6]=x_esti,x_dot_esti,y_esti,y_dot_esti,P_esti
                track_vel=np.sqrt(self.track[i,1]**2+self.track[i,3]**2)
                local_x,local_y=self.global2local(self.track[i,0],self.track[i,2],ego_status_x,ego_status_y,ego_heading)

                print('track num #',i)
                print("track x : ",self.track[i,0]," track y : ",self.track[i,2]," track z : ",self.track[i,7])
                print("track local x : ",local_x)
                print("track local y : ",local_y)
                print("track count : ",self.track[i][8])
                print("track TTC : ",track_TTC[i],"s(sec)")
                print("track Object state : ",track_Object_state[i])
                print("track velocity : ",track_vel*3.6,"km/h")
                print('')

            print('----------------------------------')
            print('')                  
            #self.global_ob_pub.publish()
            #rospy.loginfo(global_obj_list)

            # track=[x_esti,x_vel,y_esti,y_vel,x_real,y_real,P,threat_z,cnt]
            track_list=[]
            for i in range(len(self.track)):
                track_list.append(self.track[i,0]) # x_esti
                track_list.append(self.track[i,1])
                track_list.append(self.track[i,2]) # y_esti
                track_list.append(self.track[i,3])
                local_x,local_y = self.global2local(self.track[i,0],self.track[i,2],ego_status_x,ego_status_y,ego_heading)
                track_list.append(local_x[0])
                track_list.append(local_y[0])
                track_list.append(self.track[i,7])
                print('local x :',local_x[0]) 
                print('local y :',local_y[0]) 
                print('local z :',self.track[i,7]) 
                           
            self.track_msg.data = track_list
            self.track_pub.publish(self.track_msg)
            # publish track = [x_esti,x_dot_esti,y_esti,y_dot_esti,local_x,local_y,z] len = 7
            rate.sleep()

class ThreatAssessment:

    def __init__(self):
        self.RISK_TTC=1.0
        self.lane_curvature=10000
    # only TTC?
    def threatAssessment_decision(self,track_data,ego_vel,ego_x,ego_y,ego_heading): # track 정보의 위험도 판단
        # track_data = np.array([x_esti,x_dot_esti,y_esti,y_dot_esti,...])
        TTC_storage=[]
        for i in range(len(track_data)):
            x_dist,y_dist=self.global2local(track_data[i,0],track_data[i,2],ego_x,ego_y,ego_heading)
            track_vel=np.sqrt(track_data[i][1]**2+track_data[i][3]**2)

            track_TTC = x_dist[0]/abs(track_vel-(ego_vel/3.6)) # TTC=rel_dist/rel_vel
            print("TTC calculate")
            print("x distance : ",x_dist[0],"(m)")
            print("track velocity : ",track_vel,"(m/s)")
            print("ego velocity : ",ego_vel/3.6,"(m/s)")
            print("relative vel : ",abs(track_vel-(ego_vel/3.6)),"(m/s)")
            TTC_storage.append(track_TTC)

        return TTC_storage

    # lane curvature input
    def recognition_decision(self,track_data,lane_curvature,ego_x,ego_y,ego_heading):
        surround_vehicle_list=[]
        epsilon=3.5 # lane offset - tmp
        laneWidth=3.5
        sideWidth=1.75
        for i in range(len(track_data)):
            threat_x=track_data[i][0]
            threat_y=track_data[i][2]
            threat_x,threat_y=self.global2local(track_data[i,0],track_data[i,2],ego_x,ego_y,ego_heading)            
            threat_y=-1*threat_y
            # Recognition Object(only right next to ego vehicle) -> tmp
            if threat_x>0:

                if abs(lane_curvature)<epsilon:
                    threat_y = lane_curvature-np.sqrt(threat_x**2+(threat_y-lane_curvature)**2)
                
                if threat_y > -1*laneWidth-sideWidth and threat_y < -1*laneWidth+sideWidth:
                    object_state="FVL"
                elif threat_y > laneWidth-sideWidth and threat_y < laneWidth+sideWidth:
                    object_state="FVR"
                elif threat_y > -sideWidth and threat_y < sideWidth:
                    object_state="FVI"
                else:
                    object_state=None
            else:
                if threat_y > -1*laneWidth-sideWidth and threat_y < -1*laneWidth+sideWidth:
                    object_state="RVL"
                elif threat_y > laneWidth-sideWidth and threat_y < laneWidth+sideWidth:
                    object_state="RVR"
                elif threat_y > -sideWidth and threat_y < sideWidth:
                    object_state="RVI"
                else:
                    object_state=None


            surround_vehicle_list.append(object_state)

        return surround_vehicle_list

    def global2local(self,global_x,global_y,ego_x,ego_y,ego_heading):
        rot_matrix=np.array([[np.cos(ego_heading),-np.sin(ego_heading),ego_x],[np.sin(ego_heading),np.cos(ego_heading),ego_y],[0,0,1]])
        global_coord=np.array([global_x,global_y,1]).reshape(3,1)
        res=np.linalg.inv(rot_matrix)@global_coord
        local_x,local_y=res[0],res[1]

        return local_x,local_y







if __name__=='__main__':
    try:
        rospy.init_node('Maneuver') 
        Maneuver()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
