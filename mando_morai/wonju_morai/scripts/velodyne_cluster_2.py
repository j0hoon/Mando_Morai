#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float64MultiArray
from sklearn.cluster import DBSCAN

class SCANCluster:

    def __init__(self):

        self.scan_sub=rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)

        self.cluster_pub=rospy.Publisher("clusters",Float64MultiArray,queue_size=10)

        self.pc_np=None

        self.cluster_msg = Float64MultiArray()

        self.dbscan = DBSCAN(eps=0.75, min_samples=5)


    def callback(self,msg):

        self.pc_np=self.pointcloud2_to_xyz(msg)

        pc_xy=self.pc_np[:,:2]
        #pc_xyz=self.pc_np[:,:3]
        db=self.dbscan.fit_predict(pc_xy)
        #db=self.dbscan.fit_perdict(pc_xyz)
        #print("num 1",list(db).count(1))
        #print("num 0",list(db).count(0))
        n_cluster=np.max(db)+1

        cluster_list=[]
        cluster_horizontial_list=np.array([])

        for c in range(n_cluster):

            c_tmp=np.mean(pc_xy[db==c, :], axis=0)
            c_tmp=c_tmp.tolist()

            if cluster_horizontial_list.size<=0:
                if c_tmp[0] > 0.75:
                    cluster_list+= c_tmp
                    cluster_horizontial_list=np.append(cluster_horizontial_list,c_tmp[1])                
            else:

                if min(abs(c_tmp[1]-cluster_horizontial_list))>1.25 and c_tmp[0] > 0.75:
                    cluster_list+= c_tmp
                    cluster_horizontial_list=np.append(cluster_horizontial_list,c_tmp[1])

        self.cluster_msg.data=cluster_list
        print(self.cluster_msg)
        self.cluster_pub.publish(self.cluster_msg)

    def pointcloud2_to_xyz(self, cloud_msg):

        point_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True):

            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)

            angle = np.arctan2(point[1], point[0])

            if point[0] > 0 and point[2] > -1.3 and dist < 15: # init=50 , if point[0] > 0 and point[2] > -1.3 and dist < 25 and angle>-30/180*np.pi and angle<30/180*np.pi:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)

        return point_np


if __name__=='__main__':

    rospy.init_node('velodyne_cluster',anonymous=True)
    rate=rospy.Rate(10)
    scan_cluster=SCANCluster()

    rospy.spin()