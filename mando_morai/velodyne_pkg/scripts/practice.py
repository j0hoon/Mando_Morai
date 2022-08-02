#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from std_msgs.msg import Float64MultiArray

class Maneuver:

    def __init__(self):
        self.cluster_sub=rospy.Subscriber("clusters",Float64MultiArray,self.callback)
        self.cluster_msg = Float64MultiArray()        
        self.clustering_message=None
        self.FrameRate = 40

    def callback(self,msg):
        self.clustering_message=msg # [] MultiArray
        #print(self.clustering_message)

    def sub_cluster(self,msg):
        clustering_data=msg
        return clustering_data

def main():
    rospy.init_node('Maneuver')        
    rate=rospy.Rate(1)
    maneuver_planner=Maneuver()

    while not rospy.is_shutdown():
        #cluster_msg=self.sub_cluster(self.clustering_message)
        #rospy.loginfo(cluster_msg)
        rospy.loginfo('asd')
        rate.sleep()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

