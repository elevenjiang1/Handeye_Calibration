#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Here get to pose of the camera,and check their pose
"""
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose

Marker_distance=0.140#m
Allow_acc=0.001
class CheckCalibration:
    def __init__(self):
        rospy.init_node("CheckCalibration")
        self.sub_pose1=rospy.Subscriber("/aruco_simple/pose",Pose,callback=self.sub_pose1_cb)
        self.sub_pose2=rospy.Subscriber("/aruco_simple/pose2",Pose,callback=self.sub_pose2_cb)
        self.pose1=None
        self.pose2=None
        
    def sub_pose1_cb(self,data):
        self.pose1=np.array([data.position.x,data.position.y,data.position.z])

    def sub_pose2_cb(self,data):
        self.pose2=np.array([data.position.x,data.position.y,data.position.z])

    def get_poses(self):
        try:
            target=np.linalg.norm(self.pose2-self.pose1)
            if abs(target-Marker_distance)>Allow_acc:#
                print("Error is:{:.4f}".format(target-Marker_distance))
            # print("target:{}".format(target))
        except Exception as e:
            print(e)
            
        
def get_poses():
    checkCalibration=CheckCalibration()
    while not rospy.is_shutdown():
        checkCalibration.get_poses()

        rospy.sleep(0.1)

def get_trans_from_tf():
    rospy.init_node("TF")
    listener=tf.TransformListener()
    rate=rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            trans,rot=listener.lookupTransform('marker1_frame','marker2_frame',rospy.Time(0))
            print("Trans:{}".format(trans))
        except Exception as e:
            print(e)
        rospy.sleep(0.1)




if __name__ == "__main__":
    get_poses()
    # get_trans_from_tf()









