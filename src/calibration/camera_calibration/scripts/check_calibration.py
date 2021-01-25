#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Here get to pose of the camera,and check their pose
"""
import rospy
from geometry_msgs.msg import Pose

class CheckCalibration:
    def __init__(self):
        self.sub_pose1=rospy.Subscriber("/aruco_simple/pose",Pose)

    def get_poses(self):
        pass


def get_poses():
    checkCalibration=CheckCalibration()
    while not rospy.is_shutdown():
        checkCalibration.get_poses()

        rospy.sleep(0.1)



if __name__ == "__main__":
    get_poses()









