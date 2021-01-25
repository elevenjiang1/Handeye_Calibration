#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Save the Transform as 4*4 Matrix.
Before Use,Remember to change cb_w,cb_h,cb_size!!
Written by JunnanJiang,2020.12.31.
Log 2020.12.31:
    remove to save H_uw,H_wu,just save the transform between camera and world(checkboard)
    H_bw(Transform between chessboard and world) need to modify to 4*4 in the future
"""
import rospy
import cv2
import numpy as np
import yaml
import os
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

filefolder = os.path.dirname(__file__)
filename = os.path.join(filefolder, '..', '..', 'calibration_files', 'transforms.yaml')
with open(filename, 'r') as yaml_file:
    data = yaml.load(yaml_file)
K = np.array(data['camera_matrix']['data']).reshape(3, 3)
D = np.array(data['distortion_coefficients']['data']).reshape(1, 5)
K_inv = np.linalg.inv(K)
H_bw = np.array([[0, 1, 0.046], [1, 0, 0.0512], [0, 0, 1]])
H_wb = np.linalg.inv(H_bw)
print('K and D Obtained!\n')

flag_calibration_done = False

def shutdown_node():
    print('This node is shutting down')

# Callback function
def img_callback(img_msg):
    global flag_calibration_done
    if not flag_calibration_done:
        #1: Init Checkerboard points and Camera Matrix
        cb_w = 11 
        cb_h = 8
        cb_size = 0.03
        # Checkerboard points
        objpoints = np.zeros((cb_w*cb_h, 3), np.float64)
        objpoints[:, :2] = np.mgrid[0:cb_w, 0:cb_h].T.reshape(-1, 2)
        objpoints = objpoints*cb_size  
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        Rmatrix = np.zeros((3, 3), np.float64)
        rvecs = np.zeros((3, 1), np.float64) 
        tvecs = np.zeros((3, 1), np.float64) 
        bridge = CvBridge()

        try:
            cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        #2: Find Checkerboard
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(cv_img, (cb_w, cb_h), None)

        if ret != True:
            print('FindChessboardCorners Failed!\n')
            return
        print('Chessboard Corners Found!\n')
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        print('Corners Optimized!\n')
        
        #3: Calculate extrinsic
        retval, rvecs, tvecs = cv2.solvePnP(objpoints, corners2, K, D)
        cv2.Rodrigues(rvecs, Rmatrix)
        H_cw=np.eye(4)
        H_cw[:3,:3]=Rmatrix
        H_cw[:3,3]=np.transpose(tvecs)#world(checkerboard) frame in camera frame
        H_wc = np.linalg.inv(H_cw)

        #4: Save data to transform
        RT_data = (""
            + "K_inv:\n"
            + "  rows: 3\n"
            + "  cols: 3\n"
            + "  data: [" + ", ".join(["%8f" % i for i in K_inv.reshape(1,9)[0]]) + "]\n"    
            + "H_cw:\n"     #The pose of world frame (checkerboard) in camera frame
            + "  rows: 4\n"
            + "  cols: 4\n"
            + "  data: [" + ", ".join(["%8f" % i for i in H_cw.reshape(1,16)[0]]) + "]\n"
            + "H_wc:\n"     #The pose of camera frame (optical) in world frame
            + "  rows: 4\n"
            + "  cols: 4\n"
            + "  data: [" + ", ".join(["%8f" % i for i in H_wc.reshape(1,16)[0]]) + "]\n"
            + "H_bw:\n"     #The pose of world frame in board frame
            + "  rows: 3\n"
            + "  cols: 3\n"
            + "  data: [" + ", ".join(["%8f" % i for i in H_bw.reshape(1,9)[0]]) + "]\n"
            + "H_wb:\n"     #The pose of board frame in world frame
            + "  rows: 3\n"
            + "  cols: 3\n"
            + "  data: [" + ", ".join(["%8f" % i for i in H_wb.reshape(1,9)[0]]) + "]\n"
            + "")
        with open(filename, 'a') as yaml_file:
            yaml_file.write(RT_data)
        flag_calibration_done = True
        print('Calibration Done!\n')
    #rospy.on_shutdown(shutdown_node)

# Main function
def main():
    rospy.init_node('easy_worldeye')
    rospy.Subscriber('/camera/color/image_raw', Image, img_callback)
    rospy.spin()

# RUN
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
