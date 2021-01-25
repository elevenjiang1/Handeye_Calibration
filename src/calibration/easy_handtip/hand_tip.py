#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Here to achieve the hand tip calibration
reference:https://blog.csdn.net/kalenee/article/details/92591866

"""
import sys
import rospy
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
sys.path.append("/home/robot/Documents/Project/Chess/code/ChessRobot/src/ur_modern_driver/scripts")
from UR_Robot import Robot
import quaternion

class Tool_cal():
    def __init__(self):
        pass

    def cal_tran(self):
        #1: Get the A,B
        tran_data=[]
        rotm_data=[]
        for i in range(len(self.tran_tran)-1):
            tran_data.append(self.tran_tran[i+1] - self.tran_tran[i])
            rotm_data.append(self.tran_rotm[i] - self.tran_rotm[i+1])
        
        A=np.vstack(rotm_data)
        B=np.vstack(tran_data)

        #2: Calcul Ax=b (method 1)
        L = np.array(np.zeros((3,3)))
        R = np.array(np.zeros((3,1)))
        for i in range(len(tran_data)):
            L = L + np.dot(rotm_data[i],rotm_data[i])
            R = R + np.dot(rotm_data[i],tran_data[i])
        correct_reuslt=np.linalg.inv(L).dot(R)
        print("correct delta:")
        print(B-A.dot(correct_reuslt))
        print("sum:",np.sum(np.abs(B-A.dot(correct_reuslt))))

        #3: Use X=inv(A.T*A)*A.T*B
        my_result=np.linalg.inv(A.T.dot(A)).dot(A.T).dot(B)
        print("my delta:")
        print(B-A.dot(my_result))
        print("sum:",np.sum(np.abs(B-A.dot(my_result))))
        print("my_result:",my_result.T)
        return np.linalg.inv(L).dot(R),my_result

    def cal_rotm(self, tran):
        # centre
        P_otcp_To_B = np.dot(self.rot_rotm[0],tran)+self.rot_tran[0]

        # cal the dircction vector of x
        P_xtcp_To_B = np.dot(self.rot_rotm[1],tran)+self.rot_tran[1]
        vector_X = P_xtcp_To_B - P_otcp_To_B
        dire_vec_x_o = np.linalg.inv(self.rot_rotm[0]).dot(vector_X) / np.linalg.norm(vector_X)

        # cal the dircction vector of z
        P_ztcp_To_B = np.dot(self.rot_rotm[2],tran)+self.rot_tran[2]
        vector_Z = P_ztcp_To_B - P_otcp_To_B
        dire_vec_z_o = np.linalg.inv(self.rot_rotm[0]).dot(vector_Z) / np.linalg.norm(vector_Z)

        # cal the dircction vector of y
        dire_vec_y_o = np.cross(dire_vec_z_o.T,dire_vec_x_o.T)

        # modify the dircction vector of z 
        dire_vec_z_o = np.cross(dire_vec_x_o.T,dire_vec_y_o)

        # cal rotation matrix
        tool_rot = np.array(np.zeros((3,3)))
        tool_rot[:,0] = dire_vec_x_o.T
        tool_rot[:,1] = dire_vec_y_o
        tool_rot[:,2] = dire_vec_z_o
        return tool_rot

    def calibration_main(self):
        #1: Control robot to get 6 pose
        print("Connect to robot...")
        robot=Robot(sim=False,init_node=True)
        print("Robot is ok")
        while not rospy.is_shutdown():
            print("Please move to save 6 pose,1-4 is on one point and 5 is x axis,6 is z axis")
            poses_list=[]
            while True:
                temp=raw_input("Input 's' for save pose.'done' for break .Last two pose will the x and z")
                if temp=="s":
                    robot.get_joints_state()
                    pose_endlink=robot._x
                    print("Saved robot pose endlink is:",pose_endlink)
                    poses_list.append(pose_endlink)
                    print("Now poses_list number is:{}".format(len(poses_list)))
                elif temp=="done":
                    if len(poses_list)<6:
                        print("less then 6 pose,continue to save pose!!")
                        continue
                    print("Finish save {} poses".format(len(poses_list)))
                    tool_poses=np.array(poses_list)
                    break

            print("Pose save already,Begin to calculate pose...")
            break

        #2: Calculate translation
        self.tran_tran=[]
        self.tran_rotm=[]
        tool_poses_tran = tool_poses[0:-2,:]#Except 
        for pose in tool_poses_tran:
            # set translation
            self.tran_tran.append(np.array([[pose[0]],[pose[1]],[pose[2]]]))
            
            # set rotation
            rotation_matrix=quaternion.matrix_from_quaternion(np.array([pose[3],pose[4],pose[5],pose[6]]))
            self.tran_rotm.append(rotation_matrix[:3,:3])

        tool_tran,my_tran=self.cal_tran()

        #3: Calculate rotation
        self.rot_tran=[]
        self.rot_rotm=[]
        tool_poses_rot = tool_poses[-3:,:]
        print(tool_poses_rot)
        for pose in tool_poses_rot:
            # set translation
            self.rot_tran.append(np.array([[pose[0]],[pose[1]],[pose[2]]]))
            
            # set rotation
            rotation_matrix=quaternion.matrix_from_quaternion(np.array([pose[3],pose[4],pose[5],pose[6]]))
            self.rot_rotm.append(rotation_matrix[:3,:3])

        #4: Get transformation(method1)
        print("***********Method 1****************")
        tool_rot=self.cal_rotm(tool_tran)
        tool_T = np.array(np.zeros((4,4)))
        tool_T[0:3,0:3] = tool_rot
        tool_T[0:3,3:] = tool_tran
        tool_T[3:,:] = [0,0,0,1]
        print tool_T
        tip_rpy=quaternion.euler_from_matrix(tool_T)
        tip_xyz=tool_tran
        print("\n Ori is:")
        print(tip_rpy)
        print("\n xyz is:")
        print(tip_xyz.T)

        #4: Get transformation(method2)
        print("***********Method 2****************")
        tool_rot=self.cal_rotm(my_tran)
        tool_T = np.array(np.zeros((4,4)))
        tool_T[0:3,0:3] = tool_rot
        tool_T[0:3,3:] = my_tran
        tool_T[3:,:] = [0,0,0,1]
        print tool_T
        tip_rpy=quaternion.euler_from_matrix(tool_T)
        tip_xyz=my_tran
        print("\n Ori is:")
        print(tip_rpy)
        print("\n xyz is:")
        print(tip_xyz.T)

if __name__ == "__main__":
    tool_cal=Tool_cal()
    tool_cal.calibration_main()
