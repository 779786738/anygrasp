#coding:UTF-8
'''
JAKA SDK Python API custom-made software development (CSD)
Date:
    2021-10-24 (Y-M-D)
Author:
    Xiaofeng Du, Shenghan Xie
Discription:
    JAKA robot movement control library.
'''

import os
import sys
import time
import numpy as np
import json
# sys.path.append("add path of JAKA SDK")
sys.path.append("/home/yons/grasp_ws/JAKA_SDK/python3_refresh")
os.system("export LD_LIBRARY_PATH=/home/yons/grasp_ws/JAKA_SDK/python3_refresh/")
try:
    from JAKA_SDK import jkrc
except:
    raise NameError("JAKA SDK path error! current work path: ")

class JAKA():
  # parameters
    _ABS = 0
    _INRC = 1
    _homePose = None
  # functions
    ''' Funtions list:
    # get pose
            # get [X, Y, Z] pose
        def getposXYZ(self)
            # get [Roll, Pitch, Yaw] pose
        def getposRPY(self)None
    # move
            # as name
        def moveInWorldCoordinate(self, INCRPose, speedInput = 50)
            # as name
        def moveInFlangeCoordinate(self, tarPose)
            # as name
        def moveInToolCoordinate(self, toolCorPose, tarPose)'''
    def __init__(self, address,connect = True):
        self.address = address
        self.robot = None
        if(connect):
            self.jaka_connect()

    def joint_move(self, joints,sp):
        #joints = 180*joints/pi
        self.robot.joint_move(joint_pos =joints, move_mode = 0,is_block= True ,speed = sp)
        time.sleep(0.08)

    def getjoints(self):
        ret = self.robot.get_joint_position()
        if ret[0] == 0:
            return ret[1]

    def liner_move(self, pos,sp):
        # add 180
        pos_now = self.robot.get_joint_position()
        if  pos_now[0]  == 0:
            print('now pos',pos_now)
            target_joint = self.robot.kine_inverse(pos_now[1], pos)
            if target_joint[0] == 0:
                # de = []
                # for de_value in target_joint[1]:
                #     de.append()
                de = target_joint[1][5] - pos_now[1][5]
                print("1::::",de /3.1415926 *180)
                if (abs(de) > (3.1415/2)):
                    joint_pos = []
                    for i in target_joint[1]:
                        joint_pos.append(i)
                    if de > 0:
                        joint_pos[5]  -= 3.1415
                    else:
                        joint_pos[5] += 3.1415

                    print("2::: ", (joint_pos[5] - pos_now[1][5])/3.1415926*180)
                    ret , tcp_pos = self.robot.kine_forward(joint_pos)
                else:
                    tcp_pos = pos

                print("move before: ", pos_now)
                ret = self.robot.linear_move(end_pos = tcp_pos, move_mode = self._ABS, is_block = True, speed = 120)
                # ret = self.robot.linear_move(end_pos = pos, move_mode = self._ABS, is_block = True, speed = sp)
                pos_now = self.robot.get_joint_position()
                print("move after:", pos_now )
                if(ret[0] == 0):
                    pass
                else:
                    print(ret[0])
            else:
                print(target_joint[0])
            # time.sleep(0.08)
        else:
            print('[JAKA] error!!!!!',pos_now)
    '''
    INCRPose : increase position [X, Y, Z, Roll, Pitch, Yaw]
    '''
    def moveInWorldCoordinate(self, INCRPose, speedInput = 20):
        curPose = self.getpos6DoF()
        tarPose = [0,0,0,0,0,0]
        for i in range(3):
          INCRPose[i] = curPose[i]+INCRPose[i]
        ret = self.robot.linear_move(end_pos = INCRPose, move_mode = self._ABS, is_block = True, speed = speedInput)
        if(ret[0] == -4):
            print("[JAKA] Inverse solution failed")
        return ret

    '''
    tarPose : target position [X, Y, Z, Roll, Pitch, Yaw]
    '''
    def moveInFlangeCoordinate(self, tarPose):
        RPY = self.getposRPY()
        # get rotationMatrix from world to flange
        rotationMatrix = np.array(self.robot.rpy_to_rot_matrix(RPY)[1]) # w - end

        tarXYZ = np.array(tarPose[:3])
        tarRPY =np.array(tarPose[3:])

        tarRM = self.robot.rpy_to_rot_matrix(tarRPY)[1] # end - P

        worldXYZ = np.matmul(rotationMatrix,tarXYZ)

        worldRM = np.matmul(rotationMatrix,tarRM)
        print(worldRM)
        worldRPY = self.robot.rot_matrix_to_rpy(worldRM)[1]
        worldPose = np.hstack((worldXYZ,worldRPY))

        self.moveInWorldCoordinate(worldPose)

    def jj(self,tarPos,curPos,speedInput=30):

        rotationMatrix = np.array(self.robot.rpy_to_rot_matrix(curPos[3:])[1]) # w - end
        tarXYZ = np.array(tarPos[:3])
        tarRM = np.eye(3)

        worldXYZ = np.matmul(rotationMatrix,tarXYZ)
        worldRM = np.matmul(rotationMatrix,tarRM)

        worldRPY = self.robot.rot_matrix_to_rpy(worldRM)[1]
        worldPose = np.hstack((worldXYZ,worldRPY))

        for i in range(3):
          worldPose[i] = curPos[i]+worldPose[i]
        ret = self.robot.linear_move(end_pos = worldPose, move_mode = self._ABS, is_block = True, speed = speedInput)
        if(ret[0] == -4):
            print("[JAKA] Inverse solution failed")
        return ret
    # '''
    # toolCor : tool coordinate's position in flange coordinate  [X, Y, Z, Roll, Pitch, Yaw]
    # tarPose : target position in tool coordinate [X, Y, Z, Roll, Pitch, Yaw]
    # '''
    # def moveInToolCoordinate(self, toolCorPose, tarPose):
    #     # get rotationMatrix from tool to flange
    #     rotationMatrix = np.array(self.robot.rpy_to_rot_matrix(toolCorPose[3:])[1])
    #     tarXYZ = np.array(tarPose[:3])
    #     tarRPY =np.array(tarPose[3:])
    #     tarRM = self.robot.rpy_to_rot_matrix(tarRPY)[1]
    #     FlangeRM = np.matmul(rotationMatrix,tarRM)
    #     FlangeRPY = self.robot.rot_matrix_to_rpy(FlangeRM)[1]
    #     FlangeXYZ = np.matmul(rotationMatrix,tarXYZ) + toolCorPose[:3]
    #     FlangePose = np.hstack((FlangeXYZ,FlangeRPY))
    #     self.moveInFlangeCoordinate(FlangePose)

# get pose
    # get [X, Y, Z] pose
    def getposXYZ(self):
        ret = self.robot.get_tcp_position()
        if ret[0] == 0:
            return ret[1][0:3]
      # get [Roll, Pitch, Yaw] pose
    def getposRPY(self):
        # ret = self.robot.get_tcp_position()
        ret = self.robot.get_robot_status()
        if ret[0] == 0:
            return ret[1][18][3:]
        else:
            print("positon get:",ret)
        # get [X, Y, Z, Roll, Pitch, Yaw] pose
    def getpos6DoF(self):
        ret = self.robot.get_robot_status()
        return ret[1][18]
    # disconnect from the robot
    def robot_disconnect(self):
        self.robot.power_off()
        print("[JAKA] power_off successfully")
        self.robot.logout()
        print("[JAKA] logout successfully")

    def jaka_connect(self):
        self.robot = jkrc.RC(self.address)
        print("[JAKA] logining...")
        if not self.robot.login():
            print("[JAKA] login successfully")

        if not self.robot.power_on():
            print("[JAKA] power_on successfully")

# set home position
    def setHome(self, homePose = None):
        if not homePose:
            self.homePose = self.getpos6DoF()
            print(self.homePose)
        else:
            self.homePose = homePose

    def goHome(self,speedInput=5):
        ret = self.robot.linear_move(end_pos = self.homePose, move_mode = self._ABS, is_block = True, speed = speedInput)
        if(ret[0] == -4):
            print("[JAKA] Inverse solution failed")
        return ret
