import numpy as np
from math import pi
from scipy.spatial.transform import Rotation as Rot
import json

np.set_printoptions(suppress=True)

DYNAMIC = False
STATIC = True

# 齐次变换到笛卡尔坐标
def transfrom2RPY(Trans,mode = "xyz"):
  Trans = np.matrix(Trans)
  R = Trans[:3,:3]              # rotation matrix
  T = np.matrix(Trans[:3,3]).T  # translation 

  Eur = Rot.from_matrix(R)
  Eur = Eur.as_euler(mode,False)

  T = np.array(T)

  return np.hstack((list(T[0]),Eur)) # 6DoF

# 算法输出到齐次变换
def getRR(R,T):
  R = np.array(R)
  T = np.matrix(T)
  E = [[ 0, -1 , 0],
      [ 0 , 0 ,-1],
      [ 1 , 0,  0]]
  E = np.array(E)

  R_fixed = np.matmul(R,E.T) # 修正之后的R
  print(R_fixed)
  res = np.vstack((np.hstack((R_fixed,T.T)),np.array([0,0,0,1])))

  return res

def getRR_V2(RR,TT):
    R = np.matrix(RR)
    T = np.matrix(TT)
    E1 = [[0,  0,  1],
        [ 0 , -1 ,0],
        [ 1 , 0,  0]]

    E2 = [[ 0,0, -1],
        [ 0 , 1 ,0],
        [ 1 , 0,  0]]

    r = Rot.from_euler('xyz',[0,0,np.pi/2])


    E1 = np.array(E1)
    E1 = np.matmul(r.as_matrix(),E1)
    E2 = np.array(E2)
    E2 = np.matmul(r.as_matrix(),E2)

    R1_fixed = np.matmul(R,E1.T) # 修正之后的R1
    R2_fixed = np.matmul(R,E2.T) # 修正之后的R2

    trans1 = np.vstack((np.hstack((R1_fixed,T.T)),np.array([0,0,0,1])))
    trans2 = np.vstack((np.hstack((R2_fixed,T.T)),np.array([0,0,0,1])))
    if(np.array(trans1)[0][0]>0):
        return trans1
    return trans2

# 获取齐次变换矩阵
# Euler : 角度
# T ： 平移
def getTransform(m:str,Euler,T,Mode:bool):
  if Mode == True:
      m = m.lower()
  else:
      m = m.upper()
  E = np.array(Euler)

  T = np.matrix(T)

  r = Rot.from_euler(m,E)

  return np.vstack((np.hstack((r.as_matrix(),T.T)),np.array([0,0,0,1])))

def finally_succ(curPos, RR, TT):
  with open("./config.json",'r') as load_f:
    load_dict = json.load(load_f)
    Bais = load_dict['cameraPose']
  TT =np.array(TT)*1000 + np.array(Bais)
  
  Trans1 = getTransform("xyz",curPos[3:],np.array(curPos[:3]),STATIC) # on base the tool

  Trans2 = getRR(RR,TT) # on camera the target
  # on tool the camera is I
  Trans3 = np.dot(Trans1,Trans2) # on base the target

  return transfrom2RPY(Trans3)


def GG_2_Liner_move(gg, curPos):
  with open("./config.json",'r') as load_f:
    load_dict = json.load(load_f)
    Bais = load_dict['cameraPose']
  RR = list(gg.rotation_matrices)[0]
  TT = list(gg.translations)
  return finally_succ(curPos,RR,TT,toolBais)


def finally_succ_V2(curPos, RR, TT):
  with open("./config.json",'r') as load_f:
    load_dict = json.load(load_f)
    Bais = load_dict['cameraPose']
  TT =np.array(TT)*1000 + np.array(Bais)
  
  Trans1 = getTransform("xyz",curPos[3:],np.array(curPos[:3]),STATIC) # on base the tool

  Trans2 = getRR_V2(RR,TT) # on camera the target
  # on tool the camera is I
  Trans3 = np.dot(Trans1,Trans2) # on base the target

  return transfrom2RPY(Trans3)


def GG_2_Liner_move_V2(gg, curPos):
  RR = list(gg.rotation_matrices)[0]
  TT = list(gg.translations)[0]
  return finally_succ_V2(curPos,RR,TT)

