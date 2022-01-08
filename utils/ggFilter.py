from scipy.spatial.transform import Rotation as Rot
import numpy as np
def calcuAngle(v1,v2):
    v1 = np.array(v1)
    v2 = np.array(v2)
    temp = np.dot(v1,v2)
    return np.arccos(temp)/3.14*180

def inrange(g,range):
    # p = np.dot(R, p).T + center
    temp = np.dot(g.rotation_matrix,np.array([[1,0,0]]).T).T
    if calcuAngle(list(temp[0]),np.array([0,0,1]))<range:
        return True
    else:
        return False
def filter(gg,range):
    mask = []
    for g in gg:
        mask.append(inrange(g,range))
    return gg[mask]