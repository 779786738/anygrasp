import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as Rot

tran =[-464.47939099282775, -111.71404919574113, 54.77162235746357, 2.0012532658214104, 0.011144949246064462, -1.0794229163107676]

r = Rot.from_euler('xyz',tran[3:])

temp = np.matmul(r.as_matrix(),np.mat([0,0,-1]).T).T

temp = np.array(temp)

print(temp)

def calcuAngle(v1,v2):
    v1 = np.array(v1)
    v2 = np.array(v2)
    temp = np.dot(v1,v2)
    return np.arccos(temp)/3.14*180

print(calcuAngle(list(temp[0]),np.array([0,0,1])))

points = [
    [0, 0, 0],
    [0, 0, 1],
    [0, 1, 0],
    [1, 0, 0],
    
]

points.append(list(temp[0]))

print(points)

lines = [
    [0, 1],
    [0, 2],
    [0, 3],
    [0, 4]
]

colors = [
    [0,0,1],
    [0,1,0],
    [1,0,0],
    [0,0,1]
]

line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),
)

line_set.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([line_set])