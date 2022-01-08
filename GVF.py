import os
import sys
import json
from time import sleep
import numpy as np
import open3d as o3d
import argparse
from PIL import Image
import torch
from graspnetAPI import GraspGroup

import utils.ggFilter as ggFilter

import threading
from threadStop import *

#___________________Graspnet Init____________________#
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'dataset'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from graspnet import GraspNet, pred_decode
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', type=str,default= "logs/log_rs/checkpoint-rs.tar", help='Model checkpoint path')
parser.add_argument('--num_point', type=int, default=20000, help='Point Number [default: 20000]')
parser.add_argument('--num_view', type=int, default=300, help='View Number [default: 300]')
parser.add_argument('--collision_thresh', type=float, default=0.01, help='Collision Threshold in collision detection [default: 0.01]')
parser.add_argument('--voxel_size', type=float, default=0.01, help='Voxel Size to process point clouds before collision detection [default: 0.01]')
cfgs = parser.parse_args()
with open("./config.json",'r') as load_f:
    load_dict = json.load(load_f)
    intrinsic = load_dict['intrinsic_435']
# intrinsic = [[634.751,   0. ,        641.585],
#                         [  0.,         634.094, 362.22],
#                         [  0.,           0.,           1.        ]]
#[[f1  0  p1
#   0   f2 p2
#    0  0   1]]

def get_net():
    # Init the model
    net = GraspNet(input_feature_dim=0, num_view=cfgs.num_view, num_angle=12, num_depth=4,
            cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    # Load checkpoint
    checkpoint = torch.load(cfgs.checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    start_epoch = checkpoint['epoch']
    # print("-> loaded checkpoint %s (epoch: %d)"%(cfgs.checkpoint_path, start_epoch))
    # set model to eval mode
    net.eval()
    return net

def get_and_process_data_inner(color,depth,mask_dir):
    # load data
    with open("./config.json",'r') as load_f:
        load_dict = json.load(load_f)
        mask_name = load_dict['mask_name']
    workspace_mask = np.array(Image.open(os.path.join(mask_dir, mask_name)))

    color = np.array(color, dtype=np.float32) / 255.0
    color = color[:,:,::-1]
    factor_depth = [[1000.]]

    camera = CameraInfo(1280.0, 720.0, intrinsic[0][0], intrinsic[1][1], intrinsic[0][2], intrinsic[1][2], factor_depth)
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # get valid points
    mask = (workspace_mask & (depth > 0))
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    # sample points
    if len(cloud_masked) >= cfgs.num_point:
        idxs = np.random.choice(len(cloud_masked), cfgs.num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), cfgs.num_point-len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # convert data
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    end_points = dict()
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled = cloud_sampled.to(device)
    end_points['point_clouds'] = cloud_sampled
    end_points['cloud_colors'] = color_sampled

    return end_points, cloud

def get_grasps(net, end_points):
    # Forward pass
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    return gg

def collision_detection(gg, cloud):
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=cfgs.voxel_size)
    collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=cfgs.collision_thresh)
    gg = gg[~collision_mask]
    return gg

# def vis_grasps(gg, cloud,num_of_gg):
#     gg.nms()
#     gg.sort_by_score()
#     gg = gg[:num_of_gg]
#     grippers = gg.to_open3d_geometry_list()
#     o3d.visualization.draw_geometries([cloud, *grippers])

def vis_grasps(gg, cloud,num_of_gg):
    gg.nms()
    gg.sort_by_score()
    gg = gg[:num_of_gg]
    grippers = gg.to_open3d_geometry_list()
    # print(grippers[0])

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(cloud)

    for i in grippers:
        vis.add_geometry(i)
    ctr = vis.get_view_control()
    ctr.rotate(950,-120)
    ctr.translate(0,-100)
    vis.run()
    vis.destroy_window()


def demo_inner(color,depth,mask_dir,num_of_gg):
    net = get_net() # get the network

    end_points, cloud = get_and_process_data_inner(color,depth,mask_dir) # preprocess the rgbd image

    gg = get_grasps(net, end_points) # predict the graspgroup

    if cfgs.collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points))
    with open("./config.json",'r') as load_f:
        load_dict = json.load(load_f)
        coneAngle = load_dict['coneAngle']
    gg = ggFilter.filter(gg,coneAngle) # 圆锥过滤

    # vis_grasps(gg, cloud, num_of_gg)

    return gg[:num_of_gg]
