import Imath
import OpenEXR
import argparse
import array
import numpy as np
import os
from open3d import *
import open3d as o3d
import math

import copy


def get_matrix_right():
    scale_x = 1
    scale_y = 1
    scale_z = -1
    yaw = 91.238411
    pitch=15.244285
    roll=0.000003

    c_y = np.cos(np.radians(yaw )) #-270
    s_y = np.sin(np.radians(yaw  )) #-270
    c_r = np.cos(np.radians(roll))
    s_r = np.sin(np.radians(roll))
    c_p = np.cos(np.radians(pitch))
    s_p = np.sin(np.radians(pitch))
    matrix = np.matrix(np.identity(4))
    matrix[0, 3] = -51.530857 #location.x
    matrix[1, 3] = 14.047577 #location.y
    matrix[2, 3] =2.789338 #location.z
    matrix[0, 0] = scale_x * (c_p * c_y)
    matrix[0, 1] = scale_x * (c_y * s_p * s_r - s_y * c_r)
    matrix[0, 2] = scale_x * (c_y * s_p * c_r + s_y * s_r)
    matrix[1, 0] = scale_y * (s_y * c_p)
    matrix[1, 1] = scale_y * (s_y * s_p * s_r + c_y * c_r)
    matrix[1, 2] = scale_y * (s_y * s_p * c_r - c_y * s_r)
    matrix[2, 0] = scale_z * (-s_p)
    matrix[2, 1] = scale_z * (c_p * s_r)
    matrix[2, 2] = scale_z * (c_p * c_r)
    return matrix

def get_matrix_left():
    scale_x = 1
    scale_y = 1
    scale_z = -1
    yaw =-94.132767
    pitch=15.244271
    roll=0.000002

    c_y = np.cos(np.radians(yaw )) #-270
    s_y = np.sin(np.radians(yaw )) #-270
    c_r = np.cos(np.radians(roll))
    s_r = np.sin(np.radians(roll))
    c_p = np.cos(np.radians(pitch))
    s_p = np.sin(np.radians(pitch))
    matrix = np.matrix(np.identity(4))
    matrix[0, 3] =-51.117847 #location.x
    matrix[1, 3] =35.227276 #location.y
    matrix[2, 3] =2.789366 #location.z
    matrix[0, 0] = scale_x * (c_p * c_y)
    matrix[0, 1] = scale_x * (c_y * s_p * s_r - s_y * c_r)
    matrix[0, 2] = scale_x * (c_y * s_p * c_r + s_y * s_r)
    matrix[1, 0] = scale_y * (s_y * c_p)
    matrix[1, 1] = scale_y * (s_y * s_p * s_r + c_y * c_r)
    matrix[1, 2] = scale_y * (s_y * s_p * c_r - c_y * s_r)
    matrix[2, 0] = scale_z * (-s_p)
    matrix[2, 1] = scale_z * (c_p * s_r)
    matrix[2, 2] = scale_z * (c_p * c_r)
    return matrix
pcd =o3d.io.read_point_cloud("930left.pcd")
pcd2 =o3d.io.read_point_cloud("930right.pcd")
matrix_left = get_matrix_left()
matrix_right = get_matrix_right()

right_T =pcd.transform(matrix_left)
left_T = pcd2.transform(matrix_right)
o3d.io.write_point_cloud("right_T.pcd", right_T)
o3d.io.write_point_cloud("left_T.pcd", left_T)

o3d.visualization.draw_geometries([left_T,right_T])
# o3d.visualization.draw_geometries_with_editing([left_T,right_T])
# o3d.io.write_point_cloud("930left.pcd", pcd)

# o3d.io.write_point_cloud("930right.pcd", pcd2)
# pcd =o3d.io.read_point_cloud("rightcar_T.pcd")
# pcd2 =o3d.io.read_point_cloud("leftcar_T.pcd")

# pcd =o3d.io.read_point_cloud("01670042.ply")
# o3d.visualization.draw_geometries([pcd,pcd2])
# o3d.io.write_point_cloud("930right.pcd", pcd)
# pcd2 =o3d.io.read_point_cloud("930left.pcd")
# o3d.io.write_point_cloud("930left.pcd", pcd2)

# o3d.visualization.draw_geometries([pcd])
# o3d.visualization.draw_geometries([pcd2])
# o3d.visualization.draw_geometries_with_editing([pcd])
# pcd = np.asarray(pcd.points)
# print(pcd.shape)

# TT = get_matrix()
# pcd_T =pcd.transform(TT)
# o3d.io.write_point_cloud("930right_T.pcd", pcd)
# o3d.visualization.draw_geometries([pcd,pcd_T])
# print(TT)

# print(T)
# pcd_T =pcd.transform(T)
# pcd.paint_uniform_color([1,0,0])

# print("center",pcd.get_center())
# o3d.visualization.draw_geometries_with_editing([pcd_T])
# o3d.io.write_point_cloud("rightcar_T.pcd", pcd_T)


# print(T)


