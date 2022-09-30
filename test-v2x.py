############  conda create -n twin python=3.7
############  conda install -c esri nuscenes-devkit
############  conda install -c open3d-admin open3d
############  conda install matplotlib
############  conda install -c anaconda ipython

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')

import numpy as np
import os
from PIL import Image
import collections
import cv2
import open3d as o3d
import time


##########################
# print("Load a ply point cloud, print it, and render it")
# ply_point_cloud = o3d.data.PLYPointCloud()
# pcd = o3d.io.read_point_cloud(ply_point_cloud.path)
# print(pcd)
# print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries([pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])

###################




    # def translate(self, x: np.ndarray) -> None:
    #     """
    #     Applies a translation to the point cloud.
    #     :param x: <np.float: 3, 1>. Translation in x, y, z.
    #     """
    #     for i in range(3):
    #         self.points[i, :] = self.points[i, :] + x[i]

    # def rotate(self, rot_matrix: np.ndarray) -> None:
    #     """
    #     Applies a rotation.
    #     :param rot_matrix: <np.float: 3, 3>. Rotation matrix.
    #     """
    #     self.points[:3, :] = np.dot(rot_matrix, self.points[:3, :])

    # def transform(self, transf_matrix: np.ndarray) -> None:
    #     """
    #     Applies a homogeneous transform.
    #     :param transf_matrix: <np.float: 4, 4>. Homogenous transformation matrix.
    #     """
    #     self.points[:3, :] = transf_matrix.dot(np.vstack((self.points[:3, :], np.ones(self.nbr_points()))))[:3, :]


# b = np.repeat(depth[:, :, np.newaxis], 3, axis=2) # repeat to 3 dim if needed

# this is obtained from above nusc.get_sample_data
# camera_intrinsic_rgb = np.array([[1.14251841e+03, 0.00000000e+00, 8.00000000e+02],
#                                  [0.00000000e+00, 1.14251841e+03, 4.50000000e+02],
#                                  [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
# camera_intrinsic_rgb = np.array([[1142.5184053936916, 0.0, 800.0], [0.0, 1142.5184053936916, 450.0], [0.0, 0.0, 1.0]])
# from this intrinsic, we see, f = 1.14251841e+03, cx =  800 (half of image width), cy =  450 (half of image heigth),
# K = [[f, 0, Cx],
#      [0, f, Cy],
#      [0, 0, 1 ]]

# The projection matrix P is typically composed as follows:
# P = K * [R t]
# where K is a 3x3 intrinsic matrix given by

# [f 0 Cx
#  0 f Cy
#  0 0 1]

# which I have found (as there are several examples computing it), R is a 3x3 rotation matrix and t is a 3x1 translation

# Solved it! Turn out CARLA has inconsistent coordinate systems. Instead of P = K * [R t], you instead do P = K * F * [R t]
# where
# F = np.array([[ 0, 1, 0 ], [ 0, 0, -1 ], [ 1, 0, 0 ]], dtype=np.float32)


# f  = im_size_x /(2.0 * tan(fov * pi / 360))
# Cx = im_size_x / 2.0
# Cy = im_size_y / 2.0


########################################  INPUT IMG HAS TO BE JPG MODE
color_raw = o3d.io.read_image('testnb.jpg')
depth_raw = o3d.io.read_image("testnb2.jpg")
# color_raw = o3d.io.read_image('1RGBC.png')
# depth_raw = o3d.io.read_image("1D922.png")
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)
print(rgbd_image)
camera_intrinsic_rgb = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]])

#############################
#CARLAR camera settings: maximum 1000 meter, also see f above, should be 1140 meter actually
# if want to see point cloud, save depth as jpg, rather than png
# multiple 1000 as the maximum depth in original file is 1.0
# use uint16 to be the type: depth = depth.astype('uint16')
# show the "cdf" of depth values: x = np.sort(depth.ravel());plt.plot(x);plt.show()
# save in jpg lidar_to_camera
w = 1280
h = 720
# w = 980
# h = 520
fx = 600 
fy = 600
cx = 640
cy = 360
# cx = 490
# cy = 260
intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)
intrinsic.intrinsic_matrix = camera_intrinsic_rgb
cam = o3d.camera.PinholeCameraParameters()
cam.intrinsic = intrinsic
cam.extrinsic = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
# pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, cam.intrinsic)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, cam.intrinsic, cam.extrinsic)
# pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
print(np.array(pcd.points).shape)
# pcd = np.delete(np.array(pcd.points),-1,axis=1)
# print("1")
# pcd2 = o3d.geometry.PointCloud()
# pcd2.points = o3d.utility.Vector3dVector(pcd)
# o3d.io.write_point_cloud("922.pcd",pcd)
#############################

def demo_crop_geometry():
    print("Demo for manual geometry cropping")
    print(
        "1) Press 'Y' twice to align geometry with negative direction of y-axis"
    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    pcd_data = o3d.data.DemoICPPointClouds()
    pcd = o3d.io.read_point_cloud(pcd_data.paths[0])
    o3d.visualization.draw_geometries_with_editing([pcd])
# pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
#     rgbd_image,
#     o3d.camera.PinholeCameraIntrinsic(
#         o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))


# pcd = o3d.io.read_point_cloud("RIGHTC.ply")

# print(np.array(pcd.points).shape)
# o3d.visualization.draw_geometries_with_editing([pcd])
# pcd = o3d.io.read_point_cloud("cropped_2.ply")
# o3d.io.write_point_cloud("right.pcd", pcd)


# pcd = o3d.io.read_point_cloud("rightoriginal.ply")
# o3d.visualization.draw_geometries_with_editing([pcd])
# pcd = o3d.io.read_point_cloud("leftoriginal.ply")
# o3d.visualization.draw_geometries_with_editing([pcd])


# # print(np.array(pcd.points).shape)

# pcd = o3d.io.read_point_cloud("left.pcd")
# o3d.io.write_point_cloud("right.pcd", pcd)
# # # print(np.array(pcd.points).shape)
# o3d.visualization.draw_geometries_with_editing([pcd])


# o3d.visualization.draw_geometries([pcd])
# print(np.array(pcd.points)[10][2])
# print(np.array(pcd.points)[20][2])
# print(np.array(pcd.points)[30][2])W
# print(np.array(pcd.points)[40][2])
# print(np.array(color_raw))


print('done')

# color_raw = read_image("../..1/TestData/RGBD/color/00000.jpg")
# depth_raw = read_image("../../TestData/RGBD/depth/00000.png")
# rgbd_image = create_rgbd_image_from_color_and_depth(
#     color_raw, depth_raw); 
# pcd = create_point_cloud_from_rgbd_image(rgbd_image, PinholeCameraIntrinsic(
#         PinholeCameraIntrinsicParameters.PrimeSenseDefault))
# draw_geometries([pcd])



# next to do
# * segementation to crop objects
# * from other vehicle's camera, multi-view, how to combine multi-view, and show it out?
# * train MSN Morphing and Sampling Network for Dense Point Cloud



import carla

import unittest



class TestTransform(unittest.TestCase):
    def test_values(self):
        t = carla.Transform()
        self.assertEqual(t.location.x, 0.0)
        self.assertEqual(t.location.y, 0.0)
        self.assertEqual(t.location.z, 0.0)
        self.assertEqual(t.rotation.pitch, 0.0)
        self.assertEqual(t.rotation.yaw, 0.0)
        self.assertEqual(t.rotation.roll, 0.0)
        t = carla.Transform(carla.Location(y=42.0))
        self.assertEqual(t.location.x, 0.0)
        self.assertEqual(t.location.y, 42.0)
        self.assertEqual(t.location.z, 0.0)
        self.assertEqual(t.rotation.pitch, 0.0)
        self.assertEqual(t.rotation.yaw, 0.0)
        self.assertEqual(t.rotation.roll, 0.0)
        t = carla.Transform(rotation=carla.Rotation(yaw=42.0))
        self.assertEqual(t.location.x, 0.0)
        self.assertEqual(t.location.y, 0.0)
        self.assertEqual(t.location.z, 0.0)
        self.assertEqual(t.rotation.pitch, 0.0)
        self.assertEqual(t.rotation.yaw, 42.0)
        self.assertEqual(t.rotation.roll, 0.0)

    def test_print(self):
        t = carla.Transform(
            carla.Location(x=1.0, y=2.0, z=3.0),
            carla.Rotation(pitch=4.0, yaw=5.0, roll=6.0))
        s = 'Transform(Location(x=1.000000, y=2.000000, z=3.000000), Rotation(pitch=4.000000, yaw=5.000000, roll=6.000000))'
        self.assertEqual(str(t), s)


#   Location(x=34.845142, y=14.346951, z=0.001678)
# Rotation(pitch=0.000014, yaw=-164.843414, roll=-0.000061)
# Location(x=40.435448, y=15.861264, z=2.789332)
# ********************


# Location(x=-47.781830, y=20.284065, z=0.001689)
# Rotation(pitch=-0.001250, yaw=-134.697037, roll=0.001441)
# Location(x=-43.708084, y=24.401018, z=2.789348)
# ********************


    def test_list_rotation_and_translation_vector3d(self):
        error = .001
        t = carla.Transform(
            carla.Location(x=-43.708084, y=24.401018, z=2.789348),
            carla.Rotation(pitch=-0.001250, yaw=-134.697037, roll=0.001441))
        pcd = o3d.io.read_point_cloud("leftoriginal.ply")
        # point_list = [carla.Vector3D(0.0, 0.0, 2.0),
        #               carla.Vector3D(0.0, 10.0, 1.0),
        #               carla.Vector3D(0.0, 18.0, 2.0)
        #               ]
        point_list = pcd 
        t.transform(point_list)


        o3d.io.write_point_cloud("929test.pcd",  t.transform(point_list))
        # solution_list = [carla.Vector3D(-2.0, 0.0, -1.0),
        #                  carla.Vector3D(-1.0, 10.0, -1.0),
        #                  carla.Vector3D(-2.0, 18.0, -1.0)
        #                  ]

        # for i in range(len(point_list)):
        #     self.assertTrue(abs(point_list[i].x - solution_list[i].x) <= error)
        #     self.assertTrue(abs(point_list[i].y - solution_list[i].y) <= error)
        #     self.assertTrue(abs(point_list[i].z - solution_list[i].z) <= error)

a = TestTransform()
a.test_print()