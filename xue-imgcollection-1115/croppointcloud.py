import open3d as o3d

# if __name__ == "__main__":
#     sample_ply_data = o3d.data.PLYPointCloud()
#     pcd = o3d.io.read_point_cloud(sample_ply_data.path)
#     # Flip it, otherwise the pointcloud will be upside down.
#     pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
#     print(pcd)
#     axis_aligned_bounding_box = pcd.get_axis_aligned_bounding_box()
#     axis_aligned_bounding_box.color = (1, 0, 0)
#     oriented_bounding_box = pcd.get_oriented_bounding_box()
#     oriented_bounding_box.color = (0, 1, 0)
#     print(
#         "Displaying axis_aligned_bounding_box in red and oriented bounding box in green ..."
#     )
#     o3d.visualization.draw(
#         [pcd, axis_aligned_bounding_box, oriented_bounding_box])

import open3d as o3d
import numpy as np 
# if __name__ == "__main__":
#     print("Load a ply point cloud, crop it, and render it")
#     sample_ply_data = o3d.data.DemoCropPointCloud()
#     pcd = o3d.io.read_point_cloud(sample_ply_data.point_cloud_path)
#     vol = o3d.visualization.read_selection_polygon_volume(
#         sample_ply_data.cropped_json_path)
#     chair = vol.crop_point_cloud(pcd)
#     # Flip the pointclouds, otherwise they will be upside down.
#     pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
#     chair.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

#     print("Displaying original pointcloud ...")
#     # o3d.visualization.draw([pcd])
#     print("Displaying cropped pointcloud")
#     o3d.visualization.draw([chair])

def get_max_and_min(file_path):
    ply_file = o3d.io.read_point_cloud(file_path)
    o3d.visualization.draw_geometries( [ply_file] )
    points = np.asarray(ply_file.points)
    resX = []
    resY = []
    resZ = []
    temp = [np.round(point[0], 8) for point in points]
    resX.append(max(temp))
    resX.append(min(temp))
    temp = [np.round(point[1], 8) for point in points]
    resY.append(max(temp))
    resY.append(min(temp))
    temp = [np.round(point[2], 8) for point in points]
    resZ.append(max(temp))
    resZ.append(min(temp))
    print("X轴的最大值为:{0}\t最小值为:{1}".format( resX[0], resX[1]))
    print("Y轴的最大值为:{0}\t最小值为:{1}".format( resY[0], resY[1]))
    print("X轴的最大值为:{0}\t最小值为:{1}".format( resZ[0], resZ[1]))
# def read_crop_visual(file_path: str, json_path: str) -> None:
#     # 读取点云文件
#     ply = o3d.io.read_point_cloud(file_path, 'ply')
#     # 配置 crop.json 文件
#     vol = o3d.visualization.read_selection_polygon_volume(json_path)
#     # 进行裁剪
#     cut = vol.crop_point_cloud(ply)
#     # 裁剪后可视化
#     o3d.visualization.draw_geometries(
#         [cut],
#         width=1280,
#         height=720,
#         zoom=0.7,
#         front=[0.5439, -0.2333, -0.8060],
#         lookat=[2.4615, 2.1331, 1.338],
#         up=[-0.1781, -0.9708, 0.1608]
#     )


# if __name__ == '__main__':
#     file_path = '/home/ins/carla/xue-imgcollection-1115/2.ply' 
# #     # json_path = "help/crop.json"

  
#     get_max_and_min(file_path)

    # read_crop_visual(file_path, json_path)
import open3d as o3d

if __name__ == "__main__":
    file_path = '/home/ins/carla/xue-imgcollection-1115/2.ply' 
    # file_path = o3d.io.read_point_cloud(file_path)
    # o3d.visualization.draw_geometries_with_editing([file_path])
    json_path =  '/home/ins/carla/xue-imgcollection-1115/cropped_1.json'
    pcd = o3d.io.read_point_cloud(file_path)
    vol = o3d.visualization.read_selection_polygon_volume(json_path)
    car = vol.crop_point_cloud(pcd)
    o3d.visualization.draw([car])
    # Flip it, otherwise the pointcloud will be upside down.
    # pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # print(pcd)
    # axis_aligned_bounding_box = pcd.get_axis_aligned_bounding_box()
    # axis_aligned_bounding_box.color = (1, 0, 0)
    # print(axis_aligned_bounding_box)
    # oriented_bounding_box = pcd.get_oriented_bounding_box()
    # print(oriented_bounding_box)
    # oriented_bounding_box.color = (0, 1, 0)
    # print(
    #     "Displaying axis_aligned_bounding_box in red and oriented bounding box in green ..."
    # )
    # o3d.visualization.draw(
    #     [pcd, axis_aligned_bounding_box, oriented_bounding_box])