import sys
from trajectory_prediction.MPCCAR import MPC
import numpy as np
import pandas as pd
from plyfile import PlyData
from open3d import *
import open3d as o3d
import open3d as o3d
from pointcloud_completion.model import *
from pointcloud_completion.utils import *
import argparse
import random
import numpy as np
import torch
import os,pickle
import visdom
import time 
import copy
from argparse import ArgumentParser
import re 
from mmdet3d.apis import inference_detector, init_model, show_result_meshlab
from mmdet3d.core.visualizer.open3d_vis import Visualizer
import mmcv
import torch
from mmcv import Config, DictAction
from mmcv.cnn import fuse_conv_bn
from mmcv.parallel import MMDataParallel, MMDistributedDataParallel
from mmcv.runner import (get_dist_info, init_dist, load_checkpoint,
                         wrap_fp16_model)
from mmdet3d.core.bbox import box_np_ops as box_np_ops

def get_matrix(yaw,pitch,roll,x,y,z):
    scale_x = 1
    scale_y = 1
    scale_z = -1

    c_y = np.cos(np.radians(yaw )) #-270
    s_y = np.sin(np.radians(yaw  )) #-270
    c_r = np.cos(np.radians(roll))
    s_r = np.sin(np.radians(roll))
    c_p = np.cos(np.radians(pitch))
    s_p = np.sin(np.radians(pitch))
    matrix = np.matrix(np.identity(4))


    matrix[0, 3] = x #location.x
    matrix[1, 3] = y #location.y
    matrix[2, 3] = z #location.z
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

def _create_bbox_points(extent_z, extent_y,extent_x,location_x,location_y,location_z):

    cords = [
            [extent_x + location_x, extent_y + location_y, -extent_z + location_z],
            [-extent_x + location_x, extent_y + location_y, -extent_z + location_z],
            [-extent_x + location_x, -extent_y + location_y, -extent_z + location_z],
            [extent_x + location_x, -extent_y + location_y, -extent_z + location_z],
            [extent_x + location_x, extent_y + location_y, extent_z + location_z],
            [-extent_x + location_x, extent_y + location_y, extent_z + location_z],
            [-extent_x + location_x, -extent_y + location_y, extent_z + location_z],
            [extent_x + location_x, -extent_y + location_y, extent_z + location_z],
    ]
    return cords 
def resample_pcd(pcd, n):
        """Drop or duplicate points so that pcd has exactly n points"""
        idx = np.random.permutation(pcd.shape[0])
        if idx.shape[0] < n:
            idx = np.concatenate([idx, np.random.randint(pcd.shape[0], size = n - pcd.shape[0])])
        return pcd[idx[:n]]
def concatenate_pcd(pcdpath1,pcdpath2):
    pcd1 =o3d.io.read_point_cloud(pcdpath1)
    pcd2 =o3d.io.read_point_cloud(pcdpath2)
    # combining two pcd 
    start = time.time()
    p1_load = np.asarray(pcd1.points)
    p2_load = np.asarray(pcd2.points)
    p3_load = np.concatenate((p1_load,p2_load), axis=0)
    p1_color = np.asarray(pcd1.colors)
    p2_color = np.asarray(pcd2.colors)
    p3_color = np.concatenate((p1_color,p2_color), axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(p3_load)
    pcd.colors = o3d.utility.Vector3dVector(p3_color)
    end = time.time()
    print(end - start)
    # o3d.visualization.draw_geometries([pcd])
    return pcd 
def ego2world_matrix(x,y,z,yaw,pitch,roll):
    c_y = np.cos(np.radians(yaw))
    s_y = np.sin(np.radians(yaw))
    c_r = np.cos(np.radians(roll))
    s_r = np.sin(np.radians(roll))
    c_p = np.cos(np.radians(pitch))
    s_p = np.sin(np.radians(pitch))
    matrix = np.matrix(np.identity(4))
    matrix[0, 3] = x
    matrix[1, 3] = y
    matrix[2, 3] = z
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r
    return matrix



def ply_to_bin(input_path, output_path):
    plydata = PlyData.read(input_path)  # read file
    data = plydata.elements[0].data  # read data
    data_pd = pd.DataFrame(data)  # convert to DataFrame
    data_np = np.zeros(data_pd.shape, dtype=np.float32)  # initialize array to store data
    property_names = data[0].dtype.names  # read names of properties
    for i, name in enumerate(
            property_names):  # read data by property
        data_np[:, i] = data_pd[name]
    data_np.astype(np.float32).tofile(output_path)

# 

def Detection3d(frame,pcd,model):
    detected_obj_list = []
    result, data = inference_detector(model, pcd)
    score_thr= 0.14
    # show the results
    bbox_3d = result[0]['boxes_3d'].tensor.numpy()
    points = data['points'][0][0].cpu().numpy()
    num_obj = bbox_3d.shape[0]
    point_indices = box_np_ops.points_in_rbbox(points, bbox_3d)
    detected_obj_instance = [ ]
    detected_obj_bbox = [ ]
    for i in range(num_obj):
        ID = re.findall(r"id=(.+?),",pcd)
        # filename = str(i)+pcd_name[0][-6:]+'.bin'
        # save point clouds and image patches for each object
        gt_points = points[point_indices[:, i]] #
        # gt_points[:, :3] -= bbox_3d[i, :3]
        print(len(gt_points))
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(gt_points[:,:3])
        # make sure there is no bounding box outside of the map, due to the FP
        if len(pointcloud.points)>1:
            detected_obj_instance.append(pointcloud)

        
        # if len(gt_points)>79:
        # with open(filename, 'w') as f:
                
        #             gt_points.tofile(f)
        #             print(filename)
        #             print(len(gt_points))
        detected_obj_list.append("Frame"+str(frame)+"Id"+ID[0]+"Obj"+str(i))
        detected_obj_bbox.append(bbox_3d[i])
        
    # show_result_meshlab(
    #     data,
    #     result,
    #     'demo',
    #     score_thr,
    #     show=True,
    #     snapshot=True,
    #     task='det')
    return detected_obj_list,detected_obj_instance,detected_obj_bbox
class Adamap:
    def __init__(self,model_path = 'pointcloud_completion/trained_model/network.pth',opt_num_points = 8192,opt_n_primitives  = 16 ):
        self.model_path = model_path
        self.opt_num_points =opt_num_points
        self.opt_n_primitives  = opt_n_primitives
        self.opt_env = "Adamap_pointcloud_completion"
        self.network = AdaPoints(num_points = self.opt_num_points, n_primitives = self.opt_n_primitives) 
        self.network.cuda()
        self.network.apply(weights_init)
        self.vis = visdom.Visdom(port = 8097, env=self.opt_env) # set your port
        if self.model_path != '':
            self.network.load_state_dict(torch.load(self.model_path))
            print("Previous weight loaded ")
        self.network.eval()  
        self.labels_generated_points = torch.Tensor(range(1, (self.opt_n_primitives+1)*(self.opt_num_points//self.opt_n_primitives)+1)).view(self.opt_num_points//self.opt_n_primitives,(self.opt_n_primitives+1)).transpose(0,1)
        self.labels_generated_points = (self.labels_generated_points)%(opt_n_primitives+1)
        self.labels_generated_points = self.labels_generated_points.contiguous().view(-1)
        self.output_pcd = o3d.geometry.PointCloud()
    def val(self, detected_obj_instance, bboxs,demo_car_yaw):
        with torch.no_grad():
            output_list = []
            for i in range(len((detected_obj_instance))):
                pcd = detected_obj_instance[i]
                num  = 50
                partial = torch.zeros((num, 5000, 3), device='cuda')
                pcd = pcd.translate((-bboxs[i]["location_x"],-bboxs[i]["location_y"],-bboxs[i]["location_z"]))                                    
                pcd.scale(0.4,center = (0, 0 ,0)) 
                                               
                for j in range(num):
                    partial[j, :, :] = torch.from_numpy(resample_pcd(np.array(pcd.points), 5000))
                output1, expansion_penalty = self.network(partial.transpose(2,1).contiguous())
                output1 = output1.detach().cpu()
                """read coarse result """
                idx = random.randint(0, 49)
                OUTPUT = output1[idx].data.cpu().numpy() #TODO output completed result 
                self.output_pcd.points = o3d.utility.Vector3dVector(OUTPUT) 
                completed_pcd = copy.deepcopy(self.output_pcd)         
                print(self.opt_env + ' val [%d/%d]  expansion_penalty: %f' %(i + 1, len(detected_obj_instance), expansion_penalty.mean().item()))
                theta = -np.pi /2
                # R = np.array([[np.cos(theta), 0, np.sin(theta)],[0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])
                R = np.array([[1, 0, 0],[0, np.cos(theta), -np.sin(theta)],[0, np.sin(theta), np.cos(theta)]]) # a axis roate -90 
                completed_pcd.rotate(R)

                completed_pcd.scale(2.5,center = (0, 0 ,0)) 
                completed_pcd = completed_pcd.translate((bboxs[i]["location_x"],bboxs[i]["location_y"],bboxs[i]["location_z"]+3.5))   
                 
                


                angle_yaw_pcd =  bboxs[i]["yaw"]-np.deg2rad(demo_car_yaw)# 90 degrees in radians / metrics must be unified 
                rotation_matrix_pcd = np.array([[np.cos(angle_yaw_pcd), -np.sin(angle_yaw_pcd), 0],
                                            [np.sin(angle_yaw_pcd), np.cos(angle_yaw_pcd), 0],
                                            [0, 0, 1]])
                completed_pcd.rotate(rotation_matrix_pcd)
    
                # rotation_matrix_pcd = np.array([[np.cos(angle_yaw_pcd), -np.sin(angle_yaw_pcd), 0],
                #                             [np.sin(angle_yaw_pcd), np.cos(angle_yaw_pcd), 0],
                #                             [0, 0, 1]])
                # angle = np.pi / 2
                # R = np.array([[1, 0, 0],[0, np.cos(angle), -np.sin(angle)],[0, np.sin(angle), np.cos(angle)]])
                # completed_pcd.rotate(R)


                self.vis.scatter(X = partial[idx].data.cpu(), win = 'INPUT',
                            opts = dict(title = 'INPUT', markersize = 2))
                self.vis.scatter(X = np.asarray(completed_pcd.points),
                            Y = self.labels_generated_points[0:output1.size(1)],
                            win = 'OUTPUT',
                            opts = dict(title = 'OUTPUT', markersize=2))
                output_list.append(completed_pcd)
            return output_list

def point_cloud_consecutive_visualization():
    # filepath = '/home/ins/Adamap_Project/demo_data/show_seq_partial/' 
    filepath = '/home/ins/Adamap_Project/demo_data/show_seq/' 
    files = os.listdir(filepath)
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pointcloud = o3d.geometry.PointCloud()
    to_reset = True
    vis.add_geometry(pointcloud)
    so_file = sorted(files)
    for f in so_file:
        if f[-3:]== "pcd":
            pcd = o3d.io.read_point_cloud(filepath + f)  
            pcd = np.asarray(pcd.points).reshape((-1, 3))
            pointcloud.points = o3d.utility.Vector3dVector(pcd)  #
            vis.update_geometry(pointcloud)
            # pred_bboxes = np.load('frame1box.npy')
            # vis.update_geometry(pred_bboxes)
            if to_reset:
                vis.reset_view_point(True)
                to_reset = False
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.2)
def isInRange(ego_car_loc_x,ego_car_loc_y, all_car_loc_x,all_car_loc_y):
    range = 45.2
    minX = ego_car_loc_x - range
    maxX = ego_car_loc_x + range
    minY = ego_car_loc_y - range
    maxY = ego_car_loc_y + range
    if (ego_car_loc_x==all_car_loc_x) and (ego_car_loc_y==all_car_loc_y):
        inRangeFlag = False
        #equal to itself 
        return inRangeFlag
    if all_car_loc_x >= minX and all_car_loc_x<= maxX and all_car_loc_y >= minY and all_car_loc_y <= maxY:
        inRangeFlag = True
    else:
        inRangeFlag = False
    return inRangeFlag
def show_bbox(bboxs, pcds ,map,demo_car_yaw):
    linesets = []
    for bbox in bboxs:
        bbox_points  = _create_bbox_points(extent_x=bbox["extent_x"]/2,extent_y=bbox["extent_y"]/2,extent_z=bbox["extent_z"]/2,
                        location_x=bbox["location_x"],location_y=bbox["location_y"],location_z=bbox["location_z"]+3.5)
        lines = [[0,1], [1,2], [2,3], [3,0], [4,5], [5,6], [6,7], [7,4], [5,1], [0,4], [6,2], [7,3] ]
        colors = [[1, 0, 0] for i in range(len(lines))]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(bbox_points),
            lines=o3d.utility.Vector2iVector(lines),
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)
        angle_yaw = bbox["yaw"]-np.deg2rad(demo_car_yaw)# 90 degrees in radians / metrics must be unified 
        rotation_matrix = np.array([[np.cos(angle_yaw), -np.sin(angle_yaw), 0],
                                    [np.sin(angle_yaw), np.cos(angle_yaw), 0],
                                    [0, 0, 1]])
        line_set.rotate(rotation_matrix)
        linesets.append(line_set)

    o3d.visualization.draw_geometries(pcds  + linesets+[map])

def main():
    # while 1:
    #     point_cloud_consecutive_visualization()
    '''                                                       initialization                                        '''
    config = 'detection3D/configs/pointpillars/hv_pointpillars_secfpn_6x8_160e_kitti-3d-car.py'
  
    checkpoint  = 'detection3D/checkpoints/epoch_80.pth'
  
    device = 'cuda:0'
    model = init_model(config, checkpoint, device=device)
    Carpoints_completion = Adamap()
    file =  open ("carla.pkl","rb")
    dataset = pickle.load(file)
    # car_entire_map = [] ##  to save the entire carla map 

    for frame , frame_data in dataset.items():
        if int(frame) < 1400: continue 
        MSN_completed_car_list = []
        for carID,car_data in frame_data.items():
            demo_car_locx = car_data['location']["x"]
            demo_car_locy = car_data['location']["y"]
            demo_car_locz = car_data['location']["z"]
            demo_car_yaw = car_data['rotation']["yaw"]
            pcd_ply = car_data['pcd_path']
            pcd =o3d.io.read_point_cloud(pcd_ply)
            pcd_transform_matrix=get_matrix(x=car_data['location']["x"], y=car_data['location']["y"], z=car_data['location']["z"],
            pitch= car_data['rotation']["pitch"], yaw=car_data['rotation']["yaw"], roll= car_data['rotation']["roll"])
            pcd_T = pcd.transform(pcd_transform_matrix)
            pcd_bin =  pcd_ply[:-4]+'.bin'

            ply_to_bin(pcd_ply,pcd_bin)
            detected_obj_list,detected_obj_instance,detected_obj_bbox = Detection3d(frame,pcd_bin,model)
            #TODO adjust yaw->world 
            bboxs = []
            for i in range(len(detected_obj_bbox)):
                
                detected_obj_bbox[i][0] = detected_obj_bbox[i][0] + demo_car_locx
                detected_obj_bbox[i][1] = detected_obj_bbox[i][1] + demo_car_locy
                detected_obj_bbox[i][2] = detected_obj_bbox[i][2] + demo_car_locz
                bbox = {"yaw": detected_obj_bbox[i][-1], 
                        "extent_x": detected_obj_bbox[i][3], 
                        "extent_y": detected_obj_bbox[i][4],
                        "extent_z": detected_obj_bbox[i][5],
                        "location_x":detected_obj_bbox[i][0],
                        "location_y":detected_obj_bbox[i][1],
                        "location_z":detected_obj_bbox[i][2]}
                
                    
                bboxs.append(bbox)
            # TODO z 0.32? yaw? 
            for i in range(len(detected_obj_instance)):
                detected_obj_instance[i] = detected_obj_instance[i].translate((demo_car_locx,demo_car_locy,demo_car_locz))
        
            # o3d.visualization.draw_geometries(detected_obj_instance)
            # show_bbox(bboxs, detected_obj_instance, pcd_T)
            ##detected_obj_instance   a list that contains cropped pointclouds format [pointclouds,pointclouds,pointclouds,pointclouds]
            # print(detected_obj_bbox,detected_obj_instance)
            complete_carpoints = Carpoints_completion.val(detected_obj_instance, bboxs,demo_car_yaw)
            show_bbox(bboxs, complete_carpoints, pcd_T,demo_car_yaw)
            print(complete_carpoints)
   ###################################                show bouding boxes to current and world map    ###############################

            
            # # car_entire_map.append(pcd) ## to create the entire map status 
        
   
            # pcd_demo = pcd_T
            # print("  carID , x, y, yaw",carID, demo_car_locx,demo_car_locy,demo_car_yaw)
            # withInRange = isInRange(ego_car_loc_x=demo_car_locx,ego_car_loc_y=demo_car_locy,all_car_loc_x=car_data['location']["x"] ,all_car_loc_y=car_data['location']["y"])
      
            # if  withInRange and car_data['location']["y"]<55 :
            #     for i in range ( len (detected_obj_bbox)):
            #         extent_x = detected_obj_bbox[i][3] /2
            #         extent_y = detected_obj_bbox[i][4] /2
            #         extent_z = detected_obj_bbox[i][5] /2
            #         location_x = detected_obj_bbox[i][0]+2
            #         location_y = detected_obj_bbox[i][1]-2
            #         location_z = detected_obj_bbox[i][2]  
            #         yaw =  detected_obj_bbox[i][6]
            #         bbox_points  = _create_bbox_points_test(extent_x=extent_y,extent_y=extent_x,extent_z=extent_z,
            #         location_x=location_x+demo_car_locx,location_y=location_y+demo_car_locy,location_z=location_z+demo_car_locz)   
            #         print(carID,":  in " , frame,  "yaw is ",yaw,"x,y:",location_x,location_y)

                
            #         lines = [[0,1], [1,2], [2,3], [3,0], [4,5], [5,6], [6,7], [7,4], [5,1], [0,4], [6,2], [7,3] ]
            #         colors = [[0, 0, 1] for i in range(len(lines))]
            #         line_set = o3d.geometry.LineSet(
            #             points=o3d.utility.Vector3dVector(bbox_points),
            #             lines=o3d.utility.Vector2iVector(lines),
            #         )
            #         line_set.colors = o3d.utility.Vector3dVector(colors)
                
            #         # angle_yaw = np.absolute(np.deg2rad(yaw)- (np.pi/2))# 90 degrees in radians / metrics must be unified 
            #         angle_yaw = np.absolute(np.deg2rad(yaw))# 90 degrees in radians / metrics must be unified 
            #         rotation_matrix = np.array([[np.cos(angle_yaw), -np.sin(angle_yaw), 0],
            #                                     [np.sin(angle_yaw), np.cos(angle_yaw), 0],
            #                                     [0, 0, 1]])
            #         line_set.rotate(rotation_matrix)

            #         bbox.append(line_set)


            #         # # tesla = o3d.io.read_point_cloud("02691156_tesla4.pcd") 
               
            #         # tesla = complete_carpoints[i]

            #         # tesla.scale(4.7,center = (0, 0 ,0))
            #         # R = tesla.get_rotation_matrix_from_xyz((-np.pi/2,0,0))
            #         # tesla.rotate(R, (0,0,0))
            #         # tesla = copy.deepcopy(tesla).translate((location_x+demo_car_locx,location_y+demo_car_locy,location_z+demo_car_locz))  # z+2 because it's the sensor height,Need to be reduced.

            #         # angle_yaw_pcd = np.absolute(np.deg2rad(yaw))# 90 degrees in radians / metrics must be unified 
            #         # rotation_matrix_pcd = np.array([[np.cos(angle_yaw_pcd), -np.sin(angle_yaw_pcd), 0],
            #         #                             [np.sin(angle_yaw_pcd), np.cos(angle_yaw_pcd), 0],
            #         #                             [0, 0, 1]])
            #         # tesla.rotate(rotation_matrix_pcd)


            #         # MSN_completed_car_list.append(tesla)

              
              
            #     o3d.visualization.draw_geometries([pcd_demo]+ MSN_completed_car_list + bbox)
               
                # bbox_points  = _create_bbox_points(extent_x=car_data['extext']["y"],extent_y=car_data['extext']["x"],extent_z=car_data['extext']["z"],
                #     location_x=car_data['location']["x"],location_y=car_data['location']["y"],location_z=car_data['location']["z"])   

                # print(carID,":  in " , frame,  "yaw is ",car_data['rotation']["yaw"],"x,y:",car_data['location']["x"],car_data['location']["y"])

              
                # lines = [[0,1], [1,2], [2,3], [3,0], [4,5], [5,6], [6,7], [7,4], [5,1], [0,4], [6,2], [7,3] ]
                # colors = [[0, 0, 1] for i in range(len(lines))]
                # line_set = o3d.geometry.LineSet(
                #     points=o3d.utility.Vector3dVector(bbox_points),
                #     lines=o3d.utility.Vector2iVector(lines),
                # )
                # line_set.colors = o3d.utility.Vector3dVector(colors)
              
                # angle_yaw = np.absolute(np.deg2rad(car_data['rotation']["yaw"])- (np.pi/2))# 90 degrees in radians / metrics must be unified 
                # rotation_matrix = np.array([[np.cos(angle_yaw), -np.sin(angle_yaw), 0],
                #                             [np.sin(angle_yaw), np.cos(angle_yaw), 0],
                #                             [0, 0, 1]])
                # # this is the matrix for z-aixs, there are other two for y and x axises. 

                # # Apply the rotation matrix to the LineSet
                # line_set.rotate(rotation_matrix)
                # bbox.append(line_set)
               
            
                # # tesla = o3d.io.read_point_cloud("02691156_tesla4.pcd") 
               
                # tesla = complete_carpoints[0]

                # tesla.scale(4.7,center = (0, 0 ,0))
                # R = tesla.get_rotation_matrix_from_xyz((-np.pi/2,0,0))
                # tesla.rotate(R, (0,0,0))
                # tesla = copy.deepcopy(tesla).translate((car_data['location']["x"],car_data['location']["y"],car_data['location']["z"]+2))  # z+2 because it's the sensor height,Need to be reduced.

                # angle_yaw_pcd = np.absolute(np.deg2rad(car_data['rotation']["yaw"]))# 90 degrees in radians / metrics must be unified 
                # rotation_matrix_pcd = np.array([[np.cos(angle_yaw_pcd), -np.sin(angle_yaw_pcd), 0],
                #                             [np.sin(angle_yaw_pcd), np.cos(angle_yaw_pcd), 0],
                #                             [0, 0, 1]])
                # tesla.rotate(rotation_matrix_pcd)


                # MSN_completed_car_list.append(tesla)
           
                # o3d.visualization.draw_geometries([pcd_demo]+ MSN_completed_car_list + bbox)
              
            #                           '''             entire map              '''
            # bbox_points  = _create_bbox_points(extent_x=car_data['extext']["y"],extent_y=car_data['extext']["x"],extent_z=car_data['extext']["z"],
            #         location_x=car_data['location']["x"],location_y=car_data['location']["y"],location_z=car_data['location']["z"])
            # lines = [[0,1], [1,2], [2,3], [3,0], [4,5], [5,6], [6,7], [7,4], [5,1], [0,4], [6,2], [7,3] ]
            # colors = [[1, 0, 0] for i in range(len(lines))]
            # line_set = o3d.geometry.LineSet(
            #     points=o3d.utility.Vector3dVector(bbox_points),
            #     lines=o3d.utility.Vector2iVector(lines),
            # )
            # line_set.colors = o3d.utility.Vector3dVector(colors)
            # bbox.append(line_set)
        # o3d.visualization.draw_geometries(car_entire_map+ bbox)
        # o3d.visualization.draw_geometries([pcd] + bbox) ## to create the current map detection status 
        # o3d.visualization.draw_geometries([pcd_demo] + bbox) 
        # o3d.io.write_point_cloud("demo0115/{}frame{}car.ply".format(frame,333),pcd_demo)
        ''' write data of each frame '''
        # for i in range(len(bbox)):
        #     o3d.io.write_line_set("demo0115/{}frame{}car{}lineset.ply".format(frame,333,i),bbox[i])
        # o3d.io.write_triangle_mesh("testa.ply",[pcd_demo] + bbox)
        # o3d.io.write_point_cloud("demo0115/%sframe_all.pcd"%frame, pcd_demo)
        # for i in range(len(bbox)):
        #     o3d.io.write_line_set("demo0115/{}bbox{}.ply".format(frame,i),bßbox[i])
        # o3d.visualization.draw_geometries([pcd_demo]+ MSN_completed_car_list + bbox) ## to create the current map detection status  #TODO0115 completed version

        '''save data (lineset and corresponding maps )'''
        # pdemo = np.asarray(pcd_demo.points)
        # points_list  = []
        # for i in range(len(MSN_completed_car_list)):
        #     downpcd = MSN_completed_car_list[i].voxel_down_sample(voxel_size=0.05)  # no downsample the size is too big 
        #     car_points  = np.asarray(downpcd.points)
        #     points_list.append(car_points)
        # for i in range(len(points_list)):
        #     pdemo = np.concatenate((pdemo,points_list[i]), axis=0)
        # pf = o3d.geometry.PointCloud()
        # pf.points = o3d.utility.Vector3dVector(pdemo)
        # o3d.io.write_point_cloud("demoforada/{}frame{}car.ply".format(frame,333),pf)
        # for i in range(len(bbox)):
        #     o3d.io.write_line_set("demoforada/{}frame{}car{}lineset.ply".format(frame,333,i),bbox[i])

        # print("1")

        ######################################################################################################################################################

def MPC_tracking():
    '''
    car trajectory prediction
    first layer is car ID, then read the location of each frame of every car 
    '''
    file =  open ("./trajectory_prediction/carlatrajectory.pkl","rb")
    dataset = pickle.load(file)
    for carID, each_frame in dataset.items():
        car_traj_x = []
        car_traj_y = []
        for k ,v in each_frame.items():
            if (len(car_traj_x)!= 0 )and (len(car_traj_y)!= 0):
                if v[0]>0:
                    if v[0]== car_traj_x[-1]:
                        car_traj_x.append(float(car_traj_x[-1]+0.00001))
                    else:
                        if v[0]>car_traj_x[-1]:
                            car_traj_x.append(v[0])
                        else:
                            car_traj_x.append(float(car_traj_x[-1]+0.00001))
                if v[0]<0:
                    if v[0]== car_traj_x[-1]:
                        car_traj_x.append(float(car_traj_x[-1]-0.00001))
                    else:
                        if v[0]<car_traj_x[-1]:
                            car_traj_x.append(v[0])
                        else:
                            car_traj_x.append(float(car_traj_x[-1]-0.00001))
                if v[1]>0:
                    if v[1]== car_traj_y[-1]:
                        car_traj_y.append(float(car_traj_y[-1]+0.00001))
                    else:
                        if v[1]>car_traj_y[-1]:
                            car_traj_y.append(v[1]) 
                        else:
                            car_traj_y.append(float(car_traj_y[-1]+0.00001))
                if v[1]<0:
                    if v[1]< car_traj_y[-1]:
                        car_traj_y.append(v[0])
                    else:
                        car_traj_y.append(float(car_traj_y[-1]-0.00001))
            else: 
                car_traj_x.append(v[0])
                car_traj_y.append(v[1]) 
        MPC(car_traj_x,car_traj_y)
   

    
  

if __name__ == '__main__':
    # MPC_tracking()
    main()