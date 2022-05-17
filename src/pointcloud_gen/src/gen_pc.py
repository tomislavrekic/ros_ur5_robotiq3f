import cv2
import numpy
import numpy as np

import open3d as o3d


color_raw = o3d.io.read_image("../data/gen/color_40.png")
depth_raw = o3d.io.read_image("../data/gen/depth_40.png")

rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(color=color_raw, depth=depth_raw, convert_rgb_to_intensity=False)
cam_pam = o3d.camera.PinholeCameraIntrinsic()
#cam_pam.intrinsic_matrix = [[3131.58, 0.00, 1505.62] , [0.00, 3131.58, 2004.13], [0.00, 0.00, 1.00]]
#cam_pam.intrinsic_matrix = [[231.107421875, 0.0, 150.533203125], [0.0, 231.388671875, 124.0595703125], [0.0, 0.0, 1.0]]
#cam_pam.intrinsic_matrix = [[952.828, 0., 646.699 ],[0., 952.828, 342.637 ],[0., 0., 1.]]
#cam_pam.intrinsic_matrix = [[597.2974853515625, 0.0, 334.2024230957031], [0.0, 597.3826293945312, 241.11997985839844], [0.0, 0.0, 1.0]]
cam_pam.intrinsic_matrix = [[896.8549194335938, 0.0, 645.7654418945312], [0.0, 897.719970703125, 354.4916076660156], [0.0, 0.0, 1.0]]
cam_pam.height = 320
cam_pam.width = 240
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_img, cam_pam)
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
#o3d.visualization.draw_geometries([pcd])
voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.01)
cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.1)
#o3d.visualization.draw_geometries([cl])

plane_model, inliers = cl.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
[a, b, c, d] = plane_model

inlier_cloud = cl.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = cl.select_by_index(inliers, invert=True)
#o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

#o3d.visualization.draw_geometries([outlier_cloud])

#o3d.io.write_point_cloud("../data/pc/test.pcd", cl)

point = np.asarray(outlier_cloud.points)
colors = np.asarray(outlier_cloud.colors)
filt = np.where(point[:,2] > -1.2)
points_filtered = point[filt[0], :]
colors_filtered = colors[filt[0], :]

pc = o3d.geometry.PointCloud()
pc.points = o3d.utility.Vector3dVector(points_filtered)
pc.colors = o3d.utility.Vector3dVector(colors_filtered)

o3d.visualization.draw_geometries([pc])
print("end")
