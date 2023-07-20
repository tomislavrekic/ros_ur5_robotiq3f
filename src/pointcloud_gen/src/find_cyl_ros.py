import copy
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os

def img_callback(data):
    global color
    color = data
    pass

def depth_callback(data):
    global depth
    depth = data
    pass

color = Image()
depth = Image()

rospy.init_node('generate_pointcloud')
rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback)
rospy.Subscriber('/camera/color/image_raw', Image, img_callback)
rospy.sleep(1)
i = 0

save_path = "src/pointcloud_gen/data/gen2"

mesh = o3d.io.read_triangle_mesh("data/modeli/cyl/cyl_grah.ply")
pc_model = mesh.sample_points_uniformly(number_of_points=800)


while not rospy.is_shutdown():
    cv_color = np.frombuffer(color.data, dtype=np.uint8).reshape(color.height, color.width, -1)
    cv_depth = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width, 1)
    
    cv_depth = np.where(np.isfinite(cv_depth), cv_depth, 0.0)
    #cv_depth = cv2.normalize(cv_depth, 0, 1000.0, norm_type=cv2.NORM_MINMAX)
    #cv_depth = cv_depth.astype(np.uint8)

    
    #depth_prev = cv2.cvtColor(depth_prev, cv2.COLOR_RGB2GRAY)
    #cv2.imwrite(os.path.join(save_path, "depth_preview_{}.png".format(i)), depth_prev.astype(np.uint8))
    #cv2.imshow("prev", cv_color)
    #cv2.imshow("prevd", cv_depth)
    #cv2.imshow("prevdd", depth_prev)
    #cv2.waitKey(0)
    #i = i + 1
    rospy.sleep(0.1)

    color_raw = o3d.geometry.Image(cv_color)
    depth_raw = o3d.geometry.Image(cv_depth)
    what_is = np.asarray(depth_raw)
    #o3d.visualization.draw_geometries([color_raw])
    #o3d.visualization.draw_geometries([depth_raw])


    rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(color=color_raw, depth=depth_raw, convert_rgb_to_intensity=False)
    #what_is = np.asarray(rgbd_img.Image)
    cam_pam = o3d.camera.PinholeCameraIntrinsic()
    #cam_pam.intrinsic_matrix = [[896.8549194335938, 0.0, 645.7654418945312], [0.0, 897.719970703125, 354.4916076660156], [0.0, 0.0, 1.0]]
    #cam_pam.height = 320
    #cam_pam.width = 240

    cam_pam.intrinsic_matrix = [[597.9033203125, 0.0, 323.8436584472656], [0.0, 598.47998046875, 236.32774353027344], [0.0, 0.0, 1.0]]
    cam_pam.height = 640
    cam_pam.width = 480

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_img, cam_pam)
    #pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd])
    point = np.asarray(pcd.points)
    point = point * 1000
    pcd.points = o3d.utility.Vector3dVector(point)

    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.01)
    o3d.visualization.draw_geometries([voxel_down_pcd])
    #cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.1)
    #o3d.visualization.draw_geometries([cl])

    point = np.asarray(voxel_down_pcd.points)
    colors = np.asarray(voxel_down_pcd.colors)
    filter_z = 1.5
    filt = np.where(point[:,2] < filter_z)
    points_filtered = point[filt[0], :]
    colors_filtered = colors[filt[0], :]

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points_filtered)
    pc.colors = o3d.utility.Vector3dVector(colors_filtered)

    o3d.visualization.draw_geometries([pc])

    plane_model, inliers = pc.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model

    inlier_cloud = pc.select_by_index(inliers)
    #inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pc.select_by_index(inliers, invert=True)
    o3d.visualization.draw_geometries([inlier_cloud])
    o3d.visualization.draw_geometries([outlier_cloud])

    p_cl = outlier_cloud

    #voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.01)
    #p_cl, ind = pc.remove_statistical_outlier(nb_neighbors=30, std_ratio=0.1)

    o3d.visualization.draw_geometries([p_cl])

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(p_cl.cluster_dbscan(eps=0.04, min_points=20, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels/(max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    p_cl.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([p_cl])

    results = copy.deepcopy(inlier_cloud)
    for i in range(max_label+1):
        cluster = np.where(labels == i)[0]
        cluster_pc = p_cl.select_by_index(cluster, invert=False)
        #o3d.visualization.draw_geometries([cluster_pc])
        #o3d.io.write_point_cloud("../data/pc/cluster{}.pcd".format(i), cluster_pc)

        pc_obj = copy.deepcopy(cluster_pc)
        obj_points = np.asarray(pc_obj.points)
        obj_mean = np.mean(obj_points, axis=0)
        threshold = 0.017
        trans_init = np.asarray([[1, 0, 0, obj_mean[0]],
                                [0, 1, 0, obj_mean[1]],
                                [0, 0, 1, obj_mean[2]], [0.0, 0.0, 0.0, 1.0]])

        evaluation = o3d.pipelines.registration.evaluate_registration(pc_model, pc_obj, threshold, trans_init)
        print(evaluation)
        #print("point to point ICP")
        reg_p2p = o3d.pipelines.registration.registration_icp(
            pc_model, pc_obj, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
        #print(reg_p2p)
        #print("Transformation is:")
        print(reg_p2p.transformation)
        evaluation = o3d.pipelines.registration.evaluate_registration(pc_model, pc_obj, threshold, reg_p2p.transformation)
        print(evaluation)
        pc_model_transformed = copy.deepcopy(pc_model)
        pc_model_transformed.transform(reg_p2p.transformation)

        # CHAMFER DISTANCE
        dist_a = pc_obj.compute_point_cloud_distance(pc_model_transformed)
        dist_a = np.asarray(dist_a)
        dist_a = np.square(dist_a)
        dist_a = np.sum(dist_a)
        dist_b = pc_model_transformed.compute_point_cloud_distance(pc_obj)
        dist_b = np.asarray(dist_b)
        dist_b = np.square(dist_b)
        dist_b = np.sum(dist_b)
        chamfer_dist = dist_a + dist_b
        print(chamfer_dist)
        if chamfer_dist < 0.6:
            #pc_dom_plane = o3d.io.read_point_cloud("../data/pc/dom_plane.pcd")
            o3d.visualization.draw_geometries([results, pc_model_transformed])
            results = results + pc_model_transformed

        rospy.sleep(2.0)

print("end")
