import copy
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d


def find_cyl(color_img, depth_img, model_path):
    # LOAD MODEL TO LOOK FOR IN POINTCLOUD
    mesh = o3d.io.read_triangle_mesh(model_path) #IN
    pc_model = mesh.sample_points_uniformly(number_of_points=800)
    
    # LOAD COLOR AND DEPTH IMAGE, CREATE RGBD IMAGE
    #color_raw = color_img
    color_raw = o3d.geometry.Image(color_img)
    #color_raw = o3d.io.read_image("data/gen/color_40.png") #IN
    #depth_raw = depth_img
    depth_raw = o3d.geometry.Image(depth_img)
    #depth_raw = o3d.io.read_image("data/gen/depth_40.png") #IN
    rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(color=color_raw, depth=depth_raw, convert_rgb_to_intensity=False)

    # DEFINE CAMERA PARAMETERS, POSSIBLE INPUT FOR FUNCTION
    cam_pam = o3d.camera.PinholeCameraIntrinsic() #IN
    #cam_pam.intrinsic_matrix = [[896.8549194335938, 0.0, 645.7654418945312], [0.0, 897.719970703125, 354.4916076660156], [0.0, 0.0, 1.0]]
    #cam_pam.intrinsic_matrix = [[229.50390625, 0.0, 157.10546875], [0.0, 230.3828125, 124.826171875], [0.0, 0.0, 1.0]]
    cam_pam.intrinsic_matrix = [[597.9033203125, 0.0, 323.8436584472656], [0.0, 598.47998046875, 236.32774353027344], [0.0, 0.0, 1.0]]
    cam_pam.height = 320
    cam_pam.width = 240

    # CREATE POINTCLOUD FROM RGBD AND CAMERA PARAMETERS
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_img, cam_pam)
    #pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    #o3d.visualization.draw_geometries([pcd])

    # POINTCLOUD DOWNSAMPLE, FIRST REMOVAL OF STATISTICAL OUTLIERS TO REDUCE NOISE
    # DOWNSAMPLING IS PRIMARILY TO REDUCE SIZE
    pcd = pcd.voxel_down_sample(voxel_size=0.005)
    #pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=0.1)
    #o3d.visualization.draw_geometries([pcd])

    # PASSTHROUGH FILTER, REMOVE POINTS WHERE z > some_value
    point = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    filter_z = 0.8
    filt = np.where(point[:,2] < filter_z)
    points_filtered = point[filt[0], :]
    colors_filtered = colors[filt[0], :]
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points_filtered)
    pc.colors = o3d.utility.Vector3dVector(colors_filtered)
    #o3d.visualization.draw_geometries([pc])

    # FIND TABLE PLANE ON POINTCLOUD
    plane_model, inliers = pc.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model
    inlier_cloud = pc.select_by_index(inliers)
    #inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pc.select_by_index(inliers, invert=True)
    #o3d.visualization.draw_geometries([inlier_cloud])
    #o3d.visualization.draw_geometries([outlier_cloud])

    # SECOND REMOVAL OF STATISTICAL OUTLIERS, TO REDUCE NOISE AND PREPARE FOR CLUSTERING
    #voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.01)
    pc, ind = outlier_cloud.remove_statistical_outlier(nb_neighbors=10, std_ratio=0.1)
    #o3d.visualization.draw_geometries([p_cl])

    # CLUSTERING 
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pc.cluster_dbscan(eps=0.04, min_points=20, print_progress=True))
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    #colors = plt.get_cmap("tab20")(labels/(max_label if max_label > 0 else 1))
    #colors[labels < 0] = 0
    #p_cl.colors = o3d.utility.Vector3dVector(colors[:, :3])
    #o3d.visualization.draw_geometries([pc])

    # ICP FOR EVERY CLUSTER 
    results = copy.deepcopy(inlier_cloud)
    cyl_transforms = []
    for i in range(max_label+1):
        cluster = np.where(labels == i)[0]
        cluster_pc = pc.select_by_index(cluster, invert=False)
        #o3d.visualization.draw_geometries([cluster_pc])
        #o3d.io.write_point_cloud("../data/pc/cluster{}.pcd".format(i), cluster_pc)
        pc_obj = copy.deepcopy(cluster_pc)
        obj_points = np.asarray(pc_obj.points)
        obj_mean = np.mean(obj_points, axis=0)
        threshold = 0.017 # parameter
        trans_init = np.asarray([[1, 0, 0, obj_mean[0]],
                                [0, 1, 0, obj_mean[1]],
                                [0, 0, 1, obj_mean[2]], [0.0, 0.0, 0.0, 1.0]])

        #evaluation = o3d.pipelines.registration.evaluate_registration(pc_model, pc_obj, threshold, trans_init)
        #print(evaluation)
        #print("point to point ICP")
        reg_p2p = o3d.pipelines.registration.registration_icp(
            pc_model, pc_obj, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000))
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
        if chamfer_dist < 0.7:
            cyl_transforms.append(reg_p2p.transformation)
            #pc_dom_plane = o3d.io.read_point_cloud("../data/pc/dom_plane.pcd")
            #o3d.visualization.draw_geometries([results, pc_model_transformed])
            results = results + pc_model_transformed

    print("end")
    return cyl_transforms
    
