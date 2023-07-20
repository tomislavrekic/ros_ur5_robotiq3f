import open3d as o3d
import copy
import numpy as np
mesh = o3d.io.read_triangle_mesh("../data/modeli/cyl_grah.ply")
print(mesh)
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh])
pc_model = mesh.sample_points_uniformly(number_of_points=800)
o3d.visualization.draw_geometries([pc_model])

pc_obj = o3d.io.read_point_cloud("../data/pc/cluster0.pcd")

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

obj_points = np.asarray(pc_obj.points)
model_points = np.asarray(pc_model.points)
obj_mean = np.mean(obj_points, axis=0)
threshold = 0.017
trans_init = np.asarray([[1, 0, 0, obj_mean[0]],
                         [0, 1, 0, obj_mean[1]],
                         [0, 0, 1, obj_mean[2]], [0.0, 0.0, 0.0, 1.0]])
draw_registration_result(pc_model, pc_obj, trans_init)

print("initial allignment")

evaluation = o3d.pipelines.registration.evaluate_registration(pc_model, pc_obj, threshold, trans_init)
print(evaluation)
print("point to point ICP")
reg_p2p = o3d.pipelines.registration.registration_icp(
    pc_model, pc_obj, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())
print(reg_p2p)
print("Transformation is:")
print(reg_p2p.transformation)
draw_registration_result(pc_model, pc_obj, reg_p2p.transformation)

pc_model_transformed = copy.deepcopy(pc_model)
pc_model_transformed.transform(reg_p2p.transformation)
pc_dom_plane = o3d.io.read_point_cloud("../data/pc/dom_plane.pcd")
o3d.visualization.draw_geometries([pc_dom_plane, pc_model_transformed])
