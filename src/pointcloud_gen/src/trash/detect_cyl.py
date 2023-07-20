import pcl
import pcl.pcl_visualization as vis

pc = pcl.load_XYZRGB("../data/pc/cluster2.pcd")
print(pc.size)
seg = pc.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients(True)
seg.set_model_type(pcl.SACMODEL_CYLINDER)
seg.set_normal_distance_weight(0.1)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_max_iterations(10000)
seg.set_distance_threshold(0.05)
seg.set_radius_limits(0, 0.1)
indices, model = seg.segment()

cyls_cloud = pc.extract(indices, negative=False)

print(cyls_cloud.size)

visual = vis.CloudViewing()
visual.ShowColorCloud(cyls_cloud, b'cloud')

v = True
while v:
    v = not(visual.WasStopped())

