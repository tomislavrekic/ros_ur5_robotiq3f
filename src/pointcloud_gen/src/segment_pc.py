import pcl
import pcl.pcl_visualization as vis

p = pcl.load_XYZRGB("../data/pc/test.pcd")

seg = p.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_distance_threshold(0.02)
seg.set_optimize_coefficients(True)
indices, model = seg.segment()

print(model)

cloud_plane = p.extract(indices, negative=True)

dist_filter = cloud_plane.make_passthrough_filter()
dist_filter.set_filter_field_name("z")
dist_filter.set_filter_limits(-1.2, 0.0)
cloud_filtered = dist_filter.filter()

print(cloud_filtered.size)

seg = cloud_filtered.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients(False)
seg.set_model_type(pcl.SACMODEL_CYLINDER)
seg.set_normal_distance_weight(0.01)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_max_iterations(10000)
seg.set_distance_threshold(0.01)
seg.set_radius_limits(0, 0.05)
indices, model = seg.segment()

cyls_cloud = cloud_filtered.extract(indices, negative=True)

print(cyls_cloud.size)

euclid = pcl.EuclideanClusterExtraction()
p.make
#kdtr = pcl.KdTreeFLANN_PointXYZRGB(cyls_cloud)
kdtree = pcl.KdTree(cloud_filtered)

euclid.set_SearchMethod(kdtree)
euclid.set_MinClusterSize(20)
euclid.set_MaxClusterSize(500)
euclid.set_ClusterTolerance(0.02)
ind = euclid.Extract()

visual = vis.CloudViewing()
visual.ShowColorCloud(cyls_cloud, b'cloud')

v = True
while v:
    v = not(visual.WasStopped())

