import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import pyransac3d as p3d

# Read .ply file
input_file = "odm_mesh.ply"
#input_file = "point_cloud.ply"
#input_file = "odm_25dmesh.ply"
# forest = PyntCloud.from_file("odm_25dmesh.ply")
# #forest.plot()
# convex_hull_id = forest.add_structure("convex_hull")
# convex_hull = forest.structures[convex_hull_id]
# forest.mesh = convex_hull.get_mesh()
# forest.to_file("forest_hull.ply", also_save=["mesh"])
pcd = o3d.io.read_point_cloud(input_file) # Read the point cloud
xyz = np.asarray(pcd.points)
#o3d.visualization.draw_geometries([pcd])
plane = p3d.Plane()
equation1, inliers1 = plane.fit(xyz)
print(equation1, inliers1)

pl1 = pcd.select_by_index(inliers1).paint_uniform_color([1, 0, 0])
obb = pl1.get_oriented_bounding_box()
obb2 = pl1.get_axis_aligned_bounding_box()
obb.color = [0, 0, 1]
obb2.color = [0, 1, 0]
not_plane1 = pcd.select_by_index(inliers1, invert=True)

o3d.visualization.draw_geometries([not_plane1, pl1, obb, obb2])

cuboid = p3d.Cuboid()
eq2, inl2 = cuboid.fit(xyz)
print(eq2, inl2)

cub1 = pcd.select_by_index(inl2).paint_uniform_color([1, 0, 0])
obb = cub1.get_oriented_bounding_box()
obb2 = cub1.get_axis_aligned_bounding_box()
obb.color = [0, 0, 1]
obb2.color = [0, 1, 0]
not_cub1 = pcd.select_by_index(inl2, invert=True)

o3d.visualization.draw_geometries([not_cub1, cub1, obb, obb2])


# labels = np.array(pcd.cluster_dbscan(eps=0.75, min_points=4))
# max_label = labels.max()
# colors = plt.get_cmap("tab20")(labels / (max_label
#                                         if max_label > 0 else 1))
# colors[labels < 0] = 0
# pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
# o3d.visualization.draw_geometries([pcd])
#
# segment_models={}
# segments={}
# max_plane_idx=20
# rest=pcd
# for i in range(max_plane_idx):
#     colors = plt.get_cmap("tab20")(i)
#     segment_models[i], inliers = rest.segment_plane(
#     distance_threshold=1,ransac_n=3,num_iterations=5000)
#     segments[i]=rest.select_by_index(inliers)
#     segments[i].paint_uniform_color(list(colors[:3]))
#     rest = rest.select_by_index(inliers, invert=True)
#     print("pass",i,"/",max_plane_idx,"done.")
# o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest])


