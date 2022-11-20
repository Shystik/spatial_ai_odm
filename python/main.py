import numpy as np
import open3d as o3d
import json
import matplotlib.pyplot as plt
import pyransac3d as p3d
import math
import itertools
import scipy
from functools import reduce

# Read .ply file
INPUT_FILE = "odm_mesh.ply"
MAX_RADIUS = 30.0


def get_triangles_vertices(input_triangles, vertices):
    triangles_vertices = []
    for triangle in input_triangles:
        new_triangles_vertices = [
            vertices[triangle[0]],
            vertices[triangle[1]],
            vertices[triangle[2]],
        ]
        triangles_vertices.append(new_triangles_vertices)
    return np.array(triangles_vertices)


def get_point_cloud_above_plane(points: np.array, plane_coefficients: np.array):
    indexes = []
    a, b, c, d = plane_coefficients
    for enum_index, point in enumerate(points):
        eq = -((a * point[0] + b * point[1] + d) / c)
        if point[2] > eq + 2:
            indexes.append(enum_index)
    return indexes


def volume_under_triangle(triangle):
    p1, p2, p3 = triangle
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x3, y3, z3 = p3
    return abs(
        (z1 + z2 + z3) * (x1 * y2 - x2 * y1 + x2 * y3 - x3 * y2 + x3 * y1 - x1 * y3) / 6
    )


def transfer_points_to_z0(points, z):
    for point in points:
        point[2] -= z
    return points


def get_distance_between_points(point1: np.array, point2: np.array):
    return np.sqrt(
        (point1[0] - point2[0]) ** 2
        + (point1[1] - point2[1]) ** 2
        + (point1[2] - point2[2]) ** 2
    )


def get_points_in_radius(input_point: np.array, points: np.array, radius: float):
    res_indexes = []
    for index_to_process, point_to_process in enumerate(points):
        if get_distance_between_points(input_point, point_to_process) <= radius:
            res_indexes.append(index_to_process)
    return res_indexes


if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud(INPUT_FILE)
    old_points = np.asarray(pcd.points)
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=2.5, ransac_n=3, num_iterations=5000
    )
    out_liners = get_point_cloud_above_plane(old_points, plane_model)
    outlier_cloud = pcd.select_by_index(out_liners)
    outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])
    # new_points = transfer_points_to_z0(np.asarray(outlier_cloud.points), plane_model[2])
    new_points = np.asarray(outlier_cloud.points)
    res_volume = 0
    while new_points.size > 0:
        points_to_process = get_points_in_radius(new_points[0], new_points, MAX_RADIUS)
        xy_catalog = []
        for index in points_to_process:
            xy_catalog.append([new_points[index][0], new_points[index][1]])

        triangles = scipy.spatial.Delaunay(np.array(xy_catalog))
        surface = o3d.geometry.TriangleMesh()
        surface.vertices = o3d.utility.Vector3dVector(
            [new_points[index] for index in points_to_process]
        )
        surface.triangles = o3d.utility.Vector3iVector(triangles.simplices)
        volume = reduce(
            lambda a, b: a + volume_under_triangle(b),
            get_triangles_vertices(surface.triangles, surface.vertices),
            0,
        )
        res_volume += volume
        new_points = np.delete(new_points, points_to_process, 0)
    # o3d.visualization.draw_geometries([outlier_cloud.select_by_index(points_to_process)])
    print(res_volume)
