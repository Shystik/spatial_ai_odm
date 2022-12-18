import numpy as np
import open3d as o3d
import json
import matplotlib.pyplot as plt
import pyransac3d as p3d
import math
import itertools
import scipy
from functools import reduce
from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN
import random

# Read .ply file
INPUT_FILE = "odm_mesh.ply"
MAX_RADIUS = 150.0


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



def is_in_box(center, extent, point):
      if point[0] > (center[0] + extent[0]) or point[0] < (center[0] - extent[0]):
          return False
      if point[1] > (center[1] + extent[1]) or point[1] < (center[1] - extent[1]):
          return False
      if point[2] > (center[2] + extent[2]) or point[2] < (center[2] - extent[2]):
          return False
      return True


def get_volume(points_to_process: np.array, new_points: np.array):
    if len(points_to_process) <= 0:
        return 0
    xy_catalog = []
    for index in points_to_process:
        xy_catalog.append([new_points[index][0], new_points[index][1]])
    if len(xy_catalog) <= 0:
        return 0
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

    return volume

def build_drone_path(outlier_cloud):
    outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])

    colorOur = np.array(outlier_cloud.colors)
    new_points = np.asarray(outlier_cloud.points)

    bbbox = outlier_cloud.get_oriented_bounding_box()

    center = bbbox.center.copy()
    extent = bbbox.extent.copy()

    dicrection = [1, -1]
    x_direct = random.choice(dicrection)
    y_direct = random.choice(dicrection)
    currentPoint = [center[0] + (extent[0] * x_direct), center[1] + (extent[1] * y_direct), center[2] + (extent[2])]

    y_direct = 1 if y_direct == -1 else -1
    x_direct = 1 if x_direct == -1 else -1

    pointColor = [0.2, 0.5, 0.75]

    pointsWays = [currentPoint]
    colorsWays = [pointColor]

    # Изначальная модель
    o3d.visualization.draw_geometries([outlier_cloud])

    while new_points.size > 0:
        if not is_in_box(center, extent, currentPoint):
            # print('test is_in_box BREAK', x_direct, y_direct, currentPoint)
            break

        points_to_process = get_points_in_radius(currentPoint, new_points, MAX_RADIUS)
        volume = get_volume(points_to_process, new_points)

        colorOur[points_to_process] = [1, 0, 0.75]
        outlier_cloud.colors = o3d.utility.Vector3dVector(colorOur)
        if volume > 100:
            print('Set point view: ', currentPoint)
            pointsWays.append(currentPoint.copy())
            colorsWays.append(pointColor)
            # Просмотр текущего состояния (Можно включить)
            #o3d.visualization.draw_geometries([outlier_cloud])

        colors = np.array(outlier_cloud.colors)
        test_colors = colors[colors == [0.6, 0.6, 0.6]]

        if test_colors.size == 0:
            print('End build way!')
            break

        if not is_in_box(center, extent, [currentPoint[0], currentPoint[1] + MAX_RADIUS * y_direct, currentPoint[2]]):
            currentPoint[0] += (MAX_RADIUS) * x_direct
            y_direct = 1 if y_direct == -1 else -1
        else:
            currentPoint[1] += MAX_RADIUS * y_direct
            # print('test is_in_box in end check false', currentPoint)

    # Добавление точке просмотра дрона
    pointsOur = np.asarray(outlier_cloud.points)
    colorsMain = np.asarray(outlier_cloud.colors)

    pointsOur = np.concatenate((pointsOur, pointsWays), axis=0)
    colorsMain = np.concatenate((colorsMain, colorsWays), axis=0)

    outlier_cloud.points = o3d.utility.Vector3dVector(pointsOur)
    outlier_cloud.colors = o3d.utility.Vector3dVector(colorsMain)

    # Фиальная визуализация маршрута
    o3d.visualization.draw_geometries([outlier_cloud])

    # Сохранение маршрута в файл
    o3d.io.write_point_cloud("/datasets/images/way_fly.ply", outlier_cloud)

if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud(INPUT_FILE)
    old_points = np.asarray(pcd.points)
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=2.5, ransac_n=3, num_iterations=5000
    )
    out_liners = get_point_cloud_above_plane(old_points, plane_model)
    outlier_cloud = pcd.select_by_index(out_liners)
    labels = np.array(
        outlier_cloud.cluster_dbscan(eps=6.5, min_points=100, print_progress=False)
    )

    max_label = max(labels)
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    color_set = set()
    for color in outlier_cloud.colors:
        color_set.add(tuple(color))
    res_volume = 0
    for color_index, point_cloud_color in enumerate(color_set):
        local_point_index = []
        for index, color in enumerate(np.array(outlier_cloud.colors)):
            if tuple(color) == point_cloud_color and tuple(color) != (0, 0, 0):
                local_point_index.append(index)
        if local_point_index:
            local_point_cloud = outlier_cloud.select_by_index(local_point_index)
            if local_point_cloud:
                res_volume += local_point_cloud.get_oriented_bounding_box().volume()
            o3d.visualization.draw_geometries([local_point_cloud])
    print(res_volume)

    # Построение пути
    build_drone_path(outlier_cloud)
