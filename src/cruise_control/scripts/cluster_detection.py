import numpy as np
import open3d as o3d
import os

class ClusterDetection:
    def __init__(self):
        cluster_value_file = open(os.path.abspath(os.path.dirname(__file__)) + "/cluster_values.txt")
        self.min_num_points = int(cluster_value_file.readline())
        self.epsilon = float(cluster_value_file.readline())

    def find_clusters(self, lidar_points):
        o3d_point_cloud = o3d.geometry.PointCloud()

        translated_points = []
        for point in lidar_points:
            translated_points.append([point.x, point.y, point.z])

        o3d_point_cloud.points = o3d.utility.Vector3dVector(translated_points)
        labels = np.array(o3d_point_cloud.cluster_dbscan(self.epsilon, self.min_num_points, print_progress=False))

        cluster_set = set()
        for point_index in labels:
            cluster_set.add(point_index)

        return len(cluster_set) - 1
