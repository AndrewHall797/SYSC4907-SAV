import airsim
import numpy as np
import open3d as o3d
import os

# Detects clusters in a point cloud and finds bounding boxes of detected clusters
import rospy


class ClusterDetection:
    def __init__(self):
        param_file = open(os.path.abspath(os.path.dirname(__file__)) + "/cluster_values.txt")
        self.min_num_points = int(param_file.readline())
        self.epsilon = float(param_file.readline())
        self.box_output_file = open(os.path.abspath(os.path.dirname(__file__)) + "/detected_boxes.txt", "w")
        self.client = airsim.CarClient()
        self.client.confirmConnection()

    def find_clusters(self, lidar_points):
        o3d_point_cloud = o3d.geometry.PointCloud()

        # Convert points with format used by point cloud to the format used by Open3D
        translated_points = []
        for point in lidar_points:
            translated_points.append([point.x, point.y, point.z])

        o3d_point_cloud.points = o3d.utility.Vector3dVector(translated_points)
        labels = np.array(o3d_point_cloud.cluster_dbscan(self.epsilon, self.min_num_points, print_progress=False))

        cluster_dic = dict()
        point_index = 0

        # Find what points belong to what cluster. Each index of labels corresponds to the point in the point cloud.
        # For example, label[0] refers to the cluster that the first point in the point cloud is a part of.
        for index in labels:

            # "-1" refers to not being a part of any cluster
            if index == -1:
                continue

            if index not in cluster_dic:
                cluster_dic[index] = []

            cluster_dic[index].append(translated_points[point_index])
            point_index += 1

        # Bounding boxes are stored as a flat array [minX, maxX, minY, maxY, minZ, maxZ).
        # Calculate the bounding boxes of the clusters
        bounding_boxes = []

        for cluster in cluster_dic.values():
            self.find_aabb(cluster, bounding_boxes)

        self.write_array_to_file(bounding_boxes)

        return bounding_boxes

    def find_aabb(self, points, result):
        min_x = 100000
        max_x = -100000

        min_y = 100000
        max_y = -100000

        min_z = 100000
        max_z = -100000

        for p in points:
            min_x = min(p[0], min_x)
            max_x = max(p[0], max_x)

            min_y = min(p[1], min_y)
            max_y = max(p[1], max_y)

            min_z = min(p[2], min_z)
            max_z = max(p[2], max_z)

        result.append(min_x)
        result.append(min_y)
        result.append(min_z)

        result.append(max_x)
        result.append(max_y)
        result.append(max_z)

    def write_array_to_file(self, boxes):

        car_pos = self.client.simGetGroundTruthKinematics("").position

        self.box_output_file.write("{},{},{},".format(car_pos.x_val.real, car_pos.y_val.real, car_pos.z_val.real))

        component_index = 0

        for box in boxes:
            if component_index == 0:
                box += car_pos.x_val.real
            elif component_index == 1:
                box += car_pos.y_val.real
            else:
                box += car_pos.z_val.real

            component_index += 1
            if component_index % 3 == 0:
                component_index = 0

            self.box_output_file.write("{},".format(box))

        self.box_output_file.write("\n")
        self.box_output_file.flush()
