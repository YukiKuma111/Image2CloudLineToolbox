import argparse
import cv2
import json
import numpy as np
import open3d as o3d
import os
import rosbag
import tf

from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree

# 开启时保存过程中的点云文件
DEBUG = False


class Pixel2PointCloud:
    def __init__(self):
        parser = argparse.ArgumentParser(
            description="Convert pixel to point cloud data."
        )
        parser.add_argument(
            "-ppcd",
            "--panoramapointcloud",
            type=str,
            default="pcd/0.pcd",
            help="Path to the point cloud file",
        )
        parser.add_argument(
            "-bag",
            "--rosbag",
            type=str,
            default="rosbag/0.bag",
            help="Path to the point cloud file",
        )
        parser.add_argument(
            "-para",
            "--parameter",
            type=str,
            default="parameter/",
            help="Folder path to the parameters (intrinsic.txt and extrinsic.txt)",
        )
        parser.add_argument(
            "-j",
            "--json",
            type=str,
            default="sketched_line_list.json",
            help="Path to the json file for selected pixels",
        )
        parser.add_argument(
            "-s",
            "--save",
            type=str,
            default="add_lines_pcd/",
            help="Saving path to the output point cloud data",
        )

        # 获取用户输入
        args = parser.parse_args()
        self.panorama_pcd = args.panoramapointcloud
        self.bag_file = args.rosbag
        self.json_path = args.json
        self.save_path = args.save

        # 读取intrinsic.txt 和 extrinsic.txt
        self.intrinsic_file = os.path.join(args.parameter, "intrinsic.txt")
        self.extrinsic_file = os.path.join(args.parameter, "extrinsic.txt")

        # 检查文件是否存在
        if not os.path.exists(self.panorama_pcd):
            print(f"Error: The folder {self.panorama_pcd} does not exist.")
            return
        if not os.path.exists(self.intrinsic_file):
            print(f"Error: The folder {self.intrinsic_file} does not exist.")
            return
        if not os.path.exists(self.extrinsic_file):
            print(f"Error: The folder {self.extrinsic_file} does not exist.")
            return

        # 定义并初始化变量
        self.intrinsic_matrix = []
        self.distortion_coeffs = []
        self.R_mat = []
        self.tvec = []
        self.target_frame = "body"
        self.source_frame = "camera_init"

        # 获取数据
        pixel_data = self.load_json_data()
        tf_data = self.parse_tf_from_bag()
        panorama_pcd = o3d.io.read_point_cloud(self.panorama_pcd)
        projection_pcd = o3d.io.read_point_cloud(self.panorama_pcd)
        lines_pcd = o3d.geometry.PointCloud()

        # 获取参数
        self.get_parameters()
        rvec, _ = cv2.Rodrigues(self.R_mat)

        for img_tsp, data in pixel_data.items():
            # 定义并初始化变量
            roi_pcd = o3d.geometry.PointCloud()
            projection_dict = {}  # 3D 投影 2D 的关联字典
            start_end_points_list = []  # 只有和pixel一一对应的起点，中间的点，和终点
            start2end_points_list = []  # 按照一定密度生成的点
            lines_points_in_world = []  # 转置到世界坐标系下的dot lines points
            lines_panorama_pcd = o3d.geometry.PointCloud()

            # 查找与图像时间戳最接近的tf时间戳
            tf_tsp_list = list(tf_data.keys())
            tf_tsp = self.closest_tsp(img_tsp, tf_tsp_list)
            print(f"IMG Timestamp:\t{img_tsp}.png")
            print(f"TF Timestamp:\t{tf_tsp}")

            # 找到对应时间戳的 translation 和 rotation_matrix
            translation = tf_data[tf_tsp]["translation"]
            rotation_matrix = tf_data[tf_tsp]["rotation_matrix"]
            inv_rotation_matrix = np.linalg.inv(rotation_matrix)

            # True：不做坐标系的转换； False：是正常的流程（世界->body->世界）
            flag = False
            if flag:
                translation = [0, 0, 0]
                rotation_matrix = np.eye(3)
                inv_rotation_matrix = np.linalg.inv(rotation_matrix)

            # 裁剪全景点云，减少计算量
            roi_points = self.mm_filter_point_cloud(
                panorama_pcd,
                translation[0] - 10,
                translation[0] + 10,
                translation[1] - 10,
                translation[1] + 10,
                translation[2] - 10,
                translation[2] + 10,
            )

            # 将 roi_points 转换为 open3d.geometry.PointCloud 对象
            roi_pcd.points = o3d.utility.Vector3dVector(np.array(roi_points))
            if DEBUG:
                o3d.io.write_point_cloud(
                    self.save_path + img_tsp + "cliped.pcd", roi_pcd
                )  # for debug

            # 对点云进行旋转和平移操作
            points = np.asarray(roi_pcd.points)
            transformed_points = points - translation  # 平移
            rotated_points = np.dot(transformed_points, inv_rotation_matrix.T)  # 旋转
            projection_pcd.points = o3d.utility.Vector3dVector(rotated_points)
            if DEBUG:
                o3d.io.write_point_cloud(
                    self.save_path + img_tsp + "transformed_point_cloud.pcd",
                    projection_pcd,
                )  # for debug

            # 筛选投影点云（一般来说要配合相机FOV，这里先直接hardcode）
            if not flag:
                filtered_pts = self.mm_filter_point_cloud(
                    projection_pcd, -0.1, 10, -2, 2, -10, 10
                )
            else:
                filtered_pts = self.mm_filter_point_cloud(
                    projection_pcd, -1, 10, -10, 10, -10, 10
                )

            # 投影3D点到2D图像视图
            projected_pts, _ = cv2.projectPoints(
                np.array(filtered_pts),
                rvec,
                self.tvec,
                self.intrinsic_matrix,
                self.distortion_coeffs,
            )

            # 用字典关联 2D 投影点和 3D 点云坐标
            for i, point_3d in enumerate(filtered_pts):
                point_2d = tuple(projected_pts[i][0])  # 投影后的2D坐标
                projection_dict[point_2d] = point_3d

            # 构建 KDTree，方便后续查找
            self.build_kdtree_for_projection(projection_dict)

            # 打印对应的线段数据并保存最近的3D点
            for i, line in enumerate(data["lines"], 1):
                line_3d_points = []
                lines_pts = []
                # 遍历每一个像素点，并通过计算距离其最近的2D投影点来查询到对应的3D点云坐标
                for pxl in line:
                    print(f"  pixel {i}: {pxl}")
                    # 查找最近邻点
                    nearest_3d_point = self.find_nearest_2d_point([pxl[0], pxl[1]])
                    print(
                        f"  Nearest 3D point: [{nearest_3d_point[0]:.2f}, {nearest_3d_point[1]:.2f}, {nearest_3d_point[2]:.2f}]"
                    )

                    line_3d_points.append(nearest_3d_point)
                start_end_points_list.append(line_3d_points)
                # 将每个线段的对应 3D 点保存到 "points" 列表中
                pixel_data[img_tsp]["points"] = start_end_points_list

                # 生成等间隔的三维点
                start2end_points_list = self.generate_dot_lines(line_3d_points)

                # 扁平化 start2end_points_list，将所有 3D 点提取出来
                flat_line_3d_points = [
                    point for line in start2end_points_list for point in line
                ]

                # 将这些新的 3D 点追加到 lines_pts 中，并转换到世界坐标系下
                lines_pts.extend(flat_line_3d_points)
                transformed_lines_pts = self.apply_transformation(
                    lines_pts, rotation_matrix, translation
                )

                # 追加 dot lines， 用于后续保存
                lines_points_in_world.extend(transformed_lines_pts)
            lines_panorama_pcd.points = o3d.utility.Vector3dVector(
                np.array(lines_points_in_world)
            )

            # 将新的点云数据与原点云数据合并
            panorama_pcd.points.extend(lines_panorama_pcd.points)
            lines_pcd.points.extend(lines_panorama_pcd.points)

        # 保存点云结果
        o3d.io.write_point_cloud(
            self.save_path + img_tsp + "panorama_w_lines.pcd", panorama_pcd
        )
        o3d.io.write_point_cloud(
            self.save_path + img_tsp + "panorama_o_lines.pcd", lines_pcd
        )

        # 追加数据到JSON文件（不会覆盖之前的数据）
        with open(self.save_path + "point_pixel_map.json", "w") as json_file:
            json.dump(pixel_data, json_file, indent=4)

    def load_json_data(self):
        """
        读取JSON文件并返回一个字典,字典包含每个时间戳和对应的线段数据。

        参数:
        file_path: JSON文件的路径

        返回:
        一个包含时间戳为键，线段列表为值的字典
        """
        # 初始化结果字典
        result = {}

        # 读取 JSON 文件
        with open(self.json_path, "r") as file:
            data = json.load(file)

        # 将数据组织成以时间戳为键，线段为值的字典
        for entry in data:
            timestamp = entry["timestamp"]
            lines = entry["lines"]
            # 初始化每个时间戳对应的字典，并将lines加入
            result[timestamp] = {
                "lines": lines,
                "points": [],  # 初始化一个空的points列表
            }

        return result

    def closest_tsp(self, target_tsp, tsp_list):
        crr_error = 0
        pre_error = None
        for i in range(len(tsp_list)):
            tsp = tsp_list[i]
            crr_error = abs(float(target_tsp) - float(tsp))
            if pre_error == None or pre_error >= crr_error:
                pre_error = crr_error
                continue
            else:
                closest_tsp = tsp_list[i - 1]
                break
        return closest_tsp

    def get_parameters(self):
        # 读取intrinsic矩阵和distortion系数
        def parse_line(line):
            """Helper function to parse a line and convert to floats with 5 decimal places."""
            return [
                round(float(val.strip().replace(";", "")), 5) for val in line.split()
            ]

        with open(self.intrinsic_file, "r") as f:
            lines = f.readlines()
            self.intrinsic_matrix = np.array([parse_line(line) for line in lines[1:4]])
            self.distortion_coeffs = np.array(parse_line(lines[6]))

        # 读取extrinsic矩阵并提取旋转矩阵和平移矩阵
        with open(self.extrinsic_file, "r") as f:
            lines = f.readlines()
            extrinsic_matrix = np.array([parse_line(line) for line in lines[1:4]])

        self.R_mat = extrinsic_matrix[:, :3]
        self.tvec = extrinsic_matrix[:, 3]

    def parse_tf_from_bag(self):
        # 打开rosbag文件
        bag = rosbag.Bag(self.bag_file)

        listener = tf.TransformerROS()
        tf_dict = {}

        # 遍历bag文件的/tf话题
        for topic, msg, t in bag.read_messages(topics=["/tf"]):
            # msg.transforms 是一个包含多个 tf 变换的列表
            for transform in msg.transforms:
                # 检查是否是我们感兴趣的目标帧和源帧的变换
                if (
                    transform.child_frame_id == self.target_frame
                    and transform.header.frame_id == self.source_frame
                ):
                    # 提取时间戳
                    timestamp = transform.header.stamp.to_sec()

                    # 提取平移和旋转四元数
                    translation = np.array(
                        [
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z,
                        ]
                    )

                    rotation_quaternion = np.array(
                        [
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w,
                        ]
                    )

                    # 使用 tf 将四元数转换为 3x3 旋转矩阵
                    rotation_matrix = tf.transformations.quaternion_matrix(
                        rotation_quaternion
                    )[:3, :3]

                    # 将 translation 和 rotation 存储到字典
                    tf_dict[timestamp] = {
                        "translation": translation,
                        "rotation_matrix": rotation_matrix,
                    }
        bag.close()

        return tf_dict

    # 通过 2D 像素点查找对应的 3D 点云坐标
    def find_nearest_2d_point(self, target_2d):
        # 在 KDTree 中查找最近的 2D 点
        _, idx = self.kd_tree.query(target_2d)
        nearest_2d = list(self.projection_dict.keys())[idx]

        # 返回最近的 2D 点对应的 3D 点云坐标
        return self.projection_dict[nearest_2d]

    # 通过 octree 筛选 ROI 内的点云
    def mm_filter_point_cloud(self, input_pcd, xmin, xmax, ymin, ymax, zmin, zmax):
        # 创建 Axis-Aligned Bounding Box (AABB)
        bounding_box = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(xmin, ymin, zmin), max_bound=(xmax, ymax, zmax)
        )

        # 通过 AABB 在点云上直接过滤出符合条件的点
        filtered_pcd = input_pcd.crop(bounding_box)
        filtered_pts = np.asarray(filtered_pcd.points)

        return filtered_pts.tolist()

    def apply_transformation(self, points, rotation_matrix, translation_vector):
        """
        对一组 3D 点进行旋转和平移变换。

        参数:
        points: 需要变换的 3D 点列表，每个点是一个 (x, y, z) 坐标。
        rotation_matrix: 3x3 旋转矩阵 (NumPy 数组)。
        translation_vector: 3x1 平移向量 (NumPy 数组)。

        返回:
        变换后的 3D 点列表。
        """
        transformed_points = []

        for point in points:
            # 将点转换为 NumPy 数组
            point_array = np.array(point)

            # 进行旋转: R * point
            rotated_point = np.dot(rotation_matrix, point_array)

            # 进行平移: R * point + translation
            transformed_point = rotated_point + translation_vector

            # 将变换后的点添加到列表中
            transformed_points.append(transformed_point.tolist())

        return transformed_points

    def generate_dot_lines(self, start_end_points, interval=0.01):
        """
        在至少两点 start_end_points 之间生成等间隔的三维点。

        参数:
        start_end_points: 两个以上的三维点的坐标 (x, y, z) 的list
        interval: 每个新点的间隔距离，默认为 0.01

        返回:
        包含新生成点的列表
        """
        dot_lines = []

        for i in range(1, len(start_end_points)):
            # print(f"from\t{start_end_points[i-1]}\nto\t{start_end_points[i]}")
            # 将点转换为 numpy 数组
            p1 = np.array(start_end_points[i - 1])
            p2 = np.array(start_end_points[i])

            # 计算两点之间的距离
            distance = np.linalg.norm(p2 - p1)

            # 计算需要生成多少个点
            num_points = int(distance // interval)

            # 计算方向向量
            direction = (p2 - p1) / distance

            # 生成等间隔的新点
            points = [p1 + i * interval * direction for i in range(num_points + 1)]
            dot_lines.append(points)

        return dot_lines

    # 初始化 KDTree
    def build_kdtree_for_projection(self, projection_dict):
        # 将 projection_dict 中的 2D 点作为数组，并构建 KDTree
        self.kd_tree = KDTree(list(projection_dict.keys()))
        self.projection_dict = projection_dict

    # 未被使用的函数 #
    def get_sorted_pcd(self):
        # 获取所有PNG文件
        pcd = [
            os.path.join(self.pcd_folderpath, f)
            for f in os.listdir(self.pcd_folderpath)
            if f.endswith(".pcd")
        ]
        # 按文件名的前缀部分进行排序
        pcd.sort(
            key=lambda f: float(
                os.path.basename(f).split(".")[0]
                + "."
                + os.path.basename(f).split(".")[1]
            )
        )
        return pcd

    def get_timestamp(self, filepath):
        # 获取文件名作为时间戳，去掉文件扩展名
        timestamp = (
            os.path.basename(filepath).split(".")[0]
            + "."
            + os.path.basename(filepath).split(".")[1]
        )
        return timestamp

    # 通过不规则四边形来筛选ROI内的点云
    def sq_filter_point_cloud(self, input_pcd, a, b, c, d):
        filtered_pts = []
        for point in np.asarray(input_pcd.points):
            below_ab = self.point_line_relation(point, a, b)
            left_bc = self.point_line_relation(point, b, c)
            above_cd = self.point_line_relation(point, c, d)
            right_da = self.point_line_relation(point, d, a)
            if (below_ab) and (not left_bc) and (not above_cd) and (right_da):
                filtered_pts.append((point[0], point[1], point[2]))
        return filtered_pts

    def point_line_relation(self, point, lp1, lp2):
        below_right_flag = None
        x1, y1 = lp1[0], lp1[1]
        x2, y2 = lp2[0], lp2[1]
        px, py = point[0], point[1]
        # line: y = kx + b
        if (x2 - x1) != 0:
            k = (y2 - y1) / (x2 - x1)
            b = y1 - x1 * k
            l_py = k * px + b
            # 点在线的下或右，返回True，否则返回False
            if l_py >= py:
                below_right_flag = True
            else:
                below_right_flag = False
        # line 平行于 y 轴
        else:
            if px >= x1:
                below_right_flag = True
            else:
                below_right_flag = False

        return below_right_flag

    # 直接通过比较xyz大小来筛选ROI内的点云
    def old_mm_filter_point_cloud(self, input_pcd, xmin, xmax, ymin, ymax, zmin, zmax):
        filtered_pts = []
        for point in np.asarray(input_pcd.points):
            if (
                xmin < point[0] < xmax
                and ymin < point[1] < ymax
                and zmin < point[2] < zmax
            ):
                filtered_pts.append((point[0], point[1], point[2]))
        return filtered_pts

    # 通过2D像素点查找对应的3D点云坐标
    def old_find_nearest_2d_point(self, target_2d, projection_dict):
        # 计算字典中所有2D点与目标点的距离
        distances = {
            pt_2d: np.linalg.norm(np.array(pt_2d) - np.array(target_2d))
            for pt_2d in projection_dict.keys()
        }

        # 找到最近的2D点
        nearest_2d = min(distances, key=distances.get)

        # 返回最近2D点对应的3D点云坐标
        return projection_dict[nearest_2d]


if __name__ == "__main__":
    pixel2pointcloud = Pixel2PointCloud()
