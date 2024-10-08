import argparse
import cv2
import json
import numpy as np
import open3d as o3d
import os
import rosbag
import tf
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped


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

        args = parser.parse_args()
        self.panorama_pcd = args.panoramapointcloud
        self.bag_file = args.rosbag
        self.json_path = args.json
        self.save_path = args.save
        # 读取intrinsic.txt 和 extrinsic.txt
        self.intrinsic_file = os.path.join(args.parameter, "intrinsic.txt")
        self.extrinsic_file = os.path.join(args.parameter, "extrinsic.txt")

        # 检查点云文件夹是否存在
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

        self.get_parameters()
        rvec, _ = cv2.Rodrigues(self.R_mat)

        # 获取并排序文件夹中的 PCD 文件
        # self.pcd_filelist = self.get_sorted_pcd()

        pixel_data = self.load_json_data()
        tf_data = self.parse_tf_from_bag()
        panorama_pcd = o3d.io.read_point_cloud(self.panorama_pcd)
        projection_pcd = o3d.io.read_point_cloud(self.panorama_pcd)
        lines_pcd = o3d.geometry.PointCloud()

        # for img_tsp, img_data in pixel_data.items():
        for img_tsp, data in pixel_data.items():
            tf_tsp_list = list(tf_data.keys())
            tf_tsp = self.closest_tsp(img_tsp, tf_tsp_list)
            print(f"IMG Timestamp:\t{img_tsp}.png")
            print(f"TF Timestamp:\t{tf_tsp}")

            # 找到对应时间戳的 translation 和 rotation_matrix
            translation = tf_data[tf_tsp]["translation"]
            rotation_matrix = np.linalg.inv(tf_data[tf_tsp]["rotation_matrix"])

            # 计算欧拉角（需要yaw对后续找到的地面点做筛查？）
            # r = R.from_matrix(rotation_matrix)
            # euler_angles = r.as_euler("xyz", degrees=True)
            # print("euler_angles: ", euler_angles)

            # 裁剪全景点云，减少计算量
            roi_points = self.filter_point_cloud(
                panorama_pcd,
                translation[0] - 10,
                translation[0] + 10,
                translation[1] - 10,
                translation[1] + 10,
                translation[2] - 10,
                translation[2] + 10,
            )

            # 将 points 转换为 open3d.geometry.PointCloud 对象
            roi_pcd = o3d.geometry.PointCloud()
            roi_pcd.points = o3d.utility.Vector3dVector(np.array(roi_points))
            # o3d.io.write_point_cloud(img_tsp + "cliped.pcd", roi_pcd)  # for debug

            # 对点云进行旋转和平移操作
            points = np.asarray(roi_pcd.points)
            transformed_points = points - translation  # 平移
            rotated_points = np.dot(transformed_points, rotation_matrix.T)  # 旋转
            projection_pcd.points = o3d.utility.Vector3dVector(rotated_points)
            # o3d.io.write_point_cloud(
            #     img_tsp + "transformed_point_cloud.pcd", projection_pcd
            # )  # for debug

            # 筛选投影点云（一般来说要配合相机FOV，这里先直接hardcode）
            filtered_pts = self.filter_point_cloud(
                projection_pcd, -1, 10, -2, 2, -10, -0.1
            )

            # 投影3D点到2D图像视图
            projected_pts, _ = cv2.projectPoints(
                np.array(filtered_pts),
                rvec,
                self.tvec,
                self.intrinsic_matrix,
                self.distortion_coeffs,
            )

            # 构建字典，关联 2D 投影点和 3D 点云坐标
            projection_dict = {}
            for i, point_3d in enumerate(filtered_pts):
                point_2d = tuple(projected_pts[i][0])  # 投影后的2D坐标
                projection_dict[point_2d] = point_3d

            start_end_points_list = []  # 只有和pixel一一对应的起点，中间的点，和终点
            start2end_points_list = []  # 按照一定密度生成的点
            # 打印对应的线段数据并保存最近的3D点
            for i, line in enumerate(data["lines"], 1):
                line_3d_points = []
                for pxl in line:
                    print(f"  pixel {i}: {pxl}")
                    nearest_3d_point = self.find_nearest_2d_point(
                        [pxl[0], pxl[1]], projection_dict
                    )
                    print(
                        f"  Nearest 3D point: [{nearest_3d_point[0]:.2f}, {nearest_3d_point[1]:.2f}, {nearest_3d_point[2]:.2f}]"
                    )

                    # 将 nearest_3d_point 加入 pixel_data
                    line_3d_points.append(nearest_3d_point)

                start_end_points_list.append(line_3d_points)
                # 将每个线段的对应 3D 点保存到 "points" 列表中
                pixel_data[img_tsp]["points"] = start_end_points_list

                # 生成等间隔的三维点
                start2end_points_list = self.generate_dot_lines(line_3d_points)

                # 扁平化 start_end_points_list，将所有 3D 点提取出来
                flat_line_3d_points = [
                    point for line in start2end_points_list for point in line
                ]

                # 将这些新的 3D 点追加到 filtered_pts 中
                filtered_pts.extend(flat_line_3d_points)

            filtered_pcd = o3d.geometry.PointCloud()
            filtered_pcd.points = o3d.utility.Vector3dVector(np.array(filtered_pts))
            o3d.io.write_point_cloud(
                img_tsp + "filtered_pcdwpoints.pcd", filtered_pcd
            )  # for debug

        # 追加数据到JSON文件（不会覆盖之前的数据）
        with open("new.json", "w") as json_file:
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

    # 通过2D像素点查找对应的3D点云坐标
    def find_nearest_2d_point(self, target_2d, projection_dict):
        # 计算字典中所有2D点与目标点的距离
        distances = {
            pt_2d: np.linalg.norm(np.array(pt_2d) - np.array(target_2d))
            for pt_2d in projection_dict.keys()
        }

        # 找到最近的2D点
        nearest_2d = min(distances, key=distances.get)

        # 返回最近2D点对应的3D点云坐标
        return projection_dict[nearest_2d]

    def filter_point_cloud(self, input_pcd, xmin, xmax, ymin, ymax, zmin, zmax):
        filtered_pts = []
        for point in np.asarray(input_pcd.points):
            if (
                xmin < point[0] < xmax
                and ymin < point[1] < ymax
                and zmin < point[2] < zmax
            ):
                filtered_pts.append((point[0], point[1], point[2]))
        return filtered_pts

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
            print(f"from\t{start_end_points[i-1]}\nto\t{start_end_points[i]}")
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


if __name__ == "__main__":
    pixel2pointcloud = Pixel2PointCloud()
