import json
import argparse
import os
import numpy as np
import cv2


class Pixel2PointCloud:
    def __init__(self):
        parser = argparse.ArgumentParser(
            description="Convert pixel to point cloud data."
        )
        parser.add_argument(
            "-pcd",
            "--pointcloud",
            type=str,
            default="pcd/",
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
        self.pcd_folderpath = args.pointcloud
        self.json_path = args.json
        self.save_path = args.save
        # 读取intrinsic.txt 和 extrinsic.txt
        self.intrinsic_file = os.path.join(args.parameter, 'intrinsic.txt')
        self.extrinsic_file = os.path.join(args.parameter, 'extrinsic.txt')

        # 检查点云文件夹是否存在
        if not os.path.exists(self.pcd_folderpath):
            print(f"Error: The folder {self.pcd_folderpath} does not exist.")
            return
        if not os.path.exists(self.intrinsic_file):
            print(f"Error: The folder {self.intrinsic_file} does not exist.")
            return
        if not os.path.exists(self.extrinsic_file):
            print(f"Error: The folder {self.extrinsic_file} does not exist.")
            return
        
        # 定义并初始化变量
        self.pixel_data = {}
        self.intrinsic_matrix = []
        self.distortion_coeffs = [0, 0, 0, 0, 0]    # 因为解包时已去畸变
        self.R_mat = []
        self.tvec = []

        self.get_parameters()

        # 获取并排序文件夹中的 PCD 文件
        self.pcd_filelist = self.get_sorted_pcd()
        
        self.pixel_data = self.load_json_data()
        
        for img_tsp, lines in self.pixel_data.items():
            pcd_filepath = self.closest_pcd_tsp_filepath(img_tsp)
            print(f"Image Timestamp:\t{img_tsp}.png")
            print(f"Point cloud Timestamp:\t{os.path.basename(pcd_filepath)}")
            # 打印对应的线段数据
            for i, line in enumerate(lines, 1):
                # for j, pxl in enumerate(line):
                for pxl in line:
                    lidar_xyz = self.pixel_to_3d_point(pxl[0], pxl[1])
                    print(f"  pixel {i}: {pxl}")
                    print(f"  pcd   {i}: {lidar_xyz.T}")
                    # print(f"???Line {i}:{j} {line[j][0]}")
                    # print(f"???Line {i}:{j} {line[j][1]}")
                # print(f"  Line {i}: {line}")


    def load_json_data(self):
        """
        读取JSON文件并返回一个字典，字典包含每个时间戳和对应的线段数据。
        
        参数:
        file_path: JSON文件的路径
        
        返回:
        一个包含时间戳为键，线段列表为值的字典
        """
        # 初始化结果字典
        result = {}

        # 读取 JSON 文件
        with open(self.json_path, 'r') as file:
            data = json.load(file)

        # 将数据组织成以时间戳为键，线段为值的字典
        for entry in data:
            timestamp = entry["timestamp"]
            lines = entry["lines"]
            result[timestamp] = lines
        
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
    
    def closest_pcd_tsp_filepath(self, img_tsp):
        crr_error = 0
        pre_error = None
        for i in range(len(self.pcd_filelist)):
            pcd_tsp = self.get_timestamp(self.pcd_filelist[i])
            crr_error = abs(float(img_tsp) - float(pcd_tsp))
            if pre_error == None or pre_error >= crr_error:
                pre_error = crr_error
                continue
            else:
                trg_tsp = self.pcd_filelist[i-1]
                break
        return trg_tsp
    
    def get_parameters(self):
        # 读取intrinsic矩阵和distortion系数
        def parse_line(line):
            """Helper function to parse a line and convert to floats with 5 decimal places."""
            return [round(float(val.strip().replace(';', '')), 5) for val in line.split()]

        with open(self.intrinsic_file, 'r') as f:
            lines = f.readlines()
            self.intrinsic_matrix = np.array([parse_line(line) for line in lines[1:4]])
            # distortion_coeffs = np.array(parse_line(lines[6]))

        # 读取extrinsic矩阵并提取旋转矩阵和平移矩阵
        with open(self.extrinsic_file, 'r') as f:
            lines = f.readlines()
            extrinsic_matrix = np.array([parse_line(line) for line in lines[1:4]])

        self.R_mat = extrinsic_matrix[:, :3]
        self.tvec = extrinsic_matrix[:, 3]
    
    def pixel_to_3d_point(self, u, v):
        Z = 10  # 深度值?????????????????????????????????????????????????????

        # 将像素坐标转换为齐次坐标
        uv_homogeneous = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)

        # 反投影到相机坐标系
        intrinsic_inv = np.linalg.inv(self.intrinsic_matrix)
        extrinsic_inv = np.linalg.inv(self.R_mat)
        xyz_camera = intrinsic_inv @ uv_homogeneous * Z

        # 转换到世界坐标系
        xyz_lidar_coor = extrinsic_inv @ xyz_camera 
        
        return xyz_lidar_coor


if __name__ == "__main__":
    pixel2pointcloud = Pixel2PointCloud()
