# coding:utf-8
#!/usr/bin/python
     
# Extract images from a bag file and undistort them based on intrinsic parameters.
     
import rosbag
import rospy
import cv2
import argparse
import numpy as np
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class ImageCreator():
    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.description='please enter rosbag path, image save path, and intrinsic parameter file.'
        parser.add_argument("-b", "--rosbag", help="this is rosbag path", dest="bag_path", type=str, default="../rosbag/0.bag")
        parser.add_argument("-s", "--save", help="this is image save folder path",  dest="save_path", type=str, default="../img/")
        parser.add_argument("-t", "--topic", help="this is image topic",  dest="topic", type=str, default="/usb_cam/image_raw")
        parser.add_argument("-r", "--rate", help="this is output image rate",  dest="rate", type=str, default="1")
        parser.add_argument("-p", "--parameter", help="this is intrinsic parameter filepath",  dest="para", type=str, default="../parameter/intrinsic.txt")
        args = parser.parse_args()

        # Read intrinsic parameters from the file
        self.camera_matrix, self.dist_coeffs = self.read_intrinsic_parameters(args.para)
        self.bridge = CvBridge()

        with rosbag.Bag(args.bag_path, 'r') as bag:  #要读取的bag文件；
            count = 1
            for topic, msg, t in bag.read_messages():
                if topic == args.topic: #图像的topic；
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    except CvBridgeError as e:
                        print(e)
                    
                    # Undistort the image
                    undistorted_image = self.undistort_image(cv_image)

                    timestr = "%.9f" % msg.header.stamp.to_sec()   #%.9f表示小数点后带有9位，可根据精确度需要修改；
                    image_name = timestr + ".png"    #图像命名：时间戳.png

                    if count % int(args.rate) == 0:
                        # cv2.imwrite(args.save_path + image_name, undistorted_image)  #保存；
                        cv2.imwrite(args.save_path + image_name, cv_image)  #保存；
                        count = 1
                        print("Save ", image_name)
                    else:
                        count += 1

    # 读取intrinsic矩阵和distortion系数
    def parse_line(self, line):
        """Helper function to parse a line and convert to floats with 5 decimal places."""
        return [round(float(val.strip().replace(';', '')), 5) for val in line.split()]

    def read_intrinsic_parameters(self, filepath):
        """
        Read intrinsic parameters and distortion coefficients from a file.
        """
        with open(filepath, 'r') as f:
            lines = f.readlines()
            intrinsic_matrix = np.array([self.parse_line(line) for line in lines[1:4]])
            distortion_coeffs = np.array(self.parse_line(lines[6]))

        return intrinsic_matrix, distortion_coeffs

    def undistort_image(self, image):
        """
        Undistort the image using the camera matrix and distortion coefficients.
        """
        h, w = image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        map1, map2 = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, None, new_camera_matrix, (w, h), 5)
        undistorted_image = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)

        # Optionally crop the image (roi[0:2] = top-left corner, roi[2:4] = width and height)
        # x, y, w, h = roi
        # undistorted_image = undistorted_image[y:y + h, x:x + w]

        return undistorted_image

if __name__ == '__main__':
    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
