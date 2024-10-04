#!/bin/sh

bags=("$1")
topic=("$2")

cd ../rosbag/
# 检查 .bag 文件是否存在
if [[ -f "$1" ]]; then
  echo "Unpackage ROSBAG: $1"
  rosrun pcl_ros bag_to_pcd $1 $2 ../pcd
else
  echo "Error: $1 does not exist. Skipping..."
fi
