# Image2CloudLineToolbox

## 功能概述

### [sketch_lines.py](./sketch_lines.py)

该Python脚本用于在图像上绘制线条，并将选定的像素点数据保存到JSON文件中。用户可以通过鼠标点击在图像上选择点，形成线段，并且可以通过按键控制图像的浏览和保存操作。具体功能包括：

- __图像加载与展示__：从指定文件夹中加载PNG格式的图像，逐一展示给用户。
- __图像导航__：用户可以通过按键进行图像浏览。
- __线条绘制__：用户可以通过鼠标点击在图像上选择点，这些点将形成线段。用户也可以通过鼠标滚轮键结束当前线段的选择。
- __数据保存__：每张图像上绘制的线段数据会以时间戳的形式保存在一个指定的JSON文件中，每个时间戳对应一张图像的线段信息。方便后续分析和使用。

### [pixel2pointcloud.py](./pixel2pointcloud.py)

该Python脚本用于将图像中的像素点转换为三维点云数据。它通过读取点云文件、Rosbag文件及相机参数（内外参），并结合用户在图像上选择的像素，来生成与这些像素相对应的三维坐标。具体功能包括：

- __读取输入文件__：支持输入点云文件、Rosbag文件、相机内外参数文件及用户选择的像素数据（JSON格式）。
- __坐标转换__：根据Rosbag文件中的坐标变换数据，将选择的像素转换为三维点云坐标。
- __区域过滤__：根据用户设定的区域过滤条件，从全景点云中筛选出相关的点云数据。
- __点云生成__：根据像素数据生成对应的三维点，并将结果保存到新的点云文件和JSON文件中。

## 使用介绍

### 安装依赖：

确保已安装open3d、numpy、opencv-python和rosbag库，可以通过以下指令进行安装。

    ```
    pip install open3d numpy opencv-python rosbag
    ```

### 运行程序：

1. 使用命令行运行第一个脚本。可以指定图像文件夹和JSON保存路径，例如：

    ```
    python sketch_lines.py -img /path/to/image/folder -s /path/to/save/sketched_line_list.json
    ```
    
    - -img 参数用于指定存放图像的文件夹路径，默认为`img/`。
    - -s 参数用于指定保存线条数据的JSON文件路径，默认为`sketched_line_list.json`。

2. 使用命令行运行第二个脚本，可以指定各个输入文件和输出路径，例如：

    ```
    python pixel_to_pointcloud.py -ppcd /path/to/pcd/0.pcd -bag /path/to/rosbag/0.bag -para /path/to/parameter/ -j /path/to/sketched_line_list.json -s /path/to/save/
    ```

    - -ppcd：指定输入的点云文件路径，默认为`pcd/0.pcd`。
    - -bag：指定输入的ROS bag文件路径，默认为`rosbag/0.bag`。
    - -para：指定存放相机参数（intrinsic.txt和extrinsic.txt）的文件夹路径，默认为`parameter/`。
    - -j：指定存放用户选择像素的JSON文件路径，默认为`sketched_line_list.json`。
    - -s：指定保存输出点云数据的路径，默认为`add_lines_pcd/`。


### 操作说明 & 调试信息：

- sketch_lines.py
    - 运行后，程序会展示图像，用户可以用鼠标左键点击选定点。
    - 按下鼠标滚轮键以结束当前线段的绘制。
    - 按下A返回上一张图像，按下D查看下一张图像，按下Q保存数据并退出程序，其他任意键查看10张后的图像（为了加速完成全图选点任务）。

- pixel2pointcloud.py
    - 在调试模式下（DEBUG = True），程序会保存每一步处理中的点云数据，方便用户查看和分析。
    - 参数flag仅用于测试处理单帧点云的效果，默认为False，不建议开启。
