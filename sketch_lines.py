import argparse
import cv2
import json
import os


class SketchLines:
    def __init__(self):
        parser = argparse.ArgumentParser(description="Sketch lines in images.")
        parser.add_argument(
            "-img", "--image", type=str, default="img/", help="Path to the image file"
        )
        parser.add_argument(
            "-s",
            "--save",
            type=str,
            default="sketched_line_list.json",
            help="Path to the json file for selected pixels",
        )

        args = parser.parse_args()
        self.image_folderpath = args.image
        self.json_savepath = args.save

        # 定义并初始化变量
        self.data = []
        self.image_origin = None
        self.window_name = ""

        # 检查图像文件夹是否存在
        if not os.path.exists(self.image_folderpath):
            print(f"Error: The folder {self.image_folderpath} does not exist.")
            return

        # 获取并排序文件夹中的 PNG 文件
        self.image_filelist = self.get_sorted_images()

        # 初始化空数据列表并清空 JSON 文件（即覆盖原有内容）
        with open(self.json_savepath, "w") as json_file:
            json.dump(self.data, json_file)

        # 遍历每张图像
        for image in self.image_filelist:
            self.lines = []  # 重置线段列表
            self.current_line = []  # 重置当前线段
            # 读取图像
            self.image_origin = cv2.imread(image)
            if self.image_origin is None:
                print(f"Error: Failed to load image {self.image_origin}. Skipping.")
                continue

            # 动态设置窗口名称为当前图像文件名
            self.window_name = os.path.basename(image)

            # 在图像窗口中设置鼠标回调
            cv2.imshow(self.window_name, self.image_origin)
            cv2.setMouseCallback(self.window_name, self.click_event)

            # 等待用户输入，检查是否按下 "Q" 以退出
            while True:
                key = cv2.waitKey(0) & 0xFF
                # 如果按下的是 "Q"，则退出程序
                if key == ord("q"):
                    print("Q pressed. Saving data and exiting program.")
                    self.save_data(image)  # 保存数据
                    print("Sketch", len(self.lines), "lines in", self.window_name)
                    cv2.destroyAllWindows()
                    return  # 退出整个程序
                # 其他按键则跳到下一张图片
                else:
                    break

            cv2.destroyAllWindows()

            # 保存当前图像的选点数据
            self.save_data(image)
            print("Sketch", len(self.lines), "lines in", self.window_name)

    def save_data(self, image):
        # 获取文件名作为时间戳，去掉文件扩展名
        timestamp = (
            os.path.basename(image).split(".")[0]
            + "."
            + os.path.basename(image).split(".")[1]
        )

        # 将线段数据存入字典并追加到列表中
        if self.lines:
            # 检查当前线段，如果存在，则将它也保存为一个线段(防止没有按下鼠标滚轮键就按下其他键)
            if self.current_line and len(self.current_line) > 1:
                self.lines.append(self.current_line)  # 将当前线段保存
                self.data.append({"timestamp": timestamp, "lines": self.lines})
            else:
                self.data.append({"timestamp": timestamp, "lines": self.lines})

        # 追加数据到JSON文件（不会覆盖之前的数据）
        with open(self.json_savepath, "w") as json_file:
            json.dump(self.data, json_file, indent=4)

        # 清空current_line以准备下一张图像
        self.current_line = []

    # 鼠标回调函数，用于捕获点击事件并记录点的坐标
    def click_event(self, event, x, y, flags, params):
        # 当鼠标左键按下时记录点
        if event == cv2.EVENT_LBUTTONDOWN:
            # 添加点到当前线段
            self.current_line.append([x, y])
            # 在图像上绘制当前线段的点
            if len(self.current_line) > 1:
                cv2.line(
                    self.image_origin,
                    tuple(self.current_line[-2]),
                    tuple(self.current_line[-1]),
                    (0, 255, 0),
                    2,
                )
            cv2.circle(self.image_origin, (x, y), 3, (0, 0, 255), -1)
            # 显示包含点击点的图像
            cv2.imshow(self.window_name, self.image_origin)

        # 鼠标滚轮键完成当前线段
        elif event == cv2.EVENT_MBUTTONDOWN:
            if len(self.current_line) > 1:
                self.lines.append(self.current_line)  # 保存线段

            self.current_line = []  # 重置当前线段

    def get_sorted_images(self):
        # 获取所有PNG文件
        images = [
            os.path.join(self.image_folderpath, f)
            for f in os.listdir(self.image_folderpath)
            if f.endswith(".png")
        ]
        # 按文件名的前缀部分进行排序
        images.sort(
            key=lambda f: float(
                os.path.basename(f).split(".")[0]
                + "."
                + os.path.basename(f).split(".")[1]
            )
        )
        return images


if __name__ == "__main__":
    sketch_lines = SketchLines()
