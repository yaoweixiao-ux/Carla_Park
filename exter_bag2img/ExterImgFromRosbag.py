# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import argparse
import cv2
import os
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def extract_images_from_bag(bag_file, output_dir, image_topic):
    # 打开rosbag文件
    bag = rosbag.Bag(bag_file, 'r')
    bridge = CvBridge()
    count = 0

    # 读取指定话题的消息
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        try:
            # 将ROS消息转换为OpenCV图像
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            print(f"Error converting image: {e}")
            continue

        # 保存为图像文件
        image_filename = os.path.join(output_dir, f"frame_{count:06d}.png")
        cv2.imwrite(image_filename, cv_image)
        count += 1
        print(f"Image {count} saved to {image_filename}")

    # 关闭rosbag文件
    bag.close()
    print(f"Processed {count} images.")

def main():
    # 使用argparse处理命令行参数
    parser = argparse.ArgumentParser(description="Extract images from a rosbag and save them as files.")
    parser.add_argument("bag_file", help="The rosbag file to extract images from")
    parser.add_argument("output_dir", help="Directory to save the extracted images")
    parser.add_argument("image_topic", help="Image topic to subscribe to")

    args = parser.parse_args()

    # 确保输出目录存在
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    # 从rosbag中提取图像
    extract_images_from_bag(args.bag_file, args.output_dir, args.image_topic)

if __name__ == '__main__':
    main()

