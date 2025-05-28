#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

class YoloDetector:
    def __init__(self):
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/carla/ego_vehicle/rgb_omnview/image", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/carla/ego_vehicle/rgb_omnview/camera_info", CameraInfo, self.camera_info_callback)

        self.image_pub = rospy.Publisher("/carla/ego_vehicle/detected_image", Image, queue_size=1)
        self.point_pub = rospy.Publisher("/carla/parking_spot_position", PointStamped, queue_size=1)

        self.camera_matrix = None

        # Load ONNX model
        model_path = "/home/crist/Desktop/Carla_Park/src/yolo_obb_node/yolov8_carla_slot_obb.onnx"
        try:
            self.net = cv2.dnn.readNetFromONNX(model_path)
            rospy.loginfo("YOLOv8 ONNX model loaded.")
        except cv2.error as e:
            rospy.logerr(f"Failed to load ONNX model: {e}")
            exit(1)

        # 优先使用CUDA，如果无CUDA可换成CPU
        try:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            rospy.loginfo("Using CUDA backend for DNN.")
        except:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            rospy.logwarn("CUDA backend not available, switched to CPU.")

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            rospy.logwarn_throttle(5, "Camera info not received yet.")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        blob = cv2.dnn.blobFromImage(cv_image, scalefactor=1/255.0, size=(640, 640), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())

        output = outputs[0]  # (1, 6, 8400)
        output = output[0]   # (6, 8400)

        h, w = cv_image.shape[:2]
        scale_x = w / 640.0
        scale_y = h / 640.0

        for box_idx in range(output.shape[1]):
            cx = output[0, box_idx]
            cy = output[1, box_idx]
            box_w = output[2, box_idx]
            box_h = output[3, box_idx]
            theta = output[4, box_idx]
            conf = output[5, box_idx]

            if conf < 0.5:
                continue

            cx_img = cx * 640 * scale_x
            cy_img = cy * 640 * scale_y
            box_w_img = box_w * 640 * scale_x
            box_h_img = box_h * 640 * scale_y

            rrect = ((cx_img, cy_img), (box_w_img, box_h_img), theta * 180.0 / math.pi)
            box_points = cv2.boxPoints(rrect)
            box_points = np.int0(box_points)

            cv2.polylines(cv_image, [box_points], isClosed=True, color=(0,255,0), thickness=2)

            uv1 = np.array([cx_img, cy_img, 1.0])
            K_inv = np.linalg.inv(self.camera_matrix)
            xyz = K_inv.dot(uv1) * 2.0

            pt = PointStamped()
            pt.header = msg.header
            pt.point.x = float(xyz[0])
            pt.point.y = float(xyz[1])
            pt.point.z = float(xyz[2])
            self.point_pub.publish(pt)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")


if __name__ == "__main__":
    rospy.init_node("carla_slot_detect", anonymous=True)
    detector = YoloDetector()
    rospy.spin()
