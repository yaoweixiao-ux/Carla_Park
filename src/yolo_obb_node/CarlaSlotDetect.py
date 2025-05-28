import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
from ultralytics import YOLO

class YoloOBBDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/ego_vehicle/rgb_omnview/image", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/carla/ego_vehicle/rgb_omnview/camera_info", CameraInfo, self.camera_info_callback)
        self.image_pub = rospy.Publisher("/carla/ego_vehicle/detected_image", Image, queue_size=1)
        self.point_pub = rospy.Publisher("/carla/parking_spot_position", PointStamped, queue_size=1)

        self.camera_matrix = None

        model_path = "/home/crist/Desktop/Carla_Park/src/yolo_obb_node/best_0510_obb.pt"
        rospy.loginfo(f"Loading model {model_path}")
        self.model = YOLO(model_path)  # 用ultralytics的YOLO加载

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

        orig_h, orig_w = cv_image.shape[:2]

        # 使用ultralytics YOLO推理
        results = self.model(cv_image)

        anootated_img = results[0].plot()

        # print(f"[INFO] YOLO raw result: {results}")
        # result = results[0]
        # print(f"[INFO] YOLO result.boxes: {result.obb}")

        # if result.obb is not None and result.obb.xyxy is not None:
        #     for i in range(len(result.obb.xyxy)):
        #         xyxy = result.obb.xyxy[i].cpu().numpy().astype(int)
        #         angle = result.obb.xywhr[i, 4].item()
        #         conf = result.obb.conf[i].item()
        #         cls_id = int(result.obb.cls[i].item())
        #         label = self.model.names[cls_id]

        #         if conf < 0.1:
        #             continue

        #         x1, y1, x2, y2 = xyxy
        #         corners = np.array([
        #             [x1, y1],
        #             [x2, y1],
        #             [x2, y2],
        #             [x1, y2]
        #         ])

        #         # 文本拼接
        #         corners_text = ' '.join([f"{x} {y}" for x, y in corners])

        #         # 绘图
        #         cv_corners = corners.reshape((-1, 1, 2))
        #         cv2.polylines(cv_image, [cv_corners], isClosed=True, color=(255, 0, 0), thickness=2)

        #         text = f"{cls_id} {corners_text} {conf:.2f}"
        #         cv2.putText(cv_image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)



        try:
            out_msg = self.bridge.cv2_to_imgmsg(anootated_img, "bgr8")
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")

if __name__ == "__main__":
    rospy.init_node("carla_slot_obb_detector_ultralytics", anonymous=True)
    detector = YoloOBBDetector()
    rospy.spin()
