#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

class YoloDetector
{
public:
    YoloDetector(ros::NodeHandle& nh)
    {
        image_sub_ = nh.subscribe("/carla/ego_vehicle/rgb_omnview/image", 1, &YoloDetector::imageCallback, this);
        camera_info_sub_ = nh.subscribe("/carla/ego_vehicle/rgb_omnview/camera_info", 1, &YoloDetector::cameraInfoCallback, this);

        image_pub_ = nh.advertise<sensor_msgs::Image>("/carla/ego_vehicle/detected_image", 1);
        point_pub_ = nh.advertise<geometry_msgs::PointStamped>("/carla/parking_spot_position", 1);

        // Load ONNX model
        try{
            cv::dnn::Net net = cv::dnn::readNetFromONNX("/home/crist/Desktop/Carla_Park/Perception/yolov8_carla_slot_obb.onnx");
            std::cout << "Model loaded successfully!" << std::endl;
        }catch (const cv::Exception& e) {
            std::cerr << "Error loading model: " << e.what() << std::endl;
        }
        // net_ = cv::dnn::readNetFromONNX("/home/crist/Desktop/Carla_Park/Perception/yolov8_carla_slot_obb.onnx");
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA); // or DNN_TARGET_CPU

        ROS_INFO("YOLOv8 ONNX model loaded.");
    }

private:
    ros::Subscriber image_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher image_pub_;
    ros::Publisher point_pub_;

    cv::Mat camera_matrix_;
    cv::dnn::Net net_;

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->K.data()).clone();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        if (camera_matrix_.empty()) {
            ROS_WARN_THROTTLE(5.0, "Camera info not received yet.");
            return;
        }

        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat input_img = cv_ptr->image;

        // Preprocess
        cv::Mat blob = cv::dnn::blobFromImage(input_img, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);
        net_.setInput(blob);

        // Inference
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());

        // Parse output (YOLOv8: [batch, N, 6] = x, y, w, h, conf, class)
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;

        for (int i = 0; i < outputs[0].rows; ++i) {
            float conf = outputs[0].at<float>(i, 5);
            if (conf > 0.5) {
                float* data = outputs[0].ptr<float>(i);
                float cx = data[0] * input_img.cols;
                float cy = data[1] * input_img.rows;
                float w  = data[2] * input_img.cols;
                float h  = data[3] * input_img.rows;
                float theta = data[4];  // 弧度制

                // 旋转框
                cv::RotatedRect rbox(cv::Point2f(cx, cy), cv::Size2f(w, h), theta * 180.0 / CV_PI); // OpenCV使用角度
                cv::Point2f vertices[4];
                rbox.points(vertices);
                for (int j = 0; j < 4; ++j)
                    cv::line(input_img, vertices[j], vertices[(j+1)%4], cv::Scalar(0, 255, 0), 2);

                // 发布位置（基于旋转框中心）
                double z = 2.0;
                cv::Mat uv1 = (cv::Mat_<double>(3,1) << cx, cy, 1.0);
                cv::Mat K_inv = camera_matrix_.inv();
                cv::Mat xyz = K_inv * uv1 * z;

                geometry_msgs::PointStamped pt;
                pt.header = msg->header;
                pt.point.x = xyz.at<double>(0);
                pt.point.y = xyz.at<double>(1);
                pt.point.z = xyz.at<double>(2);
                point_pub_.publish(pt);
            }
        }


        // Publish processed image
        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", input_img).toImageMsg();
        image_pub_.publish(out_msg);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Carla_Slot_Detect");
    ros::NodeHandle nh;

    YoloDetector detector(nh);

    ros::spin();
    return 0;
}
