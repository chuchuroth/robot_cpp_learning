#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "utils.hpp"  // QoS

class ImageProcessNode : public rclcpp::Node {
public:
    ImageProcessNode() : Node("image_process") {
        auto img_qos = getQOS(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1, 
                              rclcpp::Duration::from_seconds(0.05));  // 图像QoS
        sub_ = image_transport::create_subscription(this, "/camera/image_raw", img_qos,
            [this](const sensor_msgs::msg::Image::ConstSharedPtr msg){
                cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
                cv::Mat gray, edges;
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                cv::Canny(gray, edges, 50, 150);  // 简单边缘检测作为障碍识别

                // 简单逻辑：边缘密度高 -> 障碍 -> 发布停止/转向命令
                double edge_ratio = cv::countNonZero(edges) / (double)(edges.rows * edges.cols);
                std_msgs::msg::Int32 cmd;
                cmd.data = (edge_ratio > 0.1) ? -50 : 80;  // 障碍时减速/转向
                pub_->publish(cmd);

                RCLCPP_INFO(this->get_logger(), "Obstacle detected: %.2f%% edges", edge_ratio * 100);
            }
        );

        auto cmd_qos = getQOS(RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10, 
                              rclcpp::Duration::from_seconds(0.1));  // 命令QoS：可靠
        pub_ = create_publisher<std_msgs::msg::Int32>("/motor_cmd", cmd_qos);
    }

private:
    image_transport::Subscriber sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessNode>());
    rclcpp::shutdown();
    return 0;
}