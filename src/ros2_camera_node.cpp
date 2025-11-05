#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "utils.hpp"  // QoS

class RealCamera : public rclcpp::Node {
public:
    RealCamera() : Node("real_camera") {
        auto qos = getQOS(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1, 
                          rclcpp::Duration::from_seconds(0.05));  // RT调优：尽力、保持最后1（图像大）、deadline 50ms
        pub_ = image_transport::create_publisher(this, "/camera/image_raw", qos);
        cap_ = cv::VideoCapture(0);  // OpenCV捕获USB相机（0为默认）
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Camera open failed!");
            return;
        }
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),  // 10FPS
            [this](){
                cv::Mat frame;
                cap_ >> frame;
                if (!frame.empty()) {
                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                    pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "Publishing real frame (%dx%d)", frame.cols, frame.rows);
                }
            }
        );
    }
    ~RealCamera() { cap_.release(); }

private:
    cv::VideoCapture cap_;
    image_transport::Publisher pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealCamera>());
    rclcpp::shutdown();
    return 0;
}