#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include "utils.hpp"  // QoS辅助
#include "pid_controller.hpp"  // PID

class MotorNode : public rclcpp::Node {
public:
    MotorNode() : Node("motor_node") {
        auto qos = getQOS(RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10, 
                          rclcpp::Duration::from_seconds(0.1));  // RT调优：可靠、保持最后10、deadline 100ms
        sub_ = create_subscription<std_msgs::msg::Int32>(
            "/motor_cmd", qos,
            [this](std_msgs::msg::Int32::SharedPtr msg){
                int target = msg->data;
                motor_.setSpeed(target);  // 调用Motor类（含PID）
                RCLCPP_INFO(this->get_logger(), "Motor target speed: %d (PID adjusted)", target);
            }
        );
    }

private:
    Motor motor_;  // 实车Motor实例
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}