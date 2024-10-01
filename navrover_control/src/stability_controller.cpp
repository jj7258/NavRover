#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class StabilityController : public rclcpp::Node
{
public:
    StabilityController()
    : Node("stability_controller")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rocker_arm_position_controller/commands", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&StabilityController::joint_state_callback, this, std::placeholders::_1));
        target_position_ = {0.0, 0.0};  // Target positions for the rocker arms
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Implement your feedback control logic here
        // For example, you can compare the current positions with the target positions and adjust accordingly
        auto current_positions = msg->position;
        auto command = std_msgs::msg::Float64MultiArray();
        command.data = target_position_;  // Adjust this based on your control logic
        publisher_->publish(command);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::vector<double> target_position_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StabilityController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}