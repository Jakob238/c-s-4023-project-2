#include "robot_controller.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); // Initialize ROS 2
    rclcpp::spin(std::make_shared<RobotController>()); // Create and run the RobotController node
    rclcpp::shutdown(); // Shutdown ROS 2
    return 0;
}