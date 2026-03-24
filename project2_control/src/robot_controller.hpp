#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

class RobotController : public rclcpp::Node {
   public:
    RobotController();

   private:
    // Subscribers - for odometry, laser scan, and keyboard input
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr key_sub_;

    // Publisher - for cmd_vel
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

    // Timer - for control loop
    rclcpp::TimerBase::SharedPtr timer_;

    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void keyboard_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

    geometry_msgs::msg::TwistStamped last_key_cmd_;
    rclcpp::Time last_key_time_;
    double key_timeout_sec_;

    // Pose tracking
    double current_yaw_;
    double current_x_position_;
    double current_y_position_;
    double last_x_position_;
    double last_y_position_;

    // Robot behavior state
    double dist_traveled_turn_;
    geometry_msgs::msg::TwistStamped last_published_cmd_;

    // Constants
    double SAFETY_DISTANCE_;
    double OBSTACLE_DISTANCE_;
    double FORWARD_SPEED_;

    // Turn speeds
    double AVOID_TURN_SPEED_;
    double ESCAPE_TURN_SPEED_;
    double RANDOM_TURN_SPEED_;

    // Escape
    bool escape_active_;
    double escape_target_yaw_;

    // Random turn state
    bool random_turn_active_;
    double random_turn_target_yaw_;

    // Random number generation
    std::random_device rd_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> random_small_turn_rad_;  // ±15 deg
    std::uniform_real_distribution<double> escape_extra_turn_rad_;  // ±30 deg

    // Helper functions
    bool teleop_active() const;
    static double normalize_angle(double a);
    static double angle_diff(double target, double current);
    bool collision_found() const;
    void update_distance_traveled(const geometry_msgs::msg::Twist& cmd);
    bool scan_ready() const;
    double min_range_in_angle_window(double angle_lo, double angle_hi) const;
    void front_left_right_mins(double& min_left, double& min_right) const;
    double front_min_range() const;
    double global_min_range() const;

    // Behavior functions (priority 1–6)
    geometry_msgs::msg::TwistStamped halt_command();
    geometry_msgs::msg::TwistStamped keyboard_command() const;
    geometry_msgs::msg::TwistStamped escape_command();
    geometry_msgs::msg::TwistStamped avoid_command();
    geometry_msgs::msg::TwistStamped random_turn_command();
    geometry_msgs::msg::TwistStamped forward_command() const;

    void control_loop();
};

#endif  // ROBOT_CONTROLLER_HPP