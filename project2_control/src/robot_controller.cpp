#include "robot_controller.hpp"

RobotController::RobotController()
    : Node("robot_controller"),
      // State variable initilization
      current_yaw_(0.0),
      current_x_position_(0.0),
      current_y_position_(0.0),
      last_x_position_(0.0),
      last_y_position_(0.0),
      dist_traveled_turn_(0.0),
      escape_active_(false),
      escape_target_yaw_(0.0),
      random_turn_active_(false),
      random_turn_target_yaw_(0.0),
      bumper_hit_(false),
      rng_(rd_()),
      random_small_turn_rad_(-M_PI / 12.0, M_PI / 12.0),  // ±15 deg per spec
      escape_extra_turn_rad_(-M_PI / 6.0, M_PI / 6.0) {   // ±30 deg per spec

    // Publisher for velocity commands
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    // Subscribers for laser scan, odometry, and keyboard input
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&RobotController::scan_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&RobotController::odom_callback, this, std::placeholders::_1));
    key_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_key", 10,
        std::bind(&RobotController::keyboard_callback, this, std::placeholders::_1));

    // Timing for teleop
    key_timeout_sec_ = 0.25;
    last_key_time_ = this->now();
    last_key_cmd_ = geometry_msgs::msg::TwistStamped();

    // Constants
    // Safety threshold 0.22–0.30m (slightly larger than robot radius)
    SAFETY_DISTANCE_ = 0.27;
    // Avoid/escape trigger within 1 foot
    OBSTACLE_DISTANCE_ = 0.3048;

    FORWARD_SPEED_ = 0.10;
    AVOID_TURN_SPEED_ = 1.2;
    ESCAPE_TURN_SPEED_ = 1.2;
    RANDOM_TURN_SPEED_ = 0.8;

    latest_scan_ = nullptr;
    latest_odom_ = nullptr;
    last_published_cmd_ = geometry_msgs::msg::TwistStamped();

    // Timer for control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&RobotController::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "RobotController started.");
}

// Callback Functions

void RobotController::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
}

// Store the latest odometry data for use in the control loop
void RobotController::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = msg;

    // Store previous position for distance tracking
    last_x_position_ = current_x_position_;
    last_y_position_ = current_y_position_;

    // Takes the current position from odometry
    // and changes it to the robot's current x and y position and yaw angle
    current_x_position_ = msg->pose.pose.position.x;
    current_y_position_ = msg->pose.pose.position.y;

    // Convert the quaternion orientation to a yaw angle in radians
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);

    // Find the roll, pitch, and yaw values from the rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
}

// Store the latest keyboard input for use in the control loop
void RobotController::keyboard_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    last_key_cmd_ = *msg;
    last_key_time_ = this->now();
}

// Utility Functions

// Detects if teleop is currently active based on time since last key input
bool RobotController::teleop_active() const {
    const double dt = (this->now() - last_key_time_).seconds();
    return dt >= 0.0 && dt <= key_timeout_sec_;
}

// Normalizes the input angle to the range (-pi, pi]
// by wrapping it around the unit circle.
double RobotController::normalize_angle(double a) {
    while(a > M_PI) a -= 2.0 * M_PI;
    while(a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// Determines the difference between the current and target angle
double RobotController::angle_diff(double target, double current) {
    return normalize_angle(target - current);
}

// Verifies that a valid, non-empty laser scan exists with a positive,
// finite resolution.
bool RobotController::scan_ready() const {
    return latest_scan_ &&
           !latest_scan_->ranges.empty() &&
           std::isfinite(latest_scan_->angle_increment) &&
           latest_scan_->angle_increment > 0.0;
}

// Returns the minimum finite range reading within the angular window [angle_lo, angle_hi]
double RobotController::min_range_in_angle_window(double angle_lo, double angle_hi) const {
    if(!scan_ready()) return std::numeric_limits<double>::infinity();

    const double a_min = latest_scan_->angle_min;
    const double inc = latest_scan_->angle_increment;
    const int n = static_cast<int>(latest_scan_->ranges.size());

    // Normalise both angles into [a_min, a_min + 2pi)
    auto wrap = [&](double a) {
        while(a < a_min) a += 2.0 * M_PI;
        while(a >= a_min + 2.0 * M_PI) a -= 2.0 * M_PI;
        return a;
    };

    double lo = wrap(angle_lo);
    double hi = wrap(angle_hi);

    // Convert normalised angles to array indices, clamped to valid bounds.
    int i_lo = static_cast<int>(std::round((lo - a_min) / inc));
    int i_hi = static_cast<int>(std::round((hi - a_min) / inc));
    i_lo = std::clamp(i_lo, 0, n - 1);
    i_hi = std::clamp(i_hi, 0, n - 1);

    double min_r = std::numeric_limits<double>::infinity();

    auto check_idx = [&](int i) {
        float r = latest_scan_->ranges[static_cast<size_t>(i)];
        if(std::isfinite(r) && r > latest_scan_->range_min)
            min_r = std::min(min_r, static_cast<double>(r));
    };

    // Iterate the two sub-ranges rather than a single contiguous block.
    if(i_lo <= i_hi) {
        for(int i = i_lo; i <= i_hi; ++i) check_idx(i);
    } else {
        for(int i = i_lo; i < n; ++i) check_idx(i);
        for(int i = 0; i <= i_hi; ++i) check_idx(i);
    }
    return min_r;
}

// Attempted fix for backwards laser readings
const double FORWARD_ANGLE_OFFSET = -1 * M_PI / 2;  // 180 deg

// Split the front +-30 degrees cone into left and right halves,
// using the corrected forward direction.
void RobotController::front_left_right_mins(double& min_left, double& min_right) const {
    const double half_cone = 30.0 * M_PI / 180.0;  // 30 degrees each side
    // Left side of the robot corresponds to angles [forward, forward + half_cone]
    // Right side corresponds to [forward - half_cone, forward]
    min_left = min_range_in_angle_window(FORWARD_ANGLE_OFFSET, FORWARD_ANGLE_OFFSET + half_cone);
    min_right = min_range_in_angle_window(FORWARD_ANGLE_OFFSET - half_cone, FORWARD_ANGLE_OFFSET);

    RCLCPP_INFO(this->get_logger(),
                "[FRONT LEFT RIGHT MINS] L=%.3f R=%.3f", min_left, min_right);  // <-- FIXED
}

// Narrow dead-ahead window: +=10 degrees around forward direction.
double RobotController::front_min_range() const {
    const double half = 10.0 * M_PI / 180.0;
    return min_range_in_angle_window(FORWARD_ANGLE_OFFSET - half, FORWARD_ANGLE_OFFSET + half);
}

// Absolute minimum across all beams for halt detection
double RobotController::global_min_range() const {
    if(!scan_ready()) return std::numeric_limits<double>::infinity();
    double m = std::numeric_limits<double>::infinity();
    for(float r : latest_scan_->ranges)
        if(std::isfinite(r) && r > latest_scan_->range_min)
            m = std::min(m, static_cast<double>(r));
    return m;
}

// Detects if the robot will experience a collision
bool RobotController::collision_found() const {
    // if(bumper_hit_) return true;

    if(scan_ready() && front_min_range() < SAFETY_DISTANCE_) return true;

    return false;
}

// Stops all of the robot velocity aspects
geometry_msgs::msg::TwistStamped RobotController::halt_command() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
}

// Gets the latest keyboard command
geometry_msgs::msg::TwistStamped RobotController::keyboard_command() const {
    return last_key_cmd_;
}

// Allows the robot to escape symmetric obstacles within 1ft
// by turing ~180 +- 30 degrees
geometry_msgs::msg::TwistStamped RobotController::escape_command() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;

    // Continue committed turn until target yaw is reached
    if(escape_active_) {
        const double err = angle_diff(escape_target_yaw_, current_yaw_);
        if(std::fabs(err) < 0.05) {
            escape_active_ = false;
            dist_traveled_turn_ = 0.0;  // fresh start after escape
            RCLCPP_INFO(this->get_logger(), "[ESCAPE] complete");
            return cmd;
        }
        cmd.twist.angular.z = (err > 0.0) ? ESCAPE_TURN_SPEED_ : -ESCAPE_TURN_SPEED_;  // <-- FIXED
        return cmd;
    }

    // Trigger condition: both halves of front cone within 1ft AND symmetric
    double min_left, min_right;
    front_left_right_mins(min_left, min_right);
    const double min_front = front_min_range();

    const bool left_close = std::isfinite(min_left) && min_left < OBSTACLE_DISTANCE_;
    const bool right_close = std::isfinite(min_right) && min_right < OBSTACLE_DISTANCE_;
    const bool front_close = std::isfinite(min_front) && min_front < OBSTACLE_DISTANCE_;

    if(left_close && right_close && front_close) {
        const double diff = std::fabs(min_left - min_right);
        if(diff < 0.12) {
            const double turn = M_PI + escape_extra_turn_rad_(rng_);  // 180° ±30°
            escape_target_yaw_ = normalize_angle(current_yaw_ + turn);
            escape_active_ = true;
            RCLCPP_WARN(this->get_logger(),
                        "[ESCAPE] triggered L=%.3f R=%.3f F=%.3f turn=%.0fdeg",
                        min_left, min_right, min_front, turn * 180.0 / M_PI);
            const double err = angle_diff(escape_target_yaw_, current_yaw_);
            cmd.twist.angular.z = (err > 0.0) ? ESCAPE_TURN_SPEED_ : -ESCAPE_TURN_SPEED_;  // <-- FIXED
        }
    }

    return cmd;
}

// Avoids asymmetric obstacles within 1ft in front.
// Turn away from the closer side.
geometry_msgs::msg::TwistStamped RobotController::avoid_command() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;

    double min_left, min_right;
    front_left_right_mins(min_left, min_right);

    const bool left_close = std::isfinite(min_left) && min_left < OBSTACLE_DISTANCE_;
    const bool right_close = std::isfinite(min_right) && min_right < OBSTACLE_DISTANCE_;

    // Nothing in the front cone
    if(!left_close && !right_close) return cmd;

    // Both sides close and symmetric yield to escape
    if(left_close && right_close && std::fabs(min_left - min_right) < 0.12) return cmd;

    // Asymmetric obstacle turn away from whichever side is closer
    const double l = left_close ? min_left : std::numeric_limits<double>::infinity();
    const double r = right_close ? min_right : std::numeric_limits<double>::infinity();
    // Obstacle closer on left, turn right
    // Obstacle closer on right, turn left
    cmd.twist.angular.z = (l < r) ? -AVOID_TURN_SPEED_ : AVOID_TURN_SPEED_;  // <-- FIXED

    RCLCPP_INFO(this->get_logger(),
                "[AVOID] L=%.3f R=%.3f az=%.2f", min_left, min_right, cmd.twist.angular.z);  // <-- FIXED

    return cmd;
}

// Creates a random turn command
geometry_msgs::msg::TwistStamped RobotController::random_turn_command() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;

    /*if(random_turn_active_) {
        const double err = angle_diff(random_turn_target_yaw_, current_yaw_);
        if(std::fabs(err) < 0.05) {
            random_turn_active_ = false;
            dist_traveled_turn_ = 0.0;
            return cmd;
        }
        cmd.twist.angular.z = (err > 0.0) ? RANDOM_TURN_SPEED_ : -RANDOM_TURN_SPEED_;  // <-- FIXED
        return cmd;
    }

    // Trigger after every 1ft of forward travel
    if(dist_traveled_turn_ >= OBSTACLE_DISTANCE_) {
        const double delta = random_small_turn_rad_(rng_);  // +- 15 degrees
        random_turn_target_yaw_ = normalize_angle(current_yaw_ + delta);
        random_turn_active_ = true;
        dist_traveled_turn_ = 0.0;
        RCLCPP_INFO(this->get_logger(),
                    "[RANDOM_TURN] delta=%.1fdeg", delta * 180.0 / M_PI);
        const double err = angle_diff(random_turn_target_yaw_, current_yaw_);
        cmd.twist.angular.z = (err > 0.0) ? RANDOM_TURN_SPEED_ : -RANDOM_TURN_SPEED_;  // <-- FIXED
    }*/
    return cmd;
}

// Moves the robot forward
geometry_msgs::msg::TwistStamped RobotController::forward_command() const {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = FORWARD_SPEED_;

    cmd.twist.angular.z = 0.0;

    // RCLCPP_WARN(this->get_logger(), "[CUR TWIST] linear.x=%.3f angular.z=%.3f", cmd.twist.linear.x, cmd.twist.angular.z);

    return cmd;
}

// Updates the distance traveled to determine if a random turn should occur
void RobotController::update_distance_traveled(
    const geometry_msgs::msg::TwistStamped& cmd) {
    if(cmd.twist.linear.x > 0.0 && std::fabs(cmd.twist.angular.z) < 1e-6) {
        const double dx = current_x_position_ - last_x_position_;
        const double dy = current_y_position_ - last_y_position_;
        dist_traveled_turn_ += std::hypot(dx, dy);
    }
}

// Control loop
// Subsumption-style priority arbitration.
// Priority order: Halt (1) > Teleop (2) > Escape (3) > Avoid (4) >
//                 RandomTurn (5) > Forward (6)
// Each higher-priority behavior suppresses all lower ones when active.

void RobotController::control_loop() {
    if(!latest_scan_ || !latest_odom_) return;

    geometry_msgs::msg::TwistStamped chosen;

    if(collision_found()) {
        chosen = halt_command();
        RCLCPP_WARN(this->get_logger(), "[HALT] min=%.3f", global_min_range());
        // bumper_hit_ = true;
    } else {
        // bumper_hit_ = false;
        geometry_msgs::msg::TwistStamped esc = escape_command();

        if(escape_active_ || std::fabs(esc.twist.angular.z) > 1e-6) {  // <-- FIXED
            random_turn_active_ = false;
            dist_traveled_turn_ = 0.0;
            chosen = esc;
        } else {
            geometry_msgs::msg::TwistStamped av = avoid_command();
            if(std::fabs(av.twist.angular.z) > 1e-6) {  // <-- FIXED
                random_turn_active_ = false;
                dist_traveled_turn_ = 0.0;
                chosen = av;
            } else {
                chosen = forward_command();
            }
        }
    }

    /*if(!latest_scan_ || !latest_odom_) return;

    update_distance_traveled(last_published_cmd_);

    geometry_msgs::msg::TwistStamped chosen;

    // 1. Halt — bumper equivalent via laser global minimum
    if(collision_found()) {
        escape_active_ = false;
        random_turn_active_ = false;
        chosen = halt_command();
        RCLCPP_WARN(this->get_logger(), "[HALT] min=%.3f", global_min_range());
    }
    // 2. Keyboard teleop
    else if(teleop_active()) {
        RCLCPP_WARN(this->get_logger(), "[TELEOP ACTIVE]");

        escape_active_ = false;
        random_turn_active_ = false;
        chosen = keyboard_command();
    } else {
        // 3. Escape
        geometry_msgs::msg::TwistStamped esc = escape_command();
        if(escape_active_ || std::fabs(esc.twist.angular.z) > 1e-6) {  // <-- FIXED
            random_turn_active_ = false;
            dist_traveled_turn_ = 0.0;
            chosen = esc;
        } else {
            // 4. Avoid fires while asymmetric obstacle present
            geometry_msgs::msg::TwistStamped av = avoid_command();
            if(std::fabs(av.twist.angular.z) > 1e-6) {  // <-- FIXED
                random_turn_active_ = false;
                dist_traveled_turn_ = 0.0;
                chosen = av;
            } else {
                // 5. Random turn
                RCLCPP_WARN(this->get_logger(), "[RANDOM TURN ACTIVE]");

                geometry_msgs::msg::TwistStamped rt = random_turn_command();
                if(random_turn_active_ || std::fabs(rt.twist.angular.z) > 1e-6) {  // <-- FIXED
                    chosen = rt;
                } else {
                    // 6. Forward — base layer
                    chosen = forward_command();
                }
            }
        }
    }
    */
    last_published_cmd_ = chosen;

    // Add Timestamp before publishing! (Crucial for hardware to accept the command)
    chosen.header.stamp = this->now();
    chosen.header.frame_id = "base_link";

    cmd_pub_->publish(chosen);
    RCLCPP_WARN(this->get_logger(), "[PUB] linear.x=%.3f angular.z=%.3f",
                chosen.twist.linear.x, chosen.twist.angular.z);  // <-- FIXED
}