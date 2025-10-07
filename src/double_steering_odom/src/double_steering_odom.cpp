#include "double_steering_odom/double_steering_odom.hpp"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace double_steering_odom
{

DoubleSteeringOdom::DoubleSteeringOdom()
    : Node("double_steering_odom"),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
      v_x_(0.0),
      v_y_(0.0),
      omega_(0.0),
      first_message_(true)
{
    // Declare and load parameters
    this->declare_parameter<double>("wheel_base", 2.39);
    this->declare_parameter<double>("track_width", 1.34);
    this->declare_parameter<double>("wheel_radius", 0.4);
    this->declare_parameter<double>("min_update_rate", 10.0);  // Minimum update rate in Hz
    
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<bool>("publish_tf", true);
    
    // Joint names
    this->declare_parameter<std::string>("fl_steer_joint", "steer_front_left_link_j");
    this->declare_parameter<std::string>("fr_steer_joint", "steer_front_right_link_j");
    this->declare_parameter<std::string>("rl_steer_joint", "steer_rear_left_link_j");
    this->declare_parameter<std::string>("rr_steer_joint", "steer_rear_right_link_j");
    
    this->declare_parameter<std::string>("fl_wheel_joint", "drive_front_left_link_j");
    this->declare_parameter<std::string>("fr_wheel_joint", "drive_front_right_link_j");
    this->declare_parameter<std::string>("rl_wheel_joint", "drive_rear_left_link_j");
    this->declare_parameter<std::string>("rr_wheel_joint", "drive_rear_right_link_j");
    
    // Get parameters
    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("track_width", track_width_);
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("min_update_rate", min_update_rate_);
    
    this->get_parameter("joint_state_topic", joint_state_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("publish_tf", publish_tf_);
    
    this->get_parameter("fl_steer_joint", fl_steer_joint_);
    this->get_parameter("fr_steer_joint", fr_steer_joint_);
    this->get_parameter("rl_steer_joint", rl_steer_joint_);
    this->get_parameter("rr_steer_joint", rr_steer_joint_);
    
    this->get_parameter("fl_wheel_joint", fl_wheel_joint_);
    this->get_parameter("fr_wheel_joint", fr_wheel_joint_);
    this->get_parameter("rl_wheel_joint", rl_wheel_joint_);
    this->get_parameter("rr_wheel_joint", rr_wheel_joint_);
    
    // Validate parameters
    if (wheel_base_ <= 0.0 || track_width_ <= 0.0 || wheel_radius_ <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), 
            "Invalid robot parameters: wheel_base=%.2f, track_width=%.2f, wheel_radius=%.2f",
            wheel_base_, track_width_, wheel_radius_);
        throw std::runtime_error("Invalid robot parameters");
    }
    
    if (min_update_rate_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid min_update_rate (%.2f Hz), setting to 10.0 Hz", min_update_rate_);
        min_update_rate_ = 10.0;
    }
    
    RCLCPP_INFO(this->get_logger(), 
        "Double Ackerman Odometry initialized with: wheelbase=%.2fm, track_width=%.2fm, wheel_radius=%.2fm, min_rate=%.1fHz",
        wheel_base_, track_width_, wheel_radius_, min_update_rate_);
    
    // Create subscriber
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_, 10,
        std::bind(&DoubleSteeringOdom::jointStateCallback, this, std::placeholders::_1));
    
    // Create publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    
    // Create TF broadcaster if requested
    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        RCLCPP_INFO(this->get_logger(), "TF broadcasting enabled: %s -> %s", 
                    odom_frame_.c_str(), base_frame_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "TF broadcasting disabled");
    }
    
    RCLCPP_INFO(this->get_logger(), "Double Steering Odometry node started");
}

DoubleSteeringOdom::~DoubleSteeringOdom()
{
    RCLCPP_INFO(this->get_logger(), "Double Steering Odometry node shutting down");
}

double DoubleSteeringOdom::getJointPosition(const sensor_msgs::msg::JointState::SharedPtr& msg, const std::string& joint_name)
{
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
    if (it != msg->name.end()) {
        size_t index = std::distance(msg->name.begin(), it);
        if (index < msg->position.size()) {
            return msg->position[index];
        }
    }
    return 0.0;
}

double DoubleSteeringOdom::getJointVelocity(const sensor_msgs::msg::JointState::SharedPtr& msg, const std::string& joint_name)
{
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
    if (it != msg->name.end()) {
        size_t index = std::distance(msg->name.begin(), it);
        if (index < msg->velocity.size()) {
            return msg->velocity[index];
        }
    }
    return 0.0;
}

void DoubleSteeringOdom::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Get current time - use message timestamp if available, otherwise use current time
    rclcpp::Time current_time;
    if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
        current_time = rclcpp::Time(msg->header.stamp);
    } else {
        current_time = this->now();
    }
    
    if (first_message_) {
        last_time_ = current_time;
        first_message_ = false;
        
        // Initialize last wheel positions
        last_wheel_positions_[fl_wheel_joint_] = getJointPosition(msg, fl_wheel_joint_);
        last_wheel_positions_[fr_wheel_joint_] = getJointPosition(msg, fr_wheel_joint_);
        last_wheel_positions_[rl_wheel_joint_] = getJointPosition(msg, rl_wheel_joint_);
        last_wheel_positions_[rr_wheel_joint_] = getJointPosition(msg, rr_wheel_joint_);
        
        RCLCPP_INFO(this->get_logger(), "First joint state received, starting odometry calculation");
        return;
    }
    
    // Calculate time step
    double dt = (current_time - last_time_).seconds();
    
    // Handle edge cases for time step
    if (dt < 0.0) {
        // Time went backwards (simulation reset or clock issue)
        RCLCPP_WARN(this->get_logger(), "Time went backwards (dt=%.3f), resetting odometry timing", dt);
        last_time_ = current_time;
        return;
    }
    
    // Calculate minimum acceptable time step based on configured rate
    double min_dt = 1.0 / (min_update_rate_ * 10.0);  // Allow 10x faster than configured rate
    double max_dt = 1.0 / min_update_rate_;            // Maximum expected time between updates
    
    if (dt < min_dt) {
        // Time step too small (likely duplicate message or paused simulation)
        // Skip this update but don't log warning on every message
        static int skip_count = 0;
        if (++skip_count % 100 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Very small time step (dt=%.6f < min_dt=%.6f), skipping update. Count: %d", 
                        dt, min_dt, skip_count);
        }
        return;
    }
    
    if (dt > max_dt * 2.0) {
        // Time step too large (likely simulation paused or node just started)
        RCLCPP_WARN(this->get_logger(), "Large time step detected (dt=%.3f > %.3f), clamping to %.3fs", 
                    dt, max_dt * 2.0, max_dt);
        dt = max_dt;  // Clamp to reasonable value instead of skipping
    }
    
    // Get steering angles
    double delta_fl = getJointPosition(msg, fl_steer_joint_);
    double delta_fr = getJointPosition(msg, fr_steer_joint_);
    double delta_rl = getJointPosition(msg, rl_steer_joint_);
    double delta_rr = getJointPosition(msg, rr_steer_joint_);
    
    // Get wheel velocities (rad/s)
    // Try to get velocities from joint_state, if not available calculate from position
    double omega_fl = getJointVelocity(msg, fl_wheel_joint_);
    double omega_fr = getJointVelocity(msg, fr_wheel_joint_);
    double omega_rl = getJointVelocity(msg, rl_wheel_joint_);
    double omega_rr = getJointVelocity(msg, rr_wheel_joint_);
    
    // If velocities not provided, calculate from position change
    if (msg->velocity.empty() || omega_fl == 0.0) {
        double pos_fl = getJointPosition(msg, fl_wheel_joint_);
        double pos_fr = getJointPosition(msg, fr_wheel_joint_);
        double pos_rl = getJointPosition(msg, rl_wheel_joint_);
        double pos_rr = getJointPosition(msg, rr_wheel_joint_);
        
        omega_fl = (pos_fl - last_wheel_positions_[fl_wheel_joint_]) / dt;
        omega_fr = (pos_fr - last_wheel_positions_[fr_wheel_joint_]) / dt;
        omega_rl = (pos_rl - last_wheel_positions_[rl_wheel_joint_]) / dt;
        omega_rr = (pos_rr - last_wheel_positions_[rr_wheel_joint_]) / dt;
        
        // Update last positions
        last_wheel_positions_[fl_wheel_joint_] = pos_fl;
        last_wheel_positions_[fr_wheel_joint_] = pos_fr;
        last_wheel_positions_[rl_wheel_joint_] = pos_rl;
        last_wheel_positions_[rr_wheel_joint_] = pos_rr;
    }
    
    // Calculate linear velocities at each wheel contact point (m/s)
    double v_fl = omega_fl * wheel_radius_;
    double v_fr = omega_fr * wheel_radius_;
    double v_rl = omega_rl * wheel_radius_;
    double v_rr = omega_rr * wheel_radius_;
    
    // DOUBLE ACKERMAN ODOMETRY CALCULATION
    // The vehicle has 4-wheel steering with front and rear axles steering in opposite directions
    
    // Calculate average steering angle for front and rear axles
    double delta_front_avg = (delta_fl + delta_fr) / 2.0;
    double delta_rear_avg = (delta_rl + delta_rr) / 2.0;
    
    // Calculate average wheel velocity for front and rear axles
    double v_front_avg = (v_fl + v_fr) / 2.0;
    double v_rear_avg = (v_rl + v_rr) / 2.0;
    
    // For double Ackerman, we can calculate the instantaneous center of curvature (ICC)
    // using the steering angles
    
    // Vehicle's longitudinal velocity (average of front and rear)
    double v_longitudinal = (v_front_avg + v_rear_avg) / 2.0;
    
    // Calculate angular velocity from steering geometry
    // For double Ackerman: omega = (v_front * tan(delta_front) + v_rear * tan(delta_rear)) / L
    // Where front and rear steer in opposite directions
    
    // Angular velocity calculation
    // Note: rear steering is opposite to front, so we add them
    if (std::abs(delta_front_avg) < 0.001 && std::abs(delta_rear_avg) < 0.001) {
        // Straight line motion
        omega_ = 0.0;
        v_x_ = v_longitudinal;
        v_y_ = 0.0;
    } else {
        // Turning motion
        // For double Ackerman: ω = (v_front * sin(δ_front) - v_rear * sin(δ_rear)) / L
        omega_ = (v_front_avg * std::sin(delta_front_avg) - v_rear_avg * std::sin(delta_rear_avg)) / wheel_base_;
        
        // Linear velocities in robot frame
        // v_x is forward velocity (average of wheels projected)
        v_x_ = v_longitudinal * std::cos((delta_front_avg + delta_rear_avg) / 4.0);
        
        // v_y is lateral velocity (typically small for Ackerman)
        v_y_ = v_longitudinal * std::sin((delta_front_avg - delta_rear_avg) / 4.0);
    }
    
    // Integrate to get pose
    // Convert velocities from robot frame to world frame
    double delta_x = (v_x_ * std::cos(theta_) - v_y_ * std::sin(theta_)) * dt;
    double delta_y = (v_x_ * std::sin(theta_) + v_y_ * std::cos(theta_)) * dt;
    double delta_theta = omega_ * dt;
    
    // Update pose
    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;
    
    // Normalize theta to [-pi, pi]
    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
    
    // Update last time
    last_time_ = current_time;
    
    // Publish odometry
    publishOdometry();
    
    // Publish TF if requested
    if (publish_tf_) {
        publishTransform();
    }
    
    // Debug output
    RCLCPP_DEBUG(this->get_logger(), 
        "Odom: x=%.2f, y=%.2f, theta=%.2f | v_x=%.2f, v_y=%.2f, omega=%.2f",
        x_, y_, theta_, v_x_, v_y_, omega_);
}

void DoubleSteeringOdom::publishOdometry()
{
    auto odom_msg = nav_msgs::msg::Odometry();
    
    // Header
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    
    // Position
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    // Orientation (convert yaw to quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    // Velocity in robot frame
    odom_msg.twist.twist.linear.x = v_x_;
    odom_msg.twist.twist.linear.y = v_y_;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = omega_;
    
    // Covariance (can be tuned based on your robot's characteristics)
    // Position covariance
    odom_msg.pose.covariance[0] = 0.001;  // x
    odom_msg.pose.covariance[7] = 0.001;  // y
    odom_msg.pose.covariance[35] = 0.01;  // yaw
    
    // Velocity covariance
    odom_msg.twist.covariance[0] = 0.001;  // vx
    odom_msg.twist.covariance[7] = 0.001;  // vy
    odom_msg.twist.covariance[35] = 0.01;  // vyaw
    
    // Publish
    odom_pub_->publish(odom_msg);
}

void DoubleSteeringOdom::publishTransform()
{
    geometry_msgs::msg::TransformStamped transform;
    
    // Header
    transform.header.stamp = this->now();
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    
    // Translation
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    
    // Rotation
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    // Broadcast
    tf_broadcaster_->sendTransform(transform);
}

} // namespace double_steering_odom
