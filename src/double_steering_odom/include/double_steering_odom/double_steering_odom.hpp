#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <map>

namespace double_steering_odom
{

class DoubleSteeringOdom : public rclcpp::Node
{
public:
    DoubleSteeringOdom();
    ~DoubleSteeringOdom();

private:
    // Callback functions
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    // Odometry calculation functions
    void calculateOdometry(double dt);
    void publishOdometry();
    void publishTransform();
    
    // Helper functions
    double getJointPosition(const sensor_msgs::msg::JointState::SharedPtr& msg, const std::string& joint_name);
    double getJointVelocity(const sensor_msgs::msg::JointState::SharedPtr& msg, const std::string& joint_name);
    
    // ROS2 communication
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Robot parameters (loaded from config)
    double wheel_base_;          // Distance between front and rear axles [m]
    double track_width_;         // Distance between left and right wheels [m]
    double wheel_radius_;        // Wheel radius [m]
    double min_update_rate_;     // Minimum update rate in Hz
    
    // Topic and frame names
    std::string joint_state_topic_;
    std::string odom_topic_;
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;
    
    // Joint names for the 4 wheels
    std::string fl_steer_joint_;  // Front left steering joint
    std::string fr_steer_joint_;  // Front right steering joint
    std::string rl_steer_joint_;  // Rear left steering joint
    std::string rr_steer_joint_;  // Rear right steering joint
    
    std::string fl_wheel_joint_;  // Front left wheel joint
    std::string fr_wheel_joint_;  // Front right wheel joint
    std::string rl_wheel_joint_;  // Rear left wheel joint
    std::string rr_wheel_joint_;  // Rear right wheel joint
    
    // Odometry state variables
    double x_;          // X position in odom frame [m]
    double y_;          // Y position in odom frame [m]
    double theta_;      // Orientation (yaw) in odom frame [rad]
    double v_x_;        // Linear velocity in x direction [m/s]
    double v_y_;        // Linear velocity in y direction [m/s]
    double omega_;      // Angular velocity [rad/s]
    
    // Previous values for numerical integration
    rclcpp::Time last_time_;
    bool first_message_;
    
    // Store last joint states for velocity calculation if not provided
    std::map<std::string, double> last_wheel_positions_;
};

} // namespace double_steering_odom
