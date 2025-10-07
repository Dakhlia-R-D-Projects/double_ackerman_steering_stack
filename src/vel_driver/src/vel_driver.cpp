#include "vel_driver/vel_driver.hpp"
#include <cmath>
#include <algorithm>

namespace gazebo
{

// constructor
VelDriver::VelDriver()
    : Node("vel_driver")
{
    // load parameters 

    //// Double Ackerman vehicle parameters
    this->declare_parameter<float>("wheel_base", 2.39);
    this->declare_parameter<float>("track_width", 1.34);
    this->declare_parameter<float>("wheel_radius", 0.4);
    this->declare_parameter<float>("max_steering_angle", 0.7);
    this->declare_parameter<float>("velocity_threshold", 1e-2);

    this->get_parameter("wheel_base", wheel_base);
    this->get_parameter("track_width", track_width);
    this->get_parameter("wheel_radius", wheel_radius);
    this->get_parameter("max_steering_angle", max_steering_angle);
    this->get_parameter("velocity_threshold", velocity_threshold);

    // Parameter validation to prevent division by zero and other issues
    if (std::abs(wheel_radius) < 1e-9) {
        RCLCPP_ERROR(this->get_logger(), "Invalid wheel_radius parameter (%f). Must be non-zero. Setting to default value 0.4.", wheel_radius);
        wheel_radius = 0.4;
    }
    if (wheel_base <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid wheel_base parameter (%f). Must be positive. Setting to default value 2.39.", wheel_base);
        wheel_base = 2.39;
    }
    if (track_width <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid track_width parameter (%f). Must be positive. Setting to default value 1.34.", track_width);
        track_width = 1.34;
    }
    if (velocity_threshold <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid velocity_threshold parameter (%f). Must be positive. Setting to default value 1e-2.", velocity_threshold);
        velocity_threshold = 1e-2;
    }
    
    RCLCPP_INFO(this->get_logger(), "Double Ackerman Parameters: wheelbase=%.2fm, track_width=%.2fm, wheel_radius=%.2fm, max_steer=%.2frad", 
                wheel_base, track_width, wheel_radius, max_steering_angle);

    //// subscribing topic names
    this->declare_parameter<std::string>("control_cmd_vel_topic", "/cmd_vel");
    std::string control_cmd_vel_topic;
    this->get_parameter("control_cmd_vel_topic", control_cmd_vel_topic);

    //// publishing topic names
    this->declare_parameter<std::string>("front_left_steer_cmd_topic", "/fwids/front_left_steer_rad/command");
    this->declare_parameter<std::string>("front_right_steer_cmd_topic", "/fwids/front_right_steer_rad/command");
    this->declare_parameter<std::string>("rear_left_steer_cmd_topic", "/fwids/rear_left_steer_rad/command");
    this->declare_parameter<std::string>("rear_right_steer_cmd_topic", "/fwids/rear_right_steer_rad/command");
    this->declare_parameter<std::string>("front_left_rotor_cmd_topic", "/fwids/front_left_rotor_radpersec/command");
    this->declare_parameter<std::string>("front_right_rotor_cmd_topic", "/fwids/front_right_rotor_radpersec/command");
    this->declare_parameter<std::string>("rear_left_rotor_cmd_topic", "/fwids/rear_left_rotor_radpersec/command");
    this->declare_parameter<std::string>("rear_right_rotor_cmd_topic", "/fwids/rear_right_rotor_radpersec/command");

    std::string front_left_steer_cmd_topic, front_right_steer_cmd_topic, rear_left_steer_cmd_topic, rear_right_steer_cmd_topic;
    std::string front_left_rotor_cmd_topic, front_right_rotor_cmd_topic, rear_left_rotor_cmd_topic, rear_right_rotor_cmd_topic;
    
    this->get_parameter("front_left_steer_cmd_topic", front_left_steer_cmd_topic);
    this->get_parameter("front_right_steer_cmd_topic", front_right_steer_cmd_topic);
    this->get_parameter("rear_left_steer_cmd_topic", rear_left_steer_cmd_topic);
    this->get_parameter("rear_right_steer_cmd_topic", rear_right_steer_cmd_topic);
    this->get_parameter("front_left_rotor_cmd_topic", front_left_rotor_cmd_topic);
    this->get_parameter("front_right_rotor_cmd_topic", front_right_rotor_cmd_topic);
    this->get_parameter("rear_left_rotor_cmd_topic", rear_left_rotor_cmd_topic);
    this->get_parameter("rear_right_rotor_cmd_topic", rear_right_rotor_cmd_topic);

    // initialize subscribers
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        control_cmd_vel_topic,1, std::bind(&VelDriver::cmdVelCallback, this, std::placeholders::_1));
    // initialize publishers
    pub_cmd_front_left_steer = this->create_publisher<std_msgs::msg::Float64>(front_left_steer_cmd_topic,1);
    pub_cmd_front_right_steer = this->create_publisher<std_msgs::msg::Float64>(front_right_steer_cmd_topic,1);
    pub_cmd_rear_left_steer = this->create_publisher<std_msgs::msg::Float64>(rear_left_steer_cmd_topic,1);
    pub_cmd_rear_right_steer = this->create_publisher<std_msgs::msg::Float64>(rear_right_steer_cmd_topic,1);
    pub_cmd_front_left_rotor = this->create_publisher<std_msgs::msg::Float64>(front_left_rotor_cmd_topic,1);
    pub_cmd_front_right_rotor = this->create_publisher<std_msgs::msg::Float64>(front_right_rotor_cmd_topic,1);
    pub_cmd_rear_left_rotor = this->create_publisher<std_msgs::msg::Float64>(rear_left_rotor_cmd_topic,1);
    pub_cmd_rear_right_rotor = this->create_publisher<std_msgs::msg::Float64>(rear_right_rotor_cmd_topic,1);
}

// destructor
VelDriver::~VelDriver()
{
    // No Contents
}

// Helper function to calculate Ackerman steering angles for left and right wheels
void VelDriver::calculateAckermannAngles(float turning_radius, bool is_front, 
                                         float &left_angle, float &right_angle)
{
    // For straight motion (infinite turning radius)
    if (std::isinf(turning_radius) || std::abs(turning_radius) > 1e6) {
        left_angle = 0.0;
        right_angle = 0.0;
        return;
    }
    
    // Ackerman geometry: inner and outer wheel angles differ
    // The wheel on the inside of the turn has a larger steering angle
    float half_track = track_width / 2.0;
    float abs_radius = std::abs(turning_radius);
    
    // Safety check: ensure we don't divide by very small numbers
    // The inner wheel radius must be greater than half the track width
    if (abs_radius <= half_track) {
        RCLCPP_WARN(this->get_logger(), 
            "Turning radius (%.2f m) too small compared to track width (%.2f m). Setting angles to max.",
            abs_radius, track_width);
        // Set to maximum steering angle
        left_angle = (turning_radius > 0) ? max_steering_angle : -max_steering_angle;
        right_angle = left_angle * 0.7; // Outer wheel angle is smaller
        if (!is_front) {
            left_angle = -left_angle;
            right_angle = -right_angle;
        }
        return;
    }
    
    if (turning_radius > 0) {
        // Turning left (counter-clockwise)
        // Inner wheel (left) has larger angle, outer wheel (right) has smaller angle
        left_angle = std::atan(wheel_base / (abs_radius - half_track));   // inner wheel
        right_angle = std::atan(wheel_base / (abs_radius + half_track));  // outer wheel
    } else {
        // Turning right (clockwise)
        // Inner wheel (right) has larger angle, outer wheel (left) has smaller angle
        right_angle = std::atan(wheel_base / (abs_radius - half_track));      // inner wheel
        left_angle = std::atan(wheel_base / (abs_radius + half_track));       // outer wheel
        // Make angles negative for right turn
        left_angle = -left_angle;
        right_angle = -right_angle;
    }
    
    // For rear axle in double Ackerman, angles are opposite sign
    // This creates the characteristic "crab steering" behavior
    if (!is_front) {
        left_angle = -left_angle;
        right_angle = -right_angle;
    }
    
    // Clamp to maximum steering angle for safety
    left_angle = std::max(-max_steering_angle, std::min(max_steering_angle, left_angle));
    right_angle = std::max(-max_steering_angle, std::min(max_steering_angle, right_angle));
}

// /cmd_vel topic callback
void VelDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // if msg contains NaN, return without publishing commands
    if (std::isnan(msg->linear.x) || std::isnan(msg->linear.y) || std::isnan(msg->angular.z))
    {
        RCLCPP_WARN(this->get_logger(), "Received NaN in Twist Command. Skip this message.");
        return;
    }

    // parse received twist message
    float v_x = msg->linear.x;   // forward velocity
    float v_y = msg->linear.y;   // lateral velocity (should be ~0 for Ackerman)
    float omega = msg->angular.z; // angular velocity around the z-axis

    // [for debug] announce received Twist message
    RCLCPP_DEBUG(this->get_logger(), "Received Twist: vx=%.2f, vy=%.2f, omega=%.2f", v_x, v_y, omega);

    // Check if robot is commanded to stop
    bool is_stopping = (std::abs(v_x) < velocity_threshold && 
                       std::abs(v_y) < velocity_threshold && 
                       std::abs(omega) < velocity_threshold);

    if (is_stopping) {
        // Stop all wheels
        cmd_front_left_rotor.data = 0.0;
        cmd_front_right_rotor.data = 0.0;
        cmd_rear_left_rotor.data = 0.0;
        cmd_rear_right_rotor.data = 0.0;
        
        // Keep last steering angles (don't change them when stopped)
        RCLCPP_DEBUG(this->get_logger(), "Robot stopping - all wheels set to zero velocity");
    } else {
        // Double Ackerman steering kinematics
        // Calculate turning radius from angular velocity
        float turning_radius;
        
        if (std::abs(omega) < 1e-6) {
            // Straight line motion
            turning_radius = std::numeric_limits<float>::infinity();
        } else {
            // For double Ackerman, the instantaneous center of rotation (ICR) 
            // is located at the intersection of the wheel axes extensions
            // Turning radius = linear_velocity / angular_velocity
            // But we need to ensure it's kinematically feasible
            turning_radius = v_x / omega;
            
            // Calculate minimum turning radius based on maximum steering angle
            // For double Ackerman: R_min = wheelbase / tan(max_steering_angle)
            // Using the center of the vehicle as reference
            float min_turning_radius = wheel_base / std::tan(max_steering_angle);
            
            // Add safety margin (half track width) to account for wheel positions
            float safe_min_radius = min_turning_radius + track_width / 2.0;
            
            // Limit the turning radius to kinematically feasible values
            if (std::abs(turning_radius) < safe_min_radius) {
                // Requested turn is too tight, limit to minimum feasible radius
                turning_radius = (turning_radius > 0) ? safe_min_radius : -safe_min_radius;
                RCLCPP_DEBUG(this->get_logger(), 
                    "Requested turning radius too tight, limited to %.2f m (min: %.2f m)", 
                    turning_radius, safe_min_radius);
            }
        }
        
        // Calculate Ackerman steering angles for front and rear axles
        float front_left_angle, front_right_angle;
        float rear_left_angle, rear_right_angle;
        
        calculateAckermannAngles(turning_radius, true, front_left_angle, front_right_angle);
        calculateAckermannAngles(turning_radius, false, rear_left_angle, rear_right_angle);
        
        // Update steering angle commands
        cmd_front_left_steer.data = front_left_angle;
        cmd_front_right_steer.data = front_right_angle;
        cmd_rear_left_steer.data = rear_left_angle;
        cmd_rear_right_steer.data = rear_right_angle;
        
        // Calculate wheel velocities
        // For double Ackerman, all wheels should rotate at velocities that match the turning geometry
        
        if (std::isinf(turning_radius)) {
            // Straight line - all wheels same speed
            float wheel_speed_base = v_x / wheel_radius;
            cmd_front_left_rotor.data = wheel_speed_base;
            cmd_front_right_rotor.data = wheel_speed_base;
            cmd_rear_left_rotor.data = wheel_speed_base;
            cmd_rear_right_rotor.data = wheel_speed_base;
        } else {
            // Different speeds for inner and outer wheels during turning
            // Each wheel travels along a circular arc with radius dependent on its position
            float half_track = track_width / 2.0;
            float half_wheelbase = wheel_base / 2.0;
            
            // Calculate the actual distance from the instantaneous center of rotation (ICR)
            // to each wheel contact point
            // ICR is on the line connecting the midpoints of front and rear axles
            float abs_turning_radius = std::abs(turning_radius);
            
            // Distance from ICR to each wheel (using Pythagorean theorem)
            // For left turn (turning_radius > 0), ICR is on the left side
            // For right turn (turning_radius < 0), ICR is on the right side
            float r_fl, r_fr, r_rl, r_rr;
            
            if (turning_radius > 0) {
                // Left turn: left wheels are inner, right wheels are outer
                r_fl = std::sqrt(std::pow(abs_turning_radius - half_track, 2) + std::pow(half_wheelbase, 2));
                r_fr = std::sqrt(std::pow(abs_turning_radius + half_track, 2) + std::pow(half_wheelbase, 2));
                r_rl = std::sqrt(std::pow(abs_turning_radius - half_track, 2) + std::pow(half_wheelbase, 2));
                r_rr = std::sqrt(std::pow(abs_turning_radius + half_track, 2) + std::pow(half_wheelbase, 2));
            } else {
                // Right turn: right wheels are inner, left wheels are outer
                r_fl = std::sqrt(std::pow(abs_turning_radius + half_track, 2) + std::pow(half_wheelbase, 2));
                r_fr = std::sqrt(std::pow(abs_turning_radius - half_track, 2) + std::pow(half_wheelbase, 2));
                r_rl = std::sqrt(std::pow(abs_turning_radius + half_track, 2) + std::pow(half_wheelbase, 2));
                r_rr = std::sqrt(std::pow(abs_turning_radius - half_track, 2) + std::pow(half_wheelbase, 2));
            }
            
            // Calculate angular velocity about the turning center
            // omega is already given, but we recalculate from the actual turning radius
            // to ensure consistency with kinematic constraints
            float actual_omega = v_x / turning_radius;  // rad/s
            float omega_abs = std::abs(actual_omega);
            
            // Linear velocity at each wheel = radius * angular_velocity
            // Then convert to wheel angular velocity = linear_velocity / wheel_radius
            cmd_front_left_rotor.data = (r_fl * omega_abs) / wheel_radius;
            cmd_front_right_rotor.data = (r_fr * omega_abs) / wheel_radius;
            cmd_rear_left_rotor.data = (r_rl * omega_abs) / wheel_radius;
            cmd_rear_right_rotor.data = (r_rr * omega_abs) / wheel_radius;
            
            // Preserve direction of motion (forward/backward)
            if (v_x < 0) {
                cmd_front_left_rotor.data = -cmd_front_left_rotor.data;
                cmd_front_right_rotor.data = -cmd_front_right_rotor.data;
                cmd_rear_left_rotor.data = -cmd_rear_left_rotor.data;
                cmd_rear_right_rotor.data = -cmd_rear_right_rotor.data;
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Turning radius: %.2f, Steer: FL=%.3f FR=%.3f RL=%.3f RR=%.3f", 
                     turning_radius, front_left_angle, front_right_angle, rear_left_angle, rear_right_angle);
    }

    // Publish commands
    pub_cmd_front_left_steer->publish(cmd_front_left_steer);
    pub_cmd_front_right_steer->publish(cmd_front_right_steer);
    pub_cmd_rear_left_steer->publish(cmd_rear_left_steer);
    pub_cmd_rear_right_steer->publish(cmd_rear_right_steer);

    pub_cmd_front_left_rotor->publish(cmd_front_left_rotor);
    pub_cmd_front_right_rotor->publish(cmd_front_right_rotor);
    pub_cmd_rear_left_rotor->publish(cmd_rear_left_rotor);
    pub_cmd_rear_right_rotor->publish(cmd_rear_right_rotor);
}
}

