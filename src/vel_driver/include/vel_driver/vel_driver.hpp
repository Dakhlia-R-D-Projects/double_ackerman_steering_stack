#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace gazebo
{
    class VelDriver : public rclcpp::Node
    {
        public:
            VelDriver();
            ~VelDriver();
            void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        private:
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cmd_front_left_steer, pub_cmd_front_right_steer, pub_cmd_rear_left_steer, pub_cmd_rear_right_steer;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cmd_front_left_rotor, pub_cmd_front_right_rotor, pub_cmd_rear_left_rotor, pub_cmd_rear_right_rotor;
            std_msgs::msg::Float64 cmd_front_left_steer, cmd_front_right_steer, cmd_rear_left_steer, cmd_rear_right_steer;
            std_msgs::msg::Float64 cmd_front_left_rotor, cmd_front_right_rotor, cmd_rear_left_rotor, cmd_rear_right_rotor;

            // Double Ackerman vehicle parameters to be loaded from yaml file
            float wheel_base;        // Distance between front and rear axles [m]
            float track_width;       // Distance between left and right wheels [m]
            float wheel_radius;      // Wheel radius [m]
            float max_steering_angle; // Maximum steering angle [rad]
            float velocity_threshold; // Threshold for zero velocity detection
            
            // Helper function to calculate Ackerman steering angles
            void calculateAckermannAngles(float turning_radius, bool is_front, 
                                         float &left_angle, float &right_angle);
    };
}