#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class DoubleAckermanController(Node):
    """
    ROS2 node to control a double Ackerman steering vehicle.
    Subscribes to cmd_vel and publishes steering angles and wheel velocities.
    """
    def __init__(self):
        super().__init__('double_ackerman_controller')
        
        # Declare and get parameters with descriptions
        self.declare_and_get_parameters()
        
                # Log the loaded parameters
        self.get_logger().info(f"Loaded parameters: wheel_base={self.wheel_base}, track_width={self.track_width}, " + 
                              f"wheel_radius={self.wheel_radius}, max_steering_angle={self.max_steering_angle}, " +
                              f"max_wheel_speed={self.max_wheel_speed}")
        
    def declare_and_get_parameters(self):
        """
        Declare and get parameters with appropriate descriptions
        """
        # Create parameter descriptors
        wheel_base_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Distance between front and rear wheels in meters')
            
        track_width_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Distance between left and right wheels in meters')
            
        wheel_radius_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Wheel radius in meters')
            
        max_steering_angle_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Maximum steering angle in radians')
            
        max_wheel_speed_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Maximum wheel angular velocity in rad/s')
            
        # Declare parameters with default values and descriptors
        self.declare_parameter('wheel_base', 0.8, wheel_base_desc)
        self.declare_parameter('track_width', 0.6, track_width_desc)
        self.declare_parameter('wheel_radius', 0.15, wheel_radius_desc)
        self.declare_parameter('max_steering_angle', 0.6, max_steering_angle_desc)
        self.declare_parameter('max_wheel_speed', 10.0, max_wheel_speed_desc)
        
        # Get parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        
    def cmd_vel_callback(self, msg):
        
        # Subscribe to cmd_vel topic
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Create publishers for steering angles
        self.fl_steer_pub = self.create_publisher(
            Float64, 
            '/fwids/front_left_steer_rad/command', 
            10)
        self.fr_steer_pub = self.create_publisher(
            Float64, 
            '/fwids/front_right_steer_rad/command', 
            10)
        self.rl_steer_pub = self.create_publisher(
            Float64, 
            '/fwids/rear_left_steer_rad/command', 
            10)
        self.rr_steer_pub = self.create_publisher(
            Float64, 
            '/fwids/rear_right_steer_rad/command', 
            10)
        
        # Create publishers for wheel velocities
        self.fl_vel_pub = self.create_publisher(
            Float64, 
            '/fwids/front_left_rotor_radpersec/command', 
            10)
        self.fr_vel_pub = self.create_publisher(
            Float64, 
            '/fwids/front_right_rotor_radpersec/command', 
            10)
        self.rl_vel_pub = self.create_publisher(
            Float64, 
            '/fwids/rear_left_rotor_radpersec/command', 
            10)
        self.rr_vel_pub = self.create_publisher(
            Float64, 
            '/fwids/rear_right_rotor_radpersec/command', 
            10)
        
        self.get_logger().info('Double Ackerman Controller initialized')
        
    def cmd_vel_callback(self, msg):
        """
        Process incoming cmd_vel messages and convert to steering angles and wheel velocities
        
        Args:
            msg: Twist message with linear and angular velocity components
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # If vehicle is not moving, don't steer
        if abs(linear_x) < 0.001 and abs(angular_z) < 0.001:
            self.set_zero_velocity_and_steering()
            return
            
        # Calculate steering angles and wheel velocities
        steering_angles = self.calculate_steering_angles(linear_x, angular_z)
        wheel_velocities = self.calculate_wheel_velocities(linear_x, angular_z, steering_angles)
        
        # Publish steering angles
        self.publish_steering_angles(steering_angles)
        
        # Publish wheel velocities
        self.publish_wheel_velocities(wheel_velocities)
        
    def calculate_steering_angles(self, linear_x, angular_z):
        """
        Calculate the steering angles for each wheel based on the Ackerman geometry
        
        Args:
            linear_x: Linear velocity in x direction
            angular_z: Angular velocity around z axis
            
        Returns:
            Dictionary with steering angles for each wheel
        """
        steering_angles = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }
        
        # If we're not turning, wheels should be straight
        if abs(angular_z) < 0.001:
            return steering_angles
            
        # If we're turning in place, use a simple symmetric model
        if abs(linear_x) < 0.001:
            turn_direction = 1.0 if angular_z > 0 else -1.0
            # Front wheels turn one way, rear wheels turn the opposite way for spot turning
            steering_angles['front_left'] = turn_direction * self.max_steering_angle
            steering_angles['front_right'] = turn_direction * self.max_steering_angle
            steering_angles['rear_left'] = -turn_direction * self.max_steering_angle
            steering_angles['rear_right'] = -turn_direction * self.max_steering_angle
            return steering_angles
        
        # Calculate turning radius based on linear and angular velocity
        # R = v / ω
        # If angular_z is very small, use a very large radius (almost straight)
        if abs(angular_z) < 0.001:
            turning_radius = float('inf')
        else:
            turning_radius = abs(linear_x) / abs(angular_z)
        
        # Determine turn direction
        turn_direction = 1.0 if angular_z > 0 else -1.0
        if linear_x < 0:
            turn_direction *= -1  # Reverse the steering when going backward
        
        # Calculate steering angles using Ackerman geometry
        # For the front wheels
        alpha_front = math.atan(self.wheel_base / (turning_radius - (turn_direction * self.track_width / 2)))
        alpha_front_inner = math.atan(self.wheel_base / (turning_radius - (turn_direction * self.track_width)))
        
        # For the rear wheels (in double Ackerman, rear wheels also steer)
        alpha_rear = math.atan(self.wheel_base / (turning_radius + (turn_direction * self.track_width / 2)))
        alpha_rear_inner = math.atan(self.wheel_base / (turning_radius + (turn_direction * self.track_width)))
        
        # Assign angles based on turn direction
        if turn_direction > 0:  # Turning left
            steering_angles['front_left'] = alpha_front_inner
            steering_angles['front_right'] = alpha_front
            steering_angles['rear_left'] = -alpha_rear_inner
            steering_angles['rear_right'] = -alpha_rear
        else:  # Turning right
            steering_angles['front_left'] = -alpha_front
            steering_angles['front_right'] = -alpha_front_inner
            steering_angles['rear_left'] = alpha_rear
            steering_angles['rear_right'] = alpha_rear_inner
        
        # Clamp steering angles to maximum
        for wheel in steering_angles:
            steering_angles[wheel] = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angles[wheel]))
        
        return steering_angles
    
    def calculate_wheel_velocities(self, linear_x, angular_z, steering_angles):
        """
        Calculate the required wheel velocities based on desired motion
        
        Args:
            linear_x: Linear velocity in x direction
            angular_z: Angular velocity around z axis
            steering_angles: Dictionary with steering angles for each wheel
            
        Returns:
            Dictionary with wheel velocities in rad/s
        """
        wheel_velocities = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }
        
        # If not moving, return zero velocities
        if abs(linear_x) < 0.001 and abs(angular_z) < 0.001:
            return wheel_velocities
        
        # Base velocity from linear component (v/r)
        base_wheel_velocity = linear_x / self.wheel_radius
        
        # Calculate instantaneous center of rotation (ICR)
        if abs(angular_z) < 0.001:
            # Moving straight
            for wheel in wheel_velocities:
                wheel_velocities[wheel] = base_wheel_velocity
        else:
            # Turning - adjust each wheel's velocity based on its distance from the ICR
            # Ensure turning_radius is never zero to avoid division by zero
            if abs(angular_z) < 0.001:
                turning_radius = float('inf')  # Effectively moving straight
            else:
                turning_radius = abs(linear_x) / abs(angular_z)
                
            # Add a safety check to prevent division by zero
            if turning_radius < 0.001:  # If radius is too small (almost rotating in place)
                turning_radius = 0.001  # Set a minimum turning radius
                
            turn_direction = 1.0 if angular_z > 0 else -1.0
            
            # Calculate the distance from each wheel to the ICR
            # This is a simplified model - for a more accurate model, the actual distance 
            # would depend on the steering angle of each wheel
            wheel_velocities['front_left'] = base_wheel_velocity * (1.0 + turn_direction * self.track_width / (2.0 * turning_radius))
            wheel_velocities['front_right'] = base_wheel_velocity * (1.0 - turn_direction * self.track_width / (2.0 * turning_radius))
            wheel_velocities['rear_left'] = base_wheel_velocity * (1.0 + turn_direction * self.track_width / (2.0 * turning_radius))
            wheel_velocities['rear_right'] = base_wheel_velocity * (1.0 - turn_direction * self.track_width / (2.0 * turning_radius))
        
        # Adjust for steering angles - wheels that are steered more should move slower
        for wheel in wheel_velocities:
            angle = steering_angles[wheel]
            # Apply cosine correction factor - a steered wheel needs to rotate faster
            # to maintain the same forward velocity
            if abs(angle) > 0.001:
                wheel_velocities[wheel] = wheel_velocities[wheel] / math.cos(angle)
        
        # Clamp wheel velocities to maximum
        for wheel in wheel_velocities:
            wheel_velocities[wheel] = max(-self.max_wheel_speed, min(self.max_wheel_speed, wheel_velocities[wheel]))
        
        return wheel_velocities
    
    def publish_steering_angles(self, steering_angles):
        """
        Publish calculated steering angles to their respective topics
        
        Args:
            steering_angles: Dictionary with steering angles for each wheel
        """
        fl_msg, fr_msg, rl_msg, rr_msg = Float64(), Float64(), Float64(), Float64()
        
        fl_msg.data = float(steering_angles['front_left'])
        fr_msg.data = float(steering_angles['front_right'])
        rl_msg.data = float(steering_angles['rear_left'])
        rr_msg.data = float(steering_angles['rear_right'])
        
        self.fl_steer_pub.publish(fl_msg)
        self.fr_steer_pub.publish(fr_msg)
        self.rl_steer_pub.publish(rl_msg)
        self.rr_steer_pub.publish(rr_msg)
        
    def publish_wheel_velocities(self, wheel_velocities):
        """
        Publish calculated wheel velocities to their respective topics
        
        Args:
            wheel_velocities: Dictionary with wheel velocities for each wheel
        """
        fl_msg, fr_msg, rl_msg, rr_msg = Float64(), Float64(), Float64(), Float64()
        
        fl_msg.data = float(wheel_velocities['front_left'])
        fr_msg.data = float(wheel_velocities['front_right'])
        rl_msg.data = float(wheel_velocities['rear_left'])
        rr_msg.data = float(wheel_velocities['rear_right'])
        
        self.fl_vel_pub.publish(fl_msg)
        self.fr_vel_pub.publish(fr_msg)
        self.rl_vel_pub.publish(rl_msg)
        self.rr_vel_pub.publish(rr_msg)
    
    def set_zero_velocity_and_steering(self):
        """
        Set all wheel velocities and steering angles to zero
        """
        zero_msg = Float64()
        zero_msg.data = 0.0
        
        # Publish zero steering
        self.fl_steer_pub.publish(zero_msg)
        self.fr_steer_pub.publish(zero_msg)
        self.rl_steer_pub.publish(zero_msg)
        self.rr_steer_pub.publish(zero_msg)
        
        # Publish zero velocity
        self.fl_vel_pub.publish(zero_msg)
        self.fr_vel_pub.publish(zero_msg)
        self.rl_vel_pub.publish(zero_msg)
        self.rr_vel_pub.publish(zero_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DoubleAckermanController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Zero out all commands before shutting down
        node.set_zero_velocity_and_steering()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
