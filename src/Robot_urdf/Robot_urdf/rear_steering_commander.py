#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState 

class RearSteeringCommander(Node):
    def __init__(self):
        super().__init__('rear_steering_commander')
        # publishers for rear steering and velocity
        self.rl_steer_pub = self.create_publisher(Float64, '/fwids/rear_left_steer_rad/command', 1)
        self.rr_steer_pub = self.create_publisher(Float64, '/fwids/rear_right_steer_rad/command', 1)
        self.joint_states_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 1)
    def joint_states_callback(self, msg: JointState):
        # Extract rear steering angles from joint states
        try:
            rl_index = msg.name.index('steer_front_left_link_j')
            rr_index = msg.name.index('steer_front_right_link_j')
            rl_steer_angle = - msg.position[rl_index]
            rr_steer_angle = - msg.position[rr_index]
            # Publish the steering angles
            rl_msg = Float64()
            rr_msg = Float64()
            rl_msg.data = rl_steer_angle
            rr_msg.data = rr_steer_angle
            self.rl_steer_pub.publish(rl_msg)
            self.rr_steer_pub.publish(rr_msg)
        except ValueError:
            self.get_logger().warn('Joint names not found in joint_states message')

def main(args=None):
    rclpy.init(args=args)
    rear_steering_commander = RearSteeringCommander()
    rclpy.spin(rear_steering_commander)
    rear_steering_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()