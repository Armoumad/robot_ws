#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class JointPositionsPublisher(Node):
    def __init__(self):
        super().__init__('joint_positions_publisher')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Joint names
        self.joint_names = [
            'shoulder_joint',
            'upper_arm_joint', 
            'elbow_joint',
            'forearm_joint',
            'wrist_joint'
        ]
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Animation parameters
        self.start_time = time.time()
        
        self.get_logger().info('Joint Positions Publisher started')
    
    def publish_joint_states(self):
        """Publish animated joint states"""
        current_time = time.time() - self.start_time
        
        # Create joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        
        # Animated positions (sinusoidal movement)
        positions = [
            0.5 * math.sin(0.5 * current_time),           # shoulder_joint
            0.3 * math.sin(0.7 * current_time + 1.0),     # upper_arm_joint
            0.8 * math.sin(0.4 * current_time + 2.0),     # elbow_joint
            0.4 * math.sin(0.6 * current_time + 3.0),     # forearm_joint
            0.6 * math.sin(0.8 * current_time + 4.0)      # wrist_joint
        ]
        
        joint_state.position = positions
        joint_state.velocity = [0.0] * len(self.joint_names)
        joint_state.effort = [0.0] * len(self.joint_names)
        
        # Publish
        self.joint_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    
    joint_publisher = JointPositionsPublisher()
    
    try:
        rclpy.spin(joint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()