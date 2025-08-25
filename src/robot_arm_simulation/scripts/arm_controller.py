#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math
import time


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Action client for trajectory control
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Joint names
        self.joint_names = [
            'shoulder_joint',
            'upper_arm_joint', 
            'elbow_joint',
            'forearm_joint',
            'wrist_joint'
        ]
        
        # Current joint positions
        self.current_positions = [0.0] * len(self.joint_names)
        
        self.get_logger().info('Arm Controller initialized. Waiting for action server...')
        
        # Wait for action server
        self.action_client.wait_for_server()
        self.get_logger().info('Action server connected!')
        
        # Start demo movements
        self.demo_timer = self.create_timer(10.0, self.demo_movements)
        
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
    
    def send_trajectory(self, positions, duration=3.0):
        """Send trajectory to arm controller"""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Start point (current position)
        start_point = JointTrajectoryPoint()
        start_point.positions = self.current_positions.copy()
        start_point.time_from_start.sec = 0
        start_point.time_from_start.nanosec = 0
        
        # End point (target position)
        end_point = JointTrajectoryPoint()
        end_point.positions = positions
        end_point.time_from_start.sec = int(duration)
        end_point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        trajectory.points = [start_point, end_point]
        goal_msg.trajectory = trajectory
        
        # Send goal
        self.get_logger().info(f'Sending trajectory: {positions}')
        future = self.action_client.send_goal_async(goal_msg)
        
        return future
    
    def move_to_home(self):
        """Move arm to home position"""
        home_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        return self.send_trajectory(home_positions, 2.0)
    
    def move_to_pose1(self):
        """Move arm to predefined pose 1"""
        pose1_positions = [1.57, 0.5, -1.0, 0.5, 0.0]  # 90° shoulder, raised arm
        return self.send_trajectory(pose1_positions, 3.0)
    
    def move_to_pose2(self):
        """Move arm to predefined pose 2"""
        pose2_positions = [-1.57, -0.5, 1.5, -0.8, 1.57]  # Complex pose
        return self.send_trajectory(pose2_positions, 3.0)
    
    def move_to_pose3(self):
        """Move arm to predefined pose 3"""
        pose3_positions = [0.0, 1.2, -2.0, 1.0, -1.57]  # Extended arm
        return self.send_trajectory(pose3_positions, 3.0)
    
    def demo_movements(self):
        """Perform demo movements sequence"""
        self.get_logger().info('Starting demo movements...')
        
        # Sequence of movements
        movements = [
            self.move_to_home,
            self.move_to_pose1, 
            self.move_to_pose2,
            self.move_to_pose3,
            self.move_to_home
        ]
        
        # Execute movements with delays
        for i, movement in enumerate(movements):
            self.get_logger().info(f'Executing movement {i+1}/{len(movements)}')
            movement()
            time.sleep(4.0)  # Wait for movement to complete


def main(args=None):
    rclpy.init(args=args)
    
    arm_controller = ArmController()
    
    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()