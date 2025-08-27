#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math
import time
import threading


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Action clients for trajectory control
        self.arm_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.gripper_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/gripper_controller/follow_joint_trajectory'
        )
        
        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Joint names (updated without wrist_joint)
        self.arm_joint_names = [
            'shoulder_joint',
            'upper_arm_joint', 
            'elbow_joint',
            'forearm_joint'
        ]
        
        self.gripper_joint_names = [
            'gripper_controller'
        ]
        
        # Current joint positions
        self.current_arm_positions = [0.0] * len(self.arm_joint_names)
        self.current_gripper_position = [0.0] * len(self.gripper_joint_names)
        
        self.get_logger().info('Arm Controller initialized. Waiting for action servers...')
        
        # Wait for action servers
        self.arm_action_client.wait_for_server()
        self.gripper_action_client.wait_for_server()
        self.get_logger().info('Action servers connected!')
        
        # Start demo movements
        self.demo_timer = self.create_timer(15.0, self.demo_movements)
        
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        # Update arm joint positions
        for i, name in enumerate(self.arm_joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_arm_positions[i] = msg.position[idx]
        
        # Update gripper position
        for i, name in enumerate(self.gripper_joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_gripper_position[i] = msg.position[idx]
    
    def send_arm_trajectory(self, positions, duration=3.0):
        """Send trajectory to arm controller"""
        if len(positions) != len(self.arm_joint_names):
            self.get_logger().error(f'Expected {len(self.arm_joint_names)} positions, got {len(positions)}')
            return None
            
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names
        
        # Start point (current position)
        start_point = JointTrajectoryPoint()
        start_point.positions = self.current_arm_positions.copy()
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
        self.get_logger().info(f'Sending arm trajectory: {positions}')
        future = self.arm_action_client.send_goal_async(goal_msg)
        
        return future
    
    def send_gripper_trajectory(self, position, duration=2.0):
        """Send trajectory to gripper controller"""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joint_names
        
        # Start point (current position)
        start_point = JointTrajectoryPoint()
        start_point.positions = self.current_gripper_position.copy()
        start_point.time_from_start.sec = 0
        start_point.time_from_start.nanosec = 0
        
        # End point (target position)
        end_point = JointTrajectoryPoint()
        end_point.positions = [position]
        end_point.time_from_start.sec = int(duration)
        end_point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        trajectory.points = [start_point, end_point]
        goal_msg.trajectory = trajectory
        
        # Send goal
        self.get_logger().info(f'Sending gripper trajectory: {position}')
        future = self.gripper_action_client.send_goal_async(goal_msg)
        
        return future
    
    def move_to_home(self):
        """Move arm to home position"""
        home_positions = [0.0, 0.0, 0.0, 0.0]  # 4 joints only
        return self.send_arm_trajectory(home_positions, 2.0)
    
    def move_to_pose1(self):
        """Move arm to predefined pose 1"""
        pose1_positions = [1.57, 0.5, -1.0, 0.5]  # 90° shoulder, raised arm
        return self.send_arm_trajectory(pose1_positions, 3.0)
    
    def move_to_pose2(self):
        """Move arm to predefined pose 2"""
        pose2_positions = [-1.57, -0.5, 1.5, -0.8]  # Complex pose
        return self.send_arm_trajectory(pose2_positions, 3.0)
    
    def move_to_pose3(self):
        """Move arm to predefined pose 3"""
        pose3_positions = [0.0, 1.2, -2.0, 1.0]  # Extended arm
        return self.send_arm_trajectory(pose3_positions, 3.0)
    
    def open_gripper(self):
        """Open gripper"""
        return self.send_gripper_trajectory(0.1, 1.5)  # Open position
    
    def close_gripper(self):
        """Close gripper"""
        return self.send_gripper_trajectory(-0.5, 1.5)  # Close position
    
    def partially_close_gripper(self):
        """Partially close gripper"""
        return self.send_gripper_trajectory(-0.2, 1.5)  # Partial close
    
    def wait_for_completion(self, future, timeout=5.0):
        """Wait for action to complete"""
        if future is None:
            return False
            
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().warning('Action timed out')
                return False
            time.sleep(0.1)
        return True
    
    def demo_movements(self):
        """Perform demo movements sequence"""
        self.get_logger().info('Starting demo movements...')
        
        # Sequence of movements with gripper actions
        movements = [
            # Movement 1: Home position with open gripper
            ('move_to_home', self.move_to_home),
            ('open_gripper', self.open_gripper),
            
            # Movement 2: Pose 1 with gripper partially closed
            ('move_to_pose1', self.move_to_pose1),
            ('partially_close_gripper', self.partially_close_gripper),
            
            # Movement 3: Pose 2 with gripper closed
            ('move_to_pose2', self.move_to_pose2),
            ('close_gripper', self.close_gripper),
            
            # Movement 4: Pose 3 with gripper open
            ('move_to_pose3', self.move_to_pose3),
            ('open_gripper', self.open_gripper),
            
            # Movement 5: Back to home
            ('move_to_home', self.move_to_home),
            ('close_gripper', self.close_gripper)
        ]
        
        # Execute movements with proper waiting
        for i, (name, movement_func) in enumerate(movements):
            self.get_logger().info(f'Executing {name} ({i+1}/{len(movements)})')
            future = movement_func()
            
            # Wait for completion
            if self.wait_for_completion(future, 6.0):
                self.get_logger().info(f'{name} completed successfully')
            else:
                self.get_logger().warning(f'{name} may not have completed')
            
            # Small delay between movements
            time.sleep(1.0)
        
        self.get_logger().info('Demo sequence completed!')
    
    def manual_control(self):
        """Manual control interface"""
        self.get_logger().info('Manual control available. Use service calls or create a separate interface.')


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