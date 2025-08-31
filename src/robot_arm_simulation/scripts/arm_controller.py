#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # ---- Action servers ----
        self.arm_action_ns = '/arm_controller/follow_joint_trajectory'
        # Use the renamed controller for the gripper
        self.gripper_action_ns = '/gripper_trajectory_controller/follow_joint_trajectory'

        # Joints du bras
        self.arm_joints = [
            'shoulder_joint',
            'upper_arm_joint',
            'elbow_joint',
            'forearm_joint',
        ]

        # Joint du gripper (nom de joint URDF)
        self.gripper_joint = ['gripper_controller']

        # Positions courantes
        self.current_arm_positions = [0.0] * len(self.arm_joints)
        self.current_gripper_position = 0.0

        # Action clients
        self.arm_client = ActionClient(self, FollowJointTrajectory, self.arm_action_ns)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, self.gripper_action_ns)

        # Subscription joint_states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.get_logger().info("Waiting for action servers...")
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("arm_controller action server not available.")
        else:
            self.get_logger().info("Arm action server connected.")

        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("gripper_trajectory_controller action server not available.")
        else:
            self.get_logger().info("Gripper action server connected.")

        # Lancer la démo une fois
        self.once_timer = self.create_timer(2.0, self.start_demo_once)

    # ---- Callbacks ----
    def joint_state_callback(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for i, jn in enumerate(self.arm_joints):
            if jn in name_to_idx:
                idx = name_to_idx[jn]
                if idx < len(msg.position):
                    self.current_arm_positions[i] = float(msg.position[idx])

        if self.gripper_joint[0] in name_to_idx:
            idx = name_to_idx[self.gripper_joint[0]]
            if idx < len(msg.position):
                self.current_gripper_position = float(msg.position[idx])

    # ---- Utils ----
    def send_trajectory(self, client, joint_names, start_pos, target_pos, duration_sec=3.0) -> bool:
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joint_names

        start = JointTrajectoryPoint()
        start.positions = list(start_pos)
        start.time_from_start = Duration(seconds=0.0).to_msg()

        end = JointTrajectoryPoint()
        end.positions = list(target_pos)
        end.time_from_start = Duration(seconds=float(duration_sec)).to_msg()

        traj.points = [start, end]
        goal.trajectory = traj

        self.get_logger().info(f"Sending trajectory to {client._action_name}: {target_pos}")

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error("No result returned by action server.")
            return False

        return True

    # ---- Movements ----
    def move_arm(self, positions: List[float], duration=3.0):
        return self.send_trajectory(
            self.arm_client, self.arm_joints, self.current_arm_positions, positions, duration
        )

    def move_gripper(self, position: float, duration=2.0):
        return self.send_trajectory(
            self.gripper_client, self.gripper_joint,
            [self.current_gripper_position], [position], duration
        )

    # ---- Poses ----
    def home(self):
        return self.move_arm([0.0, 0.0, 0.0, 0.0], 2.0)

    def pose1(self):
        return self.move_arm([1.57, 0.5, -1.0, 0.5], 3.0)

    def pose2(self):
        return self.move_arm([-1.57, -0.5, 1.5, -0.8], 3.0)

    def pose3(self):
        return self.move_arm([0.0, 1.2, -2.0, 1.0], 3.0)

    def open_gripper(self):
        return self.move_gripper(0.04, 2.0)  # exemple ouverture 4cm

    def close_gripper(self):
        return self.move_gripper(0.0, 2.0)   # fermé

    # ---- Demo ----
    def start_demo_once(self):
        self.once_timer.cancel()
        self.get_logger().info("Starting demo sequence...")

        sequence = [
            self.home,
            self.pose1,
            self.open_gripper,
            self.pose2,
            self.close_gripper,
            self.pose3,
            self.home,
        ]

        for i, step in enumerate(sequence, 1):
            self.get_logger().info(f"Step {i}/{len(sequence)}: {step.__name__}")
            ok = step()
            if not ok:
                self.get_logger().error("Movement failed, stopping sequence.")
                break


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()