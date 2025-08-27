#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, request, jsonify
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import threading

app = Flask(__name__)

class WebBridgeController(Node):
    def __init__(self):
        super().__init__('web_bridge_controller')
        
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.joint_names = [
            'shoulder_joint',
            'upper_arm_joint', 
            'elbow_joint',
            'forearm_joint',
            'wrist_joint'
        ]
        
        self.positions = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0],
            'pose1': [1.57, 0.5, -1.0, 0.5, 0.0],
            'pose2': [-1.57, -0.5, 1.5, -0.8, 1.57],
            'pose3': [0.0, 1.2, -2.0, 1.0, -1.57]
        }
        
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Connected to action server!')

    def send_trajectory(self, positions, duration=3.0):
        try:
            goal_msg = FollowJointTrajectory.Goal()
            
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names
            
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration % 1) * 1e9)
            
            trajectory.points = [point]
            goal_msg.trajectory = trajectory
            
            self.get_logger().info(f'Sending trajectory: {positions}')
            future = self.action_client.send_goal_async(goal_msg)
            return True
        except Exception as e:
            self.get_logger().error(f'Error sending trajectory: {str(e)}')
            return False

controller = None

@app.route('/control')
def control():
    try:
        cmd = request.args.get('cmd', '')
        if not cmd or cmd not in controller.positions:
            return jsonify({'error': 'Invalid command'}), 400
        
        positions = controller.positions[cmd]
        success = controller.send_trajectory(positions)
        
        if success:
            return jsonify({'message': f'Executing {cmd}'}), 200
        else:
            return jsonify({'error': 'Failed to execute command'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500

def run_flask():
    # Écouter sur toutes les interfaces
    app.run(host='0.0.0.0', port=5000)

def main(args=None):
    global controller
    
    rclpy.init(args=args)
    controller = WebBridgeController()
    
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()