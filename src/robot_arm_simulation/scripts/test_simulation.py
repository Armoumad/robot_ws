#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time
import math


class SimulationTester(Node):
    def __init__(self):
        super().__init__('simulation_tester')
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Test parameters
        self.joint_states_received = False
        self.tf_received = False
        self.test_start_time = time.time()
        self.test_duration = 30.0  # 30 secondes de test
        
        # Timer for periodic tests
        self.test_timer = self.create_timer(1.0, self.run_tests)
        
        # Joint names to check
        self.expected_joints = [
            'shoulder_joint',
            'upper_arm_joint', 
            'elbow_joint',
            'forearm_joint',
            'wrist_joint'
        ]
        
        # Expected frames
        self.expected_frames = [
            'world',
            'base_link',
            'shoulder_link',
            'upper_arm_link',
            'elbow_link',
            'forearm_link',
            'wrist_link'
        ]
        
        self.get_logger().info('🧪 Simulation Tester started')
        self.get_logger().info(f'Testing will run for {self.test_duration} seconds')
        
    def joint_state_callback(self, msg):
        """Check joint states reception"""
        if not self.joint_states_received:
            self.get_logger().info('✅ Joint states received')
            
            # Check if all expected joints are present
            missing_joints = []
            for joint in self.expected_joints:
                if joint not in msg.name:
                    missing_joints.append(joint)
            
            if missing_joints:
                self.get_logger().warn(f'❌ Missing joints: {missing_joints}')
            else:
                self.get_logger().info('✅ All expected joints found')
                
            # Check joint limits
            self.check_joint_limits(msg)
            
            self.joint_states_received = True
    
    def check_joint_limits(self, msg):
        """Check if joint positions are within expected limits"""
        joint_limits = {
            'shoulder_joint': (-math.pi, math.pi),
            'upper_arm_joint': (-math.pi/2, math.pi/2),
            'elbow_joint': (-math.pi, math.pi),
            'forearm_joint': (-math.pi/2, math.pi/2),
            'wrist_joint': (-math.pi, math.pi)
        }
        
        for i, joint_name in enumerate(msg.name):
            if joint_name in joint_limits and i < len(msg.position):
                min_limit, max_limit = joint_limits[joint_name]
                position = msg.position[i]
                
                if position < min_limit or position > max_limit:
                    self.get_logger().warn(
                        f'⚠️ Joint {joint_name} position {position:.3f} '
                        f'outside limits [{min_limit:.3f}, {max_limit:.3f}]'
                    )
    
    def check_tf_frames(self):
        """Check TF frames availability"""
        try:
            available_frames = self.tf_buffer.all_frames_as_string().split('\n')
            available_frame_names = []
            
            for frame_info in available_frames:
                if frame_info.strip():
                    frame_name = frame_info.split()[0]
                    available_frame_names.append(frame_name)
            
            missing_frames = []
            for frame in self.expected_frames:
                if frame not in available_frame_names:
                    missing_frames.append(frame)
            
            if missing_frames:
                self.get_logger().warn(f'❌ Missing TF frames: {missing_frames}')
                return False
            else:
                if not self.tf_received:
                    self.get_logger().info('✅ All expected TF frames found')
                    self.tf_received = True
                return True
                
        except Exception as e:
            self.get_logger().error(f'❌ Error checking TF frames: {e}')
            return False
    
    def test_end_effector_transform(self):
        """Test end effector transform"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'world', 
                'wrist_link',
                rclpy.time.Time()
            )
            
            # Check if transform is reasonable
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            distance = math.sqrt(x*x + y*y + z*z)
            
            if distance > 2.0:  # More than 2 meters seems unreasonable
                self.get_logger().warn(
                    f'⚠️ End effector very far from base: {distance:.3f}m'
                )
            elif distance < 0.1:  # Less than 10cm seems too close
                self.get_logger().warn(
                    f'⚠️ End effector very close to base: {distance:.3f}m'
                )
            else:
                self.get_logger().info(
                    f'✅ End effector position reasonable: {distance:.3f}m from base'
                )
                
            return True
            
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return False
    
    def run_tests(self):
        """Run periodic tests"""
        current_time = time.time()
        elapsed_time = current_time - self.test_start_time
        
        # Check if test period is over
        if elapsed_time > self.test_duration:
            self.generate_report()
            self.get_logger().info('🏁 Testing completed')
            rclpy.shutdown()
            return
        
        # Run TF tests
        if self.check_tf_frames():
            self.test_end_effector_transform()
        
        # Progress indicator
        progress = (elapsed_time / self.test_duration) * 100
        if int(elapsed_time) % 5 == 0:  # Every 5 seconds
            self.get_logger().info(f'📊 Test progress: {progress:.1f}%')
    
    def generate_report(self):
        """Generate final test report"""
        self.get_logger().info('📋 SIMULATION TEST REPORT')
        self.get_logger().info('=' * 40)
        
        # Joint states test
        if self.joint_states_received:
            self.get_logger().info('✅ Joint states: PASS')
        else:
            self.get_logger().error('❌ Joint states: FAIL')
        
        # TF test
        if self.tf_received:
            self.get_logger().info('✅ TF frames: PASS')
        else:
            self.get_logger().error('❌ TF frames: FAIL')
        
        # Overall result
        if self.joint_states_received and self.tf_received:
            self.get_logger().info('🎉 OVERALL RESULT: SIMULATION WORKING CORRECTLY')
        else:
            self.get_logger().error('💥 OVERALL RESULT: SIMULATION HAS ISSUES')
        
        self.get_logger().info('=' * 40)


def main(args=None):
    rclpy.init(args=args)
    
    tester = SimulationTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        tester.get_logger().error(f'Test error: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()