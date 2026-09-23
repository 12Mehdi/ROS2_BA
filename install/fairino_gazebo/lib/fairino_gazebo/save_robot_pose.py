#!/usr/bin/env python3
"""
Script to query the current robot tool pose and save it to a YAML config file.
Can be used to save poses for later playback with MoveIt.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
import yaml
import argparse
from datetime import datetime


class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Pose Saver Node initialized')
    
    def get_current_pose(self, reference_frame='world', target_frame='tool0'):
        """
        Get the current pose of the target frame relative to reference frame.
        
        Args:
            reference_frame: The reference frame (e.g., 'world', 'base_link')
            target_frame: The target frame (e.g., 'tool0', 'wrist_3_link')
        
        Returns:
            dict: Pose as dictionary with position and orientation
        """
        try:
            # Wait for transform to be available
            self.get_logger().info(f'Waiting for transform from {reference_frame} to {target_frame}...')
            
            # Lookup transform with timeout
            transform = self.tf_buffer.lookup_transform(
                reference_frame,
                target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=10.0)
            )
            
            # Extract pose data
            pose = {
                'position': {
                    'x': float(transform.transform.translation.x),
                    'y': float(transform.transform.translation.y),
                    'z': float(transform.transform.translation.z)
                },
                'orientation': {
                    'x': float(transform.transform.rotation.x),
                    'y': float(transform.transform.rotation.y),
                    'z': float(transform.transform.rotation.z),
                    'w': float(transform.transform.rotation.w)
                }
            }
            
            self.get_logger().info(f'Current pose retrieved successfully!')
            self.get_logger().info(f'Position: x={pose["position"]["x"]:.3f}, '
                                 f'y={pose["position"]["y"]:.3f}, '
                                 f'z={pose["position"]["z"]:.3f}')
            
            return pose
            
        except Exception as e:
            self.get_logger().error(f'Failed to get transform: {str(e)}')
            return None
    
    def save_pose_to_yaml(self, pose, filename, pose_name=None):
        """
        Save pose to YAML file in MoveIt-compatible format.
        
        Args:
            pose: Pose dictionary
            filename: Path to YAML file
            pose_name: Name for this pose (default: timestamped)
        """
        if pose is None:
            self.get_logger().error('Cannot save None pose')
            return False
        
        # Generate pose name if not provided
        if pose_name is None:
            pose_name = f'pose_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        
        # Try to load existing file
        try:
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)
                if data is None:
                    data = {}
        except FileNotFoundError:
            data = {}
        
        # Initialize saved_poses if not exists
        if 'saved_poses' not in data:
            data['saved_poses'] = {}
        
        # Add new pose
        data['saved_poses'][pose_name] = pose
        
        # Save to file
        try:
            with open(filename, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            
            self.get_logger().info(f'Pose "{pose_name}" saved to {filename}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to save YAML file: {str(e)}')
            return False


def main():
    parser = argparse.ArgumentParser(description='Save current robot tool pose to YAML file')
    parser.add_argument('--output', '-o', type=str, 
                       default='saved_poses.yaml',
                       help='Output YAML file path')
    parser.add_argument('--name', '-n', type=str, default=None,
                       help='Name for the saved pose')
    parser.add_argument('--reference-frame', '-r', type=str, default='base_link',
                       help='Reference frame (default: world)')
    parser.add_argument('--target-frame', '-t', type=str, default='tool0',
                       help='Target frame (default: tool0)')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    # Create node
    pose_saver = PoseSaver()
    
    # Spin briefly to let TF buffer fill
    import time
    for _ in range(10):
        rclpy.spin_once(pose_saver, timeout_sec=0.1)
        time.sleep(0.1)
    
    # Get current pose
    pose = pose_saver.get_current_pose(
        reference_frame=args.reference_frame,
        target_frame=args.target_frame
    )
    
    # Save to file
    if pose:
        pose_saver.save_pose_to_yaml(pose, args.output, args.name)
        print(f'\n✓ Pose saved successfully to: {args.output}')
    else:
        print('\n✗ Failed to retrieve pose')
        rclpy.shutdown()
        return 1
    
    # Cleanup
    pose_saver.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    exit(main())
