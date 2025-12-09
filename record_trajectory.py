#!/usr/bin/env python3
"""
ROS2 Trajectory Recorder for OpenVINS
======================================

This script records trajectory data from OpenVINS ROS2 topics
and saves it in TUM format for evaluation.

Usage:
    ros2 run python3 record_trajectory.py --duration 60 --output trajectory.txt
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import argparse
import signal
import sys
import os


class TrajectoryRecorder(Node):
    """
    ROS2 node to record trajectory data from OpenVINS.
    """
    
    def __init__(self, output_file: str, topic: str = '/ov_msckf/odomimu'):
        super().__init__('openvins_trajectory_recorder')
        
        self.output_file = output_file
        self.poses = []
        self.recording = True
        
        # Determine topic type and create subscription
        if 'odom' in topic.lower():
            self.subscription = self.create_subscription(
                Odometry, topic, self.odom_callback, 10)
            self.get_logger().info(f'Subscribing to Odometry topic: {topic}')
        elif 'path' in topic.lower():
            self.subscription = self.create_subscription(
                Path, topic, self.path_callback, 10)
            self.get_logger().info(f'Subscribing to Path topic: {topic}')
        elif 'pose' in topic.lower():
            # Try PoseWithCovarianceStamped first, fallback to PoseStamped
            try:
                self.subscription = self.create_subscription(
                    PoseWithCovarianceStamped, topic, self.pose_cov_callback, 10)
            except:
                self.subscription = self.create_subscription(
                    PoseStamped, topic, self.pose_callback, 10)
            self.get_logger().info(f'Subscribing to Pose topic: {topic}')
        else:
            # Default to Odometry
            self.subscription = self.create_subscription(
                Odometry, topic, self.odom_callback, 10)
            self.get_logger().info(f'Subscribing to topic as Odometry: {topic}')
        
        # Status timer
        self.timer = self.create_timer(5.0, self.print_status)
        
    def odom_callback(self, msg: Odometry):
        """Handle Odometry messages."""
        if not self.recording:
            return
            
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        
        self.poses.append([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        
    def path_callback(self, msg: Path):
        """Handle Path messages (takes last pose from path)."""
        if not self.recording or len(msg.poses) == 0:
            return
            
        pose = msg.poses[-1]
        t = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
        p = pose.pose.position
        q = pose.pose.orientation
        
        # Only add if timestamp is newer
        if len(self.poses) == 0 or t > self.poses[-1][0]:
            self.poses.append([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            
    def pose_callback(self, msg: PoseStamped):
        """Handle PoseStamped messages."""
        if not self.recording:
            return
            
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation
        
        self.poses.append([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        
    def pose_cov_callback(self, msg: PoseWithCovarianceStamped):
        """Handle PoseWithCovarianceStamped messages."""
        if not self.recording:
            return
            
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        
        self.poses.append([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        
    def print_status(self):
        """Print recording status."""
        self.get_logger().info(f'Recorded {len(self.poses)} poses')
        
    def save(self):
        """Save recorded trajectory to file."""
        self.recording = False
        
        if len(self.poses) == 0:
            self.get_logger().warn('No poses recorded!')
            return False
            
        # Convert to numpy array
        data = np.array(self.poses)
        
        # Sort by timestamp and remove duplicates
        data = data[data[:, 0].argsort()]
        _, unique_idx = np.unique(data[:, 0], return_index=True)
        data = data[unique_idx]
        
        # Create output directory if needed
        os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
        
        # Save in TUM format
        np.savetxt(self.output_file, data,
                  fmt='%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f',
                  header='timestamp tx ty tz qx qy qz qw')
        
        self.get_logger().info(f'Saved {len(data)} poses to {self.output_file}')
        
        # Print trajectory summary
        duration = data[-1, 0] - data[0, 0]
        distances = np.linalg.norm(np.diff(data[:, 1:4], axis=0), axis=1)
        total_distance = np.sum(distances)
        
        self.get_logger().info(f'Trajectory Summary:')
        self.get_logger().info(f'  Duration: {duration:.2f} s')
        self.get_logger().info(f'  Total distance: {total_distance:.2f} m')
        self.get_logger().info(f'  Average rate: {len(data)/duration:.1f} Hz')
        
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Record OpenVINS trajectory from ROS2 topics'
    )
    parser.add_argument('--output', '-o', type=str, default='trajectory.txt',
                       help='Output file path (TUM format)')
    parser.add_argument('--topic', '-t', type=str, default='/ov_msckf/odomimu',
                       help='ROS2 topic to subscribe to')
    parser.add_argument('--duration', '-d', type=float, default=0,
                       help='Recording duration in seconds (0 = until Ctrl+C)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    recorder = TrajectoryRecorder(args.output, args.topic)
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print('\nStopping recording...')
        recorder.save()
        recorder.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    print(f'Recording trajectory from {args.topic}')
    print('Press Ctrl+C to stop and save...')
    
    if args.duration > 0:
        import time
        start = time.time()
        while time.time() - start < args.duration:
            rclpy.spin_once(recorder, timeout_sec=0.1)
        recorder.save()
    else:
        rclpy.spin(recorder)
        
    recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

