#!/usr/bin/env python3
"""
this is the DWA Planner ROS2 Node.
This node integrates the DWA algorithm with ROS2 for TurtleBot navigation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math

from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import geometry_msgs.msg

from custom_dwa_planner.dwa_algorithm import DWAPlanner, DWAConfig


class DWAPlannerNode(Node):
    """ROS2 Node for DWA local planner"""
    
    def __init__(self):
        """Initialize the DWA planner node."""
        super().__init__('dwa_planner_node')
        
        # Declare parameters
        self.declare_parameter('max_speed', 0.22)
        self.declare_parameter('max_yaw_rate', 40.0)
        self.declare_parameter('robot_radius', 0.2)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 5.0)
        self.declare_parameter('goal_tolerance', 0.3)
        
        # Initialize DWA configuration
        self.config = DWAConfig()
        self.config.max_speed = self.get_parameter('max_speed').value
        self.config.max_yaw_rate = self.get_parameter('max_yaw_rate').value * math.pi / 180.0
        self.config.robot_radius = self.get_parameter('robot_radius').value
        
        # Initialize DWA planner
        self.dwa_planner = DWAPlanner(self.config)
        
        # Robot state [x, y, yaw, v, omega]
        self.robot_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Goal position
        self.goal = np.array([
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value
        ])
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.goal_reached = False
        
        # Obstacles from laser scan
        self.obstacles = np.array([])
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        
        self.scan_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback,sensor_qos)
        
        self.goal_sub = self.create_subscription(PoseStamped,'/goal_pose',self.goal_callback,10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.trajectory_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Flags
        self.odom_received = False
        self.scan_received = False
        
        self.get_logger().info('DWA Planner Node initialized')
        self.get_logger().info(f'Goal set to: ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
        
        # Publish goal marker
        self.publish_goal_marker()
    
    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry messages.
        
        Args:
            msg: Odometry message
        """
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract orientation (convert quaternion to yaw)
        orientation_q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(orientation_q)
        
        # Extract velocities
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        # Update robot state
        self.robot_state = np.array([x, y, yaw, v, omega])
        
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('Odometry data received')
    
    def scan_callback(self, msg: LaserScan):
        """
        Callback for laser scan messages.
        
        Args:
            msg: LaserScan message
        """
        # Convert laser scan to obstacle points in robot frame
        obstacles = []
        
        for i, range_val in enumerate(msg.ranges):
            # Skip invalid readings
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
            
            # Calculate angle
            angle = msg.angle_min + i * msg.angle_increment
            
            # Convert to Cartesian coordinates in robot frame
            x_robot = range_val * math.cos(angle)
            y_robot = range_val * math.sin(angle)
            
            # Transform to global frame
            x_global = self.robot_state[0] + x_robot * math.cos(self.robot_state[2]) - y_robot * math.sin(self.robot_state[2])
            y_global = self.robot_state[1] + x_robot * math.sin(self.robot_state[2]) + y_robot * math.cos(self.robot_state[2])
            
            obstacles.append([x_global, y_global])
        
        self.obstacles = np.array(obstacles)
        
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info(f'Laser scan data received ({len(obstacles)} points)')
    
    def goal_callback(self, msg: PoseStamped):
        """
        Callback for goal pose messages.
        
        Args:
            msg: PoseStamped message
        """
        self.goal = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ])
        self.goal_reached = False
        self.get_logger().info(f'New goal received: ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
        self.publish_goal_marker()
    
    def control_loop(self):
        """Main control loop for DWA planner."""
        # Publish goal marker periodically
        self.publish_goal_marker()
        
        # Check if we have received necessary data
        if not self.odom_received or not self.scan_received:
            return
        
        # Check if goal is reached
        if self.goal_reached:
            return
        
        distance_to_goal = math.sqrt(
            (self.goal[0] - self.robot_state[0])**2 +
            (self.goal[1] - self.robot_state[1])**2
        )
        
        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info('Goal reached!')
            self.goal_reached = True
            self.publish_zero_velocity()
            return
        
        # Run DWA planner
        try:
            best_u, best_trajectory = self.dwa_planner.plan(
                self.robot_state,
                self.goal,
                self.obstacles
            )
            
            # Publish velocity command
            self.publish_velocity(best_u)
            
            # Visualize trajectory
            self.publish_trajectory(best_trajectory)
            
            # Log info (changed from debug to info for visibility)
            self.get_logger().info(
                f'State: ({self.robot_state[0]:.2f}, {self.robot_state[1]:.2f}, {self.robot_state[2]:.2f}) | '
                f'Cmd: v={best_u[0]:.3f}, ω={best_u[1]:.3f} | '
                f'Goal dist: {distance_to_goal:.2f}m | Obstacles: {len(self.obstacles)}'
            )
            
        except Exception as e:
            self.get_logger().error(f'DWA planning failed: {str(e)}')
            self.publish_zero_velocity()
    
    def publish_velocity(self, u: np.ndarray):
        """
        Publish velocity command.
        
        Args:
            u: Velocity command [v, omega]
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = float(u[0])
        cmd_vel.angular.z = float(u[1])
        self.cmd_vel_pub.publish(cmd_vel)
    
    def publish_zero_velocity(self):
        """Publish zero velocity to stop the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
    
    def publish_trajectory(self, trajectory: np.ndarray):
        """
        Publish trajectory for visualization in RViz.
        
        Args:
            trajectory: Predicted trajectory
        """
        marker_array = MarkerArray()
        
        # Create line strip marker for trajectory
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'dwa_trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        
        for state in trajectory:
            point = geometry_msgs.msg.Point()
            point.x = float(state[0])
            point.y = float(state[1])
            point.z = 0.0
            marker.points.append(point)
        
        marker_array.markers.append(marker)
        
        # Create sphere markers for trajectory points
        for i, state in enumerate(trajectory[::5]):  # Sample every 5th point
            sphere_marker = Marker()
            sphere_marker.header.frame_id = 'odom'
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = 'trajectory_points'
            sphere_marker.id = i + 1
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = float(state[0])
            sphere_marker.pose.position.y = float(state[1])
            sphere_marker.pose.position.z = 0.0
            sphere_marker.scale.x = 0.05
            sphere_marker.scale.y = 0.05
            sphere_marker.scale.z = 0.05
            sphere_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
            marker_array.markers.append(sphere_marker)
        
        self.trajectory_pub.publish(marker_array)
    
    def publish_goal_marker(self):
        """Publish goal marker for visualization in RViz."""
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.goal[0])
        marker.pose.position.y = float(self.goal[1])
        marker.pose.position.z = 0.05  # Raise slightly above ground
        marker.pose.orientation.w = 1.0  # Add orientation
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.1
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        marker.lifetime.sec = 0  # Persistent marker
        
        self.goal_marker_pub.publish(marker)
    
    @staticmethod
    def quaternion_to_yaw(q):
        """
        Convert quaternion to yaw angle.
        
        Args:
            q: Quaternion
            
        Returns:
            Yaw angle in radians
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = DWAPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
