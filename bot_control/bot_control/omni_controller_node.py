#!/usr/bin/env python3
"""
Omnidirectional Wheel Controller Node for LeKiwi Robot
全向轮控制节点

Functionality:
1. Inverse Kinematics: cmd_vel → wheel velocities
2. Forward Kinematics: wheel velocities → odometry
3. Simulation/Real Robot Detection: Auto-adapt based on environment

Author: LeKiwi Team
Date: 2025-12-12
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import math


class OmniControllerNode(Node):
    """
    Omnidirectional wheel controller for 3-wheel robot with ros2_control
    """
    
    def __init__(self):
        super().__init__('omni_controller_node')
        
        # ===================== Robot Parameters =====================
        # ===================== WHEEL MAPPING (Code ↔ URDF ↔ Hardware) =====================
        # This code defines wheels in order: [wheel1, wheel2, wheel3]
        # 
        # Mapping to URDF joint names and controller.yaml:
        #   wheel1 → omni_wheel_mount-v5-2_to_wheel  (rear wheel, at robot back)
        #   wheel2 → omni_wheel_mount-v5-1_to_wheel  (right front wheel)
        #   wheel3 → omni_wheel_mount-v5_to_wheel    (left front wheel)
        #
        # Physical wheel rolling directions (from ROBOT_SPECS.md measurements):
        #   wheel1 (rear):        θ1 = 90°  (points toward +Y in base_link)
        #   wheel2 (right front): θ2 = 210° (points toward -X-Y in base_link)
        #   wheel3 (left front):  θ3 = 330° (points toward +X-Y in base_link)
        #
        # URDF wheel rotation axes (CORRECTED in lekiwi_bot.xacro):
        #   wheel1 axis: (0, 1.0, 0)          → X-axis rotation in base_link
        #   wheel2 axis: (-0.866025, 0.5, 0)  → rotated 120° from wheel1
        #   wheel3 axis: (-0.866025, -0.5, 0) → rotated 240° from wheel1
        #
        # All axes are now in the mount XY plane (Z=0), forming 120° pattern
        # No axis correction needed - axes directly correspond to physical rotation
        # ===================================================================================
        
        # Wheel radius (m)
        self.R = 0.05
        
        # Distance from robot center to each wheel (m)
        # MEASURED from URDF using analyze_wheel_kinematics.py
        # self.L1 = 0.119202  # wheel1 (rear)
        # self.L2 = 0.100002  # wheel2 (right front)
        # self.L3 = 0.099202  # wheel3 (left front)
        self.L1 = 0.126377  # wheel1 (rear)
        self.L2 = 0.125897  # wheel2 (right front)
        self.L3 = 0.125897  # wheel3 (left front)
        # Wheel rolling direction angles (radians) - CORRECTED based on fdir1
        # Calculated from wheel rotation axis and rpy transformations
        # theta = angle of rolling direction from +X axis (counter-clockwise)
        self.theta1 = np.pi/2           # wheel1: 90.0° (rear) - rolls along +Y
        self.theta2 = np.pi/6           # wheel2: 30.0° (right front) - rolls 30° from +X
        self.theta3 = 5*np.pi/6         # wheel3: 150.0° (left front) - rolls 150° from +X
        
        # Jacobian matrix for inverse kinematics
        # J * [vx, vy, omega_z]^T = [w1, w2, w3]^T
        # FORMULA: w_i = (1/R) * [cos(θ_i)*vx + sin(θ_i)*vy + L_i*ω_z]
        # Recalculated 2025-12-18 with correct theta3=150°
        # self.J = np.array([
        #     [ 0.0000000000,  20.0000000000,  2.38404000],
        #     [ 17.3205060568,  10.0000034969,  2.00004000],
        #     [-17.3205060568,  10.0000034969,  1.98404000]
        # ])
        self.J = np.array([
            [ 0.0,         20.0,        2.52754 ],  
            [ 17.32051,    10.0,        2.51794 ],  
            [-17.32051,    10.0,        2.51794 ]   
        ])
        # Pseudo-inverse for forward kinematics
        self.J_pinv = np.linalg.pinv(self.J)
        
        # ===================== Wheel Axis Correction =====================
        # CORRECTED URDF axes (from lekiwi_bot.xacro):
        #   wheel1: axis=(0, 1.0, 0)          → direct correspondence
        #   wheel2: axis=(-0.866025, 0.5, 0)  → direct correspondence  
        #   wheel3: axis=(-0.866025, -0.5, 0) → direct correspondence
        # 
        # All axes are in mount XY plane (Z=0), no tilt correction needed
        # The axis vectors directly define the rotation direction
        # No correction factors needed - 1:1 mapping between joint and wheel motion
        self.wheel_axis_correction = np.array([
            1.0,   # wheel1 (rear): no correction
            1.0,   # wheel2 (right front): no correction
            1.0    # wheel3 (left front): no correction
        ])
        
        # Debug: Print matrices
        self.get_logger().info('=== Kinematics Matrices ===')
        self.get_logger().info(f'J (inverse kinematics):\n{self.J}')
        self.get_logger().info(f'J_pinv (forward kinematics):\n{self.J_pinv}')
        self.get_logger().info(f'Wheel axis correction: {self.wheel_axis_correction}')
        
        # ===================== Odometry State =====================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # ===================== Model Selection =====================
        # Support both original complex model and simplified model
        self.declare_parameter('use_simple_model', False)
        self.use_simple_model = self.get_parameter('use_simple_model').value
        
        # Wheel joint names (model-dependent)
        if self.use_simple_model:
            self.wheel_joints = [
                'wheel_1_joint',  # wheel1 (rear)
                'wheel_2_joint',  # wheel2 (right front)
                'wheel_3_joint'   # wheel3 (left front)
            ]
        else:
            self.wheel_joints = [
                'omni_wheel_mount-v5-2_to_wheel',  # wheel1 (rear)
                'omni_wheel_mount-v5-1_to_wheel',  # wheel2 (right front)
                'omni_wheel_mount-v5_to_wheel'     # wheel3 (left front)
            ]
        
        # ===================== Environment Detection =====================
        # Check if running in simulation or real robot
        # Note: use_sim_time is automatically declared by ROS2, don't declare again
        try:
            self.is_simulation = self.get_parameter('use_sim_time').value
        except:
            # If use_sim_time not set, assume real robot
            self.is_simulation = False
        
        # ===================== Wheel Speed Compensation =====================
        # Declare wheel correction parameters (adjustable per robot/environment)
        # Default values for Gazebo simulation based on empirical testing
        # For real robot, adjust these values to compensate for hardware differences
        self.declare_parameter('wheel_correction.wheel_1', 1.0)
        self.declare_parameter('wheel_correction.wheel_2', 0.94)  # Gazebo: reduce by 6%
        self.declare_parameter('wheel_correction.wheel_3', 1.0)
        
        # Read correction factors
        self.wheel_correction = np.array([
            self.get_parameter('wheel_correction.wheel_1').value,
            self.get_parameter('wheel_correction.wheel_2').value,
            self.get_parameter('wheel_correction.wheel_3').value
        ])
        
        self.get_logger().info(f'Running in {"SIMULATION" if self.is_simulation else "REAL ROBOT"} mode')
        self.get_logger().info(f'Using {"SIMPLIFIED" if self.use_simple_model else "ORIGINAL"} model')
        self.get_logger().info(f'Wheel correction factors: {self.wheel_correction}')
        
        # ===================== Publishers & Subscribers =====================
        
        # QoS profile for cmd_vel: RELIABLE + TRANSIENT_LOCAL to avoid losing first message
        cmd_vel_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscribe to cmd_vel (from Nav2, keyboard, etc.)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            cmd_vel_qos
        )
        
        # Publish wheel velocity commands to ros2_control
        # Topic name depends on model type
        controller_topic = '/simple_omni_wheel_controller/commands' if self.use_simple_model else '/omni_wheel_controller/commands'
        self.wheel_cmd_pub = self.create_publisher(
            Float64MultiArray,
            controller_topic,
            10
        )
        self.get_logger().info(f'Publishing commands to: {controller_topic}')
        
        # Subscribe to joint states (from ros2_control)
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publish odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            'wheel/odom',  # Renamed to indicate wheel-based odometry (not fused)
            10
        )
        
        # TF broadcaster for odom→base_link
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Declare parameter for publishing TF (can be disabled for external localization)
        # Default False: EKF should publish odom→base_link TF to avoid conflicts
        self.declare_parameter('publish_odom_tf', False)
        
        self.get_logger().info('Omni Controller Node initialized')
        self.get_logger().info(f'Wheel joints: {self.wheel_joints}')
        self.get_logger().info(f'Wheel radius: {self.R} m')
        self.get_logger().info(f'Rear wheel distance: {self.L1} m')
        self.get_logger().info(f'Front wheel distance: {self.L2}, {self.L3} m')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Convert cmd_vel to wheel velocities using inverse kinematics
        
        cmd_vel (Twist) → [w1, w2, w3] (rad/s)
        """
        vx = msg.linear.x
        vy = msg.linear.y
        omega_z = msg.angular.z
        
        # Inverse kinematics: J * [vx, vy, omega_z]^T = [w1, w2, w3]^T
        velocity_vector = np.array([vx, vy, omega_z])
        wheel_speeds_ideal = self.J @ velocity_vector
        
        # Apply inverse axis correction: command must be amplified for tilted wheels
        # and adjusted for axis direction
        # If wheel is tilted 60° (z=0.5), need 2x angular velocity to achieve same ground speed
        # wheel_axis_correction already includes sign (negative for inverted axes)
        wheel_speeds = wheel_speeds_ideal / self.wheel_axis_correction
        
        # CRITICAL FIX: Negate all wheel speeds due to axis direction convention mismatch
        # The joint axis directions in URDF result in opposite rotation vs expected
        # This global negation corrects the 180° direction error observed in testing
        # wheel_speeds = -wheel_speeds
        
        # Apply wheel speed compensation (configurable via ROS2 parameters)
        # Compensates for hardware differences or simulation artifacts
        wheel_speeds = wheel_speeds * self.wheel_correction
        
        # Publish to ros2_control
        wheel_cmd_msg = Float64MultiArray()
        wheel_cmd_msg.data = wheel_speeds.tolist()
        self.wheel_cmd_pub.publish(wheel_cmd_msg)
        
        # Log for debugging
        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(omega_z) > 0.01:
            self.get_logger().info(
                f'new cmd_vel: vx={vx:.3f}, vy={vy:.3f}, omega={omega_z:.3f} → '
                f'ideal=[{wheel_speeds_ideal[0]:.2f}, {wheel_speeds_ideal[1]:.2f}, {wheel_speeds_ideal[2]:.2f}] → '
                f'corrected=[{wheel_speeds[0]:.2f}, {wheel_speeds[1]:.2f}, {wheel_speeds[2]:.2f}]',
                throttle_duration_sec=1.0
            )
    
    def joint_state_callback(self, msg: JointState):
        """
        Compute odometry from wheel velocities using forward kinematics
        
        [w1, w2, w3] (rad/s) → [vx, vy, omega_z] → update pose
        """
        # Extract wheel velocities from joint_states
        # Use model-dependent joint names
        try:
            wheel_speeds_raw = np.array([
                msg.velocity[msg.name.index(self.wheel_joints[0])],  # wheel1 (rear)
                msg.velocity[msg.name.index(self.wheel_joints[1])],  # wheel2 (right front)
                msg.velocity[msg.name.index(self.wheel_joints[2])]   # wheel3 (left front)
            ])
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to extract wheel velocities: {e}', throttle_duration_sec=5.0)
            return
        
        # Apply wheel axis correction for tilted wheels
        # joint_states reports angular velocity around tilted axis
        # Need to project to vertical axis for ground rolling velocity
        # wheel_axis_correction already includes sign (negative for inverted axes)
        wheel_speeds = wheel_speeds_raw * self.wheel_axis_correction
        
        # Forward kinematics: [vx, vy, omega_z] = J_pinv * [w1, w2, w3]
        velocity = self.J_pinv @ wheel_speeds
        vx, vy, omega_z = velocity[0], velocity[1], velocity[2]
        
        # Time update
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt > 0.2:  # Reset on large time gaps (e.g., first message)
            dt = 0.0
        
        # Update pose (Euler integration)
        # Velocity in base_link frame, transform to odom frame
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = omega_z * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish odometry
        self.publish_odometry(current_time, vx, vy, omega_z)
    
    def publish_odometry(self, current_time, vx, vy, omega_z):
        """
        Publish odometry message and TF transform
        """
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity (in base_link frame)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega_z
        
        # Covariance (uncertainty in odometry)
        # Position covariance
        odom.pose.covariance[0] = 0.001   # x (accurate from wheels)
        odom.pose.covariance[7] = 0.001   # y (accurate from wheels)
        odom.pose.covariance[35] = 0.5    # theta (UNRELIABLE! omnidirectional wheel slippage)
        
        # Velocity covariance
        odom.twist.covariance[0] = 0.001  # vx (accurate from wheels)
        odom.twist.covariance[7] = 0.001  # vy (accurate from wheels)
        odom.twist.covariance[35] = 0.5   # omega_z (UNRELIABLE! use IMU instead)
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        
        # Publish TF transform (if enabled)
        if self.get_parameter('publish_odom_tf').value:
            tf = TransformStamped()
            tf.header.stamp = current_time.to_msg()
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_link'
            
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            
            tf.transform.rotation.x = 0.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = math.sin(self.theta / 2.0)
            tf.transform.rotation.w = math.cos(self.theta / 2.0)
            
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OmniControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
