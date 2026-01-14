#!/usr/bin/env python3

import math
import subprocess

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w


class TeleportNode(Node):
    """
    Teleport + odom correction node.

    Subscribes:
      - odom_gz (nav_msgs/Odometry): raw Gazebo odometry
      - set_robot_position (geometry_msgs/PoseStamped): desired pose in "corrected/virtual odom"

    Publishes:
      - odom (nav_msgs/Odometry): corrected odom after applying inverse offset

    What it does:
      1) On odom_gz: store current Gazebo pose and republish corrected /odom.
      2) On set_robot_position: compute (x0, y0, theta0) offset so corrected odom becomes the requested pose,
         then teleport the model in Gazebo via `gz service ... set_pose`.
    """

    def __init__(self):
        super().__init__('teleport_node')

        # Namespace/model name
        self.robot_namespace = self.get_namespace().strip('/')
        if not self.robot_namespace:
            self.robot_namespace = 'my_bot'  # fallback

        # Params for Gazebo teleporting
        self.declare_parameter('gz_service', '/world/empty/set_pose')
        self.declare_parameter('model_name', self.robot_namespace)
        self.declare_parameter('z', 0.1)
        self.declare_parameter('timeout_ms', 1000)
        self.declare_parameter('process_timeout_s', 5.0)

        self.gz_service = self.get_parameter('gz_service').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.z = float(self.get_parameter('z').value)
        self.timeout_ms = int(self.get_parameter('timeout_ms').value)
        self.process_timeout_s = float(self.get_parameter('process_timeout_s').value)

        # Offsets used to map Gazebo odom -> corrected odom
        self.position_offset_x = 0.0
        self.position_offset_y = 0.0
        self.orientation_offset_x = 0.0
        self.orientation_offset_y = 0.0
        self.orientation_offset_z = 0.0
        self.orientation_offset_w = 1.0

        # Latest Gazebo pose (from odom_gz)
        self.have_gz_odom = False
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.current_robot_theta = 0.0

        # I/O
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.odom_gz_sub = self.create_subscription(
            Odometry, 'odom_gz', self.odom_gz_callback, 10
        )
        self.set_position_sub = self.create_subscription(
            PoseStamped, 'set_robot_position', self.set_position_callback, 10
        )

        self.get_logger().info(f'Teleport+Odom node started. Model: "{self.model_name}"')
        self.get_logger().info('Subscribing: /odom_gz, /set_robot_position')
        self.get_logger().info('Publishing:  /odom (corrected)')
        self.get_logger().info(f'Gazebo service: {self.gz_service}')

    def set_position_callback(self, msg: PoseStamped) -> None:
        """
        msg.pose is the desired pose in corrected odom after teleport.
        Compute offsets so that the current Gazebo odom pose maps to this desired pose.
        Then teleport in Gazebo.
        """
        if not self.have_gz_odom:
            self.get_logger().warn('Ignoring set_robot_position: no odom_gz received yet.')
            return

        # Desired corrected pose
        x_des = msg.pose.position.x
        y_des = msg.pose.position.y
        _, _, theta_des = quaternion_to_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

        # Current Gazebo odom pose BEFORE teleport
        x_g = self.current_robot_x
        y_g = self.current_robot_y
        theta_g = self.current_robot_theta

        # Orientation offset (theta0)
        theta0 = theta_g - theta_des
        cos0 = math.cos(theta0)
        sin0 = math.sin(theta0)

        # Position offset (x0, y0)
        # x_g = x0 + x_des*cos0 - y_des*sin0
        # y_g = y0 + x_des*sin0 + y_des*cos0
        x0 = x_g - (x_des * cos0 - y_des * sin0)
        y0 = y_g - (x_des * sin0 + y_des * cos0)

        # Store offsets
        self.position_offset_x = x0
        self.position_offset_y = y0

        q0_x, q0_y, q0_z, q0_w = euler_to_quaternion(0.0, 0.0, theta0)
        self.orientation_offset_x = q0_x
        self.orientation_offset_y = q0_y
        self.orientation_offset_z = q0_z
        self.orientation_offset_w = q0_w

        # Teleport model in Gazebo to the requested corrected pose position/orientation
        self._teleport_in_gazebo(x_des, y_des, msg.pose.orientation)

        self.get_logger().info(
            "Offset computed + teleport requested: "
            f"x0={x0:.3f}, y0={y0:.3f}, theta0={math.degrees(theta0):.1f}°"
        )

    def odom_gz_callback(self, msg: Odometry) -> None:
        # Store latest Gazebo pose
        self.current_robot_x = msg.pose.pose.position.x
        self.current_robot_y = msg.pose.pose.position.y
        _, _, theta = quaternion_to_euler(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.current_robot_theta = theta
        self.have_gz_odom = True

        # Extract offsets (x0, y0, theta0)
        x0 = self.position_offset_x
        y0 = self.position_offset_y
        _, _, theta0 = quaternion_to_euler(
            self.orientation_offset_x,
            self.orientation_offset_y,
            self.orientation_offset_z,
            self.orientation_offset_w,
        )

        # Apply inverse offset:
        # (x', y') = R(-theta0) * ([x, y] - [x0, y0])
        # theta' = theta - theta0
        dx = self.current_robot_x - x0
        dy = self.current_robot_y - y0

        cos0 = math.cos(theta0)
        sin0 = math.sin(theta0)

        x_new = dx * cos0 + dy * sin0
        y_new = -dx * sin0 + dy * cos0
        theta_new = theta - theta0

        new_quat_x, new_quat_y, new_quat_z, new_quat_w = euler_to_quaternion(0.0, 0.0, theta_new)

        # Build outgoing odom (avoid mutating the incoming message instance)
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        out.pose.pose.position.x = x_new
        out.pose.pose.position.y = y_new
        out.pose.pose.orientation.x = new_quat_x
        out.pose.pose.orientation.y = new_quat_y
        out.pose.pose.orientation.z = new_quat_z
        out.pose.pose.orientation.w = new_quat_w

        # Namespace prefix frames like in your EspActionSimulation
        namespace_prefix = f"{self.robot_namespace}/" if self.robot_namespace else ""
        out.header.frame_id = f"{namespace_prefix}odom"
        out.child_frame_id = f"{namespace_prefix}base_link"

        self.odom_pub.publish(out)

    def _teleport_in_gazebo(self, x: float, y: float, orientation) -> None:
        quat_x = orientation.x
        quat_y = orientation.y
        quat_z = orientation.z
        quat_w = orientation.w

        req = (
            f'name: "{self.model_name}", '
            f'position: {{x: {x}, y: {y}, z: {self.z}}}, '
            f'orientation: {{x: {quat_x}, y: {quat_y}, z: {quat_z}, w: {quat_w}}}'
        )

        cmd = [
            'gz', 'service',
            '-s', self.gz_service,
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self.timeout_ms),
            '--req', req,
        ]

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self.process_timeout_s,
            )
            if result.returncode != 0:
                self.get_logger().error(
                    f'Gazebo set_pose failed (rc={result.returncode}). stderr: {result.stderr.strip()}'
                )
        except subprocess.TimeoutExpired:
            self.get_logger().error('Gazebo set_pose service call timed out (subprocess timeout)')
        except Exception as e:
            self.get_logger().error(f'Error calling Gazebo set_pose service: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TeleportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()