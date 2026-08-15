import datetime
import math
import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class LSTMLoggerNode(Node):
    def __init__(self):
        super().__init__('lstm_logger_node')

        self.declare_parameter('log_frequency_hz', 100.0)
        self.declare_parameter('lidar_points', 30)
        self.declare_parameter('output_file', 'lstm_racer_log.npz')
        self.declare_parameter('output_csv', 'lstm_racer_log.npz')  # alias for compatibility
        self.declare_parameter('model_name', '')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('collision_topic', '')

        self.declare_parameter('goal_center_x', 0.0)
        self.declare_parameter('goal_center_y', 0.0)
        self.declare_parameter('goal_offset_x', 0.0)
        self.declare_parameter('goal_offset_y', 0.0)
        self.declare_parameter('use_goal_offset', True)
        self.declare_parameter('goal_radius', 1.0)
        self.declare_parameter('start_center_x', 0.0)
        self.declare_parameter('start_center_y', 0.0)
        self.declare_parameter('use_start_pose_as_start_center', True)
        self.declare_parameter('start_radius', 1.0)

        self.log_frequency_hz = float(self.get_parameter('log_frequency_hz').value)
        self.lidar_points = int(self.get_parameter('lidar_points').value)
        self.output_file = str(self.get_parameter('output_file').value)
        if self.output_file == '':
            self.output_file = str(self.get_parameter('output_csv').value)
        self.model_name = str(self.get_parameter('model_name').value).strip() or 'unknown_model'
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.drive_topic = str(self.get_parameter('drive_topic').value)
        self.collision_topic = str(self.get_parameter('collision_topic').value).strip()

        self.goal_center_param = (
            float(self.get_parameter('goal_center_x').value),
            float(self.get_parameter('goal_center_y').value),
        )
        self.goal_offset = (
            float(self.get_parameter('goal_offset_x').value),
            float(self.get_parameter('goal_offset_y').value),
        )
        self.use_goal_offset = bool(self.get_parameter('use_goal_offset').value)
        self.goal_radius = float(self.get_parameter('goal_radius').value)
        self.start_center_param = (
            float(self.get_parameter('start_center_x').value),
            float(self.get_parameter('start_center_y').value),
        )
        self.use_start_pose_as_start_center = bool(self.get_parameter('use_start_pose_as_start_center').value)
        self.start_radius = float(self.get_parameter('start_radius').value)
        self.goal_center = None
        self.start_center = None
        self.status_log_interval = 1.0
        self.save_interval = 1.0
        self.last_status_log_time = self.get_clock().now().nanoseconds / 1e9
        self.last_save_time = self.last_status_log_time

        self.timestamps = []
        self.step_ids = []
        self.x = []
        self.y = []
        self.yaw = []
        self.velocities = []
        self.steering_angles = []
        self.speed_commands = []
        self.collision_flags = []
        self.completion_flags = []
        self.has_left_start_zone_flags = []
        self.goal_distances = []
        self.start_distances = []
        self.lidar_data = []

        self.current_pose = None
        self.current_velocity = None
        self.current_yaw = None
        self.current_lidar = None
        self.current_steering = 0.0
        self.current_speed = 0.0
        self.collision_flag = False
        self.has_left_start_zone = False
        self.start_time = None
        self.completion_time = None
        self.completion_flag = False
        self.step_id = 0
        self.summary_written = False

        self._open_log_file()

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            self.drive_topic,
            self.drive_callback,
            10,
        )

        if self.collision_topic:
            self.collision_sub = self.create_subscription(
                Bool,
                self.collision_topic,
                self.collision_callback,
                10,
            )

        timer_period = max(0.001, 1.0 / self.log_frequency_hz)
        self.log_timer = self.create_timer(timer_period, self.log_timer_callback)

        self.get_logger().info(
            f'LSTM Logger initialized: log_frequency={self.log_frequency_hz}Hz, lidar_points={self.lidar_points}, output_file={self.output_file}'
        )

    def _open_log_file(self):
        base_name = os.path.basename(self.output_file) if self.output_file else 'lstm_racer_log'
        base_name = os.path.splitext(base_name)[0]
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_model_name = self.model_name.replace(' ', '_')
        output_filename = f'{base_name}_{safe_model_name}_{timestamp}.npz'

        if os.path.isabs(self.output_file):
            output_path = os.path.join(os.path.dirname(self.output_file), output_filename)
            output_dir = os.path.dirname(output_path)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)
        else:
            pkg_share = get_package_share_directory('mapless_lstm_racer')
            logs_dir = os.path.join(pkg_share, 'logs')
            os.makedirs(logs_dir, exist_ok=True)
            output_path = os.path.join(logs_dir, output_filename)

        self.output_file = output_path
        self.get_logger().debug(f'Logging NPZ output to: {self.output_file}')
        self._save_npz()

    def odom_callback(self, msg: Odometry):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_velocity = float(msg.twist.twist.linear.x)
        self.current_yaw = self._quat_to_yaw(msg.pose.pose.orientation)

        if self.start_center is None:
            if self.use_start_pose_as_start_center:
                self.start_center = self.current_pose
                self.get_logger().info(f'Start center set from initial pose: {self.start_center}')
            else:
                self.start_center = self.start_center_param
                self.get_logger().info(f'Start center set from parameter: {self.start_center}')

            if self.use_goal_offset:
                self.goal_center = (
                    self.start_center[0] + self.goal_offset[0],
                    self.start_center[1] + self.goal_offset[1],
                )
                self.get_logger().info(
                    f'Goal center computed relative to start: offset={self.goal_offset} goal_center={self.goal_center}'
                )
            else:
                self.goal_center = self.goal_center_param
                self.get_logger().info(f'Goal center set from parameter: {self.goal_center}')

    def scan_callback(self, msg: LaserScan):
        scan = np.array(msg.ranges, dtype=np.float32)
        scan = np.nan_to_num(scan, posinf=30.0, neginf=0.0)
        if scan.size == 0:
            return

        indices = np.linspace(0, scan.size - 1, self.lidar_points, dtype=int)
        self.current_lidar = scan[indices]

    def drive_callback(self, msg: AckermannDriveStamped):
        self.current_steering = float(msg.drive.steering_angle)
        self.current_speed = float(msg.drive.speed)

    def collision_callback(self, msg: Bool):
        if msg.data:
            if not self.collision_flag:
                self.get_logger().info('Collision detected: collision_flag raised.')
            self.collision_flag = True
        else:
            self.get_logger().debug('Collision topic received false.')

    def log_timer_callback(self):
        if self.current_pose is None or self.current_lidar is None:
            return

        timestamp = self.get_clock().now().to_msg()
        x, y = self.current_pose
        goal_dist = self._distance((x, y), self.goal_center)
        start_dist = self._distance((x, y), self.start_center)

        if not self.has_left_start_zone and start_dist > self.start_radius:
            self.has_left_start_zone = True
            if self.start_time is None:
                self.start_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info('Start zone exited; lap timing begins.')

        if self.has_left_start_zone and not self.completion_flag and goal_dist <= self.goal_radius:
            self.completion_flag = True
            self.completion_time = self.get_clock().now().nanoseconds / 1e9
            lap_time = self.completion_time - self.start_time if self.start_time is not None else float('nan')
            self.get_logger().info(f'Goal reached at step {self.step_id}. lap_time={lap_time:.3f}s')

        self.timestamps.append(timestamp.sec + timestamp.nanosec * 1e-9)
        self.step_ids.append(self.step_id)
        self.x.append(x)
        self.y.append(y)
        self.yaw.append(self.current_yaw)
        self.velocities.append(self.current_velocity)
        self.steering_angles.append(self.current_steering)
        self.speed_commands.append(self.current_speed)
        self.collision_flags.append(int(self.collision_flag))
        self.completion_flags.append(int(self.completion_flag))
        self.has_left_start_zone_flags.append(int(self.has_left_start_zone))
        self.goal_distances.append(goal_dist)
        self.start_distances.append(start_dist)
        self.lidar_data.append(self.current_lidar.astype(np.float32).copy())

        self.step_id += 1

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_status_log_time >= self.status_log_interval:
            self.get_logger().info(
                f'STATUS time={current_time:.1f}s x={x:.2f} y={y:.2f} '
                f'goal_dist={goal_dist:.2f} start_dist={start_dist:.2f} '
                f'collision={self.collision_flag} completion={self.completion_flag} '
                f'left_start={self.has_left_start_zone} step={self.step_id}'
            )
            self.last_status_log_time = current_time

        if current_time - self.last_save_time >= self.save_interval:
            self._save_npz()
            self.last_save_time = current_time

    def _save_npz(self):
        if self.output_file is None:
            return

        lidar_array = np.stack(self.lidar_data, axis=0) if self.lidar_data else np.empty((0, self.lidar_points), dtype=np.float32)
        np.savez_compressed(
            self.output_file,
            timestamps=np.array(self.timestamps, dtype=np.float64),
            step_id=np.array(self.step_ids, dtype=np.int64),
            x=np.array(self.x, dtype=np.float32),
            y=np.array(self.y, dtype=np.float32),
            yaw=np.array(self.yaw, dtype=np.float32),
            velocity=np.array(self.velocities, dtype=np.float32),
            steering_angle=np.array(self.steering_angles, dtype=np.float32),
            speed_command=np.array(self.speed_commands, dtype=np.float32),
            collision_flag=np.array(self.collision_flags, dtype=np.uint8),
            completion_flag=np.array(self.completion_flags, dtype=np.uint8),
            has_left_start_zone=np.array(self.has_left_start_zone_flags, dtype=np.uint8),
            distance_to_goal=np.array(self.goal_distances, dtype=np.float32),
            distance_to_start=np.array(self.start_distances, dtype=np.float32),
            lidar=lidar_array,
            model_name=np.array(self.model_name),
            goal_center=np.array(self.goal_center if self.goal_center is not None else [np.nan, np.nan], dtype=np.float32),
            start_center=np.array(self.start_center if self.start_center is not None else [np.nan, np.nan], dtype=np.float32),
            goal_radius=np.array(self.goal_radius, dtype=np.float32),
            start_radius=np.array(self.start_radius, dtype=np.float32),
        )
        self.get_logger().debug(f'Saved NPZ log to: {self.output_file} with {len(self.timestamps)} entries.')

    def _distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def _quat_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def destroy_node(self):
        self._save_npz()
        self._write_summary()
        super().destroy_node()

    def _write_summary(self):
        if self.summary_written:
            return
        self.summary_written = True

        if self.completion_time is not None and self.start_time is not None:
            lap_time = self.completion_time - self.start_time
            self.get_logger().info(f'Lap completed in {lap_time:.3f} seconds.')
        elif self.completion_flag:
            self.get_logger().info('Lap completed, but start time is unavailable.')
        else:
            self.get_logger().info('Run ended without goal completion.')

        if self.collision_flag:
            self.get_logger().info('Collision flag was raised during the run.')


def main(args=None):
    rclpy.init(args=args)
    node = LSTMLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
