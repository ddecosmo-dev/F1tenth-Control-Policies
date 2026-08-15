import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory

import os
import numpy as np
import onnxruntime as ort
import joblib
from collections import deque
import math
import yaml

#add basic logging!
#show outputs and errors e

#after validating baseline performance 
#need to add a method to save performance and compare to the MPC 
#may need to consider how often new scans are accepted (10ms), same with inference output 

#adding curvature and lookahead to the model
#new additions 

"""
1. Need to load a csv for the map, can be in config file
Will have 3 cols, x,y,curvature

2. Need to have a lookahead function,
searches for nodes ahead of the car.
Find the first node OVER the desired distance
reutrns the curvature value of that point

i.e we want to look 2m ahead of the car, we find the first point in the structure that is 2.1m ahead
we then return only the curvature value

3. Need to store these lookahead values and pass them to the model
store lookahead values in a list or something else,
Pass them as new inputs to the LSTM for updated models

"""

class LSTMDriveNode(Node):
    def __init__(self):
        super().__init__('lstm_drive_node')

        # --- Parameters ---
        DEFAULT_MODEL_NAME = "LSTM_1B_128D"
        DEFAULT_SEQ_LENGTH = 100
        DEFAULT_LIDAR_POINTS = 60

        # allow overriding via ROS params
        self.declare_parameter('model_name', DEFAULT_MODEL_NAME)
        self.declare_parameter('seq_length', DEFAULT_SEQ_LENGTH)
        self.declare_parameter('lidar_points', DEFAULT_LIDAR_POINTS)
        self.declare_parameter('publish_interval_ms', 15)
        self.declare_parameter('max_speed', 4.5)
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_scaler_path', '')
        self.declare_parameter('target_scaler_path', '')
        self.declare_parameter('curvature_lut_csv', '')
        self.declare_parameter('lookahead_distances', [1.0, 2.0, 3.0, 5.0, 8.0])

        # If a package-local config/config.yaml exists, load it and set
        # parameters so that `ros2 run lstm_racer lstm_drive` will use it.
        try:
            pkg_share_tmp = get_package_share_directory('lstm_racer')
            cfg_file = os.path.join(pkg_share_tmp, 'config', 'config.yaml')
            if os.path.isfile(cfg_file):
                with open(cfg_file, 'r') as cf:
                    cfg_vals = yaml.safe_load(cf)
                params_to_set = []
                if isinstance(cfg_vals, dict):
                    for k in ('model_name', 'seq_length', 'lidar_points', 'publish_interval_ms', 'max_speed', 'curvature_lut_csv', 'lookahead_distances'):
                        if k in cfg_vals:
                            val = cfg_vals[k]
                            if isinstance(val, bool):
                                ptype = rclpy.Parameter.Type.BOOL
                            elif isinstance(val, int):
                                ptype = rclpy.Parameter.Type.INTEGER
                            elif isinstance(val, float):
                                ptype = rclpy.Parameter.Type.DOUBLE
                            elif isinstance(val, list):
                                if all(isinstance(v, int) for v in val):
                                    ptype = rclpy.Parameter.Type.INTEGER_ARRAY
                                elif all(isinstance(v, (int, float)) for v in val):
                                    ptype = rclpy.Parameter.Type.DOUBLE_ARRAY
                                else:
                                    ptype = rclpy.Parameter.Type.STRING_ARRAY
                            else:
                                ptype = rclpy.Parameter.Type.STRING
                            params_to_set.append(rclpy.parameter.Parameter(k, ptype, val))
                if params_to_set:
                    self.set_parameters(params_to_set)
        except Exception as e:
            self.get_logger().debug(f"No package config/config.yaml applied: {e}")

        self.MODEL_NAME = self.get_parameter('model_name').value
        self.SEQ_LENGTH = int(self.get_parameter('seq_length').value)
        self.LIDAR_POINTS = int(self.get_parameter('lidar_points').value)
        raw_lookahead_distances = self.get_parameter('lookahead_distances').value
        if isinstance(raw_lookahead_distances, (list, tuple)):
            self.LOOKAHEAD_DISTANCES = [float(x) for x in raw_lookahead_distances]
        else:
            self.LOOKAHEAD_DISTANCES = [1.0, 2.0, 3.0, 5.0, 8.0]
        self.INPUT_DIM = 3 + self.LIDAR_POINTS + len(self.LOOKAHEAD_DISTANCES)  # [steering, velocity, yaw] + lidar + lookahead curvatures

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_velocity = 0.0
        self.current_steering = 0.0
        self.current_yaw = 0.0
        self.last_pred_steering = 0.0
        self.last_pred_velocity = 0.0
        self.last_inference_latency = 0.0
        self.sequence_buffer = deque(maxlen=self.SEQ_LENGTH)
        # Prefill sequence buffer so it always has SEQ_LENGTH timesteps
        try:
            zero_step = np.zeros(self.INPUT_DIM, dtype=np.float32)
        except Exception:
            zero_step = [0.0] * self.INPUT_DIM
        for _ in range(self.SEQ_LENGTH):
            self.sequence_buffer.append(zero_step)

        # Load curvature lookup table from config if provided
        try:
            pkg_share = get_package_share_directory('lstm_racer')
            lut_param = str(self.get_parameter('curvature_lut_csv').value).strip()
            self.curvature_lut = None
            if lut_param:
                lut_path = self._resolve_curvature_lut_path(pkg_share, lut_param)
                self.curvature_lut = self._load_curvature_lut(lut_path)
                self.get_logger().info(f"Loaded curvature LUT from {lut_path} ({len(self.curvature_lut)} points)")
            else:
                self.get_logger().warning("No curvature LUT CSV configured; lookahead curvatures will be zeros.")
        except Exception as e:
            self.get_logger().warning(f"Unable to load curvature LUT: {e}")
            self.curvature_lut = None

        # Load package config for model entries (optional). Expect a simple
        # list of model names under `models:`. If an entry matching
        # `MODEL_NAME` exists, treat it as a model_subdir and set model/scaler
        # parameter overrides. Do NOT override seq_length or lidar_points here.
        try:
            pkg_share = get_package_share_directory('lstm_racer')
            cfg_path = os.path.join(pkg_share, 'config', 'models.yaml')
            if os.path.isfile(cfg_path):
                with open(cfg_path, 'r') as f:
                    cfg = yaml.safe_load(f)
                models = cfg.get('models') if isinstance(cfg, dict) else cfg
                entry = None
                if isinstance(models, list):
                    if self.MODEL_NAME in models:
                        entry = {'model_subdir': self.MODEL_NAME}
                elif isinstance(models, dict):
                    raw = models.get(self.MODEL_NAME)
                    if isinstance(raw, str):
                        entry = {'model_subdir': raw}
                    elif isinstance(raw, dict):
                        entry = raw

                if entry and 'model_subdir' in entry:
                    model_sub = entry['model_subdir']
                    self.get_logger().info(f"Found config entry for model '{self.MODEL_NAME}' in config/models.yaml; using subdir '{model_sub}'.")
                    base = self.MODEL_NAME
                    models_root = os.path.join(pkg_share, 'models', model_sub)
                    model_file = entry.get('model_file', f"{base}.onnx")
                    input_file = entry.get('input_scaler', f"{base}_input_scaler.pkl")
                    target_file = entry.get('target_scaler', f"{base}_target_scaler.pkl")
                    # set model/scaler paths as node parameters so normal resolution will use them
                    self.set_parameters([
                        rclpy.parameter.Parameter('model_path', rclpy.Parameter.Type.STRING, os.path.join(models_root, model_file)),
                        rclpy.parameter.Parameter('input_scaler_path', rclpy.Parameter.Type.STRING, os.path.join(models_root, input_file)),
                        rclpy.parameter.Parameter('target_scaler_path', rclpy.Parameter.Type.STRING, os.path.join(models_root, target_file)),
                    ])
        except Exception as e:
            self.get_logger().warning(f"Unable to read config/models.yaml: {e}")

        # --- Resolve model/scaler paths ---
        model_path, input_scaler_path, target_scaler_path = self._resolve_model_artifacts()

        self.get_logger().info(f"Loading scalers and ONNX model: {self.MODEL_NAME}")
        try:
            self.input_scaler = joblib.load(input_scaler_path)
            self.target_scaler = joblib.load(target_scaler_path)
            self.ort_session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
            self.input_name = self.ort_session.get_inputs()[0].name
            # Inspect model input shape and adapt sequence length if necessary
            try:
                raw_shape = self.ort_session.get_inputs()[0].shape
                # normalize shape entries (replace symbolic dims with None)
                model_input_shape = [s if isinstance(s, int) else None for s in raw_shape]
                expected_seq = model_input_shape[1] if len(model_input_shape) > 1 else None
                expected_input_dim = model_input_shape[2] if len(model_input_shape) > 2 else None
                if expected_seq is not None and expected_seq != self.SEQ_LENGTH:
                    self.get_logger().info(f"Model expects seq_length={expected_seq} but node configured seq_length={self.SEQ_LENGTH}; adapting node to model.")
                    self.SEQ_LENGTH = int(expected_seq)
                    # rebuild sequence buffer with the new length and prefill zeros
                    try:
                        zero_step = np.zeros(self.INPUT_DIM, dtype=np.float32)
                    except Exception:
                        zero_step = [0.0] * self.INPUT_DIM
                    self.sequence_buffer = deque(maxlen=self.SEQ_LENGTH)
                    for _ in range(self.SEQ_LENGTH):
                        self.sequence_buffer.append(zero_step)

                if expected_input_dim is not None and expected_input_dim != self.INPUT_DIM:
                    # input feature dimension mismatch (includes steering, velocity, yaw + lidar)
                    raise ValueError(f"Model input feature dimension {expected_input_dim} != node INPUT_DIM {self.INPUT_DIM}. Adjust 'lidar_points' or use a matching model.")
            except Exception as ex_shape:
                self.get_logger().warning(f"Could not verify model input shape: {ex_shape}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model/scalers: {e}")
            raise

        # --- Publishers & Subscribers ---
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.lookahead_points_pub = self.create_publisher(MarkerArray, '/lookahead_markers', 1)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publishing timer: high-rate publish of last command (ms)
        publish_interval_ms = float(self.get_parameter('publish_interval_ms').value)
        publish_interval_s = max(0.01, publish_interval_ms / 1000.0)
        self.pub_timer = self.create_timer(publish_interval_s, self._publish_last_command)

        # Max speed clamp
        self.MAX_SPEED = float(self.get_parameter('max_speed').value)

        self.get_logger().info("🟢 LSTM Drive Node Initialized.")

    def _resolve_model_artifacts(self):
        pkg_share = get_package_share_directory('lstm_racer')
        models_root = os.path.join(pkg_share, 'models')

        # 1) explicit overrides via params
        param_model = str(self.get_parameter('model_path').value).strip()
        param_input = str(self.get_parameter('input_scaler_path').value).strip()
        param_target = str(self.get_parameter('target_scaler_path').value).strip()
        if param_model and param_input and param_target:
            self.get_logger().info("Using explicit model/scaler paths from parameters")
            return param_model, param_input, param_target

        # 2) standard candidates: flat and nested model-name directory
        base = self.MODEL_NAME
        candidates = [
            (
                os.path.join(models_root, f'{base}.onnx'),
                os.path.join(models_root, f'{base}_input_scaler.pkl'),
                os.path.join(models_root, f'{base}_target_scaler.pkl'),
            ),
            (
                os.path.join(models_root, base, f'{base}.onnx'),
                os.path.join(models_root, base, f'{base}_input_scaler.pkl'),
                os.path.join(models_root, base, f'{base}_target_scaler.pkl'),
            ),
        ]

        for m, i, t in candidates:
            if os.path.isfile(m) and os.path.isfile(i) and os.path.isfile(t):
                self.get_logger().info(f"Resolved model artifacts in: {os.path.dirname(m)}")
                return m, i, t

        # 3) recursive fallback search by basename
        model_basename = f'{base}.onnx'
        input_basename = f'{base}_input_scaler.pkl'
        target_basename = f'{base}_target_scaler.pkl'
        for root, _, files in os.walk(models_root):
            if model_basename in files and input_basename in files and target_basename in files:
                return (
                    os.path.join(root, model_basename),
                    os.path.join(root, input_basename),
                    os.path.join(root, target_basename),
                )

        raise FileNotFoundError(
            f"Could not find model artifacts for '{base}' under {models_root}. "
            f"Expected either flat files or a subdir named '{base}'."
        )

    def _resolve_curvature_lut_path(self, pkg_share, lut_param):
        if os.path.isabs(lut_param) and os.path.isfile(lut_param):
            return lut_param

        candidates = [
            os.path.join(pkg_share, lut_param),
            os.path.join(pkg_share, 'config', lut_param),
            os.path.join(pkg_share, 'maps', lut_param),
        ]
        for candidate in candidates:
            if os.path.isfile(candidate):
                return candidate

        raise FileNotFoundError(
            f"Curvature LUT CSV '{lut_param}' not found under package share or config directories."
        )

    def _load_curvature_lut(self, lut_path):
        data = np.genfromtxt(lut_path, delimiter=',', dtype=np.float32, skip_header=1)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] < 5:
            raise ValueError(f"Curvature LUT must contain at least 5 columns, with curvature in column index 4, got {data.shape[1]}")
        return data[:, [0, 1, 4]]

    def _find_lookahead_points(self, x, y, yaw):
        if self.curvature_lut is None or len(self.curvature_lut) == 0:
            return [None] * len(self.LOOKAHEAD_DISTANCES)

        distances = np.hypot(self.curvature_lut[:, 0] - x, self.curvature_lut[:, 1] - y)
        closest_idx = int(np.argmin(distances))
        points = []
        n = len(self.curvature_lut)

        for target in self.LOOKAHEAD_DISTANCES:
            accumulated = 0.0
            idx = closest_idx
            last_x, last_y = float(self.curvature_lut[idx, 0]), float(self.curvature_lut[idx, 1])
            if target <= 0.0:
                points.append((last_x, last_y, float(self.curvature_lut[idx, 2])))
                continue

            while accumulated < target:
                next_idx = (idx + 1) % n
                next_x, next_y = float(self.curvature_lut[next_idx, 0]), float(self.curvature_lut[next_idx, 1])
                segment = math.hypot(next_x - last_x, next_y - last_y)
                accumulated += segment
                idx = next_idx
                last_x, last_y = next_x, next_y
                if idx == closest_idx:
                    break

            points.append((float(self.curvature_lut[idx, 0]), float(self.curvature_lut[idx, 1]), float(self.curvature_lut[idx, 2])))
        return points

    def _publish_lookahead_markers(self, lookahead_points):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for idx, point in enumerate(lookahead_points):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.ns = 'lookahead'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(point[0]) if point is not None else 0.0
            marker.pose.position.y = float(point[1]) if point is not None else 0.0
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            marker_array.markers.append(marker)
        self.lookahead_points_pub.publish(marker_array)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_velocity = msg.twist.twist.linear.x
        # extract yaw from orientation quaternion
        # TODO: double check this angle is correct
        q = msg.pose.pose.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        raw_lidar = np.array(msg.ranges)
        # Downsample to configured lidar points (default 60)
        indices = np.linspace(0, len(raw_lidar) - 1, self.LIDAR_POINTS, dtype=int)
        downsampled_lidar = raw_lidar[indices]

        #CHECK!
        downsampled_lidar = np.nan_to_num(downsampled_lidar, posinf=15.0, neginf=0.0)

        lookahead_points = self._find_lookahead_points(self.current_x, self.current_y, self.current_yaw)
        self._publish_lookahead_markers(lookahead_points)
        lookahead_values = [float(point[2]) if point is not None else 0.0 for point in lookahead_points]

        # Feature vector order: [steering_angle, linear_velocity, yaw, lidar_scans, lookahead curvatures]
        feature_vector = np.concatenate([
            [self.current_steering, self.current_velocity, self.current_yaw],
            downsampled_lidar,
            lookahead_values,
        ])
        scaled_features = self.input_scaler.transform(feature_vector.reshape(1, -1))[0]
        
        #check that this sets up properly 
        #need to be adding timesteps to the total sequence length (100)
        #dropping oldest and replacing with newest as we move forward 
        self.sequence_buffer.append(scaled_features)
        
        if len(self.sequence_buffer) < self.SEQ_LENGTH:
            return

        # Inference
        try:
            import time
            t0 = time.perf_counter()
            model_input = np.array(self.sequence_buffer, dtype=np.float32).reshape(1, self.SEQ_LENGTH, self.INPUT_DIM)
            onnx_pred = self.ort_session.run(None, {self.input_name: model_input})[0]
            t1 = time.perf_counter()
            self.last_inference_latency = (t1 - t0) * 1000.0
        except Exception as e:
            self.get_logger().error(f"ONNX inference failed: {e}")
            return
        
        # Output Scaling: ['steering_angle', 'velocity']
        physical_commands = self.target_scaler.inverse_transform(onnx_pred)[0]
        pred_steering = float(physical_commands[0])
        pred_velocity = float(physical_commands[1])

        # clamp predicted velocity to configured max and store last predictions
        if pred_velocity > self.MAX_SPEED:
            pred_velocity = float(self.MAX_SPEED)
            self.get_logger().debug(f"Clamped predicted velocity to max_speed={self.MAX_SPEED}")

        self.last_pred_steering = pred_steering
        self.last_pred_velocity = pred_velocity
        self.current_steering = pred_steering 

        # Publish
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = pred_steering
        drive_msg.drive.speed = pred_velocity
        
        # publish immediate command and log summary
        self.drive_pub.publish(drive_msg)
        self.get_logger().info(f"Inference -> steering: {pred_steering:.4f}, speed: {pred_velocity:.4f}, seq_len: {len(self.sequence_buffer)}/{self.SEQ_LENGTH}, latency_ms: {self.last_inference_latency:.2f}")

    def _publish_last_command(self):
        # Publish last known command at high rate; use debug logging to avoid flooding
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = float(self.last_pred_steering)
        drive_msg.drive.speed = float(self.last_pred_velocity)
        self.drive_pub.publish(drive_msg)
        self.get_logger().debug(f"PublishTick -> steering: {self.last_pred_steering:.4f}, speed: {self.last_pred_velocity:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = LSTMDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()