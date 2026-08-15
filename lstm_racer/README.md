# LSTM Racer

A ROS 2 package for autonomous racing using LSTM-based imitation learning. The node subscribes to LiDAR and odometry data, buffers sequences of vehicle state, and runs ONNX-format neural network models to predict steering and velocity commands.

## Features

- **ONNX Runtime Inference**: Efficient, hardware-accelerated model inference using ONNX Runtime.
- **Sequence Buffering**: Maintains a rolling buffer of sensor history for LSTM inputs.
- **Multi-Model Support**: Easy model switching via `config/config.yaml` without rebuilding.
- **High-Rate Publishing**: Configurable timer-based command publishing (default 10 ms).
- **Velocity Clamping**: Enforce maximum speed limits for safety.
- **Robust Artifact Resolution**: Automatically locates model and scaler files across flat or nested directory structures.
- **Launch File Support**: Simple ROS 2 launch file for easy startup.

## Quick Start

### Prerequisites

- ROS 2 (humble or newer)
- Python 3.10+
- Dependencies: `onnxruntime`, `joblib`, `scikit-learn`, `numpy<=1.22.0`, `PyYAML`

### Building

```bash
cd ~/sim_ws
colcon build --packages-select lstm_racer
source install/setup.bash
```

### Running

#### Launch the drive node
```bash
ros2 launch lstm_racer lstm_drive_launch.py
```

Override the model name at launch:
```bash
ros2 launch lstm_racer lstm_drive_launch.py model_name:=LSTM_1B_32D_Pred_1
```

Or run the drive node directly:
```bash
ros2 run lstm_racer lstm_drive
```

The drive node loads `model_name` and other runtime settings from `config/config.yaml` by default.

#### Launch the logger node
```bash
ros2 launch lstm_racer lstm_logger_launch.py
```

With logger flags:
```bash
ros2 launch lstm_racer lstm_logger_launch.py \
  output_file:=lstm_racer_log.npz \
  use_start_pose_as_start_center:=true \
  use_goal_offset:=true \
  goal_offset_x:=0.0 \
  goal_offset_y:=0.0 \
  goal_radius:=1.0 \
  start_radius:=1.0 \
  collision_topic:=/collision_detected
```

#### Logger launch flag explanations

- `model_name`: identifies the model under test and is included in the output log filename.
- `output_file`: base name for the NPZ log file. The logger always stores the file under the installed package `logs/` folder and appends a timestamp and model name.
- `output_csv`: alias supported for backward compatibility.
- `use_start_pose_as_start_center`: when `true`, the first received odometry pose is used as the origin of the start zone.
- `use_goal_offset`: when `true`, the goal position is computed relative to the start center using `goal_offset_x` and `goal_offset_y`.
- `goal_offset_x`, `goal_offset_y`: offset from the start position to the goal position, in meters.
- `goal_radius`: completion is triggered when the vehicle enters this radius around the goal position.
- `start_radius`: the vehicle must exit this radius from the start center before lap timing and completion tracking begin.
- `collision_topic`: name of a `std_msgs/Bool` topic that signals collisions. The logger sets `collision_flag` only when this topic publishes `true`.

To ensure the goal is defined relative to the start position, use:
- `use_start_pose_as_start_center:=true`
- `use_goal_offset:=true`
- `goal_offset_x` / `goal_offset_y`

To enable collision detection, set:
- `collision_topic:=/collision_detected`

The logger node writes NPZ output into the package `logs/` folder. If `output_file` is a relative filename, a timestamped file is created automatically with the model name included. The old `output_csv` alias is also still accepted.

#### NPZ schema
The saved `.npz` file contains these arrays:
- `timestamps`: shape `(N,)` — time in seconds for each log step
- `step_id`: shape `(N,)` — sequential log index
- `x`, `y`: shape `(N,)` — vehicle position
- `yaw`: shape `(N,)` — vehicle heading from odometry
- `velocity`: shape `(N,)` — current linear velocity
- `steering_angle`: shape `(N,)` — current steering command
- `speed_command`: shape `(N,)` — commanded speed from the drive topic
- `collision_flag`: shape `(N,)` — 0/1 collision status
- `completion_flag`: shape `(N,)` — 0/1 goal completion status
- `has_left_start_zone`: shape `(N,)` — 0/1 start-zone exit status
- `distance_to_goal`: shape `(N,)` — distance from current pose to goal
- `distance_to_start`: shape `(N,)` — distance from current pose to start
- `lidar`: shape `(N, lidar_points)` — downsampled LiDAR vectors
- `model_name`: scalar string — the model name used for this run
- `goal_center`: shape `(2,)` — final goal center used by the logger
- `start_center`: shape `(2,)` — start center used by the logger
- `goal_radius`: scalar float
- `start_radius`: scalar float

### Configuration

Edit `config/config.yaml` to set runtime parameters:
```yaml
model_name: LSTM_1B_32D_Pred_1   # Model name (subdir under models/)

**NOTE: ** Do not change these for the most part, only applicable if training changes
seq_length: 100                  # LSTM sequence length
lidar_points: 60                 # Number of downsampled LiDAR points
publish_interval_ms: 10          # Command publish rate (ms)
max_speed: 4.5                   # Maximum velocity clamp (m/s)
```

### Topics

- **Subscriptions**:
  - `/scan` (sensor_msgs/LaserScan): LiDAR range data
  - `/ego_racecar/odom` (nav_msgs/Odometry): Vehicle odometry and pose

- **Publications**:
  - `/drive` (ackermann_msgs/AckermannDriveStamped): Steering angle and velocity commands

### Models

Place ONNX model files under `models/<MODEL_NAME>/`:
```
models/
  LSTM_1B_128D/
    LSTM_1B_128D.onnx
    LSTM_1B_128D_input_scaler.pkl
    LSTM_1B_128D_target_scaler.pkl
  LSTM_1B_32D_Pred_1/
    LSTM_1B_32D_Pred_1.onnx
    LSTM_1B_32D_Pred_1_input_scaler.pkl
    LSTM_1B_32D_Pred_1_target_scaler.pkl
```

The node supports both flat and nested layouts and will auto-detect model artifacts.

## TODO

- [x] **Add logging for outputs, position, velocity etc**
  - Implement debug/info logging for predicted commands
  - Log vehicle position and velocity states
  - Add optional CSV logging for post-analysis

- [ ] **Perform additional tests w/ models on different maps**
  - Test model generalization across varied environments
  - Benchmark performance on diverse track layouts
  - Identify failure modes and edge cases

- [ ] **Compare to each other and MPC**
  - Run comparative benchmarks between models
  - Measure LSTM performance vs. Model Predictive Control
  - Analyze latency, accuracy, and robustness trade-offs

- [ ] **Update config for sim-to-real**
  - Calibrate sensor preprocessing for real hardware
  - Adjust speed clamps and safety limits
  - Validate model transfer learning to physical platform

## Development Notes

- The node adapts its internal `seq_length` to match the loaded ONNX model's expected input shape.
- Feature order: `[steering_angle, linear_velocity, yaw, lidar_scans...]`
- Input scalers and target scalers are loaded via `joblib` and applied before/after inference.
- Inference latency is logged for performance monitoring.

## License

TODO

## Author

TODO
