# F1Tenth Autonomous Racing Algorithms

This repository contains the ROS packages and experiments I built for 16663 F1Tenth Autonomous Racing at Carnegie Mellon University.

These packages are individual control policies and experiments for use on the F1tenth platform, 
an autonomous racing competition utilizing Traxxis RC cars with Nvidia Jetson Nanos for compute
and 2D lidars for primary sensing. 

This repo is meant to be installed alongside the F1tenth Sim, that defines the connection for both
simulation and on-vehicle deployment. 

## What this repo is for

This project is focused on autonomous racing behavior for F1Tenth, including:

- collision avoidance and safety logic
- wall-following navigation
- local planning based on free-space / gap selection
- waypoint tracking and pure pursuit
- path generation and logging tools
- experimental planning and learning-based racing code

## What is in this repo

This repo contains the following ROS packages:

- safety_node — safety controller that monitors the car state and intervenes when the vehicle is in an unsafe condition
- wall_follow — wall-following behavior used to track boundaries and remain inside a drivable corridor
- gap_follow — local planner that evaluates LiDAR observations and chooses a safe gap to drive through
- pure_pursuit_pf — pure pursuit path-tracking controller that follows a waypoint path
- path_csv_node — utility for recording clicked waypoints to CSV for later use by waypoint-following controllers
- pure_pursuit_sim — simulation-oriented version of the pure pursuit stack for testing in the simulator
- f1tenth_rrt_dd — RRT-based planning package used as an experimental planner
- lstm_racer / mapless_lstm_racer — Control policies from ApproxiMPC project. LSTM models trained from MPC
teachers. Designed to perform similar to MPC with lower util. Mapless uses pure Lidar while standard uses 
waypoints and curvature to improve performance.

## What requires external dependencies

This repo depends on external F1Tenth setup in a few specific ways:

- ROS 2 is required for all of the packages here
- Need to install [F1Tenth Gym](https://github.com/f1tenth/f1tenth_gym). This repo
defines the ROS2 harness for F1tenth control policies, can be used for sims and real applications.
- For policies that use map information or waypoints, pure_pursuit, rrt, standard lstm_race,
[Particle Filter](https://github.com/f1tenth/particle_filter.git) also needs to be installed and configured.

## Install the F1Tenth simulator and dependencies

1. Install ROS 2 Humble

2. Install F1tenth Gym

3. Install Particle Filter Package

4. Install Control Policies 

```bash
cd ~/f1tenth_ws/src
git clone <this-repo-url>
```

## Luanching Control Policies

Launch commands:
TODO: List all of them
```bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch safety_node safety_node_launch.py
ros2 launch wall_follow wall_follow_launch.py
ros2 launch gap_follow gap_follow_launch.py
ros2 launch pure_pursuit_pf pure_pursuit_launch.py

#driver is main control policy, logger is for analytics
ros2 launch lstm_racer lstm_drive_launch.py
ros2 launch lstm_racer lstm_logger_launch.py

#driver is main control policy, logger is for analytics
ros2 launch mapless_lstm_racer lstm_drive_launch.py
ros2 launch mapless_lstm_racer lstm_logger_launch.py

#includes control policy, and visualizer 
ros2 launch rrt rrt.launch.py
ros2 launch rrt viz.launch.py
```

### Typical workflow

1. Start the F1Tenth simulator
2. source the workspace
3. launch the algorithm package you want to test
4. tune parameters for the track or obstacle behavior
5. run the car in simulation or on hardware

## Support Package
### path_csv_node

The path_csv_node package is a utility for generating a waypoint file from clicked points.

It subscribes to the /clicked_point topic and logs each clicked location into a CSV file. The output is written to:

```bash
~/waypoints.csv
```

The format is:

```text
x,y
0.0,0.0
1.2,0.4
2.5,1.0
```

This is useful for creating a path for pure pursuit or other waypoint-following controllers. In practical use, you click waypoints in RViz or the simulator UI, save the file, and then use that CSV as the target path for path-following logic.

### F1tenth YOLO Trainer

Not included in this repo, as it is a separate application. It is a project completed for training
and applying a YOLO model for vehicle detection.

[YOLO Trainer Repo](https://github.com/ddecosmo-dev/F1Tenth-YOLO-Trainer)

