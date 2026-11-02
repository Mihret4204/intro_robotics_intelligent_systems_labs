# Lab 1 Exercise: Program diffdrive_bot

This repository root is a self-contained ROS 2 package tree with the four packages required by the
exercise:

- `diffdrive_bot`: copied lab robot, extended with a rear depth camera and an IMU.
- `lab_interfaces`: copied lab message, service and action definitions.
- `my_robot_interfaces`: `RobotStatus`, `SetTurnSpeed` and `Rotate`.
- `my_robot_nodes`: `turn_away`, `rotate_server`, `square_driver` and the square launch file.

## Requirements

Use Ubuntu 22.04 and ROS 2 Humble, with Gazebo Harmonic and the ROS-Gazebo integration, as described
in `diffdrive_bot/README.md`. In each terminal:

```bash
source /opt/ros/humble/setup.bash
```

## Build

Run the build from the repository root, which contains all four packages:

```bash
cd ~/ros2_ws/src/intro_robotics_intelligent_systems_labs
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
```

Install ROS/Gazebo dependencies using the apt instructions in `diffdrive_bot/README.md`; avoid
`rosdep` installing a Gazebo package set that conflicts with Gazebo Harmonic. After changing a
Python node or package metadata, rebuild and source `install/setup.bash` again. The URDF, world,
bridge configuration and launch files use symlink installation, so restart their launch after
editing. The package directories are unique at the repository root, so no `COLCON_IGNORE` file is
necessary.

## Run and check each exercise step

For ROS commands, open a second terminal and source both setup files as above. The checks that
start a node or simulation require Gazebo and the lab dependencies to be installed.

### Step 1: depth camera and IMU

```bash
ros2 launch diffdrive_bot display.launch.py
```

In another sourced terminal:

```bash
ros2 run tf2_ros tf2_echo base_link depth_camera_link
ros2 run tf2_ros tf2_echo base_link imu_link
```

Then start the simulated robot:

```bash
ros2 launch diffdrive_bot gazebo.launch.py
```

Check `/depth_camera/image`, `/depth_camera/points`, `/depth_camera/camera_info`, and `/imu` in
RViz or from another sourced terminal:

```bash
ros2 topic hz /imu
ros2 topic echo /imu
```

Add Image and PointCloud2 displays for `/depth_camera/image` and `/depth_camera/points` in RViz.

### Steps 2–5: obstacle avoidance, parameters, status message and services

Start the simulation and the original lab nodes:

```bash
ros2 launch diffdrive_bot control.launch.py
```

In a second sourced terminal, run the exercise node and enable it:

```bash
ros2 run my_robot_nodes turn_away
ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: true}"
```

To check the adjustable speeds and status:

```bash
ros2 param list /turn_away
ros2 param set /turn_away forward_speed 0.4
ros2 topic echo /robot_status
ros2 interface show my_robot_interfaces/msg/RobotStatus
```

To check its services:

```bash
ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 1.0}"
ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 3.0}"
ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: false}"
```

### Step 6: rotate action server

Start `diffdrive_bot` with `control.launch.py`, then in a second sourced terminal:

```bash
ros2 run my_robot_nodes rotate_server
```

In a third sourced terminal, request a quarter-turn:

```bash
ros2 action send_goal --feedback /rotate my_robot_interfaces/action/Rotate "{angle: 1.57}"
```

### Step 7: square action client and launch file

The launch file starts `control.launch.py`, `rotate_server`, and `square_driver` together:

```bash
ros2 launch my_robot_nodes square.launch.py side_length:=0.8
```

To inspect the available nodes and graph while it runs:

```bash
rqt_graph
```

Only run one `/cmd_vel` producer at a time: leave `/drive_enable` disabled and do not send a
separate `/drive_distance` goal while `turn_away` or `square_driver` is driving.

## Submission notes

The code and packages are present and buildable from this folder. Capture the two requested
screenshots (`RViz` with depth image and point cloud, and `rqt_graph` while the square runs) on a
desktop session with Gazebo/RViz available; a screenshot cannot be truthfully supplied without a
running graphical simulation. In the runtime check, Gazebo started and spawned the robot, but RViz
exited because `/snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0` did not export
`__libc_pthread_init` from `GLIBC_PRIVATE`. This is a host Snap/runtime-library conflict, not a node or URDF build error; resolve the conflicting
library path or use a clean Ubuntu/ROS environment to run RViz. The depth image (`320x240`,
`32FC1`), point cloud, and IMU (`linear_acceleration.z` about `9.8`; about 47 Hz) arrived in ROS.
However, Gazebo did not publish `/depth_camera/camera_info` during this runtime check, despite the
requested topic being set in the URDF and bridged. This sensor-runtime issue needs confirmation in
the target Gazebo installation. The requested screenshots must be captured on a working desktop
session; they are not fabricated here.

To publish this local folder, create an empty GitHub repository, then set its URL as `origin` and
push the branch:

```bash
cd ~/ros2_ws/src/intro_robotics_intelligent_systems_labs
git remote add origin https://github.com/<your-account>/<repository>.git
git push -u origin HEAD
```
