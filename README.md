# Introduction to Robotics and Intelligent Systems Labs

ROS 2 Humble and Gazebo Harmonic material for the BSc in Software Engineering course
**Introduction to Robotics and Intelligent Systems**, academic year 2026.

The completed Lab 1 workspace is organized directly in this repository. Its four ROS packages are
at the top level, so there is one discoverable copy of each package and no `COLCON_IGNORE` marker
is needed:

| Package | Purpose |
|---------|---------|
| [diffdrive_bot](diffdrive_bot) | Differential-drive robot description, Gazebo world, bridge configuration, and lab nodes |
| [lab_interfaces](lab_interfaces) | Lab obstacle message, speed service, and drive-distance action |
| [my_robot_interfaces](my_robot_interfaces) | Exercise robot-status message, turn-speed service, and rotate action |
| [my_robot_nodes](my_robot_nodes) | Exercise obstacle node, rotate action server, square action client, and launch file |

## Requirements

- Ubuntu 22.04 with ROS 2 Humble
- Gazebo Harmonic and ROS-Gazebo integration, as described in
  [diffdrive_bot/README.md](diffdrive_bot/README.md#what-you-need)

## Build and test from this repository

```bash
cd ~/ros2_ws/src/intro_robotics_intelligent_systems_labs
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
```

For a conventional workspace where this repository is checked out under `~/ros2_ws/src`, run
`colcon build --symlink-install` from `~/ros2_ws` instead. Do not put another copy of these packages
beside this repository in the same workspace.

## Guides

- [Lab 1 overview and robot tutorial](LAB_01_README.md)
- [Original exercise specification](EXERCISE.md)
- [Completed exercise build and run commands](EXERCISE_README.md)
- [Robot package guide](diffdrive_bot/README.md)

## Runtime notes

The code builds and the square-drive simulation was verified. On the machine used to prepare this
workspace, RViz failed to start because a Snap `libpthread.so.0` conflicted with the ROS system
libraries, and Gazebo did not publish `/depth_camera/camera_info` during the live check. See the
completed exercise guide for the observed details.

## License

The packages are released under the MIT license; see [LICENSE](LICENSE).
