# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Workspace for **Martha**, AndesRobot's holonomic base for RoboCup@Home 2027: 4 mecanum wheels,
RPLIDAR A2M8, IMU, ESP32 with 4 motor drivers, an aluminium tower, a laptop on top. Shared by the
**Navigation** and **Structure** teams. ROS 2 **Humble**, Gazebo **Classic 11**, everything in Docker
(`docker/`, image `martha:humble`, container `martha`); `ros2_ws/` is bind-mounted as `/ros2_ws`.

It mirrors `alejapinzonf/milo_ws` (Milo's workspace) on purpose: same `andesrobot_*` package names,
same Docker image plus Martha's extras, same `sim.sh` interface. The goal is to merge both robots
later into one codebase selected with `robot:=milo|martha`, so Martha-specific files are named
`martha_*` inside the shared package names. The team architecture and the interface contract
(topic names, types, owners) are at https://alejapinzonf.github.io/Robot_arquitecture_ROS2/ — follow
it: Navigation never touches motors, it publishes `/cmd_vel` and reads `/base/odom`.

The PPO local planner and its training live in a separate thesis repo (`Mocolax/martha_nav`). Do not
add torch / stable-baselines3 to this image; a policy enters here only as an exported `.npz` run with
numpy.

## Conventions

- Docs, comments and log messages in **Spanish**, written for students, with dense explanatory
  comments (match `milo_ws`). `README.md` is step-by-step for non-experts.
- Contract names from day one: `/cmd_vel` (Twist with `linear.y`), `/base/odom` + TF
  `odom -> base_footprint`, `/base/imu`, `/nav/scan` (frame `laser`). `base_footprint` is the URDF root.
- **Never write a colon followed by a space in a `.xacro` file, comments included.** With
  `drive:=mecanum`, `gazebo_ros2_control` parses the expanded URDF as YAML and fails silently.
  `andesrobot_description/test/test_urdf.py` checks it.
- Python within 99 columns. No ament_lint tests; pytest only.

## Commands

All from `~/martha_ws` on the host:

```bash
./sim.sh            # Gazebo (RoboCup house) + Martha (planar_move) + RViz
./sim.sh teleop     # keyboard -> /cmd_vel (Shift = sideways)
./sim.sh modelo     # URDF in RViz only
./sim.sh pruebas    # pytest of every package with a test/ folder
./sim.sh mecanum    # wheels with rollers via ros2_control (no /cmd_vel bridge yet)
./sim.sh shell | build | stop | cpu | software | sin-gazebo | gpu
```

**Never run `colcon build` natively inside `~/martha_ws/ros2_ws`**: the container reuses
`ros2_ws/install/` and host-built paths break it.

## Layout

| Package | Role |
|---|---|
| `andesrobot_description` | `martha.urdf.xacro` (geometry; args `drive`, `tower`, `sim`, `lidar_samples`), `martha.sim.xacro` (Gazebo plugins with contract topics), `config/martha_controllers.yaml`, `rsp`/`display` launches |
| `andesrobot_gazebo` | `sim.launch.py`, `worlds/robocup_casa.world` (copied from milo_ws, generated there by `tools/gen_casa_robocup.py`; spawn outside the living-room door at x=-7.2 y=-1.862) |
