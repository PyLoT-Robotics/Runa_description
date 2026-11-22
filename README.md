# Runa_description

ROS 2 description of **Runa**, PyLoT Robotics' mobile manipulator.
ROS package: `runa_description`. Target: Ubuntu 24.04 / ROS 2 Jazzy.

The model includes a differential-drive base, pan/tilt camera head, five-axis arm,
parallel gripper, lift and sensor frames. It is a **primitive geometric model**,
not a detailed CAD/mesh release. Existing mounting transforms and arm joint limits
are retained; base/caster geometry, masses and inertias are simulation approximations.
Do not use this model as a calibrated dynamics or manufacturing specification.

## Install and display

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
mkdir -p ~/runa_ws/src
cd ~/runa_ws/src
git clone https://github.com/PyLoT-Robotics/Runa_description.git
cd ~/runa_ws
colcon build --symlink-install --packages-select runa_description
source install/setup.bash
ros2 launch runa_description display.launch.py
```

Use `gui:=false rviz:=false` for robot_state_publisher only; in that mode another
node must supply joint states. Display launch never starts a serial driver.

## Frames and joints

| Component | Names / units |
|---|---|
| Root / body | `base_footprint` → `base_link` |
| Drive | `left_wheel_joint`, `right_wheel_joint`; radius 0.05 m, track 0.39 m |
| Head | `base_to_pan_joint` (+Z), `pan_to_tilt_joint` (+Y); radians |
| Arm | `joint1`–`joint5`; radians |
| Gripper | `gripper_left_joint`, `gripper_right_joint` (mimic); 0–0.025 m per finger |
| Lift | `lift_joint`; 0–0.18 m |
| Sensors | `laser`, `livox_frame`, `bno055_link`, `camera_link`, `d415_link` |
| Optical | `d455_color_optical_frame`, `d415_color_optical_frame` |
| Tool | `tcp` |

The base frame is at ground level. The source of geometry/joint limits is
`urdf/runa.urdf.xacro` and `urdf/arm.xacro`. No model weights, private application
code, credentials, or robot-specific serial configuration are included.

## Gazebo Harmonic

Use [Runa_sim](https://github.com/PyLoT-Robotics/Runa_sim) for simulation.
Its launch expands this model with `sim:=true` and an absolute `controllers_file`.
Default `sim:=false` keeps this package independent of Gazebo and ros2_control.
The simulator supplies physical controllers, RGB-D/LiDAR/IMU sensors and ROS bridges.

## Validation

After building and sourcing the workspace:

```bash
python3 -m pytest -q src/Runa_description/test
xacro src/Runa_description/urdf/runa.urdf.xacro > /tmp/runa.urdf
```

Tests check a single connected tree, actuator limits, positive inertias, mimic
joints, and absence of hardware plugins. They do not establish real-world dimensions
or dynamics accuracy.

## License

[Apache-2.0](LICENSE).

## Acknowledgements

Thanks to PyLoT Robotics contributors for Runa's mechanical and ROS work, and to
the ROS / Gazebo communities. The companion simulation uses the
[PyLoT fork of SOBITS Gazebo Worlds](https://github.com/PyLoT-Robotics/sobits_gazebo_worlds),
originally developed by [TeamSOBITS](https://github.com/TeamSOBITS/sobits_gazebo_worlds)
under BSD-3-Clause. World assets remain in that dependency and retain their own terms.
