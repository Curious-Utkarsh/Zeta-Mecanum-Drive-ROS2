# Zeta — Mecanum-Drive Robot in ROS 2 (Gazebo)

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Status](https://img.shields.io/badge/status-early%20prototype-lightgrey)](#status)

A four-wheel mecanum robot modelled in URDF and simulated in Gazebo, with my own mecanum kinematics controller: it converts body velocity commands into four wheel speeds and integrates wheel motion back into odometry and a TF transform.

## Key features

- URDF/Xacro model with ros2_control (one velocity controller per wheel)
- Python mecanum controller: inverse kinematics (`cmd_vel` → wheel speeds) and forward kinematics (wheels → `odom` + TF)
- Joystick teleoperation through `twist_mux`; warehouse world

## Run

```bash
ros2 launch zeta_description gazebo.launch.py
ros2 launch zeta_controller controller.launch.py
ros2 launch zeta_controller joystick.launch.py
```

## Status

Early prototype (Jan 2025): model, controller and teleop are in place; navigation was not added.

## Related

[RF holonomic drive](https://github.com/Curious-Utkarsh/rf-mecanum-drive-robot) — an earlier hardware mecanum robot · [Aura](https://github.com/Curious-Utkarsh/aura-mobile-manipulator) — built in the same month with the same package layout

---
[Kumar Utkarsh](https://github.com/Curious-Utkarsh) · [LinkedIn](https://www.linkedin.com/in/kumar-utkarsh-124aab227/)
