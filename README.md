# SO-ARM101 MoveIt 2 & Isaac Sim

ROS 2 workspace for controlling **SO-ARM101** with **MoveIt 2**, using either a physical robot or an Isaac Sim backend.

This repository contains the robot description, MoveIt configuration, custom `ros2_control` hardware interface, Isaac Sim topic-based control configuration, and a simple joint-control GUI.

---

## Packages

```text
soarm_moveit_isaacsim/
└── src/
    ├── so_arm_description/
    ├── so_arm_hardware/
    ├── so_arm_moveit_config/
    └── so_arm_gui_control/
```

### `so_arm_description`

SO-ARM101 robot description package.

Contains:

- URDF / Xacro
- meshes
- real-hardware `ros2_control` description
- Isaac Sim `ros2_control` description
- Isaac Sim USD assets

---

### `so_arm_hardware`

Custom `ros2_control` hardware interface for the physical SO-ARM101.

The hardware plugin communicates with the servo bus and converts between:

```text
ROS Joint Position [rad]
        ↕
Servo Motor Position
```

It provides position command/state interfaces used by `ros2_control` and MoveIt 2.

---

### `so_arm_moveit_config`

MoveIt 2 configuration for SO-ARM101.

Includes:

- SRDF
- kinematics configuration
- joint limits
- ros2 controller configuration
- RViz configuration
- real-hardware launch
- Isaac Sim launch

Two launch configurations are provided:

```text
demo_real.launch.py
```

for the physical robot, and

```text
demo_sim.launch.py
```

for Isaac Sim.

---

### `so_arm_gui_control`

Simple PyQt-based joint-control GUI.

The GUI:

- displays current joint positions from `/joint_states`
- provides joint sliders
- sends trajectories to the arm and gripper controllers

---

## Control Modes

### Physical SO-ARM101

In real-hardware mode:

```text
MoveIt 2
   ↓
ros2_control
   ↓
so_arm_hardware/SoArmSystem
   ↓
SO-ARM101
```

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch so_arm_moveit_config demo_real.launch.py
```

The current hardware configuration uses:

```text
Port     : /dev/so101_follower
Baudrate : 1000000
```

---

## Isaac Sim

In simulation mode, MoveIt 2 communicates with Isaac Sim through ROS 2 topics using `topic_based_ros2_control`.

```text
MoveIt 2
   ↓
ros2_control
   ↓
ROS 2 Topics
   ↓
Isaac Sim
```

### ROS 2 Topics

Isaac Sim should receive joint commands from:

```text
/isaac_joint_commands
```

and publish the current robot joint states to:

```text
/isaac_joint_states
```

The basic communication direction is:

```text
MoveIt / ros2_control
        ↓
/isaac_joint_commands
        ↓
Isaac Sim SO-ARM101

Isaac Sim SO-ARM101
        ↓
/isaac_joint_states
        ↓
ros2_control / MoveIt
```

Start Isaac Sim with the ROS 2 Bridge enabled and configure the SO-ARM101 articulation to use these topics.

Then run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch so_arm_moveit_config demo_sim.launch.py
```

---

## GUI

Run the joint-control GUI with:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run so_arm_gui_control gui
```

The GUI uses:

```text
State:
  /joint_states

Commands:
  /arm_controller/joint_trajectory
  /gripper_controller/joint_trajectory
```

---

## Build

### Requirements

- ROS 2 Humble
- MoveIt 2
- ros2_control / ros2_controllers
- `topic_based_ros2_control`
- PyQt5
- Isaac Sim with ROS 2 Bridge
- SCServo SDK for physical SO-ARM101 control

Clone and build:

```bash
git clone https://github.com/HYEONHEE5739/soarm_moveit_isaacsim.git
cd soarm_moveit_isaacsim

source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

For physical hardware control, the current package expects the SCServo SDK at:

```text
~/SCServo_Linux
```


---

## Summary

This repository provides the packages required to use **SO-ARM101 with MoveIt 2** in both physical and Isaac Sim environments.

- `so_arm_description` — robot description
- `so_arm_hardware` — physical `ros2_control` hardware interface
- `so_arm_moveit_config` — MoveIt 2 configuration and launch files
- `so_arm_gui_control` — simple joint-control GUI

For Isaac Sim integration, use:

```text
Command : /isaac_joint_commands
State   : /isaac_joint_states
```
