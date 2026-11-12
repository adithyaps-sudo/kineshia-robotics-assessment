# Kineshia Robotics – 3-DOF Planar Arm Controller

A ROS 2 and PyQt5 application for controlling and visualizing a simulated 3-DOF planar robotic arm.

This project was developed as a take-home assessment for the Robotics Software Engineer (ROS 2 & GUI Systems) role.

## Features

- Inverse kinematics using the provided kinematics library.
- Joint-limit and ground-clearance validation.
- Smooth quintic trajectory interpolation.
- Joint-state publishing at 50 Hz.
- Real-time robotic arm visualization.
- Live joint-angle and end-effector telemetry.
- Manual Cartesian target control.
- Simulated pick-and-place sequence.
- Concurrent ROS 2 and Qt event-loop execution.
- ROS 2 launch-based startup.

## Robot specifications

| Property | Value |
|---|---|
| Degrees of freedom | 3 |
| Link lengths | 3.0, 2.0, 1.5 |
| Joint 1 limits | 0° to 180° |
| Joint 2 limits | -120° to 120° |
| Joint 3 limits | -120° to 120° |
| Ground constraint | No arm link below y = 0 |

The supplied `planar_arm.py` kinematics library is used without modification.

## Project structure

```text
kineshia_ws/
├── README.md
└── src/
    └── planar_arm_control/
        ├── launch/
        │   └── bringup.launch.py
        ├── planar_arm_control/
        │   ├── __init__.py
        │   ├── planar_arm.py
        │   ├── controller_node.py
        │   └── gui_node.py
        ├── resource/
        │   └── planar_arm_control
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

## Software requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- NumPy
- PyQt5
- PyQtGraph

Install the required Python GUI dependencies:

```bash
sudo apt update
sudo apt install python3-pyqt5 python3-pyqtgraph python3-numpy
```

## Build

From the workspace root:

```bash
cd ~/kineshia_ws

source /opt/ros/jazzy/setup.bash

colcon build --packages-select planar_arm_control --symlink-install

source install/setup.bash
```

## Run

Launch the controller and GUI together:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kineshia_ws/install/setup.bash

ros2 launch planar_arm_control bringup.launch.py
```

The launch file starts both ROS 2 nodes and configures the Qt `xcb` platform backend for desktop compatibility.

## ROS 2 interfaces

| Topic | Message type | Direction | Description |
|---|---|---|---|
| `/target_pose` | `geometry_msgs/PointStamped` | GUI → Controller | Cartesian target position |
| `/joint_states` | `sensor_msgs/JointState` | Controller → GUI | Current joint positions and velocities |
| `/controller/status` | `std_msgs/String` | Controller → GUI | Motion and controller status |

The controller publishes joint states at 50 Hz.

## Controller design

The controller receives Cartesian targets through `/target_pose` and calculates joint configurations using the supplied inverse-kinematics implementation.

Before executing a movement, it validates:

- Numerical validity of the IK solution.
- Joint-angle limits.
- Ground-clearance constraints.
- End-effector position accuracy.
- Intermediate configurations along the planned trajectory.

Targets outside the maximum radial workspace are projected onto the workspace boundary.

A quintic time-scaling function is used to generate smooth joint-space trajectories:

```text
s(u) = 10u³ - 15u⁴ + 6u⁵
```

The trajectory runs for 3 seconds by default. Joint positions and velocities are published through `/joint_states`.

If an IK solution or its trajectory violates the constraints, the controller rejects the movement and publishes a status message.

## GUI design

The PyQt5 and PyQtGraph interface provides:

- Live 2D robotic arm visualization.
- Cartesian target input.
- Manual target commands.
- Pick and place controls.
- Automated pick-and-place sequence.
- Real-time joint-angle plots.
- End-effector X/Y position telemetry.
- Controller status display.

The ROS 2 executor runs in a background Python thread, while the Qt event loop runs in the main thread. This allows the GUI to remain responsive while receiving ROS 2 messages.

## Pick-and-place demonstration

The GUI provides a simulated pick-and-place sequence:

1. Move the arm to the pick target `(4, 2)`.
2. Simulate object attachment.
3. Move the arm to the place target `(-3, 3)`.
4. Simulate object release.
5. Display sequence completion.

The pick-and-place behavior is simulated in software. No physical gripper or object manipulation hardware is connected.

## Validation and test results

The following targets were tested using the running ROS 2 application:

| Target | Observed behavior | Result |
|---|---|---|
| `(4, 2)` | Arm reached the pick position | Passed |
| `(-3, 3)` | Arm reached the place position | Passed |
| `(7, 3)` | Projected to approximately `(5.974, 2.560)` and reached | Passed |
| `(1, 1)` | Rejected because the returned IK configuration did not satisfy joint limits | Rejected as expected |
| `(0, -2)` | Rejected by constraint validation | Rejected as expected |

The edge-target test demonstrates workspace-boundary projection. Invalid configurations are prevented from being executed.

## Simulation and hardware integration

This implementation is a software simulation. The controller and GUI communicate through ROS 2 topics and do not directly command physical motors.

The controller is separated from the GUI through ROS 2 interfaces. This makes it possible to introduce a hardware interface without redesigning the GUI.

For a physical robot, a hardware interface would:

1. Receive desired joint positions from the controller.
2. Convert joint commands into motor-driver signals.
3. Communicate with motor drivers using an appropriate interface.
4. Read encoder or sensor feedback.
5. Publish measured joint positions through `/joint_states`.

A real hardware implementation would also require calibration, feedback control, safety limits, emergency-stop handling, and hardware-specific testing.

## Notes

- The supplied `planar_arm.py` file has not been modified.
- The application is designed for simulated operation.
- Pick-and-place represents simulated object attachment and release.
- Physical motor control and gripper operation are outside the scope of this implementation.
