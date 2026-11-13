# Technical Design Note
## 3-DoF Planar Robot Arm — ROS 2 Control and GUI

### 1. System Architecture

The application is developed using ROS 2 Jazzy and Python. It consists of two main components: a controller node and a PyQt5/PyQtGraph graphical user interface.

The controller node receives target end-effector positions through the `/target_pose` topic and generates joint-space trajectories. It publishes joint positions through `/joint_states` at 50 Hz. Controller status is communicated through `/controller/status`.

The GUI subscribes to joint states and controller status, displays the robot arm in real time, and allows the operator to send target positions and execute an automatic pick-and-place sequence.

### 2. Inverse Kinematics and Constraints

The provided `planar_arm.py` library is used for forward and inverse kinematics without modification.

Before executing a target, the controller validates the resulting joint configuration against the specified joint limits and ground constraint. Unreachable targets are handled by workspace projection where applicable, while invalid configurations are rejected.

### 3. Trajectory Planning

A quintic time-scaling trajectory is used to generate smooth joint-space motion between the current and target configurations.

The interpolation function is:

s(u) = 10u³ − 15u⁴ + 6u⁵

where u is normalized time between zero and one.

This provides smooth position, velocity, and acceleration transitions at the trajectory endpoints. The controller publishes the interpolated joint positions at a fixed rate of 50 Hz.

### 4. GUI and Concurrent Execution

The operator interface is developed using PyQt5 and PyQtGraph. It displays the robot arm, end-effector coordinates, joint angles, target positions, controller status, and real-time joint-angle plots.

The ROS 2 executor runs in a separate background thread, while the Qt event loop runs in the main thread. This allows the GUI to remain responsive while receiving ROS feedback and updating the visualization.

### 5. Pick-and-Place Demonstration

The automated sequence moves the arm to the pickup position (4.0, 2.0), simulates gripping an object, transports it to the placement position (-3.0, 3.0), and simulates releasing it.

The GUI visualizes the cube attachment and movement using the end-effector position calculated through forward kinematics. The gripper animation is simulated; no physical gripper is connected.

The edge target (7.0, 3.0) is also tested to demonstrate reachable-workspace handling.

### 6. Design Decisions and Limitations

The controller and GUI are separated into independent ROS 2 nodes to improve modularity. The supplied kinematics library is kept unchanged, and the trajectory generation is separated from the visualization.

The current implementation is simulation-only. A physical robot deployment would require a hardware driver, encoder feedback, motor control, calibration, communication handling, and safety mechanisms. The existing target and trajectory interfaces provide a basis for integrating such a backend, but the hardware interface has not yet been implemented or tested.
