#!/usr/bin/env python3

import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from planar_arm_control.planar_arm import PlanarArm


LINK_LENGTHS = [3.0, 2.0, 1.5]


class ControllerNode(Node):

    def __init__(self):
        super().__init__("controller_node")

        # Parameters
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("trajectory_duration", 3.0)

        self.publish_rate = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.trajectory_duration = float(
            self.get_parameter("trajectory_duration").value
        )

        if self.publish_rate <= 0 or self.trajectory_duration <= 0:
            raise ValueError("Publish rate and trajectory duration must be positive.")

        # Robot model
        self.arm = PlanarArm(LINK_LENGTHS)

        # Joint names
        self.joint_names = ["joint_1", "joint_2", "joint_3"]

        # Current joint position (radians)
        self.current_q = np.zeros(3)
        self.current_velocity = np.zeros(3)

        # Trajectory state
        self.trajectory_active = False
        self.trajectory_start = None
        self.trajectory_start_q = self.current_q.copy()
        self.trajectory_goal_q = self.current_q.copy()

        # ROS publishers
        self.joint_pub = self.create_publisher(
            JointState, "/joint_states", 10
        )

        self.status_pub = self.create_publisher(
            String, "/controller/status", 10
        )

        # ROS subscriber
        self.target_sub = self.create_subscription(
            PointStamped,
            "/target_pose",
            self.target_callback,
            10
        )

        # Timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.control_loop
        )

        self.publish_status("Ready")
        self.publish_joint_state()

        self.get_logger().info(
            "Controller started. Waiting for targets on /target_pose."
        )

    def publish_status(self, message):
        status = String()
        status.data = message
        self.status_pub.publish(status)
        self.get_logger().info(message)

    def target_callback(self, msg):

        target = np.array([msg.point.x, msg.point.y], dtype=float)

        if not np.all(np.isfinite(target)):
            self.publish_status("Rejected: target contains invalid values.")
            return

        # Project targets outside the maximum radial workspace.
        reachable_target = self.arm.reachable_target(target)

        was_projected = not np.allclose(
            target, reachable_target, atol=1e-8
        )

        # Calculate IK from the current joint position.
        try:
            goal_q = np.asarray(
                self.arm.inverse_kinematics(
                    reachable_target,
                    initial_guess=self.current_q
                ),
                dtype=float
            )
        except (ValueError, np.linalg.LinAlgError) as error:
            self.publish_status(f"IK failed: {error}")
            return

        # Validate IK output.
        if goal_q.shape != (3,) or not np.all(np.isfinite(goal_q)):
            self.publish_status("Rejected: IK returned invalid joint angles.")
            return

        if not self.arm.within_joint_limits(goal_q):
            self.publish_status("Rejected: joint limits exceeded.")
            return

        if not self.arm.arm_above_base(goal_q):
            self.publish_status("Rejected: arm violates ground constraint.")
            return

        achieved = np.asarray(self.arm.end_effector(goal_q))
        error = np.linalg.norm(achieved - reachable_target)

        if error > 1e-3:
            self.publish_status(
                f"Rejected: IK position error is too large ({error:.4f})."
            )
            return

        # Validate the complete joint-space trajectory.
        if not self.validate_trajectory(self.current_q, goal_q):
            self.publish_status(
                "Rejected: interpolated trajectory violates constraints."
            )
            return

        # Start a new trajectory from the current arm position.
        self.trajectory_start_q = self.current_q.copy()
        self.trajectory_goal_q = goal_q.copy()
        self.trajectory_start = self.get_clock().now()
        self.trajectory_active = True

        prefix = "Projected target. " if was_projected else ""
        self.publish_status(
            prefix
            + f"Moving to ({reachable_target[0]:.3f}, "
            + f"{reachable_target[1]:.3f})."
        )

    def validate_trajectory(self, start_q, goal_q):

        # Check 101 points along the planned path.
        for u in np.linspace(0.0, 1.0, 101):

            s = 10 * u**3 - 15 * u**4 + 6 * u**5
            q = start_q + s * (goal_q - start_q)

            if not self.arm.within_joint_limits(q):
                return False

            if not self.arm.arm_above_base(q):
                return False

        return True

    def control_loop(self):

        if self.trajectory_active:

            elapsed = (
                self.get_clock().now() - self.trajectory_start
            ).nanoseconds / 1e9

            u = min(elapsed / self.trajectory_duration, 1.0)

            # Quintic time scaling.
            s = 10 * u**3 - 15 * u**4 + 6 * u**5

            ds = (
                30 * u**2
                - 60 * u**3
                + 30 * u**4
            ) / self.trajectory_duration

            delta_q = self.trajectory_goal_q - self.trajectory_start_q

            self.current_q = (
                self.trajectory_start_q + s * delta_q
            )

            self.current_velocity = ds * delta_q

            if u >= 1.0:
                self.current_q = self.trajectory_goal_q.copy()
                self.current_velocity = np.zeros(3)
                self.trajectory_active = False
                self.publish_status("Target reached successfully.")

        self.publish_joint_state()

    def publish_joint_state(self):

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = self.joint_names
        msg.position = self.current_q.tolist()
        msg.velocity = self.current_velocity.tolist()
        msg.effort = []

        self.joint_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = ControllerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
