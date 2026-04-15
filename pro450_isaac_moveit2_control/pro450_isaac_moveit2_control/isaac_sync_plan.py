#!/usr/bin/env python3
"""Bridge MoveIt FollowJointTrajectory actions to Isaac joint commands."""

import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

MIN_REQUIRE_VERSION = "4.0.1"
JOINT_ORDER = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class SyncPlanBridge(Node):
    """Accept MoveIt trajectories and forward them to Isaac and the real robot."""

    def __init__(self):
        super().__init__("sync_plan_bridge")

        self.declare_parameter("action_name", "/arm_group_controller/follow_joint_trajectory")
        self.declare_parameter("command_topic", "/joint_command")
        self.declare_parameter("sync_real_robot", True)
        self.declare_parameter("ip", "192.168.0.232")
        self.declare_parameter("port", 4500)
        self.declare_parameter("robot_speed", 50)

        self.action_name = self.get_parameter("action_name").value
        self.command_topic = self.get_parameter("command_topic").value
        self.sync_real_robot = bool(self.get_parameter("sync_real_robot").value)
        self.robot_speed = int(self.get_parameter("robot_speed").value)
        self.previous_positions = [0.0] * len(JOINT_ORDER)

        self.command_publisher = self.create_publisher(JointState, self.command_topic, 10)
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.mycobot_450 = None
        if self.sync_real_robot:
            self._connect_robot()

        self.get_logger().info(
            f"Bridge ready: action={self.action_name}, joint_command={self.command_topic}, "
            f"sync_real_robot={self.sync_real_robot}"
        )

    def _connect_robot(self):
        """Connect to the physical Pro450 only when hardware sync is enabled."""
        import pymycobot
        from packaging import version

        current_version = pymycobot.__version__
        self.get_logger().info(f"current pymycobot library version: {current_version}")
        if version.parse(current_version) < version.parse(MIN_REQUIRE_VERSION):
            raise RuntimeError(
                "The version of pymycobot library must be greater than "
                f"{MIN_REQUIRE_VERSION}. Current version: {current_version}"
            )

        from pymycobot import Pro450Client

        ip = self.get_parameter("ip").value
        port = int(self.get_parameter("port").value)
        self.get_logger().info(f"Connecting Pro450: ip={ip}, port={port}")
        self.mycobot_450 = Pro450Client(ip, port)
        time.sleep(0.05)
        if self.mycobot_450.is_power_on() != 1:
            self.mycobot_450.power_on()
        time.sleep(0.05)
        if self.mycobot_450.get_fresh_mode() != 1:
            self.mycobot_450.set_fresh_mode(1)
        time.sleep(0.05)
        self.mycobot_450.set_limit_switch(2, 0)

    def goal_callback(self, goal_request):
        """Reject empty trajectories early so MoveIt gets a clear error."""
        if not goal_request.trajectory.points:
            self.get_logger().warning("Rejected empty trajectory goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        """Allow MoveIt to cancel an executing trajectory."""
        self.get_logger().info("Received trajectory cancel request")
        return CancelResponse.ACCEPT

    def _ordered_positions(self, joint_names, positions):
        """Reorder incoming joint arrays into the Isaac/robot joint order."""
        joint_map = {
            name: positions[index]
            for index, name in enumerate(joint_names)
            if index < len(positions)
        }
        ordered = []
        for index, joint_name in enumerate(JOINT_ORDER):
            ordered.append(joint_map.get(joint_name, self.previous_positions[index]))
        return ordered

    def _publish_joint_command(self, positions):
        """Publish one JointState command for Isaac and optionally mirror to hardware."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_ORDER)
        msg.position = list(positions)
        msg.velocity = []
        msg.effort = []
        self.command_publisher.publish(msg)
        self.previous_positions = list(positions)

        if self.mycobot_450 is not None:
            robot_angles = [round(math.degrees(value), 2) for value in positions]
            self.mycobot_450.send_angles(robot_angles, self.robot_speed)

    def _make_feedback_point(self, positions):
        feedback_point = JointTrajectoryPoint()
        feedback_point.positions = list(positions)
        return feedback_point

    def execute_callback(self, goal_handle):
        """Execute a MoveIt trajectory point-by-point against Isaac."""
        trajectory = goal_handle.request.trajectory
        trajectory_names = list(trajectory.joint_names) or list(JOINT_ORDER)
        start_time = time.monotonic()

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(JOINT_ORDER)

        for point in trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = getattr(result, "SUCCESSFUL", 0)
                result.error_string = "Trajectory canceled"
                return result

            target_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            while time.monotonic() - start_time < target_time:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = FollowJointTrajectory.Result()
                    result.error_code = getattr(result, "SUCCESSFUL", 0)
                    result.error_string = "Trajectory canceled"
                    return result
                time.sleep(0.002)

            ordered_positions = self._ordered_positions(trajectory_names, point.positions)
            self._publish_joint_command(ordered_positions)

            feedback.desired = point
            feedback.actual = self._make_feedback_point(ordered_positions)
            feedback.error = JointTrajectoryPoint()
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = getattr(result, "SUCCESSFUL", 0)
        result.error_string = ""
        self.get_logger().info("Trajectory execution forwarded to Isaac successfully")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SyncPlanBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
