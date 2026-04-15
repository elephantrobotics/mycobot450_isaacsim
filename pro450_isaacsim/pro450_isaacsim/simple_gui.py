#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import queue
import sys
import threading
import tkinter as tk
import time
from tkinter import messagebox
import pymycobot
from packaging import version

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


MIN_REQUIRE_VERSION = '4.0.1'
CURRENT_VERSION = pymycobot.__version__
print(f'current pymycobot library version: {CURRENT_VERSION}')
if version.parse(CURRENT_VERSION) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        f'The version of pymycobot library must be greater than {MIN_REQUIRE_VERSION} or higher. '
        f'The current version is {CURRENT_VERSION}. Please upgrade the library version.'
    )
print('pymycobot library version meets the requirements!')
from pymycobot import Pro450Client
from pymycobot.robot_info import RobotLimit

class WindowNode(Node):
    """ROS2 node with Tkinter GUI interface for controlling a MyCobot robotic arm.

    This node provides services to send joint angles, coordinates, and gripper
    commands to the robot, as well as to query the current robot state. It also
    integrates a Tkinter GUI for user input and displays robot data in real time.
    """

    def __init__(self, handle):
        """Initialize the WindowNode, ROS2 service clients, and GUI.

        Args:
            handle (tk.Tk): Tkinter window instance to attach the GUI.
        """
        super().__init__('simple_gui')

        self.declare_parameter('ip', '192.168.0.232')
        self.declare_parameter('port', 4500)
        self.declare_parameter('sync_isaac', True)
        self.declare_parameter('command_topic', '/joint_command')

        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        self.sync_isaac = self.get_parameter(
            "sync_isaac"
        ).get_parameter_value().bool_value
        self.command_topic = self.get_parameter(
            "command_topic"
        ).get_parameter_value().string_value

        self.get_logger().info(f"Connecting Pro450 directly: ip={ip}, port={port}")
        self.mycobot_450 = Pro450Client(ip, port)
        time.sleep(0.05)
        if self.mycobot_450.is_power_on() != 1:
            self.mycobot_450.power_on()
        time.sleep(0.05)
        # Prefer fresh mode for teleop so the latest command wins.
        if self.mycobot_450.get_fresh_mode() != 0:
            self.mycobot_450.set_fresh_mode(0)
        time.sleep(0.05)
        self.mycobot_450.set_limit_switch(2, 0)

        self.command_pub = None
        if self.sync_isaac:
            self.command_pub = self.create_publisher(JointState, self.command_topic, 10)

        # Tkinter window setup
        self.win = handle
        self.win.resizable(0, 0)  # Fixed window size

        self.speed = 50

        # Default speed variable
        self.speed_d = tk.StringVar()
        self.speed_d.set(str(self.speed))

        # Robotic arm data
        self.record_coords = [
            [0, 0, 0, 0, 0, 0],
            self.speed
        ]
        self.res_angles = [
            [0, 0, 0, 0, 0, 0],
            self.speed
        ]
        # Command queue and background worker
        self.cmd_queue = queue.Queue()
        threading.Thread(target=self.worker, daemon=True).start()

        # self.get_date()  # Initialize data from the robot
        self.record_coords[0] = self.get_initial_coords()
        self.res_angles[0] = self.get_initial_angles()

        # Screen dimensions
        self.ws = self.win.winfo_screenwidth()
        self.hs = self.win.winfo_screenheight()

        # Calculate window position
        x = (self.ws / 2) - 190
        y = (self.hs / 2) - 250
        self.win.geometry("470x440+{}+{}".format(int(x), int(y)))

        # GUI layout and widgets
        self.set_layout()
        self.need_input()
        self.show_init()

        # Buttons for joint settings
        tk.Button(self.frmLT, text="Set Joints", width=10, command=self.get_joint_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )

        # Buttons for coordinate settings
        tk.Button(self.frmRT, text="Set Coords", width=10, command=self.get_coord_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )

        # Periodic GUI update
        self.update_gui()

        # Robot model and limits
        self.robot_name = "Pro450Client"
        self.angles_min = RobotLimit.robot_limit[self.robot_name]["angles_min"]
        self.angles_max = RobotLimit.robot_limit[self.robot_name]["angles_max"]
        self.coords_min = RobotLimit.robot_limit[self.robot_name]["coords_min"]
        self.coords_max = RobotLimit.robot_limit[self.robot_name]["coords_max"]

    def worker(self):
        """Background thread that executes queued robot commands."""
        while rclpy.ok():
            try:
                cmd = self.cmd_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                if cmd[0] == "angles":
                    _, values, speed = cmd
                    self.speed = speed
                    self.send_angles(values)

                elif cmd[0] == "coords":
                    _, values, speed = cmd
                    self.record_coords[0] = values
                    self.record_coords[1] = speed
                    self.send_coords()

                elif cmd[0] == "gripper":
                    _, state = cmd
                    self.set_force_gripper(state)

            except Exception as e:
                self.get_logger().warn(f"worker error: {e}")

    def publish_joint_command(self, angles_deg):
        """Mirror a 6-axis target to Isaac as radians."""
        if self.command_pub is None:
            return

        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        command.position = [math.radians(value) for value in angles_deg[:6]]
        command.velocity = []
        command.effort = []
        self.command_pub.publish(command)

    def sync_isaac_from_robot(self):
        """Read back actual robot angles and mirror them to Isaac."""
        angles = self.get_initial_angles()
        if all(value != -1 for value in angles):
            self.publish_joint_command(angles)

    def get_initial_coords(self):
        """Fetch current coordinates from the robot.

        Returns:
            list: [[x, y, z, rx, ry, rz], speed]
        """
        coords = self.mycobot_450.get_coords()
        if coords and len(coords) == 6 and all(value != -1 for value in coords):
            return coords
        else:
            self.get_logger().error('Failed to get coordinates')
            return [-1, -1, -1, -1, -1, -1]

    def get_initial_angles(self):
        """Fetch current joint angles from the robot.

        Returns:
            list: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        """
        angles = self.mycobot_450.get_angles()
        if angles and len(angles) == 6 and all(value != -1 for value in angles):
            return angles
        else:
            self.get_logger().error("Failed to get angles")
            return [-1, -1, -1, -1, -1, -1]

    def send_coords(self):
        """Send coordinates to the robot, ensuring they are within limits."""
        coords = self.record_coords[0]
        try:
            angles = self.get_initial_angles()
            if all(value != -1 for value in angles):
                target_angles = self.mycobot_450.solve_inv_kinematics(coords, angles)
            # self.get_logger().info(f'angles: {target_angles}')
            self.mycobot_450.send_coords(coords, self.record_coords[1])
    
            self.publish_joint_command(target_angles)
        except Exception:
            self.get_logger().error('Failed to set coordinates')

    def send_angles(self, angles):
        """Send joint angles to the robot.

        Args:
            angles (list): List of 6 joint angle values.
        """
        try:
            self.mycobot_450.send_angles(angles, self.speed)
            self.publish_joint_command(angles)
        except Exception:
            self.get_logger().error('Failed to set angles')
    
    def set_layout(self):
        """Set the interface layout"""
        self.frmLT = tk.Frame(width=200, height=200)
        self.frmLC = tk.Frame(width=200, height=200)
        self.frmLB = tk.Frame(width=200, height=200)
        self.frmRT = tk.Frame(width=200, height=200)
        self.frmLT.grid(row=0, column=0, padx=1, pady=3)
        self.frmLC.grid(row=1, column=0, padx=1, pady=3)
        self.frmLB.grid(row=1, column=1, padx=2, pady=3)
        self.frmRT.grid(row=0, column=1, padx=2, pady=3)

    def need_input(self):
        """Display input fields for joint angles and robot coordinates."""

        # Joint labels and input variables
        joint_names = ["Joint 1", "Joint 2",
                       "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
        self.all_j = []
        self.joint_vars = []

        for i, name in enumerate(joint_names):
            tk.Label(self.frmLT, text=name).grid(row=i)
            var = tk.StringVar()
            var.set(self.res_angles[0][i])
            self.joint_vars.append(var)
            entry = tk.Entry(self.frmLT, textvariable=var)
            entry.grid(row=i, column=1, pady=3)
            self.all_j.append(entry)

        # Coordinate labels and input variables
        coord_names = ["x", "y", "z", "rx", "ry", "rz"]
        self.all_c = []
        self.coord_vars = []

        for i, name in enumerate(coord_names):
            tk.Label(self.frmRT, text=f" {name} ").grid(row=i)
            var = tk.StringVar()
            var.set(self.record_coords[0][i])
            self.coord_vars.append(var)
            entry = tk.Entry(self.frmRT, textvariable=var)
            entry.grid(row=i, column=1, pady=3, padx=0)
            self.all_c.append(entry)

        # Speed input
        tk.Label(self.frmLB, text="speed").grid(row=0, column=0)
        self.get_speed = tk.Entry(
            self.frmLB, textvariable=self.speed_d, width=10)
        self.get_speed.grid(row=0, column=1)

    def safe_get_angle(self, angle_list, index, default="-1°"):
        """Safely get an angle from a nested list.

        This method attempts to retrieve an angle value from `angle_list[0][index]`.
        If the list is empty, the index is out of range, or an error occurs,
        it returns a default value.

        Args:
            angle_list (list[list[float]]): Nested list of angles.
            index (int): Index of the angle to retrieve.
            default (str, optional): Default value to return if retrieval fails. Defaults to "-1°".

        Returns:
            str: The angle as a string with a degree symbol, or the default value.
        """
        try:
            if angle_list and len(angle_list) > index:
                return "{}°".format(round(angle_list[index], 2))
        except Exception as e:
            self.get_logger().warn("safe_get_angle error: {}".format(e))
        return default

    def safe_get_coord(self, coords_list, index, default="0.0"):
        """Safely get a coordinate from a nested list.

        This method attempts to retrieve a coordinate value from `coords_list[0][index]`.
        If the list is empty, the index is out of range, the value is -1, or an error occurs,
        it returns a default value.

        Args:
            coords_list (list[list[float]]): Nested list of coordinates.
            index (int): Index of the coordinate to retrieve.
            default (str, optional): Default value to return if retrieval fails. Defaults to "0.0".

        Returns:
            str: The coordinate as a string, or the default value.
        """
        try:
            # self.get_logger().info("safe_get_coord: {}".format(coords_list))
            if coords_list and len(coords_list) > 0:
                value = coords_list[index]
                if value != -1:
                    return str(round(value, 2))
        except Exception as e:
            self.get_logger().warn("safe_get_coord error: {}".format(e))
        return default

    def show_init(self):
        """Display the robot arm joint angles and coordinate data in the GUI."""

        # Joint labels
        joint_names = ["Joint 1", "Joint 2",
                       "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
        self.cont_all = []
        self.all_jo = []

        for i, name in enumerate(joint_names):
            tk.Label(self.frmLC, text=name).grid(row=i)
            var = tk.StringVar(self.frmLC)
            var.set(self.safe_get_angle(self.res_angles[0], i))
            self.cont_all.append(var)
            lbl = tk.Label(
                self.frmLC,
                textvariable=var,
                font=("Arial", 9),
                width=7,
                height=1,
                bg="white"
            )
            lbl.grid(row=i, column=1, padx=5, pady=5)
            self.all_jo.append(lbl)

        # Add speed to joint variables
        self.cont_all.append(self.speed)

        # Coordinate labels
        coord_names = ["x", "y", "z", "rx", "ry", "rz"]
        self.coord_all = []
        coord_vars = []

        for i, name in enumerate(coord_names):
            tk.Label(self.frmLC, text=f"  {name} ").grid(row=i, column=3)
            var = tk.StringVar(self.frmLC)
            var.set(self.safe_get_coord(self.record_coords[0], i))
            self.coord_all.append(var)
            lbl = tk.Label(
                self.frmLC,
                textvariable=var,
                font=("Arial", 9),
                width=7,
                height=1,
                bg="white"
            )
            lbl.grid(row=i, column=4, padx=5, pady=5)
            coord_vars.append(lbl)

        # Add speed to coordinate variables
        self.coord_all.append(self.speed)

        # Unit display (mm)
        unit_var = tk.StringVar(value="mm")
        for i in range(6):
            tk.Label(self.frmLC, textvariable=unit_var,
                     font=("Arial", 9)).grid(row=i, column=5)

    def get_coord_input(self):
        """Read coordinates input from the GUI and send them to the robotic arm.

        Retrieves the coordinate values from the GUI input fields, validates
        them against predefined min/max limits, and reads the speed input.
        Sends the coordinates and speed as a command to the robot through
        a queue for execution.

        Displays error messages via `show_error` if input is invalid.

        Raises:
            ValueError: If the coordinate or speed inputs cannot be converted
            to numbers.
        """
        try:
            c_value = [float(i.get()) for i in self.all_c]
        except ValueError:
            self.get_logger().error("Please enter a number for the coordinates")
            return

        for idx, val in enumerate(c_value):
            if not (self.coords_min[idx] <= val <= self.coords_max[idx]):
                self.get_logger().error(
                    f"Coordinate {['X','Y','Z','RX'][idx]} input value is out of range "
                    f"{self.coords_min[idx]}~{self.coords_max[idx]}"
                )
                return

        try:
            speed_str = self.get_speed.get()
            if not speed_str:
                self.get_logger().error("Please enter a speed value (1-100)")
                return

            speed = int(float(speed_str))
            if not (1 <= speed <= 100):
                self.get_logger().error("Speed input value must be between 1 and 100")
                return
        except ValueError:
            self.get_logger().error("Speed must be a number")
            return

        self.speed = speed
        self.cmd_queue.put(("coords", c_value, self.speed))

    def get_joint_input(self):
        """Read joint angles input from the GUI and send them to the robotic arm.

        Retrieves the joint angles from the GUI input fields, validates them
        against predefined min/max limits, and reads the speed input.
        Sends the joint angles and speed as a command to the robot through
        a queue for execution.

        Displays error messages via `show_error` if input is invalid.

        Raises:
            ValueError: If the joint angle or speed inputs cannot be converted
            to numbers.
        """
        try:
            j_value = [float(i.get()) for i in self.all_j]
        except ValueError:
            self.get_logger().error("Please enter a number for the joint angle")
            return

        for idx, val in enumerate(j_value):
            if not (self.angles_min[idx] <= val <= self.angles_max[idx]):
                self.get_logger().error(
                    f"Joint {idx+1} input value is out of range "
                    f"{self.angles_min[idx]}~{self.angles_max[idx]}"
                )
                return

        try:
            speed_str = self.get_speed.get()
            if not speed_str:
                self.get_logger().error("Please enter a speed value (1-100)")
                return

            speed = int(float(speed_str))
            if not (1 <= speed <= 100):
                self.get_logger().error("Speed input value must be between 1 and 100")
                return
        except ValueError:
            self.get_logger().error("Speed must be a number")
            return

        self.speed = speed
        self.cmd_queue.put(("angles", j_value, self.speed))


    def update_gui(self):
        """Periodically refresh the GUI and update joint angles and coordinates.

        This method queries the current joint angles and coordinates from the
        robot, updates the Tkinter variables for display, and schedules itself
        to run again after 300 ms.
        """
        try:
            angles = self.get_initial_angles()
            coords = self.get_initial_coords()
            if angles and len(angles) == 6 and all(value != -1 for value in angles):
                self.res_angles[0] = angles
                for i, var in enumerate(self.cont_all[:6]):
                    var.set(self.safe_get_angle(self.res_angles[0], i))

            if coords and len(coords) == 6 and all(value != -1 for value in coords):
                self.record_coords[0] = coords
                for i, var in enumerate(self.coord_all[:6]):
                    var.set(self.safe_get_coord(self.record_coords[0], i))
        except Exception as e:
            self.get_logger().warn(f"update_gui error: {e}")

        # Schedule next update in 300 ms
        self.win.after(300, self.update_gui)


def main(args=None):
    """Initialize the ROS2 node and launch the Tkinter GUI for MyCobot.

    This function initializes the rclpy client library, creates the main
    Tkinter window, initializes the WindowNode to handle ROS2 communication
    and GUI interactions, and starts the Tkinter main loop. The GUI can be
    safely interrupted using Ctrl+C (KeyboardInterrupt).

    Args:
        args (list[str], optional): Command line arguments to pass to rclpy.
            Defaults to None.
    """
    rclpy.init(args=args)
    window = tk.Tk()
    window.title("mycobot Isaac GUI")
    node = WindowNode(window)

    try:
        window.mainloop()
    except KeyboardInterrupt:
        # Allow graceful exit on Ctrl+C
        print("Exiting...")
        rclpy.shutdown()  # Shutdown ROS2 client library
        sys.exit(0)       # Exit the program


if __name__ == "__main__":
    main()
