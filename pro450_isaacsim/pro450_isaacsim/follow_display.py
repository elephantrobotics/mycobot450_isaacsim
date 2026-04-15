import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '4.0.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))

if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'Current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import Pro450Client


class Talker(Node):
    """ROS2 node to publish joint states and visualize end-effector position."""

    def __init__(self):
        """Initialize the Talker node and connect to MyCobotPro450."""
        super().__init__("follow_display")
        self.declare_parameter('ip', '192.168.0.232')
        self.declare_parameter('port', 4500)
        self.declare_parameter('sync_isaac', True)
        self.declare_parameter('command_topic', '/joint_command')

        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        self.sync_isaac = self.get_parameter("sync_isaac").get_parameter_value().bool_value
        self.command_topic = self.get_parameter("command_topic").get_parameter_value().string_value

        self.get_logger().info("ip:%s, port:%d" % (ip, port))
        self.mycobot_450 = Pro450Client(ip, port)
        if self.mycobot_450.is_power_on !=1:
            self.mycobot_450.power_on()
        # self.mycobot_450.set_motor_enabled(254, 0)
        time.sleep(0.05)
        self.mycobot_450.set_free_move_mode(1)
        time.sleep(0.05)

        self.command_pub = None
        if self.sync_isaac:
            self.command_pub = self.create_publisher(JointState, self.command_topic, 10)
        # self.get_logger().info("All servos released.\n")
        self.get_logger().info("Please press the button at the end of the machine to drag the joint.\n请按下机器末端按钮进行关节拖拽运动")

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

    def start(self):
        """Start publishing joint command_topic.

        Publishes:
            JointState messages to 'command_topic' topic.
        """
        rate = self.create_rate(30)

        self.get_logger().info("Publishing ...")
        while rclpy.ok():
            rclpy.spin_once(self)
            try:
                # Get robot joint angles
                angles = self.mycobot_450.get_angles()
                if isinstance(angles, list) and len(angles) > 0:
                    self.publish_joint_command(angles)
                else:
                    self.get_logger().warn("Failed to get valid angles: {}".format(angles))

                rate.sleep()
            except Exception as e:
                print(e)


def main(args=None):
    """Main function to run the Talker node.

    Args:
        args (list, optional): Command-line arguments for ROS2. Defaults to None.
    """
    rclpy.init(args=args)

    talker = Talker()
    talker.start()
    rclpy.spin(talker)

    talker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
