from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    res = []
    
    ip_launch_arg = DeclareLaunchArgument(
        name="ip",
        default_value="192.168.0.232",
        description='IP address used by the device'
    )
    res.append(ip_launch_arg)

    port_launch_arg = DeclareLaunchArgument(
        name="port",
        default_value="4500",
        description='Port number used by the device'
    )
    res.append(port_launch_arg)

    sync_isaac_launch_arg = DeclareLaunchArgument(
        name="sync_isaac",
        default_value="true",
        description="Mirror the real robot joint angles to Isaac via /joint_command",
    )
    res.append(sync_isaac_launch_arg)

    command_topic_launch_arg = DeclareLaunchArgument(
        name="command_topic",
        default_value="/joint_command",
        description="Isaac command topic mirrored from the real robot",
    )
    res.append(command_topic_launch_arg)

    teleop_keyboard_node = Node(
        package="pro450_isaacsim",
        executable="teleop_keyboard",
        name="teleop_keyboard",
        output="screen",
        emulate_tty=True,
        prefix="x-terminal-emulator -e",
        parameters=[{
            "ip": LaunchConfiguration("ip"),
            "port": LaunchConfiguration("port"),
            "sync_isaac": LaunchConfiguration("sync_isaac"),
            "command_topic": LaunchConfiguration("command_topic"),
        }],
    )
    res.append(teleop_keyboard_node)

    return LaunchDescription(res)
