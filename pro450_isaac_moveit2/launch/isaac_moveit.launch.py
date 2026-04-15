from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_sync_bridge = LaunchConfiguration("start_sync_bridge")
    sync_real_robot = LaunchConfiguration("sync_real_robot")
    robot_ip = LaunchConfiguration("ip")
    robot_port = LaunchConfiguration("port")
    robot_speed = LaunchConfiguration("robot_speed")
    package_share = FindPackageShare("pro450_isaac_moveit2")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Isaac Sim /clock if available.",
            ),
            DeclareLaunchArgument(
                "start_sync_bridge",
                default_value="true",
                description="Start the MoveIt to Isaac trajectory bridge.",
            ),
            DeclareLaunchArgument(
                "sync_real_robot",
                default_value="false",
                description="Also forward the planned trajectory to the real Pro450.",
            ),
            DeclareLaunchArgument("ip", default_value="192.168.0.232"),
            DeclareLaunchArgument("port", default_value="4500"),
            DeclareLaunchArgument("robot_speed", default_value="50"),
            SetParameter(name="use_sim_time", value=use_sim_time),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [package_share, "launch", "rsp.launch.py"]
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [package_share, "launch", "static_virtual_joint_tfs.launch.py"]
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [package_share, "launch", "move_group.launch.py"]
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [package_share, "launch", "moveit_rviz.launch.py"]
                    )
                )
            ),
            Node(
                package="pro450_isaac_moveit2_control",
                executable="isaac_sync_plan",
                name="sync_plan_bridge",
                output="screen",
                condition=IfCondition(start_sync_bridge),
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"sync_real_robot": sync_real_robot},
                    {"ip": robot_ip},
                    {"port": robot_port},
                    {"robot_speed": robot_speed},
                    {"action_name": "/arm_group_controller/follow_joint_trajectory"},
                    {"command_topic": "/joint_command"},
                ],
            ),
        ]
    )
