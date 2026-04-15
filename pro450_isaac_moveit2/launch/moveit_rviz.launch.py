from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mycobot_pro450", package_name="pro450_isaac_moveit2").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
