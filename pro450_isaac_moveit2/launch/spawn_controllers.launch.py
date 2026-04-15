from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mycobot_pro450", package_name="pro450_isaac_moveit2").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
