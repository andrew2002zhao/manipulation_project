
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur3e", package_name="real_moveit_config").to_moveit_configs()
    
    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True},
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription(
        [move_group_node]
    )