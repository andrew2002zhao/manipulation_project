from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    
    container = ComposableNodeContainer(
        package="rclcpp_components",
        name="container",
        executable="component_container_mt",
        namespace="",
        parameters=[{'use_sim_time': True}, {"dedicated_threads": 5}],
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_scripts",
                plugin="MoveitScripts::PickAndPlacePerception",
                name="pick_and_place_perception",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {'use_sim_time': True},
                ],
            )
        ]

    )
    grasping_node = Node(
        name="basic_grasping",
        package="simple_grasping",
        executable="basic_grasping_perception_node",
        namespace="",
        output="screen"
    )
   
    return LaunchDescription([
            container,
            grasping_node
        ])