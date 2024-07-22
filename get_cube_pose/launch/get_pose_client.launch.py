from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    container = ComposableNodeContainer(
        package="rclcpp_components",
        name="container",
        executable="component_container",
        namespace="",
        composable_node_descriptions=[
            ComposableNode(
                package="get_cube_pose",
                plugin="GetCubePose::GetPoseClient",
                name="get_cube_pose"
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