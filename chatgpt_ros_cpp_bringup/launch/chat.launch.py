from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
)

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    api_key_arg = DeclareLaunchArgument(
        'api_key', default_value=TextSubstitution(text='THIS_IS_TEST_KEY'))
    my_node = ComposableNodeContainer(
        name='chat',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='chatgpt_ros_cpp_node',
                plugin='chatgpt_ros_cpp_node::ChatGptServer',
                name='chatgpt_server',
                namespace='',
                parameters=[{
                    'api_key': LaunchConfiguration('api_key'),
                }],
            )
        ])
    rqt_service_caller = Node(
        package='rqt_service_caller',
        executable='rqt_service_caller',
        name='service_caller',
    )
    ld = LaunchDescription()
    ld.add_action(api_key_arg)
    ld.add_action(my_node)
    ld.add_action(rqt_service_caller)

    return ld
