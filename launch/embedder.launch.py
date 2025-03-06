import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # use config file if provided
    launch_config_yaml = DeclareLaunchArgument('config_yaml', 
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory("bob_vector_db"),
                "config", "vectordb.yaml")))

    # launch with terminal
    launch_terminal = DeclareLaunchArgument('terminal', 
        default_value="false")

    node = Node(
        package='bob_vector_db',
        executable='embedder',
        name='embedder',
        parameters=[
            LaunchConfiguration('config_yaml')]
    )

    terminal = Node(
        condition=IfCondition(LaunchConfiguration("terminal")),
        package='bob_topic_tools',
        executable='terminal',
        name='terminal_embed',
        remappings=[
            ('topic_in_cr', 'embed'),
            ('topic_out', 'embed')],
        parameters=[
            {'title': 'Vector DB Embedder Terminal'},
            LaunchConfiguration('config_yaml')]
    )

    return LaunchDescription([
        launch_config_yaml,
        launch_terminal,
        node, 
        terminal
    ])
