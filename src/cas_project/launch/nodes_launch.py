from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cas_project',
            namespace='/',
            executable='global_path_publisher',
            name='global_path_publisher_node'
        ),
        #Node(
        #    package='cpp_pubsub',
        #    namespace='/',
        #    executable='talker',
        #    name='global_path_publisher_node'
        #),
    ])
