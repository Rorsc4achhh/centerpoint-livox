from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="data_loader",
            node_executable="data_loader_node",
            node_name="data_loader_node",
            output="screen",
            parameters=[
                {"yaml_path": "/home/txy/workspace/param/config.yml"}
            ]
        )
    ])