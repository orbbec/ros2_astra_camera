from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    astra_params_file = (get_package_share_directory("astra_camera") +
                         "/params/dabai_u3_params.yaml")
    return LaunchDescription([
        Node(
            package="astra_camera",
            namespace="camera",
            name="camera",
            executable="astra_camera_node",
            output="screen",
            parameters=[astra_params_file],
        ),
    ])
