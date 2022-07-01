from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    params_file = (get_package_share_directory(
        "astra_camera") + "/params/halley_params.yaml")
    container = ComposableNodeContainer(
        name='astra_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::OBCameraNodeFactory',
                name='astra_camera',
                parameters=[params_file]),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzNode',
                name='point_cloud_xyz'),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb')
        ],
        output='screen',
        prefix=["xterm -e gdb -ex run --args"]
    )
    return LaunchDescription([container])
