from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def generate_container_node(camera_suffix, params_file):
    with open(params_file, 'r') as file:
        config_params = yaml.safe_load(file)['/camera/camera' + camera_suffix]['ros__parameters']
    container = ComposableNodeContainer(
        name='astra_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::OBCameraNodeFactory',
                name='camera',
                namespace='camera' + camera_suffix,
                parameters=[config_params]
            ),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzNode',
                namespace='camera' + camera_suffix,
                name='point_cloud_xyz'),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzrgbNode',
                namespace='camera' + camera_suffix,
                name='point_cloud_xyzrgb')
        ],
        output='screen'
    )
    return container


def generate_launch_description():
    params1_file = get_package_share_directory("astra_camera") + "/params/multi_camera/halley1_params.yaml"
    params2_file = get_package_share_directory("astra_camera") + "/params/multi_camera/halley2_params.yaml"
    camera_suffix1 = '1'
    camera_suffix2 = '2'
    container1 = generate_container_node(camera_suffix1, params1_file)
    container2 = generate_container_node(camera_suffix2, params2_file)
    return LaunchDescription([container1, container2])
