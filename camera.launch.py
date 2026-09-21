import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    serial_number = LaunchConfiguration("serial_number")

    params = {
        "camera_name": "top_camera",
        "serial_number": serial_number,
        "enable_color": True,
        "enable_depth": True,
        "enable_ir": False,
        "color_width": 3840,
        "color_height": 2160,
        "color_fps": 30,
        "color_format": "MJPG",
        "depth_width": 640,
        "depth_height": 576,
        "depth_fps": 30,
        "depth_registration": True,
        "align_mode": "SW",
        "align_target_stream": "COLOR",
        "enable_frame_sync": True,
        "enable_point_cloud": True,
        "enable_colored_point_cloud": False,
        "log_level": "info",
        "publish_tf": True,
    }

    # Remap topics directly to:
    # /vision/top_camera/rgb
    # /vision/top_camera/depth
    # /vision/top_camera/points
    remappings = [
        ("color/image_raw", "rgb"),
        ("color/camera_info", "rgb/camera_info"),
        ("depth/image_raw", "depth"),
        ("depth/camera_info", "depth/camera_info"),
        ("depth/points", "points"),
    ]

    compose_node = ComposableNode(
        package="orbbec_camera",
        plugin="orbbec_camera::OBCameraNodeDriver",
        name="top_camera",
        parameters=[params],
        remappings=remappings,
    )

    container = ComposableNodeContainer(
        name="top_camera_container",
        namespace="vision/top_camera",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[compose_node],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_number", description="Femto Bolt SN"),
        RegisterEventHandler(OnProcessExit(on_exit=[Shutdown(reason="Camera exited")])),
        GroupAction([
            PushRosNamespace("vision/top_camera"),
            container,
        ]),
    ])
