from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    default_mode_arg = DeclareLaunchArgument(
        "default_mode",
        default_value="/rc/ackermann_cmd",
        description="Default AckermannDriveStamped input topic for the drive manager.",
    )

    drive_manager_node = Node(
        package="drive_manager",
        executable="drive_manager",
        name="drive_manager",
        output="screen",
        parameters=[
            {
                "default_mode": LaunchConfiguration("default_mode"),
            }
        ],
    )

    speed_guard_node = Node(
        package="vision_speed_guard",
        executable="speed_guard",
        name="speed_guard",
        output="screen",
    )

    yolo_node = Node(
        package="yolo_vision",
        executable="run_yolo",
        name="yolo_node",
        output="screen",
        parameters=[
            {
                "publish_plot": False,
                "publish_mask": False,
                "publish_detections": True,
            }
        ],
    )

    return LaunchDescription([
        default_mode_arg,
        yolo_node,
        speed_guard_node,
        drive_manager_node,
    ])