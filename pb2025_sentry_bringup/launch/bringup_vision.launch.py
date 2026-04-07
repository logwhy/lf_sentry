import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pb2025_sentry_bringup")
    serial_bringup_dir = get_package_share_directory("standard_robot_pp_ros2")
    vision_bringup_dir = get_package_share_directory("pb2025_vision_bringup")
    nav_bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    # Create the launch configuration variables
    robot_name = LaunchConfiguration("robot_name")
    detector = LaunchConfiguration("detector")
    use_hik_camera = LaunchConfiguration("use_hik_camera")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    # Declare launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="lf2026_old_sentry",
        description="The file name of the robot xmacro to be used",
    )

    declare_detector_cmd = DeclareLaunchArgument(
        "detector",
        default_value="opencv",
        description="Type of detector to use (option: 'opencv', 'openvino','tensorrt')",
    )

    declare_use_hik_camera_cmd = DeclareLaunchArgument(
        "use_hik_camera",
        default_value="True",
        description="Whether to bringup hik camera node",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "node_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="False",
        description="Whether to start the robot state publisher",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Specify the actions
    start_serial_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                serial_bringup_dir, "launch", "standard_robot_pp_ros2.launch.py"
            )
        ),
        launch_arguments={
            "robot_name": robot_name,
            "namespace": namespace,
            "params_file": params_file,
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    start_vision_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision_bringup_dir, "launch", "rm_vision_reality_launch.py")
        ),
        launch_arguments={
            "detector": detector,
            "use_hik_camera": use_hik_camera,
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_robot_state_pub": use_robot_state_pub,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    # Start RViz if needed
    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": os.path.join(bringup_dir, "rviz", "sentry_default_view.rviz"),
        }.items(),
    )

    # Record rosbag if needed
    record_rosbag_cmd = Node(
        package="rosbag2_composable_recorder",
        executable="composable_recorder_node",
        name="rosbag_recorder",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_detector_cmd)
    ld.add_action(declare_use_hik_camera_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch serial and vision nodes
    ld.add_action(start_serial_driver_cmd)
    ld.add_action(start_vision_launch_cmd)

    # Add RViz and rosbag recording if needed
    ld.add_action(start_rviz_cmd)
    ld.add_action(record_rosbag_cmd)


    return ld
