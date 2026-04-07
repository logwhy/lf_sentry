import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # ========================
    # Package directories
    # ========================
    bringup_dir = get_package_share_directory("pb2025_sentry_bringup")
    serial_dir = get_package_share_directory("standard_robot_pp_ros2")
    navigation_dir = get_package_share_directory("pb2025_nav_bringup")
    behavior_dir = get_package_share_directory("pb2025_sentry_behavior")
    merge_cloud_dir = get_package_share_directory("merge_cloud")

    # ========================
    # Launch configurations
    # ========================
    robot_name = LaunchConfiguration("robot_name")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")

    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    enable_rosbag = LaunchConfiguration("enable_rosbag")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # ========================
    # Environment variables
    # ========================
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )
    colorized_output_envvar = SetEnvironmentVariable(
        "RCUTILS_COLORIZED_OUTPUT", "1"
    )

    # ========================
    # Declare arguments
    # ========================
    declare_cmds = [
        DeclareLaunchArgument(
            "robot_name",
            default_value="lf2026_old_sentry",
            description="Robot name for serial bringup",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Top-level namespace",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "params", "node_params.yaml"),
            description="Full path to the ROS2 parameters file",
        ),
        DeclareLaunchArgument(
            "slam",
            default_value="True",
            description="Whether to run SLAM mode",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="basef1_1116",
            description="Map/world name",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=[
                TextSubstitution(text=os.path.join(bringup_dir, "map", "")),
                world,
                TextSubstitution(text=".yaml"),
            ],
            description="Full path to map yaml",
        ),
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value=[
                TextSubstitution(text=os.path.join(bringup_dir, "pcd", "")),
                world,
                TextSubstitution(text=".pcd"),
            ],
            description="Full path to prior PCD map file",
        ),
        DeclareLaunchArgument(
            "use_robot_state_pub",
            default_value="False",
            description="Whether navigation launch should start robot_state_publisher",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="False",
            description="Whether to start RViz",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="True",
            description="Use composed bringup for Nav2",
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="True",
            description="Respawn nodes if they crash",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level",
        ),
        DeclareLaunchArgument(
            "enable_rosbag",
            default_value="False",
            description="Whether to start rosbag recorder node",
        ),
    ]

    # ========================
    # 1) Merge cloud
    # Equivalent to:
    # ros2 launch merge_cloud launch.py
    # ========================
    start_merge_cloud_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(merge_cloud_dir, "launch", "launch.py")
        )
    )

    # ========================
    # 2) Serial / lower machine / gimbal
    # Equivalent to:
    # ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
    # ========================
    start_serial_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                serial_dir,
                "launch",
                "standard_robot_pp_ros2.launch.py",
            )
        ),
        launch_arguments={
            "robot_name": robot_name,
            "namespace": namespace,
            "params_file": params_file,
            "use_rviz": "False",
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    # ========================
    # 3) Navigation / SLAM
    # Equivalent to:
    # ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=False
    # ========================
    start_navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_dir,
                "launch",
                "rm_navigation_reality_launch.py",
            )
        ),
        launch_arguments={
            "slam": slam,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_robot_state_pub": use_robot_state_pub,
            "use_rviz": "False",
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    # ========================
    # 4) Behavior tree
    # Equivalent to:
    # ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py
    # ========================
    start_behavior_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                behavior_dir,
                "launch",
                "pb2025_sentry_behavior_launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "log_level": log_level,
        }.items(),
    )

    # ========================
    # Optional RViz
    # ========================
    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": os.path.join(
                bringup_dir, "rviz", "sentry_default_view.rviz"
            ),
        }.items(),
    )

    # ========================
    # Optional rosbag
    # ========================
    record_rosbag_cmd = Node(
        package="rosbag2_composable_recorder",
        executable="composable_recorder_node",
        name="rosbag_recorder",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(enable_rosbag),
    )

    # ========================
    # Startup ordering
    # 为了减少抢资源/TF/话题未就绪问题，稍微错开启动
    # ========================
    delayed_serial_cmd = TimerAction(
        period=1.0,
        actions=[start_serial_driver_cmd],
    )

    delayed_navigation_cmd = TimerAction(
        period=2.0,
        actions=[start_navigation_launch_cmd],
    )

    delayed_behavior_cmd = TimerAction(
        period=3.0,
        actions=[start_behavior_launch_cmd],
    )

    delayed_rviz_cmd = TimerAction(
        period=4.0,
        actions=[start_rviz_cmd],
    )

    delayed_rosbag_cmd = TimerAction(
        period=4.0,
        actions=[record_rosbag_cmd],
    )

    # ========================
    # LaunchDescription
    # ========================
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    for cmd in declare_cmds:
        ld.add_action(cmd)

    ld.add_action(start_merge_cloud_cmd)
    ld.add_action(delayed_serial_cmd)
    ld.add_action(delayed_navigation_cmd)
    ld.add_action(delayed_behavior_cmd)
    ld.add_action(delayed_rviz_cmd)
    ld.add_action(delayed_rosbag_cmd)

    return ld