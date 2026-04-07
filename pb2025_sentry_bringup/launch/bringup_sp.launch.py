import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # ========================
    # Directories
    # ========================
    bringup_dir = get_package_share_directory("pb2025_sentry_bringup")
    serial_bringup_dir = get_package_share_directory("standard_robot_pp_ros2")
    navigation_bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    bt_bringup_dir = get_package_share_directory("pb2025_sentry_behavior")

    # sp_vision_25 默认路径（你给的：~/pb2025_sentry_ws/src/sp_vision_25）
    default_sp_root = os.path.expanduser("~/pb2025_sentry_ws/src/sp_vision_25")
    default_sp_exec = os.path.join(default_sp_root, "build", "sentry_dual_cameras_ros")

    # ========================
    # Launch configurations
    # ========================
    robot_name = LaunchConfiguration("robot_name")

    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # sp vision
    use_sp_vision = LaunchConfiguration("use_sp_vision")
    sp_vision_exec = LaunchConfiguration("sp_vision_exec")
    sp_vision_workdir = LaunchConfiguration("sp_vision_workdir")

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
    # Environment
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
        ),
        DeclareLaunchArgument(
            "slam",
            default_value="True",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="basef1_1116",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=[
                TextSubstitution(text=os.path.join(bringup_dir, "map", "")),
                world,
                TextSubstitution(text=".yaml"),
            ],
        ),
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value=[
                TextSubstitution(text=os.path.join(bringup_dir, "pcd", "")),
                world,
                TextSubstitution(text=".pcd"),
            ],
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "params", "node_params.yaml"),
        ),
        DeclareLaunchArgument(
            "use_robot_state_pub",
            default_value="False",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="False",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="True",
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="True",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
        ),
        # sp vision switch & path
        DeclareLaunchArgument(
            "use_sp_vision",
            default_value="True",
            description="Whether to start sp_vision_25 sentry_dual_cameras_ros",
        ),
        DeclareLaunchArgument(
            "sp_vision_exec",
            default_value=default_sp_exec,
            description="Absolute path to sp_vision_25 executable (sentry_dual_cameras_ros)",
        ),
        DeclareLaunchArgument(
            "sp_vision_workdir",
            default_value=default_sp_root,
            description="Working directory for sp_vision_25 executable",
        ),
    ]

    # ========================
    # Serial (下位机 / 云台 / stand_ros2_pp)
    # ========================
    start_serial_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                serial_bringup_dir,
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
    # SP vision (纯 CMake 二进制：./build/sentry_dual_cameras_ros)
    # ========================
    # 注意：因为不是 ament package，用 ExecuteProcess 直接跑二进制
    start_sp_vision_cmd = ExecuteProcess(
        condition=IfCondition(use_sp_vision),
        cmd=[sp_vision_exec],
        cwd=sp_vision_workdir,
        output="screen",
        # 需要的话你可以给它额外参数： cmd=[sp_vision_exec, "--flag", "xxx"]
        additional_env={
            # 如果你的 sp 程序依赖工作区下的库，可在这里补 LD_LIBRARY_PATH
            # "LD_LIBRARY_PATH": os.environ.get("LD_LIBRARY_PATH", ""),
        },
    )

    # ========================
    # Navigation (Nav2 / SLAM)
    # ========================
    start_navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_bringup_dir,
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
    # Behavior Tree
    # ========================
    start_behavior_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bt_bringup_dir,
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
    # RViz（可选）
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
    # Rosbag
    # ========================
    record_rosbag_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "rosbag2_composable_recorder",
            "composable_recorder_node",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        output="screen",
        condition=IfCondition(TextSubstitution(text="True")),
    )

    # 如果你更想保留你原来 Node 的写法（更“ros2-native”），也可以换回 Node。
    # 这里我用 ExecuteProcess 是为了保持“纯进程”的一致性（尤其你 sp 也是进程）。

    # ========================
    # LaunchDescription
    # ========================
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    for cmd in declare_cmds:
        ld.add_action(cmd)

    # 顺序建议：串口先起（下位机），再起 sp vision（订阅/发布更稳定），再起 nav/BT
    ld.add_action(start_serial_driver_cmd)
    ld.add_action(start_sp_vision_cmd)
    ld.add_action(start_navigation_launch_cmd)
    ld.add_action(start_behavior_launch_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(record_rosbag_cmd)

    return ld
