from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open_rqt_plot = DeclareLaunchArgument("open_rqt_plot", default_value="false")

    return LaunchDescription(
        [
            open_rqt_plot,
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                output="screen",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--qx",
                    "0",
                    "--qy",
                    "0",
                    "--qz",
                    "0",
                    "--qw",
                    "1",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
            ),
            Node(
                package="erl_geometry_ros",
                executable="house_expo_2d_node",
                name="house_expo_2d_node",
                output="screen",
                parameters=[
                    {
                        "default_qos_reliability": "reliable",
                        "default_qos_durability": "transient_local",
                        "map_file": PathJoinSubstitution(
                            [
                                FindPackageShare("erl_geometry"),
                                "data",
                                "house_expo_room_1451.json",
                            ]
                        ),
                        "publish_pose": True,
                        "pose_publish_rate": 40.0,
                        "path_feed_rate": 20.0,
                        "init_x": 6.61,
                        "init_y": 6.46,
                        "init_yaw": 0.0,
                        "path_topic": "path",
                        "scan_topic": "scan",
                    }
                ],
            ),
            Node(
                package="erl_active_mapping_ros",
                executable="frontier_based_grid_2d_node",
                name="frontier_based_grid_2d_node",
                output="screen",
                parameters=[
                    {
                        "default_qos_reliability": "reliable",
                        "default_qos_durability": "transient_local",
                        "robot_frame": "laser",
                        "auto_replan": True,
                        "stop_exploration_ratio": 0.95,
                        "agent_config_file": PathJoinSubstitution(
                            [
                                FindPackageShare("erl_active_mapping"),
                                "config",
                                "frontier_based_grid_2d.yaml",
                            ]
                        ),
                        "map_min": [-0.5, -0.5],
                        "map_max": [12.0, 12.0],
                        "map_resolution": 0.05,
                        "use_external_map": False,
                        "scan_topic": "scan",
                        "path_topic": "path",
                        "max_observed_area": 118.0,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("erl_active_mapping_ros"),
                            "rviz2",
                            "frontier_based_grid_2d.rviz",
                        ]
                    ),
                ],
            ),
            Node(
                package="rqt_plot",
                executable="rqt_plot",
                name="rqt_plot",
                output="screen",
                arguments=["/observed_ratio/data"],
                condition=IfCondition(LaunchConfiguration("open_rqt_plot")),
            ),
        ],
    )
