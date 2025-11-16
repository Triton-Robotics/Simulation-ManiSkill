from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Simulation parameters
    sim_params = [
        # General parameters
        DeclareLaunchArgument(
            "human_gui",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "spawn_scenario",
            default_value="center_1v1",
        ),
        DeclareLaunchArgument(
            "sim_time_scale",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "control_freq",
            default_value="150",
        ),
        DeclareLaunchArgument(
            "sim_freq",
            default_value="300",
        ),
        DeclareLaunchArgument(
            "cpu_sim",
            default_value="false",
        ),
        # ---
        # CV Camera parameters
        DeclareLaunchArgument(
            "enable_cv_cam",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "cv_exposure",
            default_value="0.005",
        ),
        DeclareLaunchArgument(
            "cv_fov_horizontal",
            default_value="31",
        ),
        DeclareLaunchArgument(
            "cv_fov_vertical",
            default_value="20",
        ),
        DeclareLaunchArgument(
            "cv_ray_tracing",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "cv_resolution_x",
            default_value="1920",
        ),
        DeclareLaunchArgument(
            "cv_resolution_y",
            default_value="1200",
        ),
        # ---
        # LiDAR parameters
        DeclareLaunchArgument(
            "enable_lidar",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "lidar_pointcloud_resolution",
            default_value="20",
        ),
        # ---
        # ROS parameters
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "primary_robot_color",
            default_value="[0.0, 0.0, 1.0, 1.0]",
        ),
        DeclareLaunchArgument(
            "secondary_robot_color",
            default_value="[0.0, 0.0, 1.0, 1.0]",
        )
    ]

    node = Node(
        package="sim_node",
        executable="sim_node",
        parameters=[
            {
                "human_gui": LaunchConfiguration("human_gui"),
                "spawn_scenario": LaunchConfiguration("spawn_scenario"),
                "sim_time_scale": LaunchConfiguration("sim_time_scale"),
                "control_freq": LaunchConfiguration("control_freq"),
                "sim_freq": LaunchConfiguration("sim_freq"),
                "cpu_sim": LaunchConfiguration("cpu_sim"),
                "enable_cv_cam": LaunchConfiguration("enable_cv_cam"),
                "cv_exposure": LaunchConfiguration("cv_exposure"),
                "cv_fov_horizontal": LaunchConfiguration("cv_fov_horizontal"),
                "cv_fov_vertical": LaunchConfiguration("cv_fov_vertical"),
                "cv_ray_tracing": LaunchConfiguration("cv_ray_tracing"),
                "cv_resolution_x": LaunchConfiguration("cv_resolution_x"),
                "cv_resolution_y": LaunchConfiguration("cv_resolution_y"),
                "enable_lidar": LaunchConfiguration("enable_lidar"),
                "lidar_pointcloud_resolution": LaunchConfiguration(
                    "lidar_pointcloud_resolution"
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "primary_robot_color": LaunchConfiguration("primary_robot_color"),
                "secondary_robot_color": LaunchConfiguration("secondary_robot_color"),
            }
        ],
        output="screen",
    )

    return LaunchDescription(sim_params + [node])
