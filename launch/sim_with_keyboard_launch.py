from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    sim_launch_path = (
        get_package_share_directory("sim_node") + "/launch/sim_node_launch.py"
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path=sim_launch_path)
    )

    keyboard_controls_node = Node(package="sim_node", executable="keyboard_controls")

    ld.add_action(sim_launch)
    ld.add_action(keyboard_controls_node)

    return ld
