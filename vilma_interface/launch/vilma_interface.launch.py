import os

import errno
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
  
    interface_share_dir = get_package_share_directory("vilma_interface")
    interface_param_file = os.path.join(
        interface_share_dir, "config", "interface.param.yaml"
    )
    interface_node = Node(
        package="vilma_interface",
        executable="vilma_interface_node",
        output="both",
        parameters=[interface_param_file],
    )

    hmi_launch = []

    if LaunchConfiguration("hmi").perform(context):
        hmi_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(get_package_share_directory("vilma_hmi")),
                    "/launch/vilma_hmi.launch.py",
                ]
            ),
            launch_arguments={"interface": "vilma_autoware"}.items(),
        )

    return [
        interface_node,
        # hmi_launch,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=interface_node,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ]


def generate_launch_description():

    return LaunchDescription(
        [DeclareLaunchArgument("hmi", default_value="False")]
        + [OpaqueFunction(function=launch_setup)]
    )
