
import os

import ament_index_python.packages
import launch
import launch.event_handlers
import launch_ros.actions

def generate_launch_description():
    interface_share_dir = ament_index_python.packages.get_package_share_directory('vilma_interface')
    interface_param_file = os.path.join(interface_share_dir, 'config', 'interface-param.yaml')
    interface_node = launch_ros.actions.Node(package='vilma_interface',
                                             executable='vilma_interface_node',
                                             output='both',
                                             parameters=[interface_param_file])
    
    
    return launch.LaunchDescription([interface_node,
                                     
                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=interface_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                             )),
                                     ])