import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='slip_gui',
            executable='slip_gui_node',
            name='slip_gui_node'),
  ])