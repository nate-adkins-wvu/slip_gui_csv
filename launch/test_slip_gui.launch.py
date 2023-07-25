import launch
import launch_ros.actions

def generate_launch_description():

    ld = launch.LaunchDescription()

    ld.add_action(
        launch_ros.actions.Node(
            package='slip_gui',
            executable='data_recording_node',
            name='data_recording_node'),
    )

    ld.add_action(
        launch_ros.actions.Node(
            package='slip_gui',
            executable='fake_wheelz_node',
            name='fake_wheelz_node'
        )
    )

    return ld 
