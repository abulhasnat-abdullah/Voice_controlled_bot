import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='voice_controlled_robot',
            executable='speak',
            name='SpeechNode',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='voice_controlled_robot',
            executable='control',
            name='ControlNode',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='voice_controlled_robot',
            executable='energy',
            name='EnergyNode',
            output='screen'
        ),
    ])

#ros2 launch voice_controlled_robot vc_launch.py