import launch
import launch.actions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Launch the service server node
        launch_ros.actions.Node(
            package='robot_patrol',
            executable='service_server_node',
            name='service_server_node',
            output='screen'
        ),
        # Launch the patrolling node
        launch_ros.actions.Node(
            package='robot_patrol',
            executable='patrolling_node',
            name='patrolling_node',
            output='screen'
        ),
    ])
