from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Brings up the policy_manager node, which spawns/stops RoboJuDo pipelines
    (the bundled policy_runtime) on demand via ROS topics.
    """
    return LaunchDescription([
        Node(
            package='policypilot',
            executable='policy_manager',
            name='policy_manager',
            output='screen',
        ),
    ])
