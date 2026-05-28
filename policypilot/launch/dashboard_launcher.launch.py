from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Brings up the PyQt6 operator dashboard (control_panel).

    The dashboard publishes lifecycle topics; it does not own any robot
    state itself. Run alongside bringup_launcher (which provides
    policy_manager, loco_client, etc.).
    """
    return LaunchDescription([
        Node(
            package='policypilot',
            executable='control_panel',
            name='control_panel',
            output='screen',
        ),
    ])
