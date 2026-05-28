from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def _default_interface():
    env_iface = os.environ.get("POLICYPILOT_INTERFACE")
    if env_iface:
        return env_iface

    config_path = os.path.join(
        get_package_share_directory("policypilot"), "config", "config.yaml"
    )
    try:
        import yaml

        with open(config_path, "r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file) or {}
        general = config.get("general", {})
        iface = general.get("interface")
        if iface:
            return str(iface)
    except Exception:
        pass

    return "eth0"

def generate_launch_description():
    interface = LaunchConfiguration("interface")
    use_robot = LaunchConfiguration("use_robot")
    arm_controlled = LaunchConfiguration("arm_controlled")
    enable_arm_ui = LaunchConfiguration("enable_arm_ui")
    ik_use_waist = LaunchConfiguration("ik_use_waist")
    ik_alpha = LaunchConfiguration("ik_alpha")
    ik_max_dq_step = LaunchConfiguration("ik_max_dq_step")
    arm_velocity_limit = LaunchConfiguration("arm_velocity_limit")

    return LaunchDescription([
        DeclareLaunchArgument(
            "interface",
            default_value=_default_interface(),
            description="Network interface for Unitree SDK. Override with interface:=... on multi-NIC hosts.",
        ),
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument("arm_controlled", default_value="both"),
        DeclareLaunchArgument("enable_arm_ui", default_value="true"),
        DeclareLaunchArgument("ik_use_waist", default_value="false"),
        DeclareLaunchArgument("ik_alpha", default_value="0.2"),
        DeclareLaunchArgument("ik_max_dq_step", default_value="0.05"),
        DeclareLaunchArgument("arm_velocity_limit", default_value="2.0"),

        Node(
            package='policypilot',
            executable='loco_client',
            name='loco_client',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'arm_controlled': arm_controlled,  # string ('left'|'right'|'both')
                'enable_arm_ui': ParameterValue(enable_arm_ui, value_type=bool),
                'ik_use_waist': ParameterValue(ik_use_waist, value_type=bool),
                'ik_alpha': ParameterValue(ik_alpha, value_type=float),
                'ik_max_dq_step': ParameterValue(ik_max_dq_step, value_type=float),
                'arm_velocity_limit': ParameterValue(arm_velocity_limit, value_type=float),
            }],
            output='screen'
        ),

        Node(
            package='policypilot',
            executable='nav2point',
            name='nav2point',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='policypilot',
            executable='dijkstra_planner',
            name='dijkstra_planner',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
            }],
            output='screen'
        ),
    ])
