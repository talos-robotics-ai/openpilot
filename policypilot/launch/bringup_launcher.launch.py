from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def _load_config():
    config_path = os.path.join(
        get_package_share_directory("policypilot"), "config", "config.yaml"
    )
    try:
        import yaml

        with open(config_path, "r", encoding="utf-8") as config_file:
            return yaml.safe_load(config_file) or {}
    except Exception:
        return {}


def _default_interface():
    env_iface = os.environ.get("POLICYPILOT_INTERFACE")
    if env_iface:
        return env_iface

    config = _load_config()
    general = config.get("general", {})
    iface = general.get("interface")
    if iface:
        return str(iface)

    return "eth0"


def _default_node_enabled(key: str) -> str:
    config = _load_config()
    nodes = config.get("nodes", {})
    value = nodes.get(key, True)
    return "true" if bool(value) else "false"


def generate_launch_description():
    pkg_share = get_package_share_directory('policypilot')
    interface = LaunchConfiguration("interface")
    start_livox = LaunchConfiguration("start_livox")
    start_locomotion = LaunchConfiguration("start_locomotion")
    start_robot_state = LaunchConfiguration("start_robot_state")
    start_policy = LaunchConfiguration("start_policy")
    start_manipulation = LaunchConfiguration("start_manipulation")
    start_dashboard = LaunchConfiguration("start_dashboard")

    livox_launcher = os.path.join(pkg_share, 'launch', 'livox_launcher.launch.py')
    locomotion_launcher = os.path.join(pkg_share, 'launch', 'locomotion_launcher.launch.py')
    robot_state_launcher = os.path.join(pkg_share, 'launch', 'robot_state_launcher.launch.py')
    policy_launcher = os.path.join(pkg_share, 'launch', 'policy_launcher.launch.py')
    manipulation_launcher = os.path.join(pkg_share, 'launch', 'manipulation_launcher.launch.py')
    dashboard_launcher = os.path.join(pkg_share, 'launch', 'dashboard_launcher.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            "interface",
            default_value=_default_interface(),
            description="Network interface for Unitree SDK. Forwarded to state/locomotion/manipulation launchers.",
        ),
        DeclareLaunchArgument(
            "start_livox",
            default_value=_default_node_enabled("livox_launcher_enabled"),
            description="Whether to start the Livox launcher.",
        ),
        DeclareLaunchArgument(
            "start_locomotion",
            default_value=_default_node_enabled("locomotion_launcher_enabled"),
            description="Whether to start the locomotion launcher (loco_client + planners).",
        ),
        DeclareLaunchArgument(
            "start_robot_state",
            default_value=_default_node_enabled("robot_state_launcher_enabled"),
            description="Whether to start the robot_state launcher.",
        ),
        DeclareLaunchArgument(
            "start_policy",
            default_value=_default_node_enabled("policy_launcher_enabled"),
            description="Whether to start the policy_manager (spawns RoboJuDo pipelines).",
        ),
        DeclareLaunchArgument(
            "start_manipulation",
            default_value=_default_node_enabled("manipulation_launcher_enabled"),
            description="Whether to start manipulation nodes.",
        ),
        DeclareLaunchArgument(
            "start_dashboard",
            default_value=_default_node_enabled("dashboard_launcher_enabled"),
            description="Whether to start the PyQt control panel.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launcher),
            condition=IfCondition(start_livox),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(locomotion_launcher),
            launch_arguments={"interface": interface}.items(),
            condition=IfCondition(start_locomotion),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_launcher),
            launch_arguments={"interface": interface}.items(),
            condition=IfCondition(start_robot_state),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(policy_launcher),
            condition=IfCondition(start_policy),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manipulation_launcher),
            launch_arguments={"interface": interface}.items(),
            condition=IfCondition(start_manipulation),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dashboard_launcher),
            condition=IfCondition(start_dashboard),
        ),
    ])
