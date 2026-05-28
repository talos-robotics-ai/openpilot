from setuptools import find_packages, setup
from glob import glob

package_name = 'policypilot'


def expand(patterns):
    files = []
    for p in patterns:
        files.extend(glob(p, recursive=True))
    return files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'policy_runtime', 'policy_runtime.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),

        # Launch Files
        (f'share/{package_name}/launch', [
            'launch/robot_state_launcher.launch.py',
            'launch/policy_launcher.launch.py',
            'launch/locomotion_launcher.launch.py',
            'launch/mola_launcher.launch.py',
            'launch/livox_launcher.launch.py',
            'launch/manipulation_launcher.launch.py',
            'launch/dashboard_launcher.launch.py',
            'launch/bringup_launcher.launch.py',
        ]),

        # URDF / XML
        (f'share/{package_name}/description_files/urdf',
         expand(['description_files/urdf/*.urdf', 'description_files/urdf/*.xacro'])),
        (f'share/{package_name}/description_files/xml',
         expand(['description_files/xml/*.xml'])),

        # Meshes
        (f'share/{package_name}/description_files/meshes',
         expand(['description_files/meshes/**/*.STL'])),

        # Configuration Files
        (f'share/{package_name}/config', expand(['config/*.yaml'])),

        # Pipeline configs
        (f'share/{package_name}/pipelines', expand(['pipelines/*.yaml'])),

        # RViz
        (f'share/{package_name}/rviz', expand(['rviz/*.rviz'])),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Massimiliano Menzolini',
    maintainer_email='massimiliano.menzolini@example.com',
    description='ROS 2 application layer for the Unitree G1, with bundled RoboJuDo policy runtime.',
    license='BSD 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # State nodes
            'robot_state = policypilot.state.robot_state:main',

            # Manipulation nodes (DDS-side arm and hand control, not VR teleop)
            'interactive_marker = policypilot.manipulation.interactive_marker:main',
            'dx3_controller = policypilot.manipulation.dx3_hand:main',
            'arm_controller = policypilot.manipulation.arm_controller:main',
            'arm_controller_test = policypilot.manipulation.arm_controller_test:main',
            'arm_controller_dds_test = policypilot.manipulation.arm_controller_dds_test:main',

            # Policy nodes (spawn / supervise RoboJuDo pipelines from policy_runtime/)
            'policy_manager = policypilot.policy.policy_manager:main',

            # Dashboard (PyQt6 operator control panel — AMO WALK button etc.)
            'control_panel = policypilot.dashboard.control_panel:main',

            # Locomotion nodes (Unitree loco RPC + planners)
            'loco_client = policypilot.locomotion.loco_client:main',
            'loco_rpc_test = policypilot.locomotion.loco_rpc_test:main',
            'sport_service_test = policypilot.locomotion.sport_service_test:main',
            'dijkstra_planner = policypilot.locomotion.dijkstra_planner:main',
            'nav2point = policypilot.locomotion.nav2point:main',
            'create_map = policypilot.locomotion.create_map:main',
            'mola_fixed = policypilot.locomotion.fix_mola_odometry:main',
        ],
    },
)
