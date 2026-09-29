"""Unitree G1 (Dex1-1 grippers) at a table with a screwdriver-bit tray, in Gazebo Harmonic."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            GroupAction, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from g1_gazebo.robot_description import build_robot_description

# Front-right three-quarter view of the robot, bit tray and table (eye at 1.75, -1.3, 1.5 m).
GUI_CAMERA_POSE = ('pose: {position: {x: 1.75, y: -1.3, z: 1.5}, '
                   'orientation: {w: 0.3527, x: -0.1498, y: 0.0573, z: 0.9219}}')


def _setup(context):
    pkg_share = get_package_share_directory('g1_gazebo')
    world = os.path.join(pkg_share, 'worlds', 'g1_table.sdf')

    # Bit tray and screwdriver bit models used by the world.
    actions = [AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                         os.path.join(pkg_share, 'models'))]

    # Server and GUI run as separate processes so a GUI/rendering crash does not kill the sim.
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': f'-s -r -v 3 {world}', 'on_exit_shutdown': 'true'}.items(),
    ))
    if context.launch_configurations['gui'].lower() in ('true', '1'):
        # Gazebo's Ogre2 renderer needs GLX, so on Wayland the GUI must go through XWayland.
        actions.append(GroupAction([
            SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={'gz_args': '-g -v 3'}.items(),
            ),
        ]))
        # Point the GUI camera at the robot and table once the GUI is up.
        actions.append(ExecuteProcess(cmd=['bash', '-c', (
            'for i in $(seq 90); do '
            'gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera '
            '--reptype gz.msgs.Boolean --timeout 1000 --req "' + GUI_CAMERA_POSE + '" '
            '| grep -q true && exit 0; sleep 1; done')]))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': build_robot_description(pkg_share),
                     'use_sim_time': True}],
        output='screen',
    )

    # /clock, head D435i (RGB-D + IMU) and wrist D405 (RGB-D) topics.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml'),
                     'use_sim_time': True}],
        output='screen',
    )

    # The world->pelvis joint in the URDF already carries the standing height.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'g1'],
        output='screen',
    )

    spawn_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'body_controller',
                   'left_arm_controller', 'right_arm_controller',
                   'left_gripper_controller', 'right_gripper_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return actions + [
        robot_state_publisher,
        bridge,
        spawn_robot,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot,
                                           on_exit=[spawn_controllers])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo GUI (false = headless).'),
        OpaqueFunction(function=_setup),
    ])
