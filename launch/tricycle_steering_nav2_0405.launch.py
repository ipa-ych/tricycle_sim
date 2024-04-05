# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    world_file = os.path.join(get_package_share_directory("tricycle_sim"),"world","obstacles.world")
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                    # launch_argumkents={'world': world_file}.items()
             )

    tricycle_sim_path = os.path.join(
        get_package_share_directory('tricycle_sim'))

    xacro_file = os.path.join(tricycle_sim_path,
                              'urdf',
                              'min_tricycle_imu_0404.xacro.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'tricycle',
                                #    '-x', '-10', '-y', '-10', '-z', '0'
                                   ],
                        output='screen')

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
        
    )

    load_tricycle_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'tricycle_steering_controller'],
        output='screen'
    )

    twist_mux_params = os.path.join(get_package_share_directory('tricycle_sim'),'config','twist_mux_stamped.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
        )
    
    tricycle_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tricycle_steering_controller"],
        remappings=[('/tricycle_steering_controller/reference','/cmd_vel_stamped')]
    )

    teleop_key = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        parameters=[{'stamped': True, 'frame_id': 'base_link'}],
        remappings=[('/cmd_vel','/tricycle_steering_controller/reference')]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=[
        #     '-d',
        #     os.path.join(tricycle_sim_path, 'config/config.rviz'),
        # ],
        output='screen',
    )

    cmd_vel_bridge_TwistStamped = Node(
        package="tricycle_sim",
        executable="cmd_vel_bridge_TwistStamped_node.py",
        output='screen'
    )

    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_config = LaunchConfiguration('params_file', default='/home/nhg-yc/ros2_ws/tricycle_ws/src/tricycle_sim/config/navi_tricycle_steering.yaml')
    nav2_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={'params_file': nav2_config}.items()
        )

    teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')
    joy_config = LaunchConfiguration('joy_config', default='xbox')

    teleop_joy = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(teleop_twist_joy_dir, 'launch', 'teleop-launch.py')
            ),
            launch_arguments={'joy_config': joy_config}.items()
        )


    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_tricycle_controller],
            )
        ),
        gazebo,
        rviz2,
        node_robot_state_publisher,
        spawn_entity,
        twist_mux,
        teleop_joy,
        cmd_vel_bridge_TwistStamped,
        # nav2_node
    ])
