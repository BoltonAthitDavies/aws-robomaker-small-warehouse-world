# Copyright (c) 2018 Intel Corporation
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

"""Autonomous navigation for the ackermann robot: sim + TF + Nav2, headless.

    ros2 launch aws_robomaker_small_warehouse_world nav2.launch.py
    ros2 launch aws_robomaker_small_warehouse_world nav2.launch.py start_sim:=False

WHY THIS IS A SEPARATE FILE
    small_warehouse.launch.py is a documented interface -- the README has an
    argument table and an RTF table keyed on `headless`, and
    no_roof_small_warehouse.launch.py forwards all nine of its arguments. Changing
    its defaults would break both. Composing it with IncludeLaunchDescription also
    preserves the ordering its _override_speed_limits OpaqueFunction depends on
    (that function mutates IGN_GAZEBO_RESOURCE_PATH and MUST run before gz starts).

    So `headless` is flipped to True HERE, in the wrapper, which makes the cheap
    path the default for autonomous runs without touching the shared contract.
    Pass headless:=False if you really want the GUI (README: RTF 0.40 vs 0.90).

    max_speed/max_accel are left at 10.0/3.0 on purpose. Nav2 never asks for more
    than 0.6 m/s, so neither limit ever binds, and leaving them alone keeps the SDF
    shadowing machinery in small_warehouse.launch.py entirely out of the picture.

WHY THE SERVERS ARE SPELLED OUT INSTEAD OF REUSING navigation_launch.py
    Two reasons. Its lifecycle_manager hardcodes a node list that does NOT include
    map_server (upstream keeps map_server in localization_launch.py, next to the
    AMCL we are deliberately not running), so reusing it means running a second
    lifecycle manager just for the map. And its argument names vary across patch
    releases, which is exactly the sort of thing that is hard to debug when the
    symptom is "the action server never appears". Eight explicit Node actions are
    longer but they are the whole truth, and the node list in
    params/nav2_ackermann.yaml is then actually the list being used.

SETUP -- this will not run until you have done both:
    sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
        ros-humble-nav2-smac-planner ros-humble-nav2-regulated-pure-pursuit-controller
    python3 ~/wil_project/bake_map.py && colcon build --symlink-install \
        --packages-select aws_robomaker_small_warehouse_world
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml

# The launch directory is not an importable Python package, so put it on the path
# to reach stale_process_reaper.py sitting next to this file. __file__ resolves to
# the source tree under --symlink-install and to share/ otherwise, i.e. to
# whichever copy is actually being run, so this needs no ament lookup.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stale_process_reaper import reaper  # noqa: E402


def generate_launch_description():
    pkg_share = get_package_share_directory('aws_robomaker_small_warehouse_world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    start_sim = LaunchConfiguration('start_sim')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    verbosity = LaunchConfiguration('verbosity')
    robot_name = LaunchConfiguration('robot_name')
    publish_joint_states = LaunchConfiguration('publish_joint_states')
    reap_stale = LaunchConfiguration('reap_stale')

    declare_reap_stale_cmd = DeclareLaunchArgument(
        'reap_stale', default_value='True',
        description='Before starting anything, kill nav2 servers, gz and bridges left over from a previous '
                    'launch -- Ctrl-C does not reliably reap them and the leftovers '
                    'fight the new run over /clock and /tf. See '
                    'launch/stale_process_reaper.py. Set False only if you are '
                    'deliberately running a second stack on this ROS_DOMAIN_ID.')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use /clock. Every Nav2 node needs this to match the sim.')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Have the lifecycle manager configure+activate the stack itself')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'params', 'nav2_ackermann.yaml'),
        description='Nav2 parameters. Read the header comment before editing: the '
                    'lookahead and smoother settings are load-bearing for an '
                    'Ackermann platform, not taste.')

    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg_share, 'maps', 'baked', 'map.yaml'),
        description='Occupancy map. Generate with bake_map.py, which slices the '
                    "world's collision meshes -- do NOT point this at maps/002 "
                    '(wrong frame) or maps/005 (SLAM, shelves are hollow).')

    declare_start_sim_cmd = DeclareLaunchArgument(
        'start_sim', default_value='True',
        description='Start Gazebo too. False attaches to an already-running sim.')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Server-only Gazebo. True is the default HERE (unlike '
                    'small_warehouse.launch.py) because the GUI halves the RTF.')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            pkg_share, 'worlds', 'small_warehouse_static', 'small_warehouse_static.world'),
        description='World file; must be the one the map was baked from')

    declare_verbosity_cmd = DeclareLaunchArgument(
        'verbosity', default_value='3', description='Gazebo console verbosity, 0-4')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='ackermann_robot_001',
        description='World-scoped robot name, used to build its gz topic names')

    declare_publish_joint_states_cmd = DeclareLaunchArgument(
        'publish_joint_states', default_value='True',
        description='See localization_tf.launch.py')

    # yaml_filename cannot be a plain launch substitution inside a params file, so
    # the map path is rewritten into a temporary copy of the yaml. use_sim_time is
    # rewritten the same way as a belt-and-braces override of the per-node values.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time,
                        'yaml_filename': map_yaml,
                        # see the note in nav2_ackermann.yaml: the via-point
                        # radius lives in the BT XML, not in any parameter
                        'default_nav_through_poses_bt_xml': os.path.join(
                            pkg_share, 'behavior_trees',
                            'nav_through_poses_ackermann.xml')},
        convert_types=True)

    start_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'small_warehouse.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'reap_stale': reap_stale,
            'headless': headless,
            'world': world,
            'verbosity': verbosity,
            'robot_name': robot_name,
            'bridge_sensors': 'True',
            # Cameras through image_transport, so /camN/image_raw/compressed
            # exists here too and the sim matches the real rig. The raw topic is
            # unaffected, so Nav2 itself neither knows nor cares.
            'compressed_images': 'True',
            'bridge_cmd_vel': 'True',        # this is how Nav2 reaches the robot
            'bridge_ground_truth': 'True',   # this is what localizes it
        }.items(),
        condition=IfCondition(start_sim))

    start_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization_tf.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'reap_stale': reap_stale,
            'publish_joint_states': publish_joint_states,
        }.items())

    lifecycle_nodes = ['map_server', 'controller_server', 'smoother_server',
                       'planner_server', 'behavior_server', 'bt_navigator',
                       'velocity_smoother']

    nav2_nodes = GroupAction([
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[configured_params]),

        # cmd_vel out of the controller goes to cmd_vel_nav, through the smoother,
        # and only then onto /cmd_vel -- which small_warehouse.launch.py's
        # cmd_vel_bridge already forwards to the robot. Getting this chain wrong
        # bypasses the smoother, and on this platform the smoother is what stops
        # every start-from-rest applying full steering lock.
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[configured_params],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),

        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen', parameters=[configured_params]),

        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[configured_params]),

        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[configured_params]),

        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[configured_params]),

        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=[configured_params],
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')]),

        # configured_params FIRST, and it is load-bearing: this node was previously
        # given an inline dict only, so bond_timeout: 20.0 in the params file never
        # reached it and the manager silently used nav2's 4.0 s default. That is the
        # spurious "SERVER map_server IS DOWN after ... 4000 ms" teardown the params
        # file warns about, and it tears the whole stack down under load.
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[configured_params,
                         {'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'node_names': lifecycle_nodes}]),
    ])

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_start_sim_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_verbosity_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_publish_joint_states_cmd)
    ld.add_action(declare_reap_stale_cmd)

    # First action that does anything: clear the previous generation of the
    # nav2 servers before the includes below reap and restart the sim and the
    # TF publishers. The three reap groups are disjoint on purpose -- see
    # launch/stale_process_reaper.py, SCOPE.
    ld.add_action(OpaqueFunction(function=reaper('nav2'),
                                 condition=IfCondition(reap_stale)))

    ld.add_action(start_sim_cmd)
    ld.add_action(start_localization_cmd)
    ld.add_action(nav2_nodes)
    return ld
