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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aws_robomaker_small_warehouse_world')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    verbosity = LaunchConfiguration('verbosity')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Bridge /clock from Gazebo so ROS nodes can use simulation time')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run the Gazebo server only, without the GUI')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            pkg_share, 'worlds', 'small_warehouse', 'small_warehouse.world'),
        description='Full path to the world file to load')

    declare_verbosity_cmd = DeclareLaunchArgument(
        'verbosity',
        default_value='3',
        description='Gazebo console verbosity, 0-4. 4 is debug level and very noisy.')

    gz_sim_launch = PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py'))

    # The world path is interpolated into gz_args here. The Gazebo Classic
    # version of this file declared a 'world' argument but never forwarded it,
    # so gzserver always came up with an empty world and
    # no_roof_small_warehouse.launch.py was a no-op.
    start_gz_gui_cmd = IncludeLaunchDescription(
        gz_sim_launch,
        launch_arguments={
            'gz_args': ['-r -v ', verbosity, ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(headless))

    # Server only. '-s' disables rendering entirely, so --headless-rendering is
    # not needed here unless you spawn camera/lidar sensors on a machine with
    # no display, in which case the Sensors system does need it.
    start_gz_server_cmd = IncludeLaunchDescription(
        gz_sim_launch,
        launch_arguments={
            'gz_args': ['-s -r -v ', verbosity, ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(headless))

    # gz-sim has no gazebo_ros_init equivalent, so without this bridge /clock
    # never appears on the ROS side and every node honouring use_sim_time
    # blocks on startup waiting for a clock that never ticks.
    start_clock_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        condition=IfCondition(use_sim_time))

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_verbosity_cmd)

    ld.add_action(start_gz_gui_cmd)
    ld.add_action(start_gz_server_cmd)
    ld.add_action(start_clock_bridge_cmd)

    return ld
