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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('aws_robomaker_small_warehouse_world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    verbosity = LaunchConfiguration('verbosity')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Bridge /clock from Gazebo so ROS nodes can use simulation time')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run the Gazebo server only, without the GUI')

    declare_verbosity_cmd = DeclareLaunchArgument(
        'verbosity',
        default_value='3',
        description='Gazebo console verbosity, 0-4. 4 is debug level and very noisy.')

    start_warehouse_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'small_warehouse.launch.py')),
        launch_arguments={
            'world': os.path.join(
                pkg_share, 'worlds', 'no_roof_small_warehouse',
                'no_roof_small_warehouse.world'),
            'use_sim_time': use_sim_time,
            'headless': headless,
            'verbosity': verbosity,
        }.items())

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_verbosity_cmd)

    ld.add_action(start_warehouse_cmd)

    return ld
