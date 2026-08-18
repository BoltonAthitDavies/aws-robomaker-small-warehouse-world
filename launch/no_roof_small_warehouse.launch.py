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
    bridge_sensors = LaunchConfiguration('bridge_sensors')
    bridge_cmd_vel = LaunchConfiguration('bridge_cmd_vel')
    bridge_ground_truth = LaunchConfiguration('bridge_ground_truth')
    robot_name = LaunchConfiguration('robot_name')
    max_speed = LaunchConfiguration('max_speed')
    max_accel = LaunchConfiguration('max_accel')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Bridge /clock from Gazebo so ROS nodes can use simulation time')

    # Both bridges default False here, unlike small_warehouse.launch.py: this world
    # contains no ackermann_robot, so there would be nothing on the far side of them.
    declare_bridge_sensors_cmd = DeclareLaunchArgument(
        'bridge_sensors',
        default_value='False',
        description="Bridge the ackermann robot's cameras and IMU onto ROS topics")

    declare_bridge_cmd_vel_cmd = DeclareLaunchArgument(
        'bridge_cmd_vel',
        default_value='False',
        description="Bridge ROS /cmd_vel to the robot's Gazebo cmd_vel so ROS teleop can drive it")

    declare_bridge_ground_truth_cmd = DeclareLaunchArgument(
        'bridge_ground_truth',
        default_value='False',
        description="Bridge the robot's true-pose odometry onto ROS as /ground_truth/odometry")

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='ackermann_robot_001',
        description='World-scoped name of the robot model, used to build its gz topic names')

    declare_max_speed_cmd = DeclareLaunchArgument(
        'max_speed',
        default_value='10.0',
        description="Robot speed cap in m/s, forward and reverse. Above ~10 expect "
                    "tyre slip on the 1 ms physics step.")

    declare_max_accel_cmd = DeclareLaunchArgument(
        'max_accel',
        default_value='3.0',
        description='Robot acceleration cap in m/s^2. Decides how much run-up the '
                    'top speed needs: 10 m/s at 3 m/s^2 takes 3.3 s and about 17 m.')

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
            'bridge_sensors': bridge_sensors,
            'bridge_cmd_vel': bridge_cmd_vel,
            'bridge_ground_truth': bridge_ground_truth,
            'robot_name': robot_name,
            'max_speed': max_speed,
            'max_accel': max_accel,
        }.items())

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_verbosity_cmd)
    ld.add_action(declare_bridge_sensors_cmd)
    ld.add_action(declare_bridge_cmd_vel_cmd)
    ld.add_action(declare_bridge_ground_truth_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_max_speed_cmd)
    ld.add_action(declare_max_accel_cmd)

    ld.add_action(start_warehouse_cmd)

    return ld
