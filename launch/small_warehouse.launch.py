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
    bridge_sensors = LaunchConfiguration('bridge_sensors')
    bridge_cmd_vel = LaunchConfiguration('bridge_cmd_vel')
    robot_name = LaunchConfiguration('robot_name')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Bridge /clock from Gazebo so ROS nodes can use simulation time')

    declare_bridge_sensors_cmd = DeclareLaunchArgument(
        'bridge_sensors',
        default_value='True',
        description="Bridge the ackermann robot's cameras and IMU onto ROS topics")

    declare_bridge_cmd_vel_cmd = DeclareLaunchArgument(
        'bridge_cmd_vel',
        default_value='True',
        description="Bridge ROS /cmd_vel to the robot's Gazebo cmd_vel so ROS teleop can drive it")

    # The robot's gz topics are scoped by the name the WORLD gives it, which is the
    # <include><name> in small_warehouse.world, not the <model name> in model.sdf.
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='ackermann_robot_001',
        description='World-scoped name of the robot model, used to build its gz topic names')

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

    # Server only. '-s' alone would leave the Sensors system with no render
    # context, so the ackermann robot's cameras would publish nothing;
    # --headless-rendering gives it an offscreen one.
    start_gz_server_cmd = IncludeLaunchDescription(
        gz_sim_launch,
        launch_arguments={
            'gz_args': ['-s -r --headless-rendering -v ', verbosity, ' ', world],
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

    # The ackermann robot's sensors (models/ackermann_robot/model.sdf). These are
    # the topics vins_fusion_ros2/config/wil_sim/stereo_imu.yaml subscribes to.
    # '[' is gz -> ROS only, which is all a sensor ever needs. Humble's
    # parameter_bridge has no YAML config option, so the mappings are listed here
    # rather than in a config file.
    start_sensor_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        output='screen',
        arguments=[
            # Note the camera_info names: gz derives them by REPLACING the last
            # element of the sensor's <topic>, not by appending, so a sensor on
            # 'cam0/image_raw' publishes info on '/cam0/camera_info'.
            '/cam0/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cam0/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cam1/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cam1/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # Capitalised 'IMU' -- that is the Fortress-era gz message name.
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        condition=IfCondition(bridge_sensors))

    # Actuation, kept separate from sensor_bridge: opposite direction, and this way
    # you can drop control without losing the sensors (e.g. during a scripted replay).
    #
    # ']' is ROS -> gz, the mirror of the sensor bridge's '[' -- see `parameter_bridge`
    # with no arguments for the full notation. Getting it backwards yields a bridge
    # that comes up cleanly and simply never delivers anything.
    #
    # The gz side is model-scoped, so it is remapped to a plain /cmd_vel on ROS and
    # stock teleop nodes need no remapping of their own. Each remap rule has to be a
    # TUPLE -- launch_ros's normalize_remap_rule rejects a list outright.
    start_cmd_vel_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=[['/model/', robot_name,
                    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist']],
        remappings=[(['/model/', robot_name, '/cmd_vel'], '/cmd_vel')],
        condition=IfCondition(bridge_cmd_vel))

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_verbosity_cmd)
    ld.add_action(declare_bridge_sensors_cmd)
    ld.add_action(declare_bridge_cmd_vel_cmd)
    ld.add_action(declare_robot_name_cmd)

    ld.add_action(start_gz_gui_cmd)
    ld.add_action(start_gz_server_cmd)
    ld.add_action(start_clock_bridge_cmd)
    ld.add_action(start_sensor_bridge_cmd)
    ld.add_action(start_cmd_vel_bridge_cmd)

    return ld
