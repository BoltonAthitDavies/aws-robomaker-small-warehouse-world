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

"""TF tree for the ackermann robot: map -> odom -> base_footprint -> links.

Split out of nav2.launch.py on purpose. This is the file you run on its own when
TF is broken, and it is the file you would REPLACE to swap ground truth for AMCL
or for VINS-as-localization. Nothing in here knows that Nav2 exists.

    ros2 launch aws_robomaker_small_warehouse_world localization_tf.launch.py
    ros2 run tf2_tools view_frames

The chain, with exactly one publisher per edge:

    map  --(static identity)-->  odom          ground_truth_localization
    odom --(50 Hz)------------>  base_footprint  ground_truth_localization
    base_footprint --> base_link --> sensors    robot_state_publisher  (/tf_static)
    base_link --> wheels, knuckles              robot_state_publisher  (/tf, 10 Hz)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aws_robomaker_small_warehouse_world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_joint_states = LaunchConfiguration('publish_joint_states')
    flatten_2d = LaunchConfiguration('flatten_2d')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use /clock. Must match every other node or tf2 will report '
                    'extrapolation errors that look like a TF bug.')

    declare_publish_joint_states_cmd = DeclareLaunchArgument(
        'publish_joint_states', default_value='True',
        description="Run joint_state_publisher. Nav2 does not need the wheel frames, "
                    "but without /joint_states robot_state_publisher logs 'the "
                    "complete state of the robot is not yet available' forever.")

    declare_flatten_2d_cmd = DeclareLaunchArgument(
        'flatten_2d', default_value='False',
        description='Zero z/roll/pitch in the published TF. Off by default: the gz '
                    'plugin runs with <dimensions>3</dimensions> and Nav2 takes yaw '
                    'only, so the tilt is harmless and hiding it hides the truth.')

    # The URDF is read at launch time and passed as a string. robot_state_publisher
    # will warn about the SDF-only <gazebo> extension tags in it; that is expected.
    urdf_path = os.path.join(pkg_share, 'urdf', 'ackermann_robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}])

    # Publishes zeros, so the steered wheels always render straight even mid-turn.
    # Cosmetic only: the robot's SDF has no JointStatePublisher system, so there is
    # no real joint state to bridge in the first place.
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(publish_joint_states))

    start_ground_truth_localization_cmd = Node(
        package='aws_robomaker_small_warehouse_world',
        executable='ground_truth_localization',
        name='ground_truth_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'flatten_2d': flatten_2d}])

    # CROSS-CHECK, deliberately left commented out. ros_gz_bridge really can map
    # ignition.msgs.Pose_V to tf2_msgs/TFMessage (the Factory<> symbol is in
    # libros_gz_bridge.so), so if the node above looks wrong, enable this instead
    # and compare. It is not the default because it publishes only TF -- Nav2's
    # controller_server also needs odometry on /odom, which the node provides.
    #
    # start_gt_tf_bridge_cmd = Node(
    #     package='ros_gz_bridge', executable='parameter_bridge', name='gt_tf_bridge',
    #     arguments=['/ground_truth/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
    #     remappings=[('/ground_truth/pose', '/tf')])

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_publish_joint_states_cmd)
    ld.add_action(declare_flatten_2d_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_ground_truth_localization_cmd)
    return ld
