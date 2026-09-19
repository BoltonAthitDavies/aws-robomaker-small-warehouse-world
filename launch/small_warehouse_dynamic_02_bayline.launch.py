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
import re
import shutil
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# The launch directory is not an importable Python package, so put it on the path
# to reach stale_process_reaper.py sitting next to this file. __file__ resolves to
# the source tree under --symlink-install and to share/ otherwise, i.e. to
# whichever copy is actually being run, so this needs no ament lookup.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stale_process_reaper import reaper  # noqa: E402

# Where the speed-patched copy of the robot model is written. Stable rather than a
# fresh mkdtemp per run, so repeated launches do not litter /tmp and so the path in
# the log stays the same while you are debugging.
_MODEL_OVERRIDE_ROOT = os.path.join(
    tempfile.gettempdir(), 'aws_warehouse_ackermann_override')


def _override_speed_limits(context, *args, **kwargs):
    """Shadow the installed ackermann_robot model with a speed-patched copy.

    AckermannSteering reads min/max velocity and acceleration once at plugin load
    and exposes no topic or service to change them, and sdformat 12 has no
    parameter substitution. So the only way to drive them from a launch argument
    is to rewrite the SDF before gz parses it, then put the rewritten copy earlier
    on IGN_GAZEBO_RESOURCE_PATH than the installed one, since gz takes the first
    `model://` match it finds.

    The copy is regenerated from the INSTALLED model every launch, so editing the
    real model.sdf (and rebuilding) still takes effect and this cannot go stale.
    When the requested values already match the model, nothing is generated and
    nothing is shadowed -- the common case leaves the resource path untouched.
    """
    pkg_share = get_package_share_directory('aws_robomaker_small_warehouse_world')
    src_dir = os.path.join(pkg_share, 'models', 'ackermann_robot')
    src_sdf = os.path.join(src_dir, 'model.sdf')

    speed = float(context.perform_substitution(LaunchConfiguration('max_speed')))
    accel = float(context.perform_substitution(LaunchConfiguration('max_accel')))

    with open(src_sdf) as f:
        sdf = f.read()

    def current(tag):
        match = re.search(r'<{0}>\s*([-\d.eE+]+)\s*</{0}>'.format(tag), sdf)
        if match is None:
            raise RuntimeError(
                '<{}> not found in {} -- max_speed/max_accel cannot be applied. '
                'Was the AckermannSteering block edited?'.format(tag, src_sdf))
        return float(match.group(1))

    if (speed, accel) == (current('max_velocity'), current('max_acceleration')):
        return []

    for tag, value in (('min_velocity', -speed), ('max_velocity', speed),
                       ('min_acceleration', -accel), ('max_acceleration', accel)):
        sdf = re.sub(r'<{0}>\s*[-\d.eE+]+\s*</{0}>'.format(tag),
                     '<{0}>{1}</{0}>'.format(tag, value), sdf, count=1)

    out_dir = os.path.join(_MODEL_OVERRIDE_ROOT, 'ackermann_robot')
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'model.sdf'), 'w') as f:
        f.write(sdf)
    # model.config is what makes the directory resolvable as a `model://` URI.
    shutil.copyfile(os.path.join(src_dir, 'model.config'),
                    os.path.join(out_dir, 'model.config'))

    # Prepend, so this copy wins over the installed one. Fortress reads
    # IGN_GAZEBO_RESOURCE_PATH; GZ_SIM_RESOURCE_PATH is set too because the
    # package's env-hook populates both.
    actions = [LogInfo(msg='max_speed={} m/s, max_accel={} m/s^2: gz will load a '
                           'patched robot model from {} instead of the installed '
                           'one.'.format(speed, accel, out_dir))]
    for var in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH'):
        existing = os.environ.get(var, '')
        actions.append(SetEnvironmentVariable(
            var,
            _MODEL_OVERRIDE_ROOT + (os.pathsep + existing if existing else '')))
    return actions


def _sensor_bridges(context, *args, **kwargs):
    """The camera and IMU bridges, in one of two shapes set by compressed_images.

    parameter_bridge is a message-TYPE bridge: it knows sensor_msgs/Image and
    nothing whatever about image_transport, so it can only ever produce RAW
    images. That is the one place the simulated pipeline differs from the real
    one -- the real rig's bags carry /camN/image_raw/compressed, and
    orbslam3_ros2/launch/wil_stereo_imu.launch.py subscribes to them with
    image_transport="compressed".

    compressed_images:=True closes that gap by bringing the images across through
    ros_gz_image's image_bridge instead, which publishes through image_transport
    and therefore advertises /camN/image_raw AND .../compressed (and .../theora)
    from a single node. The sim can then be consumed by exactly the launch
    configuration the real rig uses.

    The images MOVE between the two bridges rather than being published by both:
    image_bridge names its ROS topic after the gz topic it reads, so leaving the
    Image lines in parameter_bridge as well would put two publishers on
    /cam0/image_raw and every subscriber would see each frame twice. camera_info
    and the IMU stay on parameter_bridge in both shapes -- image_bridge carries
    images and nothing else.

    JPEG quality belongs to the compressed publisher, not to the bridge, so it is
    tuned with a parameter on the image_bridge node (default quality 80):
        ros2 param list /cam_image_bridge      # exact name of the jpeg_quality param
    """
    compressed = IfCondition(
        LaunchConfiguration('compressed_images')).evaluate(context)

    # '[' is gz -> ROS only, which is all a sensor ever needs.
    #
    # Note the camera_info names: gz derives them by REPLACING the last
    # element of the sensor's <topic>, not by appending, so a sensor on
    # 'cam0/image_raw' publishes info on '/cam0/camera_info'.
    bridge_args = [
        '/cam0/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        '/cam1/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        # Capitalised 'IMU' -- that is the Fortress-era gz message name.
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
    ]
    if not compressed:
        bridge_args = [
            '/cam0/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cam1/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        ] + bridge_args

    nodes = [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        output='screen',
        arguments=bridge_args)]

    if compressed:
        # The gz topic name doubles as the ROS base topic name, so these are the
        # same '/camN/image_raw' the raw shape produces -- only now with the
        # image_transport suffixes hanging off them.
        nodes.append(Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='cam_image_bridge',
            output='screen',
            arguments=['/cam0/image_raw', '/cam1/image_raw']))

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('aws_robomaker_small_warehouse_world')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    verbosity = LaunchConfiguration('verbosity')
    bridge_sensors = LaunchConfiguration('bridge_sensors')
    bridge_cmd_vel = LaunchConfiguration('bridge_cmd_vel')
    bridge_ground_truth = LaunchConfiguration('bridge_ground_truth')
    bridge_wheel_odom = LaunchConfiguration('bridge_wheel_odom')
    bridge_joint_states = LaunchConfiguration('bridge_joint_states')
    cmd_vel_bridge_topic = LaunchConfiguration('cmd_vel_bridge_topic')
    bridge_model_poses = LaunchConfiguration('bridge_model_poses')
    robot_name = LaunchConfiguration('robot_name')
    reap_stale = LaunchConfiguration('reap_stale')

    declare_reap_stale_cmd = DeclareLaunchArgument(
        'reap_stale', default_value='True',
        description='Before starting anything, kill gz and ros_gz_bridge processes left over from a previous '
                    'launch -- Ctrl-C does not reliably reap them and the leftovers '
                    'fight the new run over /clock and /tf. See '
                    'launch/stale_process_reaper.py. Set False only if you are '
                    'deliberately running a second stack on this ROS_DOMAIN_ID.')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Bridge /clock from Gazebo so ROS nodes can use simulation time')

    declare_bridge_sensors_cmd = DeclareLaunchArgument(
        'bridge_sensors',
        default_value='True',
        description="Bridge the ackermann robot's cameras and IMU onto ROS topics")

    declare_compressed_images_cmd = DeclareLaunchArgument(
        'compressed_images',
        default_value='True',
        description='Publish the cameras through image_transport (ros_gz_image '
                    'image_bridge) so /camN/image_raw/compressed exists alongside '
                    'the raw topic, exactly as the real rig does. On by default '
                    'so the simulated and real pipelines are the same shape; the '
                    'raw topic is published either way, so consumers that want '
                    'raw need no change. Set False to skip the JPEG encode (one '
                    'per frame per camera). See _sensor_bridges().')

    declare_bridge_cmd_vel_cmd = DeclareLaunchArgument(
        'bridge_cmd_vel',
        default_value='True',
        description="Bridge ROS /cmd_vel to the robot's Gazebo cmd_vel so ROS teleop can drive it")

    declare_bridge_ground_truth_cmd = DeclareLaunchArgument(
        'bridge_ground_truth',
        default_value='True',
        description="Bridge the robot's true-pose odometry onto ROS as /ground_truth/odometry")

    declare_bridge_wheel_odom_cmd = DeclareLaunchArgument(
        'bridge_wheel_odom',
        default_value='False',
        description="Bridge AckermannSteering's wheel (dead-reckoning) odometry "
                    'onto ROS as /model/<robot_name>/odometry. Off by default: it '
                    'is not a sensor and not the reference, and an extra Odometry '
                    'topic on the graph gets auto-adopted by viewer.py as an '
                    'estimator feed.')

    declare_bridge_joint_states_cmd = DeclareLaunchArgument(
        'bridge_joint_states',
        default_value='False',
        description='Bridge the wheel and steering encoders onto ROS as '
                    '/model/<robot_name>/joint_state (sensor_msgs/JointState): '
                    'per-joint position, velocity and effort. Position times the '
                    '0.0585 m wheel radius is distance travelled by that wheel.')

    declare_cmd_vel_bridge_topic_cmd = DeclareLaunchArgument(
        'cmd_vel_bridge_topic',
        default_value='/cmd_vel',
        description='ROS topic the cmd_vel bridge listens on. Point it at '
                    '/cmd_vel_exec to insert script/drivetrain_sim.py between the '
                    'teleop and the simulator, giving the drivetrain a deadband '
                    'and motor lag. Teleop keeps publishing /cmd_vel either way.')

    declare_bridge_model_poses_cmd = DeclareLaunchArgument(
        'bridge_model_poses',
        default_value='False',
        description='Bridge gz dynamic model poses onto ROS as a TFMessage, so a 2D '
                    'viewer can draw MOVING models at their live pose instead of '
                    'the pose authored in the world file. Off by default: it is '
                    'only useful if something in the world actually moves.')

    # The robot's gz topics are scoped by the name the WORLD gives it, which is the
    # <include><name> in small_warehouse.world, not the <model name> in model.sdf.
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='ackermann_robot_001',
        description='World-scoped name of the robot model, used to build its gz topic names')

    # Defaults must match the AckermannSteering block in models/ackermann_robot/model.sdf,
    # otherwise every launch needlessly generates a patched copy of it.
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

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            pkg_share, 'worlds', 'small_warehouse_dynamic', 'small_warehouse_dynamic_02_bayline.world'),
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
    # Built by an OpaqueFunction because compressed_images decides how many nodes
    # there are and what arguments they take, and Humble's parameter_bridge has
    # no YAML config option to push that decision out of the launch file.
    start_sensor_bridge_cmd = OpaqueFunction(
        function=_sensor_bridges,
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
        remappings=[(['/model/', robot_name, '/cmd_vel'], cmd_vel_bridge_topic)],
        condition=IfCondition(bridge_cmd_vel))

    # Ground truth, from the OdometryPublisher system in model.sdf. Separate from
    # sensor_bridge because it is not a sensor: it is the reference you SCORE the
    # sensors against, and you may well want it off while recording a bag that is
    # meant to look like real hardware.
    #
    # Not to be confused with the AckermannSteering wheel odometry on
    # /model/<robot_name>/odometry, which is dead reckoning and is not bridged.
    start_wheel_odom_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wheel_odom_bridge',
        output='screen',
        arguments=[['/model/', robot_name,
                    '/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry']],
        condition=IfCondition(bridge_wheel_odom))

    start_joint_state_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        output='screen',
        arguments=[['/model/', robot_name,
                    '/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model']],
        condition=IfCondition(bridge_joint_states))

    start_ground_truth_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_bridge',
        output='screen',
        arguments=[
            '/ground_truth/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        condition=IfCondition(bridge_ground_truth))

    # THIS WORLD USES pose/info, NOT dynamic_pose/info -- deliberately.
    #
    # dynamic_pose/info carries only NON-STATIC entities. That was the right choice
    # while the bay props were rigid bodies, but they are now <static>true</static>
    # driven kinematically by KinematicTrajectory (static props cost no contact
    # solving: 30 dynamic props on the floor measured RTF 0.294 vs 1.005 static,
    # which is what dragged the 30 Hz cameras down to ~9 Hz). Static entities are
    # absent from dynamic_pose/info, so subscribing to it shows all 30 props FROZEN
    # at their spawn poses while gz actually moves them -- worse than no overlay,
    # because it looks authoritative.
    #
    # pose/info carries everything, moving or not. MEASURED side by side here:
    #   dynamic_pose/info   58 Hz    8 entities/msg    ~115 KB/s
    #   pose/info           58 Hz  ~149 entities/msg   ~1.0 MB/s
    # 9x the traffic, and worth it -- this bridge is opt-in
    # (bridge_model_poses, default False) so it costs nothing unless asked for.
    #
    # Pose_V maps onto tf2_msgs/TFMessage, and each entry's child_frame_id is the
    # model name. This is NOT published on /tf and is not meant for TF: it is a pose
    # feed that happens to reuse a convenient message type.
    start_model_pose_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='model_pose_bridge',
        output='screen',
        arguments=[
            '/world/default/pose/info@tf2_msgs/msg/TFMessage'
            '[ignition.msgs.Pose_V',
        ],
        condition=IfCondition(bridge_model_poses))

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_verbosity_cmd)
    ld.add_action(declare_bridge_sensors_cmd)
    ld.add_action(declare_compressed_images_cmd)
    ld.add_action(declare_bridge_cmd_vel_cmd)
    ld.add_action(declare_bridge_ground_truth_cmd)
    ld.add_action(declare_bridge_wheel_odom_cmd)
    ld.add_action(declare_bridge_joint_states_cmd)
    ld.add_action(declare_cmd_vel_bridge_topic_cmd)
    ld.add_action(declare_bridge_model_poses_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_max_speed_cmd)
    ld.add_action(declare_max_accel_cmd)
    ld.add_action(declare_reap_stale_cmd)

    # Must run BEFORE gz starts, and before the shadowing below touches
    # anything: a surviving gz from the last run holds the same fixed topic
    # names this one is about to advertise.
    ld.add_action(OpaqueFunction(function=reaper('sim'),
                                 condition=IfCondition(reap_stale)))

    # Must run BEFORE gz starts: it sets the resource path the gz process inherits.
    ld.add_action(OpaqueFunction(function=_override_speed_limits))

    ld.add_action(start_gz_gui_cmd)
    ld.add_action(start_gz_server_cmd)
    ld.add_action(start_clock_bridge_cmd)
    ld.add_action(start_sensor_bridge_cmd)
    ld.add_action(start_cmd_vel_bridge_cmd)
    ld.add_action(start_ground_truth_bridge_cmd)
    ld.add_action(start_wheel_odom_bridge_cmd)
    ld.add_action(start_joint_state_bridge_cmd)
    ld.add_action(start_model_pose_bridge_cmd)

    return ld
