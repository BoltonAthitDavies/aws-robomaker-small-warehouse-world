"""TF tree for the ackermann robot with SLAM localization instead of ground truth.

The counterpart to localization_tf.launch.py, which says of itself that it is "the
file you would REPLACE to swap ground truth for AMCL or for VINS-as-localization".
This is that replacement. Nothing in here knows that Nav2 exists, and nothing in
here reads /ground_truth/odometry.

    ros2 launch aws_robomaker_small_warehouse_world localization_rtabmap.launch.py \
        frontend:=vins mode:=localize database_path:=~/wil_project/map/rtabmap_tiny.db
    ros2 run tf2_tools view_frames

The chain, with exactly one publisher per edge:

    map  --(20 Hz)------------->  odom            rtabmap  (the correction)
    odom --(front-end rate)---->  vio_body        VINS *or* ORB-SLAM3, never both
    vio_body --(static)-------->  base_footprint  this file
    base_footprint --> base_link --> sensors      robot_state_publisher (/tf_static)
    base_link --> wheels, knuckles                robot_state_publisher (/tf, 10 Hz)
    vio_body --> cam{0,1}_optical                 rtab_camera_shim.py (/tf_static)

WHY THE FRONT-ENDS ARE RENAMED
    VINS publishes world -> body; ORB-SLAM3 publishes orbslam3_world ->
    base_footprint. Nav2 and rtabmap both expect map -> odom -> <robot>. Rather than
    bridge three different namings with static identities, each front-end is told
    its frame names directly -- both expose world_frame_id / body_frame_id as ROS
    parameters -- so they publish odom -> vio_body and the rest of the tree is
    front-end independent. `frontend:=` then changes exactly one thing: which
    executable runs.

    ORB-SLAM3's body_frame_id DEFAULT IS WRONG and this fixes it as a side effect.
    It defaults to 'base_footprint', but slam_wrapper.cpp computes T_w_b using
    T_b_c0 from the config's IMU.T_b_c1 -- so the pose it publishes is the IMU body,
    not base_footprint. Under its own default it therefore places the robot 0.338 m
    forward and 0.192 m high. Naming it vio_body and bridging explicitly is correct
    for both front-ends and wrong for neither.

THE 0.338 / 0.192 CONSTANT
    models/ackermann_robot/model.sdf puts imu_link at base_footprint + (0.338, 0,
    0.192) with zero rpy (base_footprint -> base_link 0.097, base_link -> imu_link
    (0.338, 0, 0.095)). Cross-checked independently: the URDF chain
    base_footprint -> cam0 minus base_footprint -> imu_link equals
    config/wil_sim/stereo_imu.yaml's body_T_cam0 translation (0.158, 0.0936, 0.2475)
    exactly, and that matrix's rotation equals cam0_joint -> cam0_undis_joint. So
    the two paths to the camera agree, which makes

        ros2 run tf2_ros tf2_echo cam0_undis cam0_optical

    an assertion that must return IDENTITY at runtime: the URDF reaches that frame
    through base_footprint, the shim reaches it through vio_body and the calibration
    file. It is a free, continuous check on the whole chain -- and it is exactly the
    check that would have caught the cv2.FileStorage extrinsics corruption
    (rtabmap_ros/MEMORY_CLAUDE §0 fix 13) in seconds instead of after a retracted
    measurement campaign.

NO GROUND TRUTH
    ground_truth_localization is deliberately NOT started, so nothing publishes the
    old static map->odom identity and nothing publishes /odom. If you are driving
    the live sim, pass bridge_ground_truth:='False' to small_warehouse.launch.py as
    well, so /ground_truth/odometry cannot be consumed by accident.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stale_process_reaper import reaper  # noqa: E402

# base_footprint expressed in the IMU body frame. See THE 0.338 / 0.192 CONSTANT.
BODY_TO_BASE = ('-0.338', '0.0', '-0.192')

# rtab_sim.launch.py lives in the workspace, not in this package's share dir: it is
# not a colcon package, it is a loose launch file beside the shim it starts.
DEFAULT_RTAB_SIM = os.path.expanduser(
    '~/wil_project/rtabmap_ros/rtab_sim.launch.py')


def launch_setup(context, *args, **kwargs):
    frontend = LaunchConfiguration('frontend').perform(context).lower()
    if frontend not in ('vins', 'orbslam3'):
        raise RuntimeError(
            f"frontend must be vins|orbslam3, got {frontend!r}")

    mode = LaunchConfiguration('mode').perform(context)
    database = LaunchConfiguration('database_path').perform(context)
    rtab_sim = os.path.expanduser(
        LaunchConfiguration('rtab_sim_launch').perform(context))
    if not os.path.exists(rtab_sim):
        raise RuntimeError(f'rtab_sim_launch not found: {rtab_sim}')

    print(f'[localization_rtabmap] frontend = {frontend}')
    print(f'[localization_rtabmap] mode     = {mode}')
    print(f'[localization_rtabmap] database = {database}')
    print('[localization_rtabmap] NO ground truth: '
          'ground_truth_localization is not started')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # VINS subscribes to raw sensor_msgs/Image and has NO image_transport support
    # (grep CompressedImage in vins_estimator.cpp: nothing). It therefore works
    # against the live sim, which bridges raw when compressed_images:=False, and
    # CANNOT consume the recorded bags at all -- those carry only
    # /camN/image_raw/compressed. Decompressing here rather than repointing VINS at
    # the shim's /stereo/*/image_rect keeps VINS' input byte-identical to the live
    # path, so a bag-replay run stays comparable to the recorded baselines.
    # ORB-SLAM3 needs none of this: it takes image_transport:=compressed directly.
    republish = []
    if _truthy(context, 'decompress') and frontend == 'vins':
        for cam in ('cam0', 'cam1'):
            republish.append(Node(
                package='image_transport', executable='republish',
                name=f'{cam}_republish', output='screen',
                arguments=['compressed', 'raw'],
                remappings=[('in/compressed', f'/{cam}/image_raw/compressed'),
                            ('out', f'/{cam}/image_raw')],
                parameters=[{'use_sim_time': use_sim_time}]))
        print('[localization_rtabmap] decompressing /camN/image_raw/compressed '
              '-> /camN/image_raw for VINS (bag replay)')

    if frontend == 'vins':
        # Started directly rather than via vins_fusion_ros2.launch.py, which
        # hardcodes the EuRoC config and exposes no config_file argument.
        cfg = os.path.join(
            get_package_share_directory('vins_fusion_ros2'),
            'config', 'wil_sim', 'stereo_imu.yaml')
        front = Node(
            package='vins_fusion_ros2', executable='vins_fusion_ros2_node',
            name='vins_fusion_ros2_node', output='screen', emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time,
                         'config_file': cfg,
                         'world_frame_id': 'odom',
                         'body_frame_id': 'vio_body'}])
    else:
        cfg = os.path.join(
            get_package_share_directory('orbslam3_ros2'),
            'config', 'wil_sim', 'stereo_imu.yaml')
        vocab = os.path.expanduser(
            '~/wil_project/thirdparty/ORB_SLAM3/Vocabulary/ORBvoc.txt')
        front = Node(
            package='orbslam3_ros2', executable='orbslam3_node',
            name='orbslam3', output='screen', emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time,
                         'config_file': cfg,
                         'vocabulary_file': vocab,
                         'use_imu': True,
                         'imu_accel_scale': 1.0,
                         'use_viewer': False,
                         'image0_topic': '/cam0/image_raw',
                         'image1_topic': '/cam1/image_raw',
                         'imu_topic': '/imu',
                         'image_transport':
                             LaunchConfiguration('image_transport'),
                         'publish_tf': True,
                         'world_frame_id': 'odom',
                         'body_frame_id': 'vio_body'}])

    rtab = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtab_sim),
        launch_arguments={
            'mode': mode,
            'database_path': database,
            'odom_frame': 'odom',
            'body_frame': 'vio_body',
            'base_frame': 'base_footprint',
            'ground_height': LaunchConfiguration('ground_height'),
            'detection_rate': LaunchConfiguration('detection_rate'),
            'shim_rate': LaunchConfiguration('shim_rate'),
            'read_only': LaunchConfiguration('read_only'),
            'optimize_max_error': LaunchConfiguration('optimize_max_error'),
            'rviz': 'false',          # the composing launch file owns RViz
            'output_path': '',
        }.items())

    if not _truthy(context, 'start_frontend'):
        return [rtab]
    return republish + [front, rtab]


def _truthy(context, name):
    return LaunchConfiguration(name).perform(context).lower() in (
        'true', '1', 'yes')


def generate_launch_description():
    pkg_share = get_package_share_directory(
        'aws_robomaker_small_warehouse_world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_joint_states = LaunchConfiguration('publish_joint_states')
    reap_stale = LaunchConfiguration('reap_stale')

    with open(os.path.join(pkg_share, 'urdf', 'ackermann_robot.urdf')) as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}])

    joint_state_publisher = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(publish_joint_states))

    # The one edge neither the front-end nor the URDF publishes. Static because the
    # IMU is bolted to the chassis; latched, so it survives a late subscriber.
    body_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='vio_body_to_base_footprint', output='screen',
        arguments=['--x', BODY_TO_BASE[0], '--y', BODY_TO_BASE[1],
                   '--z', BODY_TO_BASE[2],
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'vio_body',
                   '--child-frame-id', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}])

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('reap_stale', default_value='True'),
        DeclareLaunchArgument('publish_joint_states', default_value='True'),
        DeclareLaunchArgument(
            'frontend', default_value='vins',
            description='vins | orbslam3. Both are renamed to publish '
                        'odom -> vio_body, so this changes only which '
                        'executable runs.'),
        DeclareLaunchArgument(
            'mode', default_value='localize',
            description='Passed to rtab_sim.launch.py: construct | extend | '
                        'localize.'),
        DeclareLaunchArgument(
            'database_path', default_value='~/output/rtabmap/wil_sim.db',
            description='RTAB-Map database. For extend/localize this is a map '
                        'that already exists -- copy it first, there is no '
                        'save_database service and the DB is written on exit.'),
        DeclareLaunchArgument(
            'read_only', default_value='false',
            description='mode:=localize only; opens the database read-only.'),
        DeclareLaunchArgument(
            'start_frontend', default_value='True',
            description='False runs the TF tree and rtabmap without a front-end, '
                        'for driving odom -> vio_body from something else '
                        '(e.g. script/gt_tf_publisher.py as a test fixture).'),
        DeclareLaunchArgument(
            'decompress', default_value='false',
            description='VINS only. Republish /camN/image_raw/compressed as raw, '
                        'which VINS needs because it has no image_transport '
                        'support. Required for BAG REPLAY; leave false against '
                        'the live sim, which already bridges raw when '
                        'compressed_images:=False.'),
        DeclareLaunchArgument(
            'image_transport', default_value='compressed',
            description='orbslam3 only. The bags carry only '
                        '/camN/image_raw/compressed.'),
        DeclareLaunchArgument(
            'ground_height', default_value='0.29',
            description='Grid/MaxGroundHeight in base_footprint. 0.29 = the '
                        'measured 0.1 in the IMU body frame plus the 0.192 m '
                        'the body sits above base_footprint.'),
        DeclareLaunchArgument(
            'optimize_max_error', default_value='10.0',
            description='Loop-closure rejection threshold. Defaults to 10 here, '
                        'not rtabmap\'s 3.0: a drifting VIO front-end needs a '
                        'larger correction to close a lap, and at 3.0 every '
                        'closure VINS proposed on dataset_static_tiny_repeat was '
                        'rejected (36 detected, 0 accepted).'),
        DeclareLaunchArgument('detection_rate', default_value='1.0'),
        DeclareLaunchArgument('shim_rate', default_value='4.0'),
        DeclareLaunchArgument(
            'rtab_sim_launch', default_value=DEFAULT_RTAB_SIM,
            description='rtab_sim.launch.py lives in the workspace, not in this '
                        "package's share directory."),
    ])

    # Exactly one publisher per edge is the whole contract of this file, and a
    # survivor from the last run breaks it in a way that reads as a SLAM bug
    # rather than as a duplicate node.
    ld.add_action(OpaqueFunction(function=reaper('localization'),
                                 condition=IfCondition(reap_stale)))
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(body_to_base)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
