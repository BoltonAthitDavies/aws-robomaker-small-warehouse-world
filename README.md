# AWS RoboMaker Small Warehouse World

![Gazebo01](docs/images/small_warehouse_gazebo.png)


This Gazebo world is well suited for organizations who are building and testing robot applications for warehouse and logistics use cases. 

> **This package targets Gazebo (Ignition) Fortress / gz-sim 6, not Gazebo Classic.**
> On Fortress the simulator CLI is `ign gazebo` — note that `/usr/bin/gz` is Gazebo
> *Classic*'s CLI on Ubuntu 22.04, so `gz sim ...` will not work here.

## 3D Models included in this Gazebo World

| Model (/models)       | Picture           |
| :------------- |:-------------:|
| **aws_robomaker_warehouse_Bucket_01**    | ![Model: Buckets](docs/images/models_buckets.png)
| **aws_robomaker_warehouse_ClutteringA_01, aws_robomaker_warehouse_ClutteringC_01, aws_robomaker_warehouse_ClutteringD_01**     | ![Model: Box Clusters](docs/images/models_boxes.png) |
| **aws_robomaker_warehouse_DeskC_01**    | ![Model: Desk](docs/images/models_desk.png)
| **aws_robomaker_warehouse_GroundB_01**    | ![Model: Ground Paint](docs/images/models_warehouse_ground_paint.png)
| **aws_robomaker_warehouse_TrashCanC_01**   | ![Model: Humans](docs/images/models_trashcan.png)
| **aws_robomaker_warehouse_Lamp_01**    | ![Model: Ceiling Lamp](docs/images/models_ceiling_lamp.png)
| **aws_robomaker_warehouse_PalletJackB_01**    | ![Model: Pallet Jack](docs/images/models_lift.png)
| **aws_robomaker_warehouse_ShelfD_01, aws_robomaker_warehouse_ShelfE_01, aws_robomaker_warehouse_ShelfF_01**    | ![Model: Pallet Jack](docs/images/models_shelves.png)

## Building and Launching the Gazebo World with your ROS Applications

* Create or update a **.rosinstall** file in the root directory of your ROS workspace. Add the following line to **.rosintall**:
    ```
    - git: {local-name: src/aws-robomaker-small-warehouse-world, uri: 'https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git', version: ros2}
    ```
* Change the directory to your ROS workspace and run `rosws update`

* Add the following include to the ROS2 launch file you are using:
    ```python
    import os

    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    def generate_launch_description():
        warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
        warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')

        warehouse_world_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([warehouse_launch_path, '/small_warehouse.launch.py'])
        )

        ld = LaunchDescription()

        ld.add_action(warehouse_world_cmd)

        return ld
    ```

* Build your application using `colcon`
    ```bash
    rosws update
    rosdep install --from-paths . --ignore-src -r -y
    colcon build
    ```

## Example: Running this world directly in Gazebo without a ROS application

To open this world in Gazebo, change the directory to this repository's root folder and run:

```bash
export IGN_GAZEBO_RESOURCE_PATH=`pwd`/models
ign gazebo -r worlds/small_warehouse/small_warehouse.world
```

`IGN_GAZEBO_RESOURCE_PATH` must contain the `models` directory — that is the parent
the `model://` URIs in the worlds and meshes resolve against. After building the
package and sourcing the workspace, the environment hook sets this for you.

## Example: Running this world directly using ROS without a simulated robot

To launch this base Gazebo world without a robot, clone this repository and run the following commands. **Note: ROS 2 and Gazebo Fortress must already be installed on the host.**

```bash
# build for ROS2
rosdep install --from-paths . --ignore-src -r -y
colcon build

# run in ROS2
source install/setup.sh
ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py
```

### Launch arguments

| Argument | Default | Description |
| :------- | :------ | :---------- |
| `world` | `worlds/small_warehouse/small_warehouse.world` | Full path to the world file to load |
| `headless` | `False` | Run the Gazebo server only, without the GUI |
| `use_sim_time` | `True` | Start a `ros_gz_bridge` for `/clock` so ROS nodes can use simulation time |
| `verbosity` | `3` | Gazebo console verbosity, 0-4. Use `4` when debugging resource resolution. |
| `bridge_sensors` | `True` (`False` for `no_roof_*`) | Start a `ros_gz_bridge` for the ackermann robot's cameras and IMU. Defaults off for `no_roof_small_warehouse`, which contains no robot. |
| `bridge_cmd_vel` | `True` (`False` for `no_roof_*`) | Start a `ros_gz_bridge` so ROS `/cmd_vel` drives the robot. Same defaulting rule as `bridge_sensors`. |
| `robot_name` | `ackermann_robot_001` | World-scoped name of the robot, used to build its Gazebo topic names. This is the world's `<include><name>`, not the `<model name>` in `model.sdf`. |
| `bridge_ground_truth` | `True` (`False` for `no_roof_*`) | Publish the robot's true-pose odometry on ROS as `/ground_truth/odometry`. |
| `max_speed` | `10.0` | Robot speed cap in m/s, forward and reverse. |
| `max_accel` | `3.0` | Robot acceleration cap in m/s². Decides how much run-up the top speed needs. |

Both launch files accept all nine. For example:

```bash
ros2 launch aws_robomaker_small_warehouse_world no_roof_small_warehouse.launch.py headless:=True
```

Because gz-sim has no `gazebo_ros_init` equivalent, `/clock` only reaches ROS through
the bridge this launch file starts. Set `use_sim_time:=False` if you bridge the clock
yourself.

### Robot sensors

`small_warehouse.world` includes the `ackermann_robot` model, which carries a stereo
camera pair and an IMU. With `bridge_sensors:=True` (the default) these appear on ROS as:

| Topic | Type | Rate |
| :---- | :--- | :--- |
| `/cam0/image_raw`, `/cam1/image_raw` | `sensor_msgs/Image` | 20 Hz, 1280x720, 90° HFOV |
| `/cam0/camera_info`, `/cam1/camera_info` | `sensor_msgs/CameraInfo` | 20 Hz |
| `/imu` | `sensor_msgs/Imu` | 200 Hz, REP-103 axes |

cam1 sits 93.6 mm to cam0's right — the real WiL rig's calibrated baseline. The matching
VINS-Fusion configuration is `vins_fusion_ros2/config/wil_sim/stereo_imu.yaml`.

Fortress ships no wide-angle camera implementation, so these are **ideal pinhole** cameras
and cannot reproduce the real rig's ~180° fisheye.

### Driving the robot

With `bridge_cmd_vel:=True` (the default), ROS `/cmd_vel` is bridged to the robot's
Gazebo `cmd_vel`, so any standard teleop works. In a second terminal:

```bash
ros2 run aws_robomaker_small_warehouse_world teleop
```

That is a thin wrapper around `teleop_twist_keyboard` with defaults matched to this
robot's steering geometry (0.5 m/s, 0.8 rad/s). Override them positionally:

```bash
ros2 run aws_robomaker_small_warehouse_world teleop 1.0       # faster
ros2 run aws_robomaker_small_warehouse_world teleop 1.0 1.2   # faster + sharper
```

To change what a bare `teleop` does, edit `DEFAULT_SPEED` / `DEFAULT_TURN` at the top of
[`scripts/teleop`](scripts/teleop). The equivalent bare command, if you would rather not
use the wrapper, is:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.5 -p turn:=0.8
```

No topic remapping is needed either way — the node publishes the relative name `cmd_vel`,
which resolves to `/cmd_vel`, which is what the bridge subscribes to.

> Use `ros2 run`, not `ros2 launch`. A node started by `ros2 launch` never receives the
> terminal's keystrokes: launch spawns processes with `stdin` connected to a pipe, and
> `emulate_tty` allocates a PTY for stdout/stderr only. `ros2 run` execs the process
> directly, so it inherits your terminal.

**This robot is Ackermann-steered and cannot turn on the spot**, so part of the stock key
map does nothing. You must hold a forward or reverse key to steer:

| Keys | Effect |
| :--- | :----- |
| `i` / `,` | forward / reverse |
| `u` `o` | forward-left / forward-right |
| `m` `.` | reverse-left / reverse-right |
| `j` / `l` | **nothing** — pure rotation, with no linear velocity to steer against |
| `U I O J K L M < >` | **nothing** — shifted keys are holonomic strafe; y-velocity is ignored |
| `k` or any unbound key | stop |
| `q`/`z`, `w`/`x`, `e`/`c` | scale both / linear only / angular only, by 10% |

The suggested `turn:=0.8` comes from the geometry in `model.sdf`: `steering_limit` is
0.6109 rad (35°) over a `wheel_base` of 0.42 m, so the yaw rate is capped at
`ω = v·tan(δ)/L = 1.667·v` — 0.83 rad/s at the default 0.5 m/s. The node's own default of
`turn:=1.0` just saturates the steering. Minimum turning radius is `L/tan(δ) = 0.60 m`.

`speed`, `turn`, `stamped` and `frame_id` are declared **read-only**, so they can only be
set at startup as above — `ros2 param set` will be rejected, which is exactly why the
defaults live in a wrapper script. Adjust speed while driving with the
`q`/`z`/`w`/`x`/`e`/`c` keys instead.

Note there is no command timeout: the last twist persists until you send another. If the
terminal loses focus mid-command the robot keeps driving, so press `k` to stop.

To drive without the bridge — or to check the Gazebo side directly — publish natively:

```bash
ign topic -t /model/ackermann_robot_001/cmd_vel -m ignition.msgs.Twist \
  -p 'linear: {x: 0.6}, angular: {z: 0.35}'
```

### Speed limits

The robot is capped at `max_speed` (default 10 m/s) and `max_accel` (3 m/s²). Both are
launch arguments:

```bash
ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py \
  max_speed:=4.0 max_accel:=6.0
```

`AckermannSteering` reads these once at plugin load and offers no runtime API, and
sdformat 12 has no parameter substitution, so the launch file implements this by writing
a patched copy of the model to `/tmp/aws_warehouse_ackermann_override` and putting it
earlier on `IGN_GAZEBO_RESOURCE_PATH`. It logs when it does so. When the arguments match
the values already in `model.sdf` nothing is generated and the installed model is used
directly, so the default path involves no shadowing.

Two things that bite in practice:

- **Acceleration, not top speed, is usually the limit.** At 3 m/s² reaching 10 m/s takes
  3.3 s and about 17 m of run-up — more than most straight runs in this warehouse. Raise
  `max_accel` together with `max_speed` or you will never see the top speed.
- **`/model/<name>/odometry` is dead reckoning**, integrated from the wheel joints. It
  reports the *commanded* speed even when the tyres are slipping and the robot is barely
  moving. For real motion use `/ground_truth/odometry` (below).

Measured true ground speed against commanded, from a standing start:

| Commanded | True |
| :-- | :-- |
| 0.5 m/s | 0.40 m/s |
| 1.0 m/s | 0.89 m/s |
| 2.0 m/s | 1.52 m/s |
| 5.0 m/s | 3.41 m/s |

The shortfall is mostly the acceleration ramp inside the measurement window rather than
slip. Note the wheel joints deliberately carry zero damping and friction — see the note
above `front_left_wheel_joint` in `model.sdf`, since restoring the original 0.2/0.1 pins
the robot at ~0.5 m/s no matter what you command.

### Ground truth

`/ground_truth/odometry` (`nav_msgs/Odometry`, 50 Hz, `odom` → `base_footprint`) is the
robot's **exact** pose and twist. It comes from Gazebo's `OdometryPublisher` system, which
reads the model's true world pose out of the ECM and finite-differences it — so unlike the
wheel odometry it cannot drift and cannot be fooled by slip. No noise is added.

Use it to score VINS, or anything else, against reality:

```bash
ros2 topic echo /ground_truth/odometry --field pose.pose.position
```

There are now **two** odometry sources and they are not interchangeable:

| Topic | Source | Use it for |
| :---- | :----- | :--------- |
| `/ground_truth/odometry` (ROS) | true world pose | the reference you measure against |
| `/model/<name>/odometry` (Gazebo only, not bridged) | wheel joint integration | simulating a real robot's odometry, drift and all |

Verified: while driving, `/ground_truth/odometry` matched `/world/default/pose/info` to
four decimal places, while the wheel odometry over-reported the distance by about 6%.

Two gotchas worth knowing if you edit the plugin block:

- The plugin **must** be named `ignition::gazebo::systems::OdometryPublisher`. Unlike
  `AckermannSteering`, which registers both aliases, this library registers only the
  `ignition::` one in Fortress 6.17.1. The `gz::sim::` spelling fails with *"Could not
  find a plugin with that name or alias"*, the world still loads, and you simply get no
  topic.
- Do not let `<odom_topic>` default. The default is `/model/<name>/odometry`, which is
  exactly what `AckermannSteering` already publishes, and the two would collide.

### Running on a discrete GPU

Camera rendering dominates this simulation. Measured on an Intel iGPU:

| Configuration | Real-time factor | Cost per simulated second |
| :------------ | :--------------- | :------------------------ |
| Warehouse only, no robot | 0.78 | 1.28 s |
| Warehouse + robot + two 1280x720 cameras @ 20 Hz | 0.18 | 5.56 s |

The cameras account for roughly 77% of the total work, so moving them onto a discrete
GPU is the single biggest speed-up available. Tuning `<max_step_size>` is not worth it by
comparison: even infinitely fast physics only takes 0.18 to about 0.23.

First check what is actually rendering:

```bash
glxinfo -B | grep "OpenGL renderer"
```

`llvmpipe` or `swrast` means CPU software rendering. A vendor name such as
`Mesa Intel(R) UHD Graphics` means an integrated GPU — hardware, but usually the weakest
option on a laptop with hybrid graphics.

On an NVIDIA Optimus laptop the discrete card is often bound to the open-source `nouveau`
driver, which cannot reclock modern cards and is typically *slower* than the iGPU. Check
with `lsmod | grep -E "nvidia|nouveau"`; if you see `nouveau` and `nvidia-smi` does not
exist, install the proprietary driver first:

```bash
ubuntu-drivers devices          # see what is recommended for your card
sudo ubuntu-drivers install
sudo reboot
nvidia-smi                      # must now exist before going further
```

Then launch with PRIME render offload:

```bash
prime-run ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py
```

or, if `prime-run` is not installed (it ships with `nvidia-prime`):

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py
```

The variables are inherited by the `ign gazebo` subprocess that the launch file starts,
so setting them on the `ros2 launch` command is enough. Confirm the offload actually took
effect before trusting any timing:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo -B | grep "OpenGL renderer"
# expect an NVIDIA renderer string, not Intel

ign topic -e -t /stats -n 4 | grep real_time_factor   # compare against the table above
```

Note `nvidia-smi` alone is a weak check: a GPU that is present and idle still reports
fine. The renderer string is what tells you the offload worked.

**Headless is a separate path.** `headless:=True` passes `--headless-rendering`, which
makes Ogre use EGL rather than GLX, and `__GLX_VENDOR_LIBRARY_NAME` does not apply to EGL.
If offload works in GUI mode but not headless, select the EGL vendor explicitly:

```bash
__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
  ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py headless:=True
```

> These GPU instructions are **untested on this machine** — it currently has no NVIDIA
> driver installed, so there was no way to verify them. Everything else in this section
> (the RTF figures, the 77% split) was measured. Treat the offload commands as a starting
> point and use the renderer-string check to confirm.

If you would rather not touch drivers, the zero-risk alternative is to render fewer
pixels: 640x480 is a third of the pixels of 1280x720 and should roughly triple the
real-time factor. Change `<image>` in `models/ackermann_robot/model.sdf` **and** the
matching `fx`/`fy`/`cx`/`cy` in `vins_fusion_ros2/config/wil_sim/cam*_pinhole.yaml`, or
VINS will silently misproject. At 640x480 with the same 90 degree FOV those become
`fx=fy=320, cx=320, cy=240`.

**Visit the [AWS RoboMaker website](https://aws.amazon.com/robomaker/) to learn more about building intelligent robotic applications with Amazon Web Services.**

## Notes
- Lighting might vary on different system(s) (e.g brighter on system without CPU and darker on system with GPU)
- Adjust lighting parameters in .world file as you need
- The two worlds are lit differently on purpose. `no_roof_small_warehouse` uses a
  shadow-casting directional `sun`; `small_warehouse` cannot, because the roof mesh
  would put the whole interior in shadow, so it uses a higher `<scene><ambient>` plus
  a non-shadowing `warehouse_fill` light instead.
- `aws_robomaker_warehouse_Lamp_01` is a lamp *mesh* only — it contains no `<light>`
  and emits nothing. All illumination is defined at world level.
- Each world embeds a `<gui>` block. gz-sim uses that block *instead of*
  `~/.ignition/gazebo/6/gui.config` rather than merging with it, so any GUI plugin you
  want must be listed in the world.
- Both worlds are named `default`, so they cannot be run simultaneously — they would
  contend for the same `/world/default/*` topics and services.
