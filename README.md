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
| `bridge_model_poses` | `False` | Bridge gz's dynamic model poses onto ROS as a `TFMessage`, so `viewer.py --live-poses` can draw MOVING models where they actually are. Only useful if you have made something in the world non-static. |

Both launch files accept all ten. For example:

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

Camera rendering dominates this simulation, but **only while something is subscribed to
the image topics**. gz-sim renders a camera lazily: with no subscriber the sensor costs
essentially nothing. Measured on an Intel iGPU, same world and robot throughout:

| Camera subscribers | Real-time factor |
| :----------------- | :--------------- |
| none (`bridge_sensors:=False`) | **1.00** |
| one image topic bridged | **0.07** |

So the cheapest speed-up by far is simply not to bridge the cameras when you do not need
vision — driving, teleop, vehicle tuning:

```bash
ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py bridge_sensors:=False
```

That runs at real time. Note the trap in the other direction: anything that subscribes
will re-enable rendering, including a stray `ign topic -e -t /cam0/image_raw` or an
`rqt_image_view` you left open in another terminal. If the simulation suddenly crawls,
look for a subscriber before blaming the machine.

Tuning `<max_step_size>` is not worth it by comparison — with the cameras unsubscribed
you are already at real time, and with them subscribed the cost is rendering, not physics.

Once NVIDIA's proprietary driver is installed (`nvidia-smi` exists), **the Gazebo server
already uses the discrete GPU in headless mode with no extra flags**. Headless rendering
goes through EGL, and glvnd tries `/usr/share/glvnd/egl_vendor.d/10_nvidia.json` before
`50_mesa.json`, so NVIDIA wins by default. Confirm rather than assume:

```bash
nvidia-smi | grep -i gazebo                       # gz should appear, holding GPU memory
ls -l /proc/$(pgrep -f '^ign gazebo' | head -1)/fd | grep -o 'renderD[0-9]*'
```

`renderD128` is normally the Intel iGPU and `renderD129` the discrete card; check with
`cat /sys/class/drm/renderD129/device/uevent | grep DRIVER`. An open `renderD129` alone is
weak evidence -- the node gets opened during capability probing even when rendering happens
elsewhere. `nvidia-smi` showing real memory against the process is the reliable signal.

Measured here, RTX 3070 + Intel iGPU, cameras bridged so they actually render:

| How it was launched | RTF | Where the server rendered |
| :------------------ | :-- | :------------------------ |
| `headless:=True`, no flags | **0.90** | NVIDIA, EGL picks it automatically |
| `headless:=True` + offload vars | 0.65 | NVIDIA (same 201 MiB) |
| GUI, no flags | 0.40 | GUI path on Intel |
| GUI + offload vars | 0.27 | NVIDIA, server and GUI |

("offload vars" = `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`. The two
headless rows are 8-sample means; the GUI rows are single runs.)

Two results worth knowing, because both are counter-intuitive:

- **Running the GUI is what costs, not the GPU choice.** Headless is roughly twice the
  real-time factor of any GUI configuration: the GUI's 3D view re-renders the whole scene
  every frame on top of the two camera sensors.
- **The offload variables make things slower in every configuration measured** -- 0.65 vs
  0.90 headless, 0.27 vs 0.40 with the GUI. They are also redundant for the server: both
  headless runs showed the identical 201 MiB against `ign gazebo server` in `nvidia-smi`
  and the same `renderD129`, so the server was on the discrete card either way. The
  variables only add the GLX offload path, whose frames must be copied back across the bus
  to the Intel-driven display. Do not set them for headless work; measure before assuming
  offload helps anywhere.

The GLX offload variables only affect GLX clients such as the GUI and `glxinfo`:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo -B | grep "OpenGL renderer"
# NVIDIA GeForce RTX 3070 Laptop GPU/PCIe/SSE2   <- offload working
```

They do nothing for a headless server, which uses EGL. So for camera-heavy work the
fastest configuration here is simply:

```bash
ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py headless:=True
```

These are single runs and the real-time factor drifts a few percent between samples, so
treat small differences as noise and re-measure on your own machine with
`ign topic -e -t /stats -n 5 | grep real_time_factor`.

If you would rather not touch drivers, the zero-risk alternative is to render fewer
pixels: 640x480 is a third of the pixels of 1280x720 and should roughly triple the
real-time factor. Change `<image>` in `models/ackermann_robot/model.sdf` **and** the
matching `fx`/`fy`/`cx`/`cy` in `vins_fusion_ros2/config/wil_sim/cam*_pinhole.yaml`, or
VINS will silently misproject. At 640x480 with the same 90 degree FOV those become
`fx=fy=320, cx=320, cy=240`.

**Visit the [AWS RoboMaker website](https://aws.amazon.com/robomaker/) to learn more about building intelligent robotic applications with Amazon Web Services.**

### 2D viewer instead of the Gazebo GUI

The GUI costs roughly half the real-time factor (table above), and because every sensor
rate scales with RTF, that is IMU and camera samples you do not get. `viewer.py` at the
workspace root replaces it with a top-down 2D view built from ROS topics: the GroundB
floor texture, obstacle footprints parsed from the world file, the robot from
`/ground_truth/odometry`, keyboard driving, and a live RTF + per-topic rate panel.

```bash
ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py headless:=True
python3 ~/wil_project/viewer.py                       # add --map ros once nav2 is up
```

Measured against a headless sim: **26% of one core**, taking RTF from 0.436 to 0.418 --
about a 4% cost, against the GUI's ~55%.

Controls: `w/s` throttle/brake, `a/d` steer, `space` handbrake, middle-drag pan, wheel
zoom, `f` fit, `[` `]` rotate the view 90 deg, `g` grid, `o` obstacles, `m` map, `t`
trails, `v` VINS, `c` camera thumbnail, `r` re-align VINS, left-drag a Nav2 goal,
shift-left-drag to queue a waypoint, `Enter` plan then drive the route, `Backspace`
drop the last waypoint, `Delete` clear the route, `Esc` cancel, `q` quit.

#### Viewer arguments

All optional; the defaults are what `python3 viewer.py` with no flags uses.

**Topics and sources**

| Argument | Default | Description |
| :------- | :------ | :---------- |
| `--gt-topic` | `/ground_truth/odometry` | Odometry the robot pose, trail and RTF clock come from. The viewer takes its clock from these header stamps rather than `/clock` — see the `/clock` note below. |
| `--vins-topic` | `auto` | VINS odometry to overlay. `auto` probes `/vins_estimator/odometry`, then `/odometry`, then any other `nav_msgs/Odometry` publisher. |
| `--cmd-vel-topic` | `/cmd_vel` | Where keyboard teleop publishes, and the topic whose rate is shown. |
| `--imu-topic` | `/imu` | IMU topic for the `--watch-imu` rate row. |
| `--cam-info-topics` | `/cam0/camera_info,/cam1/camera_info` | Comma-separated `CameraInfo` topics watched for rates — the cheap way to see whether the cameras are keeping up without subscribing to images. |
| `--plan-topic` | `/plan` | Nav2 path to draw. |
| `--pose-topic` | `/world/default/dynamic_pose/info` | Bridged gz dynamic pose feed used by `--live-poses`. |
| `--world` | packaged `small_warehouse.world` | World file the obstacle footprints are parsed from. |
| `--texture` | packaged `GroundB_01` PNG | Floor texture tiled under the scene. |

**Navigation and teleop**

| Argument | Default | Description |
| :------- | :------ | :---------- |
| `--nav-action` | `/navigate_to_pose` | Nav2 action server that left-drag goals are sent to. |
| `--nav-frame` | `map` | `frame_id` stamped on those goals. |
| `--no-nav` | off | Never create the Nav2 action client. Use when Nav2 is not running, or to keep left-drag from sending anything. |
| `--nav-through-action` | `/navigate_through_poses` | Action used to DRIVE a queued waypoint route. |
| `--compute-route-action` | `/compute_path_through_poses` | `planner_server` action used to PREVIEW a route without moving the robot. |
| `--planner-id` | `GridBased` | Planner plugin used for route previews. Must match `planner_plugins` in `nav2_ackermann.yaml`. |
| `--no-teleop` | off | Never create a `/cmd_vel` publisher at all. Without it the publisher is created lazily on the first key press, so an idle viewer already does not compete with `drive.py`. |

**Map and floor rendering**

| Argument | Default | Description |
| :------- | :------ | :---------- |
| `--map` | `none` | `none`, `ros` to subscribe to the Nav2 `/map` topic, or a path to a `map.yaml` to load from disk. Toggled at runtime with `m`. |
| `--rotate {0,90,180,270}` | `0` | Starting view orientation; also bound to `[` and `]`. See below. |
| `--floor-ppm` | `100.0` | Pixels per metre the floor texture is baked at. Lower it to cut the one-time build cost and memory of the floor pixmap. |
| `--floor-flip {none,u,v,uv}` | `none` | Mirror the floor texture's UV directions. COLLADA's UV origin is bottom-left while an image's row 0 is the top, so which way `+v` runs depends on the loader. Because this is a repeating pattern, getting it wrong mirrors the pattern rather than changing scale or offset — set it against a Gazebo screenshot only if you care. |
| `--no-floor` | off | Skip the floor texture entirely (plain background). |

**Overlays and performance**

| Argument | Default | Description |
| :------- | :------ | :---------- |
| `--fps` | `15` | Repaint rate, and the single biggest knob on viewer CPU — see below. |
| `--thumb {none,cam0,cam1}` | `none` | Live camera thumbnail. Costs ~9% of a core. Toggled at runtime with `c`. |
| `--thumb-topic` | derived from `--thumb` | Image topic for the thumbnail; defaults to `/<thumb>/image_raw`. Set it to use a camera the `--thumb` choices do not cover. |
| `--thumb-decimate` | `4` | Keep every Nth pixel in each direction when decoding the thumbnail. Raise it to make an already-subscribed camera cheaper to draw. |
| `--live-poses` | off | Draw moving models at their live gz pose instead of the pose the world file authored. Needs `bridge_model_poses:=True`. ~13% of a core -- see below. |
| `--watch-imu` | off | Add an `/imu` row to the rate panel. Costs ~21% of a core at RTF 0.4 and ~45-50% at RTF 1.0 — rclpy is ~2.2 ms per message. RTF tells you the same thing more cheaply. |
| `--trail-min-step` | `0.02` | Metres of motion before a new trail point is recorded. Raising it keeps long runs' trails shorter. |
| `--vins-align {first,none}` | `first` | `first` fits a rigid VINS→world transform the first time VINS has moved 0.5 m, so the two trails are comparable; `none` draws VINS in its own frame. Re-align at runtime with `r`. |
| `--no-sim-time` | off | Ignored — accepted so older command lines still run. The viewer never uses sim time; that is deliberate, for the `/clock` reason below. |

`--rotate {0,90,180,270}` sets the starting orientation, e.g. `--rotate 180` to put the
world's +y at the bottom of the screen. Only the world layers turn; the panel, scale bar
and camera thumbnail stay upright, and mouse picking is un-rotated to match, so Nav2 goals
still land where you click.

Two measurements shaped it, and both are worth knowing before changing it:

- **rclpy costs ~2.2 ms of CPU per message**, so message count is the only thing that
  matters. gz publishes `/clock` at ~700 Hz real-time-equivalent, which is **70% of one
  core for that single subscription** -- and `use_sim_time:=True` creates one internally
  whether you ask or not. The viewer therefore does not use sim time and takes its clock
  from the odometry header stamps it already receives. An early build that did subscribe
  cost 118% of a core and pushed RTF from 0.44 to 0.32: it was causing the very problem it
  exists to diagnose.
- **Repainting is ~8.5 ms a frame** at 1200x900, mostly the full-window blit, so `--fps`
  is the biggest rendering knob. Default is 15.

The panel judges each topic against `expected_hz * RTF`, because a topic keeping pace with
a half-speed sim is healthy, not half-broken. Anything reading far below 100% there is a
real problem; everything moving together just means the sim is slow.

Optional layers, both off by default because of the per-message cost above:

| flag | what it adds | measured cost |
| :--- | :----------- | :------------ |
| `--watch-imu` | an `/imu` row in the rate panel | +21% of a core at RTF 0.4, ~+50% at RTF 1.0 |
| `--thumb cam0` | live camera thumbnail | +9% of a core |
| `--live-poses` | moving models drawn where they actually are | ~13% of a core (~58 Hz feed) |

Note `--watch-imu` adds only a RATE row -- the viewer shows no IMU data (no orientation,
no accel/gyro traces). RTF is the cheaper proxy: every rate measured tracked
`expected * RTF` at 100-103%, so if RTF is healthy the IMU is keeping up.

#### Waypoint routes, and planning before the robot moves

Left-drag sends one goal immediately. **Shift**-left-drag queues a waypoint instead,
and `Enter` plans the whole route *without moving the robot* -- that is
`ComputePathThroughPoses` on `planner_server`, which is a pure query. Measured: a
3-waypoint route returned a 241-pose, 24.4 m path with `/ground_truth/odometry`
unchanged to 16 digits and nothing on `/cmd_vel`. Press `Enter` again to drive it via
`NavigateThroughPoses`. `Backspace` drops the last waypoint, `Delete` clears the route.

Waypoints are numbered on the map, with a dotted line showing the leg order. A dashed
ring means the heading was derived automatically; drag instead of clicking to pin one,
and the ring goes solid.

**Heading is what makes a route plannable, and the viewer derives it for you.** The
planner is `DUBIN` with `allow_reversing: false`, so it has to *leave* each via-point
on the heading you give it -- an arbitrary yaw there is the usual reason a route whose
positions are all reachable will not plan at all. A click-placed waypoint therefore
gets the bearing *through* the point: towards the next waypoint, or along the incoming
leg for the last one. Pin a heading only when you actually want a specific arrival
angle, and expect it to fail more often when you do.

`FollowWaypoints` is deliberately not used, and `nav2_waypoint_follower` is not
launched. It drives each waypoint as a separate `NavigateToPose` goal, so the robot
stops and replans at every one and every arrival has to satisfy both the goal checker
and a feasible Dubins heading. `NavigateThroughPoses` treats them as via-points on one
path, which is what a car wants.

#### Moving models

By default every footprint is drawn where the **world file** put it, because
`load_obstacles()` parses the world once at startup. So if you make a model non-static
and it starts moving -- a `VelocityControl` plugin on a clutter box, say -- the sim
moves it and the viewer does not show that. `--live-poses` fixes it:

```bash
ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py \
    headless:=True bridge_model_poses:=True
python3 ~/wil_project/viewer.py --live-poses
```

The feed is gz's `/world/default/dynamic_pose/info`, bridged as a `tf2_msgs/TFMessage`
whose `child_frame_id` is the model name. It carries **only non-static entities** --
in the stock world that is the robot and its links, 10 entries here against
`/world/default/pose/info`'s 77, of which 67 can never move. Measured at ~58 Hz with
RTF 0.98, the same order as the ground-truth odometry, and it scales with RTF like
everything else, so the panel judges it against `expected * RTF` as usual.

Two implementation notes, because both are load-bearing:

- Footprints are **not** re-read from the meshes. Each one is stored with the pose the
  world file authored it at, and a live pose is applied as a rigid transform of that
  polygon. Rotation is included, and only the *delta* from the authored yaw is applied,
  so a model authored at a non-zero yaw is not double-rotated.
- A model is treated as moving only when its live pose differs from its authored pose.
  That matters because a non-static model sitting still is in the feed every tick, and
  treating it as live would pull it out of the cached static pixmap for nothing. Models
  move between the cached layer and the per-frame layer only when that changes.

Nav2 still cannot see any of this. The costmap is a static layer plus inflation, so a
moving obstacle is invisible to the planner and the robot will drive into it -- and
`bake_map.py` reads the same authored poses, so the baked map has the obstacle at its
original spot. Observed while testing: the robot wedged against a drifting clutter box
with `/ground_truth/odometry` frozen while nav2 cycled Spin and BackUp recoveries
reporting "Collision Ahead". `--live-poses` is a *diagnostic* -- it lets you see the
collision coming; it does not make nav2 avoid it.

### Navigation (Nav2)

```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-nav2-smac-planner ros-humble-nav2-regulated-pure-pursuit-controller
python3 ~/wil_project/bake_map.py
colcon build --symlink-install --packages-select aws_robomaker_small_warehouse_world
ros2 launch aws_robomaker_small_warehouse_world nav2.launch.py     # headless by default
python3 ~/wil_project/viewer.py --map ros                          # left-drag to set a goal
```

`nav2.launch.py` brings up three layers in one command: the sim (via
`small_warehouse.launch.py`, `headless` defaulting to True here), the TF tree (via
`localization_tf.launch.py`), and seven Nav2 servers plus a lifecycle manager. Goals flow
`bt_navigator -> planner_server (Smac Hybrid-A*) -> controller_server (RPP) ->
velocity_smoother -> /cmd_vel -> cmd_vel_bridge`. Use `start_sim:=False` to attach to a
sim that is already running.

Static map only -- the robot has no lidar, so the costmap runs a static layer plus
inflation and nothing else -- and localization is ground truth, not AMCL, so that planner
and controller behaviour can be judged without localization error in the way. See
`launch/localization_tf.launch.py` (the file to replace when you want AMCL or VINS
instead) and the header of `params/nav2_ackermann.yaml`, which explains why the lookahead
and smoother settings are load-bearing rather than taste.

`bake_map.py` slices the world's collision meshes at robot height rather than using either
existing map: `maps/002` is in a different frame entirely, and `maps/005` is a SLAM product
in which the shelves are hollow post outlines, so planning on it routes the robot into
0.94 m aisles it cannot turn around in.

**Verified against the installed nav2 1.1.20.** Every plugin parameter name in
`nav2_ackermann.yaml` was checked against the installed libraries; all seven servers reach
`active`, SmacPlannerHybrid/Dubin, RPP and SimpleSmoother all load, and both costmaps come
up 286x423 at 0.05 m, origin (-7.0, -10.5) -- i.e. exactly the baked map. Best measured
run, three goals from spawn:

| goal | result | speed (sim) | final error |
| :--- | :----- | :---------- | :---------- |
| hall centre `(-0.72, -2.47)` | SUCCEEDED | 0.58 m/s | 0.30 m |
| S-curve past ShelfF `(-3.58, -5.32)` | SUCCEEDED | 0.52 m/s | 0.29 m |
| far corner `(0.23, -9.32)` | SUCCEEDED | 0.50 m/s | 0.29 m |

against `desired_linear_vel: 0.60` and `xy_goal_tolerance: 0.30`.

#### Four traps, all of which cost real debugging time here

**Measure in SIM time, not wall time.** This is the big one. At RTF 0.2-0.4 a 12 m goal
legitimately takes 2-3 minutes of wall clock. Timing a goal on the wall clock, or dividing
distance by wall seconds, makes a perfectly healthy controller look broken -- it reports
~0.10 m/s and "timeouts" for something actually running at 0.5 m/s and arriving fine.
Budget goals in sim seconds, taken from `/ground_truth/odometry` header stamps.

**`bond_timeout` must be generous, and it must actually reach the node.** The
nav2_bringup default of 4.0 s is checked in WALL time while the sim runs at RTF 0.2-0.4,
so a server merely descheduled for a moment looks dead. Observed on a loaded desktop:

```
CRITICAL FAILURE: SERVER map_server IS DOWN after not receiving a heartbeat for 4000 ms.
Shutting down related nodes.
```

after which `bt_navigator` sits `inactive` and every goal comes back REJECTED -- which
looks nothing like a timeout. Set to 20.0 here.

Setting it in `nav2_ackermann.yaml` is not sufficient on its own: `lifecycle_manager` was
the one node in `nav2.launch.py` launched with an inline parameter dict rather than
`configured_params`, so the params file never reached it and it silently used the 4.0 s
default -- and then tore the stack down exactly as above. It now gets `configured_params`
like every other node. Verify rather than assume, because the failure is silent:

```bash
ros2 param get /lifecycle_manager_navigation bond_timeout   # must say 20.0, not 4.0
```

**Goal HEADING decides feasibility, not just position.** A car cannot arrive at an
arbitrary yaw in a tight spot. Planning *to* the spawn `(1.80, 9.00)`, which has only
0.55 m of clearance, succeeds at yaw +45 and +90 deg and fails at 0, 135, 180 and -90 --
each failure burning the full `max_planning_time` (3 s) before reporting "no valid path
found". Position is fine; the heading is what is unreachable. When scripting goals, use
the bearing from the robot to the goal rather than a fixed yaw.

**The curvature bound.** `AckermannSteering` does not reject an infeasible turn -- it
clamps the radius and the robot quietly under-turns, with no error anywhere. The check:

```bash
ros2 topic echo /cmd_vel      # require |angular.z| / |linear.x| <= 1.6673
```

Note 1.6673 is `tan(35 deg)/0.42`, the REAR-AXLE bound. Nav2 plans `base_link`, which sits
mid-wheelbase and sweeps the larger radius 0.6355 m (curvature 1.5736). Do not mix them.
Also: slowing down makes saturation *worse*, since the steering angle depends on
`angular.z / linear.x` -- lowering `desired_linear_vel` is never the fix.

#### Known open issues

- **Terminal approach exceeds the curvature bound.** Worst observed `|wz|/|vx|` is 6.67
  against a limit of 1.6673, which is `min_approach_linear_velocity` (0.15) against a yaw
  rate still allowed to reach 1.00. Near the goal the remaining path is shorter than
  `min_lookahead_dist`, so the `k <= 2/L_d` guarantee stops holding. The robot still
  reaches goals within tolerance -- the plugin clamps and RPP corrects at 20 Hz -- but it
  applies full steering lock while crawling. Fixing it properly means bounding the yaw
  rate as a function of commanded speed rather than with a constant cap.
- **`use_collision_detection` is currently `true` and is not settled.** With it on, one
  run logged 155-195 "detected collision ahead!" -> "Controller patience exceeded" ->
  abort -> backup -> replan, in corridors with 0.7-4.9 m of real clearance; with it off,
  three goals in a row succeeded and the worst costmap cost under the robot footprint was
  95, never reaching the inscribed value of 99. That argues the vetoes were false
  positives against the controller's own corner-cutting. But the two runs were not
  otherwise identical, and the comparison run with it back on was invalidated by the
  `bond_timeout` failure above, so this needs one clean A/B before drawing a conclusion.

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
