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

Both launch files accept all four. For example:

```bash
ros2 launch aws_robomaker_small_warehouse_world no_roof_small_warehouse.launch.py headless:=True
```

Because gz-sim has no `gazebo_ros_init` equivalent, `/clock` only reaches ROS through
the bridge this launch file starts. Set `use_sim_time:=False` if you bridge the clock
yourself.

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
