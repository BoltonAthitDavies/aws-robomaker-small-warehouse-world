^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aws_robomaker_small_warehouse_world
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0
-----
* Port from Gazebo Classic 11 to Gazebo (Ignition) Fortress / gz-sim 6. Gazebo
  Classic is no longer supported.
* Rewrite the 28 mesh URIs in the models from relative ``file://models/...`` to
  ``model://...``. The old form only resolved under Classic's
  ``GAZEBO_RESOURCE_PATH`` search and is unresolvable by libsdformat12.
* Fix inertia tensors on ``GroundB_01`` and ``RoofB_01``. Both violated the
  inertia triangle inequality; Classic ignored this, libsdformat12 rejects it and
  refuses to load the world.
* Flatten the ``<model><include/></model>`` wrappers in both worlds to plain
  ``<include>`` entries. The wrappers defaulted to non-static, which would have
  made the whole warehouse a dynamic body under gz-sim.
* Bump models and worlds to SDF 1.7 and drop the removed ``pose`` ``frame`` attribute.
* Add the required gz-sim system plugins, a ``<scene>``, per-world lighting, and a
  Fortress ``<gui>`` block to both worlds.
* Replace the ``gzserver``/``gzclient`` launch files with ``ros_gz_sim``, and
  actually forward the ``world`` argument — previously it was declared but never
  passed, so ``no_roof_small_warehouse.launch.py`` had no effect.
* Start a ``ros_gz_bridge`` for ``/clock`` so ``use_sim_time`` works.
* Swap the ``GAZEBO_MODEL_PATH`` environment hook for
  ``IGN_GAZEBO_RESOURCE_PATH``/``GZ_SIM_RESOURCE_PATH``.
* Drop ``rviz/basic_data.rviz`` (an RViz 1 config that RViz 2 cannot load).

Forthcoming
-----------
* Use floor friction value from Gazebo empty world (`#13 <https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/issues/13>`_)
* ROS2 branch
* Adding gazebo model path to stop requiring modification of gazebo model path env var (`#11 <https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/issues/11>`_)
* Merge pull request `#5 <https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/issues/5>`_ from RoverRobotics-forks/foxy-devel
  Foxy devel
* fixing missing gazebo path exports not covered in env_hooks
* Merge branch 'foxy-devel' of https://github.com/aws-robotics/aws-robomaker-small-warehouse-world into foxy-devel
* merging with branch PR `#5 <https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/issues/5>`_.  Also updated readme
* updated for merge request `#2 <https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/issues/2>`_
* updated cmake according to review
* Tuning lighting for system(s) with/without GPU (`#2 <https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/issues/2>`_) (`#6 <https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/issues/6>`_)
  - Adjust ceiling lamp's parameters so that it works system(s) with/without GPU
* update to ament_cmake and added env-hooks
* Add support for launching in ROS2 foxy
* Add support for launch with ROS2
* Merge pull request `#1 <https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/issues/1>`_ from konduri/no-roof-warehouse
  warehouse world with no roof
* no roof world and launch
* Add initial package contents
* Initial commit
* Contributors: Amazon GitHub Automation, Anson Wong, Hai Quang, Kim (kenvink), Joep Tool, Matthew Murphy, Ojas Joshi, Sam Gundry, Steve Macenski, konduri, padiln, root
