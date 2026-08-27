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

"""Reap processes left behind by a previous launch, before starting new ones.

    from stale_process_reaper import reaper
    ld.add_action(OpaqueFunction(function=reaper('sim')))

WHY THIS EXISTS
    Ctrl-C on a launch does not reliably reap everything it started. `ign gazebo`
    in particular ignores SIGTERM often enough that it survives its own launch,
    and so do the ros_gz_bridge processes. The next launch then comes up alongside
    the previous generation instead of replacing it, and because these nodes talk
    on FIXED topic names -- /clock, /tf, and the deliberately unscoped
    /ground_truth/odometry (see models/ackermann_robot/model.sdf) -- the two
    generations silently fight over the same signals.

    The symptoms do not look like "there are two of everything". They look like a
    TF bug, which is what makes this worth automating:

      Two clock_bridge processes relaying the SAME gz clock publish it onto ROS
      /clock with independent latency, so consecutive samples arrive out of order
      by one sim step. tf2 reads that as time running backwards:

          [bt_navigator] Detected jump back in time. Clearing TF buffer.

      and wipes the entire buffer, repeatedly. Navigation cannot work.

      Two ground_truth_localization nodes attached to sims at different sim times
      both publish odom->base_footprint. The buffer keeps the newer timeline and
      rejects every transform from the real one:

          [planner_server] TF_OLD_DATA ignoring data from the past for frame
          base_footprint at time 0.100000

    Both were real, and both cost a debugging session before anyone thought to run
    `ros2 node list | sort | uniq -c`. Hence: clean up on the way IN, where it is
    automatic, rather than trusting the way out.

WHY ON THE WAY IN AND NOT ON THE WAY OUT
    An exit handler only runs if the exit is orderly, and these processes survive
    precisely when it is not (SIGKILL of the launch, a crash, a closed terminal).
    A preflight has no such failure mode: whatever the last run left behind, the
    next run starts from a known-clean graph.

WHY OpaqueFunction AND NOT ExecuteProcess
    OpaqueFunction runs synchronously while the launch description is being
    visited, so the reap is guaranteed to have FINISHED before any later action
    starts gz. An ExecuteProcess would only be spawned in order, and would race
    the very processes it is meant to clear.

SCOPE -- what each group may kill, and why it is safe to run them in sequence
    The groups are disjoint, which matters because nav2*.launch.py visits its own
    'nav2' reap, then includes small_warehouse*.launch.py (which reaps 'sim' and
    starts gz), then includes localization_tf.launch.py (which reaps
    'localization'). That last reap runs AFTER gz has been started, so if the
    groups overlapped it would kill the sim that was just launched.

    Only processes in the SAME ROS_DOMAIN_ID are considered, so a stack you are
    deliberately running on another domain is left alone. This process and every
    one of its ancestors are protected unconditionally -- without that, the
    pattern for the nav2 servers would match the `ros2 launch` command line that
    is running this code.

    'localization' and 'nav2' match on node names (robot_state_publisher,
    planner_server, ...) that are not unique to this package. That is deliberate:
    a second node publishing the same frames or offering the same action server IS
    the failure being prevented, whoever started it. If you are intentionally
    running another stack on this domain, launch with reap_stale:=False.
"""

import os
import re
import signal
import time

from launch.actions import LogInfo

# Matched against the full command line, which for a launch_ros Node always
# carries `-r __node:=<name>`. Anchoring on that rather than on the executable
# path is what keeps these from matching an unrelated `ros2 run`.
_PKG = 'aws_robomaker_small_warehouse_world'

GROUPS = {
    # small_warehouse.launch.py and its _static/_dynamic twins: the gz server and
    # GUI (both carry the world path, which lives under this package's share dir)
    # plus the bridges it names.
    'sim': (
        r'(ign gazebo|gz sim).*' + _PKG,
        r'parameter_bridge.*__node:='
        r'(clock|sensor|cmd_vel|ground_truth|model_pose)_bridge\b',
        # compressed_images:=True adds this one alongside sensor_bridge; it holds
        # /camN/image_raw just as the parameter_bridge shape does, so a survivor
        # duplicates every frame for the next run.
        r'image_bridge.*__node:=cam_image_bridge\b',
    ),
    # localization_tf.launch.py. ground_truth_localization is ours; the other two
    # are stock nodes we run under stock names -- see SCOPE above.
    'localization': (
        r'__node:=ground_truth_localization\b',
        r'__node:=robot_state_publisher\b',
        r'__node:=joint_state_publisher\b',
    ),
    # The eight lifecycle nodes spelled out in nav2*.launch.py.
    'nav2': (
        r'__node:=(map_server|controller_server|smoother_server|planner_server'
        r'|behavior_server|bt_navigator|velocity_smoother'
        r'|lifecycle_manager_navigation)\b',
    ),
}


def _cmdline(pid):
    """Full command line of pid, or '' if it is gone or not readable."""
    try:
        with open('/proc/%d/cmdline' % pid, 'rb') as f:
            # NUL-separated; the trailing NUL would otherwise leave a bare space.
            return f.read().decode('utf-8', 'replace').replace('\0', ' ').strip()
    except (IOError, OSError):
        return ''


def _ppid(pid):
    """Parent of pid, or 0. Read from status, not stat: a process whose comm
    contains a space or a bracket makes stat's field offsets unreliable."""
    try:
        with open('/proc/%d/status' % pid, 'r') as f:
            for line in f:
                if line.startswith('PPid:'):
                    return int(line.split()[1])
    except (IOError, OSError, ValueError):
        pass
    return 0


def _domain(pid):
    """ROS_DOMAIN_ID of pid as a string ('0' when unset, matching rclcpp's
    default), or None if the environment cannot be read -- which in practice
    means another user's process, and those cannot be signalled anyway."""
    try:
        with open('/proc/%d/environ' % pid, 'rb') as f:
            env = f.read().decode('utf-8', 'replace')
    except (IOError, OSError):
        return None
    for entry in env.split('\0'):
        if entry.startswith('ROS_DOMAIN_ID='):
            return entry.split('=', 1)[1] or '0'
    return '0'


def _protected():
    """This process and every ancestor of it.

    The launch that is running this code must never reap itself. Its command line
    is `ros2 launch aws_robomaker_small_warehouse_world nav2_static.launch.py`,
    which contains the package name, and its shell and terminal are above it.
    """
    protected = set()
    pid = os.getpid()
    while pid > 1 and pid not in protected:
        protected.add(pid)
        pid = _ppid(pid)
    return protected


def _victims(patterns):
    compiled = [re.compile(p) for p in patterns]
    protected = _protected()
    mine = os.environ.get('ROS_DOMAIN_ID', '0') or '0'
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in protected:
            continue
        cmd = _cmdline(pid)
        if not cmd:                       # kernel thread, or already exited
            continue
        if not any(c.search(cmd) for c in compiled):
            continue
        if _domain(pid) != mine:
            continue
        found.append((pid, cmd))
    return found


def _alive(pid):
    """True only if pid is a process that could still be doing something.

    Not `os.kill(pid, 0)`: that succeeds on a ZOMBIE, which is a process that has
    already exited and is only waiting to be reaped by its parent. Counting those
    as alive makes every polite process look like it ignored SIGTERM, and the
    escalation message below is meant to be believable when it appears.
    """
    try:
        with open('/proc/%d/status' % pid, 'r') as f:
            for line in f:
                if line.startswith('State:'):
                    return not line.split()[1].startswith('Z')
    except (IOError, OSError, IndexError):
        return False
    return False


def reap(group, grace_sec=3.0):
    """Kill the previous generation of `group`. Returns lines describing what died.

    SIGTERM first, then SIGKILL whatever is still standing after grace_sec. The
    escalation is not defensive programming: a stalled `ign gazebo` was observed
    surviving SIGTERM for 73 minutes, and that exact process is why this file
    exists.
    """
    victims = _victims(GROUPS[group])
    if not victims:
        return []

    lines = []
    for pid, cmd in victims:
        lines.append('reaped stale %s process %d: %s' % (group, pid, cmd[:110]))
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass

    deadline = time.time() + grace_sec
    pending = [pid for pid, _ in victims]
    while pending and time.time() < deadline:
        pending = [pid for pid in pending if _alive(pid)]
        if pending:
            time.sleep(0.1)

    for pid in pending:
        lines.append('  %d ignored SIGTERM, sending SIGKILL' % pid)
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass

    return lines


def reaper(group, grace_sec=3.0):
    """An OpaqueFunction body that reaps `group`. Silent when nothing was stale."""
    def _reap(context, *args, **kwargs):
        return [LogInfo(msg=line) for line in reap(group, grace_sec)]
    return _reap
