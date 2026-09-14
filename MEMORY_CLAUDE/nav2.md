# Nav2 on the Ackermann robot — work log

Companion to [README.md](README.md), which covers the warehouse environment itself.
This file covers Nav2: what was wrong, how it was diagnosed, what was measured, and
what is still open. Same convention as the other log — the *measurements* and the
*root causes* are the parts worth quoting later, so they are kept verbatim.

Submodule HEAD at time of writing: `2ca63f5 update bay size compact to the map size`.

The presenting complaint, in the user's words: *"when I plan, the first path shows it
can go to goal, but when it's near the goal it changes the path"*, and later *"it only
shows this behaviour when the angular goal is very different than the initial angular"*.
That second observation turned out to be the key to the whole investigation.

---

## 1. Summary of what was found

Four independent faults, none of which produced an error message. In order of how much
they cost:

| # | Fault | Symptom it produced |
| :- | :---- | :------------------ |
| 1 | Via-point check sampled at 0.333 Hz | robot orbits a waypoint it already reached |
| 2 | Goal heading demand is infeasible near the goal | robot circles the final goal forever |
| 3 | `bond_timeout` never reached the lifecycle manager | whole stack tears itself down under load |
| 4 | Baked map stale against the world | nav2 refuses to plan across clear floor |

A fifth is still open at the time of writing: the velocity smoother cannot execute the
tightest turn the planner is allowed to plan (§6).

---

## 2. Fault 1 — via-points are checked once every 3 seconds

**This was the main bug.** It has two layers and the first one hides the second.

**Layer one: `xy_goal_tolerance` does not apply to via-points.** It governs the FINAL
pose only. Intermediate poses of a `NavigateThroughPoses` route are discarded purely by
proximity, by the `RemovePassedGoals` BT node, whose `radius` nav2's stock tree
hard-codes at **0.7 m** with no parameter to override it. Loosening either goal
tolerance therefore does nothing whatsoever for waypoint behaviour — which is why an
afternoon of tuning `xy_goal_tolerance` and `yaw_goal_tolerance` changed nothing.

0.7 m is generous on a differential robot and too small on this car:

- minimum turning radius is 0.635 m at `base_link`, so a via-point missed by a metre
  costs a ~4 m loop to return to a point the robot was effectively already at;
- RPP steers at a lookahead of `min_lookahead_dist` 1.30 m, so it deliberately cuts
  corners by up to that much. Passing a via-point at more than 0.7 m is **normal
  tracking, not an error**.

**Layer two, and the actual cause: the check is rate-limited.** In nav2's stock tree
`RemovePassedGoals` sits INSIDE `<RateController hz="0.333">`, so the proximity test is
evaluated once every 3 s — while the tree itself ticks at `bt_loop_duration` (10 ms),
300× faster. The robot is only inside a via-point's radius for `2*radius/speed` seconds,
so the test is missed outright whenever

```
2 * radius / speed  <  1 / hz
```

At `desired_linear_vel: 1.5` m/s with radius 1.60 that window is 2.1 s against a 3 s
sampling period. Being inside the radius is irrelevant if nothing looks while you are
there.

MEASURED, 3-waypoint route that reverses the robot's direction, radius 1.60, before the
fix:

```
t=  4s pos=( 7.85,  3.99) remaining=3 turn= +157.5 deg  d=4.8 8.4 12.3
t=  8s pos=( 4.91,  8.21) remaining=3 turn= +192.4 deg  d=0.4 3.8 7.8   <- 0.40 m away, NOT consumed
t= 12s pos=( 7.59, 11.64) remaining=3 turn=  +72.9 deg  d=4.3 2.4 4.9   <- overshot
t= 16s pos=( 6.03,  7.88) remaining=2 turn=  -71.2 deg  d=0.8 4.2 8.2   <- looped back, consumed
t= 20s pos=( 5.02, 11.08) remaining=2 turn= -210.9 deg
t= 24s pos=( 5.14, 15.69) remaining=1 turn= -170.2 deg
total heading swept: -170 deg (0.47 full circles), 24 s
```

Note `number_of_poses_remaining` stuck at 3 while the robot sat 0.40 m from the
waypoint, and the heading swinging +157/+192/+73/−71/−211 as it orbited.

**Raising the radius alone does not fix this.** At 0.333 Hz you would need
`radius > 1.5 * speed` = 2.25 m, which starts swallowing waypoints placed deliberately.

**Fix**, both parts, in `behavior_trees/nav_through_poses_ackermann.xml` (a copy of
nav2's stock tree, otherwise byte-identical):

1. radius raised 0.7 → **1.60 m**, above `min_lookahead_dist`, to cover RPP's
   corner-cutting;
2. `RemovePassedGoals` moved **outside** the `RateController`, so it is evaluated every
   tick while `ComputePathThroughPoses` stays throttled at 0.333 Hz — which is what the
   rate limiter is actually for.

Same route after the fix:

```
t=  4s pos=( 7.73,  3.30) remaining=3 turn= +137.5 deg
t=  8s pos=( 5.30,  7.21) remaining=2 turn= +213.3 deg  d=0.8   <- consumed first pass
t= 12s pos=( 5.59, 12.49) remaining=1 turn= +181.6 deg  d=0.6   <- consumed first pass
t= 16s pos=( 5.12, 15.66) remaining=1 -> SUCCEEDED
total heading swept: 180 deg, 16 s
```

Every waypoint consumed on the first pass, 24 s → 16 s, and the 180 deg is the
legitimate turnaround the route asks for — monotonic, not orbiting.

Wiring, two traps worth recording:

- The tree is selected with `default_nav_through_poses_bt_xml`. **Plain YAML has no
  `$(find-pkg-share)` substitution** — the literal string passes straight through and
  fails only when the first goal arrives, as an opaque *"Error loading XML file"*. The
  value is a placeholder rewritten at launch by `RewrittenYaml` in all three nav2 launch
  files. Verify rather than assume: `ros2 param get /bt_navigator
  default_nav_through_poses_bt_xml` must return an absolute path.
- **XML comments cannot contain `--`.** BehaviorTree.CPP's parser tolerates it;
  `xml.etree.ElementTree` does not. Worth knowing before annotating a BT heavily.

---

## 3. Fault 2 — the goal heading, not the goal position, decides feasibility

The user's own observation. Confirmed with a controlled comparison: identical start pose,
identical goal position, only the goal yaw differs.

| goal heading | outcome |
| :----------- | :------ |
| matching the approach bearing | **SUCCEEDED in ~3 s**, 5 replans, stopped 0.24 m out |
| 180° from the approach | never converged — 75 s, **74 replans**, stalled at 0.56 m |

A Dubins path is forward-only, so the only way it can shed a large heading difference is
a loop of radius ≥ 0.635 m. Near the goal there is no aisle width for one, so each
replan produces another loop, tracking error moves the robot, and the next replan
produces another.

Plan-only sweep (`ComputePathToPose`, no motion), same goal, heading swept away from the
approach bearing:

| goal heading | DUBIN path | plan time | REEDS_SHEPP path | plan time |
| :--- | :--- | :--- | :--- | :--- |
| +0° | 4.74 m | 0.9 s | 4.72 m | 0.6 s |
| +45° | 4.77 m | 0.03 s | 4.79 m | 0.44 s |
| +90° | 5.28 m | **20.0 s** | 5.22 m | 1.1 s |
| +135° | 6.29 m (×1.3) | 0.03 s | 5.50 m | 1.05 s |
| +180° | **7.15 m (×1.5)** | **20.0 s** | 5.56 m (×1.2) | 0.06 s |

Those 20 s plan times are far past `max_planning_time: 3.0`, against a BT replanning at
1 Hz — so the controller chases a stale path while the planner saturates.

**A hypothesis that the data killed.** The package README's "known open issues" blamed a
curvature-bound violation on the approach (`min_approach_linear_velocity` 0.15 against a
yaw rate still allowed to reach 1.00, giving k = 6.67 against a limit of 1.6673). That is
NOT what causes this failure: worst measured `|wz|/|vx|` across the Dubins runs was
**1.53**, inside the 1.6673 limit. The curvature issue is real but separate; the goal
heading is the cause here.

**Mitigation applied** (`viewer.py`): a plain click set the goal heading to the robot's
*current* yaw, which is arbitrary relative to where you clicked and frequently hostile.
It now uses the bearing from robot to goal — the same rule `_resolve_route` already used
for click-placed waypoints. Deliberate drags are untouched.

**Not applied, and why.** `REEDS_SHEPP` + `allow_reversing: true` plans every heading in
~1 s with ≤1.2× detour, and got the robot to 0.56 m where Dubins stalled at 2.07 m. But
execution was never verified: **RPP never emitted a single reverse command** in the test
run, and `SmacPlannerHybrid` takes ~40 s longer to configure with Reeds-Shepp. Left at
`DUBIN` / `allow_reversing: false` pending that check.

---

## 4. Fault 3 — `bond_timeout` was silently ignored

`nav2.launch.py` gave `lifecycle_manager` an inline parameter dict instead of
`configured_params` — the only node in the file that did. So `bond_timeout: 20.0` in
`params/nav2_ackermann.yaml` never reached it and it silently used nav2's 4.0 s default:

```
CRITICAL FAILURE: SERVER map_server IS DOWN after not receiving a heartbeat for 4000 ms.
Shutting down related nodes.
```

after which `bt_navigator` sits `inactive` and every goal comes back REJECTED — which
looks nothing like a timeout. The bond heartbeat is checked in WALL time while the sim
runs at RTF 0.2–0.4, so a server merely descheduled for a moment looks dead.

**Fix**: pass `configured_params` first, then the inline overrides. Verify, because the
failure is silent:

```bash
ros2 param get /lifecycle_manager_navigation bond_timeout   # must say 20.0, not 4.0
```

---

## 5. Fault 4 — the baked map was stale against the world

`maps/baked_static/map.pgm` baked 15:07; `small_warehouse_static.world` edited 15:11.
Diffing a fresh bake against the installed one:

```
phantom (nav2 sees, world does not):  57.08 m²
MISSING (world has, nav2 blind to):    0.00 m²
disagreement: 3.2% of the grid
```

The edit had *removed* obstacles, so nav2 was treating 57 m² of clear floor as blocked —
detouring around or refusing routes that are perfectly drivable, with nothing in any log
to explain it. No collision risk (nothing missing), purely over-conservative.

Rebaked → 0.000% disagreement. `maps/baked_dynamic` was already clean both ways.

**Navigability of the new warehouse**, measured on the rebaked map (844×844 @ 0.05 m,
origin (−21.1, −21.1), 42.2 m square):

```
free 90.5%   occupied 9.5%   unknown 0.0%
clearance to nearest obstacle, over free space:
   p50 3.02 m   p75 5.04 m   p90 6.79 m   p99 8.79 m   max 9.94 m
free area with clearance > 0.25 m (footprint half-width)  95.1%
free area with clearance > 0.635 m (can turn around)      88.6%
free area with clearance > 0.80 m (inflation_radius)      85.2%
```

Far more forgiving than the old warehouse, where the README recorded 0.94 m aisles the
robot could not turn in. No aisle-width problem in the new one.

---

## 6. Tooling added — `script/check_limits.py`

`model.sdf` is the only place the robot is actually defined. Four other files carry
hand-copied consequences of it and nothing connected them:

```
models/ackermann_robot/model.sdf        wheel_base, steering_limit, speed/accel caps
  -> drive.py                           WHEEL_BASE, STEER_LIMIT
  -> viewer.py                          ROBOT_X0/X1/HALF_W
  -> params/nav2_ackermann.yaml         turning radius, lookahead, footprint, velocities
  -> launch/small_warehouse.launch.py   max_speed / max_accel defaults
```

17 checks, exits non-zero on failure. It asserts **relations**, not values, and reports
the margin on each — deliberately, because `minimum_turning_radius: 0.75` against a
geometric 0.6355 is an 18% safety margin, not a stale copy, and a tool that "fixed" it by
overwriting would be destroying engineering judgement. It parses the files rather than
importing them, so it runs without rclpy, PyQt5 or a built workspace.

Geometry it recomputes from scratch, reproducing the package README's hand-derived
figures exactly:

```
R_rear = L/tan(d)                   = 0.5998 m   (rear axle)
R_base = sqrt(R_rear^2 + (L/2)^2)   = 0.6355 m   (base_link, what nav2 plans)
k_max  = tan(d)/L                   = 1.6673 /m  (the curvature bound)
```

Verified against deliberately broken trees: tightening `<steering_limit>` from 35° to 23°
and changing nothing else produces exactly the four consequences one would otherwise
discover by watching the robot miss corners:

```
FAIL  nav2 minimum_turning_radius covers base_link             0.7500 m >= 1.0153 m  (margin -26.1%)
FAIL  RPP min_lookahead_dist keeps emitted curvature feasible  1.3000 m >= 2.0307 m  (margin -36.0%)
FAIL  smoother yaw cap respects the curvature bound            1.0000 rad/s <= 0.6040 rad/s
FAIL  drive.py STEER_LIMIT matches model.sdf                   0.6109 rad == 0.4000 rad
```

Both failure modes it guards are silent, which is the point: `AckermannSteering` never
rejects an infeasible Twist (it clamps the radius and the robot quietly under-turns), and
`max_speed`/`max_accel` work by rewriting a copy of `model.sdf` before gz parses it, so a
launch/file mismatch silently shadows the model every run.

---

## 7. Open items

- **The smoother cannot execute the planner's tightest turn.** Found while writing this
  log, currently FAILING:

  ```
  FAIL  smoother can execute the planner tightest turn   0.6667 /m >= 1.3333 /m  (margin -50.0%)
        -- else effective radius is 1.50 m against a planned 0.75 m
  ```

  `max_velocity: [1.5, 0.0, 1.00]` — the linear cap was raised to 1.5 m/s but the yaw cap
  left at 1.00, whose comment still reads `1.6673 * 0.60`. Since `scale_velocities: true`
  preserves the ratio, the sharpest curvature that survives the smoother is
  `1.00/1.5 = 0.667 /m`, i.e. a **1.50 m** turning radius — 2.5× the robot's true 0.60 m,
  and twice the 0.75 m the planner is allowed to plan. The robot therefore cannot track
  its own plan through tight turns. Fix is `max_velocity: [1.5, 0.0, 2.50]`
  (`1.6673 * 1.5`), but that has not been tested and interacts with `min_lookahead_dist`,
  which was derived at 0.60 m/s.
- **`lookahead_time: 2.20` is stale.** Its comment derives it as `0.60 * 2.20 = 1.32`
  from the old cruise speed. `desired_linear_vel` is now 1.5.
- **Terminal-approach curvature violation is unresolved.** Worst `|wz|/|vx|` observed
  27.57 in one Reeds-Shepp run, 6.67 recorded earlier, against a 1.6673 limit. A real fix
  needs the yaw cap tied to commanded speed, for which RPP has no parameter — a custom
  plugin or an accepted compromise.
- **Reeds-Shepp is unfinished.** Plans measurably better (§3) but execution unverified.
- **Costmap `width: 15` / `height: 22`** are leftovers sized for the old 14×21 m
  warehouse. Harmless (the static layer resizes both to the incoming map) but misleading.
- **Nothing checks map-vs-world freshness.** Exactly the drift class `check_limits.py`
  exists for, and arguably worse than what it does cover, since a stale map fails as
  "nav2 won't plan there" rather than as an error.
- **Test hygiene.** The robot wedged against shelves repeatedly during these runs and
  froze the odometry, which invalidated several measurements before it was noticed.
  Always confirm the robot can still move (a manual `/cmd_vel` forward burst) before
  trusting a "nav2 is stuck" reading.
