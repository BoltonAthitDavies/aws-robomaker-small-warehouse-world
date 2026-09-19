# Warehouse environment — work log

Record of the changes made to this package, why each was made, and the numbers
measured along the way. Written to support a later write-up: the *measurements*
and the *root causes* are the parts worth quoting, so they are kept verbatim
rather than summarised.

Submodule HEAD at time of writing: `2ca63f5 update bay size compact to the map size`.

Nav2 has its own log: **[nav2.md](nav2.md)** — the waypoint-orbiting bug and its root
cause, goal-heading feasibility, the silently-ignored `bond_timeout`, the stale baked
map, and `script/check_limits.py`.

---

## 1. What was built

A stereo-VIO / Nav2 test environment: a large warehouse whose floor carries
painted bay outlines and walkway lines, populated with props that move on
scripted, repeatable, collision-free trajectories.

Pipeline (all scripts in `script/`, run from the workspace root):

```
git -C aws-robomaker-small-warehouse-world checkout -- models/   # pristine meshes
python3 script/scale_warehouse.py 3.0105741651 2.0               # enlarge the shell
python3 script/add_bays.py                                       # 9 extra bay outlines
python3 script/reshape_floor.py                                  # halve lanes, grow bays
python3 script/reshuffle_bays.py --seed 5 --speed 2.5 ...        # place props, route them
python3 script/route_all.py <world> ...                          # re-solve motion only
python3 script/bake_map.py --origin -21.1 -21.1 --size 844 844   # nav2 occupancy grid
```

`scale_warehouse.py` and `reshape_floor.py` are **not idempotent** — their factors
are absolute against pristine geometry. Always `git checkout -- models/` first.

---

## 2. Geometry

### Warehouse shell
Original interior 13.713 × 20.642 m → **41.2833 m square** (1,704 m², 6× the
original floor area). Achieved with per-axis factors **X ×3.0105741651, Y ×2.0**,
written into the COLLADA vertex arrays of wall / ground / roof, visual *and*
collision. Height unchanged at 9.020 m.

The package has **no parametric dimensions** — `model.sdf` is a pointer file and
every size is frozen into `<float_array>` vertex data in centimetres
(`<unit meter="0.01">`).

### Floor markings
Two materials share one mesh, with **disjoint UV index sets**:

| material | texture | role |
|---|---|---|
| `#946568` | `GroundB_01.png` 1024² | tiling concrete |
| `#946569` | `GroundB_02.png` 512² | 4-band colour **atlas** for the paint |

Atlas bands (u selects the colour): hazard stripes 0.000–0.342, green 0.342–0.590,
blue 0.590–0.820, yellow 0.820–1.000.

### Bays and zones
14 bays. `reshape_floor.py` halved every walkway and grew the bays into the freed
floor with 0.30 m clearance:

| | before | after |
|---|---|---|
| lane A / B | 2.584 / 2.510 m | 1.292 / 1.255 m |
| lane C / D / E | 1.684 / 1.793 / 1.686 m | 0.842 / 0.897 / 0.843 m |
| west bay | 6.00 × 6.00 m (36 m²) | **7.94 × 7.24 m (57 m²)** |
| stock bay | 9.03 × 6.00 m (54 m²) | **14.57 × 7.34 m (107 m²)** |
| east bay | 9.03 × 6.00 m (54 m²) | **13.89 × 7.66 m (106 m²)** |

**Zones** = regions enclosed by the green walkway lines, derived by carving the
green strips out of the floor and taking connected components. There are **four**
containing bays (plus two bayless edge slivers of 100 m² and 14 m²):

```
west    bays 1-5     347 m²
stock   bays 6-10    587 m²
east_s  bays 11-13   351 m²
east_n  bay  14      225 m²
```

The non-obvious one: **walkway D cuts across the east column**, isolating bay 14.
That is why east is two zones, and why bay 14 can never make a same-zone
bay-to-bay move.

---

## 3. Measurements worth quoting

### Simulator performance — the cost is CONTACTS, not motion

| configuration | RTF | camera (nominal 30 Hz) |
|---|---|---|
| 30 dynamic props, `max_step_size` 0.001 | **0.307** | 9.2 Hz |
| 30 dynamic, step 0.002 | 0.529 | 15.9 Hz |
| 30 dynamic, step 0.004 | 0.996 | 29.9 Hz |
| 9 dynamic, step 0.001 | 0.692 | 20.7 Hz |
| 30 dynamic **in zero gravity** (no floor contact) | **1.006** | — |
| 30 dynamic, at rest, no plugin | 0.302 | — |
| 30 **static** | 1.005 | 30.3 Hz |
| 30 static, kinematically driven | **1.010** | 30.3 Hz |

Three conclusions, each measured rather than assumed:

1. **Moving is free; being in contact is not.** Dynamic bodies floating in zero
   gravity ran at RTF 1.006; the same bodies resting on the floor ran at 0.294.
   A resting contact is a constraint re-solved every step (1000×/sim-second at
   `max_step_size` 0.001), not a cached fact.
2. **Cost grows worse than linearly** with dynamic body count: 0 → 9 → 30 props
   gave RTF 1.005 → 0.692 → 0.307, i.e. 0.050 then 0.086 cost per body.
3. **The cameras were never the problem.** Two 1280×720 @ 30 Hz sensors run at
   full rate when the props are static.

### The camera "slowdown" was not a slowdown

Measured camera frame interval in **sim time**: median 0.0330 s against a nominal
1/30 = 0.0333. The camera never missed a beat. Sensors are scheduled on the
simulation clock, so at RTF 0.30 one wall second contains 0.30 sim seconds and you
receive ~9 frames. Recorded data with `use_sim_time` is unaffected; only
wall-clock throughput changes.

### Friction is not what the SDF says
Bisection in this world: props immobile at `F = m(3 + 0.6g)`, moving at
`F = m(10 + 0.6g)`. That brackets the effective µ at roughly **1.0–1.6** — neither
the prop's declared 0.6 nor the ground's 100. Lowering the ground's µ to 0.2
changed nothing, so it is not a simple min/max of the two surfaces.
`reshuffle_bays.py` therefore uses a measured `MU_EFF = 1.6`.

Floor friction is otherwise **uniform**: one `<collision>`, one friction block, and
the painted markings exist only in the *visual* mesh (154 triangles) while the
collision mesh is a plain 12-triangle box. Driving over a line changes nothing.
`mu=100 / mu2=50` is anisotropic on paper, but every wheel-ground contact resolves
to `min(100, 1) = 1` because the wheels declare µ=1.

---

## 4. Bugs found, and their root causes

Ordered roughly as encountered. Most were silent failures.

**`*2` in a DAE does not evaluate.** COLLADA `<float_array>` is whitespace-separated
literal floats; expressions do not parse. Hand-edited `699.023682*2` produced a
file Gazebo could not load.

**Scaling the atlas UVs repainted the floor.** A uniform ×2 on all UVs changed
**72 of 82** line faces' colours and left 18 straddling a band edge. The two
materials share one UV array but use disjoint index sets, so only the concrete's
indices may be scaled. Bay/walkway UVs must never be scaled.

**Props spawn 20–29 mm *inside* the floor** in the stock poses. Invisible while
static; for a dynamic body it is a permanent penetration the solver fights every
step. Fixed by computing pose z as `FLOOR_TOP − collision_z_min` (FLOOR_TOP =
0.034223).

**`VelocityControl` pins velocity, not pose.** It rewrites `LinearVelocityCmd` and
`AngularVelocityCmd` every `PreUpdate` with z = 0, cancelling gravity. The velocity
a contact imparts is erased next step, but the displacement and rotation already
produced within that step are not — so props *ratchet* upward and tilt. This was
the "objects moving mid-air" symptom. It is climbing out of the floor, not over
other objects.

**`<initial_linear>` is in the MODEL'S LOCAL FRAME.** World velocity is
`R(yaw) · initial_linear`. Proof: decoding the stock world's 11 movers as
body-frame yields world velocity (0, −1.011) for *every* one — they all sweep −y
with yaw compensated. Writing world vectors while assigning random yaws made props
in one bay scatter in different directions.

**`Bucket_01`'s link is named `body`**, every other prop's is `link`.
`TrajectoryFollower` requires `<link_name>`; get it wrong and only the buckets
silently fail to move.

**Editing the wrong launch file.** `nav2_dynamic.launch.py` does *not* include
`small_warehouse_dynamic.launch.py` — it includes the base `small_warehouse.launch.py`
and forwards `world`. Changes to the `_dynamic` launch were never loaded.

**`dynamic_pose/info` excludes static entities.** Once props became `<static>true`
and kinematically driven, they vanished from that topic, so `viewer.py --live-poses`
drew all 30 frozen at spawn while gz moved them. Now bridges
`/world/default/pose/info` instead. Measured cost: 58 Hz / ~149 entities / ~1.0 MB/s
versus 58 Hz / 8 entities / ~115 KB/s. `bridge_model_poses` (default False) also had
to be *forwarded* by the nav2 launches; it was being silently swallowed.

**A commented block ends `</include> -->`.** A regex scanning raw XML from a
commented `<include>` cannot close there and runs on to swallow the *next live*
block — 3 of 12 live includes were lost this way. All scripts now blank comment
bodies to spaces first, preserving offsets.

**`fill_bays.py` carried its own stale bay table.** It hard-coded the
*pre-reshape* rects (6.00 m bays at `y0 = -17.643`). After `reshape_floor.py` moved
and grew every bay, the script still parsed, still placed 56 props, and put every
one of them in the wrong place — a wrong answer, not an error. Rewritten to derive
the rects from `reshuffle_bays` (which reads them out of the ground mesh), and it
now asserts containment and same-bay non-overlap rather than trusting the pitch
arithmetic.

**`%.4f` waypoints caused 0.1 mm formation shear.** Each waypoint rounds
independently, so the *difference* across a group varied. Now `%.6f`.

---

## 5. Motion: why kinematic, and the scheduling constraints

`TrajectoryFollower` (force-based) was tried and abandoned. It works, but:
it needs `<static>false>` (the 3.4× RTF cost above); it is a bang-bang controller
with no braking, so props overshoot badly (one bound for a bay centred at y = 11.13
reached y = 15.95) and **5 of 30 ended outside any bay**; and its default 60 N gives
a 1 kg prop 54 m/s².

Replaced by a purpose-written plugin, `gz_kinematic_trajectory/KinematicTrajectory.cc`,
installed to `~/.ignition/gazebo/plugins/`. Props stay `<static>true</static>` and
their pose is written each step via `components::WorldPoseCmd` — the same mechanism
the `set_pose` service uses.

Verified: a static model moved this way **still carries its collision** (a ball
dropped onto a teleported static platform landed on top at the new location, z =
1.2500). Formation drift measured at **0.000000 m** — perfectly rigid.

The trade-off: a pose-driven body has no velocity, so Gazebo cannot compute a
collision *response*. Contact with the robot resolves as overlap.

### SDF parameters
`<speed> <loop> <waypoints> <phase> <dwell_far> <dwell_home>` (and `<dwell>` as a
default for both).

### Scheduling results (proved by construction, not by trial)

- **Two groups sharing a corridor cannot be staggered with a symmetric dwell.**
  The leader must turn for home after the follower (`d0 > d1 + lag`) while both
  keep the same period (`d0 == d1`). Both cannot hold.
- **Unequal periods drift into a collision.** The nesting scheme forcing the
  follower to half the leader's period put its second round trip head-on with the
  leader returning — measured collision at t ≈ 47.8 s.
- **Zero lag is safe** (identical motion ⇒ constant separation), but reads as
  moving together, not as a sequence.
- **Asymmetric dwell resolves it.** Splitting the wait lets both keep period
  28.00 s while turning at different moments. Current `_02`: leader departs 0.00,
  follower 3.00, follower turns 14.50, leader turns 16.48. Verified over a
  **2000 s** analytic sweep: no overlap ever.

Other invariants the router enforces: corridors pairwise disjoint (swept AABB,
0.25 m margin), no waypoint outside a bay interior, one heading per zone where
requested, travel along the zone's long axis, and no overtaking within a
sequenced zone.

---

## 6. Current state

**Worlds** (`worlds/small_warehouse_dynamic/`): `small_warehouse_dynamic.world`
(canonical) plus scenario variants `_00`, `_01`, `_02`.
`small_warehouse_static.world` is the frozen twin of the canonical world —
identical props and poses, no plugins — generated in the same pass.
`small_warehouse_static_01.world` is the **fully-stocked** variant (2026-09-03):
all 14 bays occupied at grid capacity, 88 props = 74 ground + 14 stacked buckets,
versus 30 props in 6 bays for the others. Built with
`python3 script/fill_bays.py --world <path>`.

**Maps**: `maps/baked` (canonical), `maps/baked_static`, `maps/baked_dynamic`,
`maps/baked_static_01` (occupied 110020 / 712336 = 15.4%).
`maps/002` and `maps/005` are obsolete SLAM products describing the *original*
13.98 × 20.907 m building and cannot be regenerated without re-running SLAM.
All bakes use `--origin -21.1 -21.1 --size 844 844`.

**Archives**: `worlds/map_v1_tinywarehouseOriginal/` (world files only) and
`worlds/map_v2_largewarehousesmallbaysize/` (world files **plus the matching
ground mesh** and a README). The mesh matters: a `.world` does not contain the
floor — bays and walkways are geometry inside the shared
`aws_robomaker_warehouse_GroundB_01` visual mesh, so archiving worlds alone leaves
them pointing at whatever the shared model currently is. `map_v1` has this gap.

**Launch**: `bridge_model_poses:=True` is required for `viewer.py --live-poses`.

```bash
ros2 launch aws_robomaker_small_warehouse_world nav2_dynamic.launch.py bridge_model_poses:=True
python3 script/viewer.py --rotate 180 --live-poses \
  --world .../small_warehouse_dynamic/small_warehouse_dynamic.world \
  --map   .../maps/baked_dynamic/map.yaml --floor-ppm 40
```

`viewer.py` also renders the painted floor markings (key `l`), rasterised from the
mesh and sampled from the real atlas, so the colours are the actual paint.

---

## 7. Open items

- **Stock's two periods differ by 4×10⁻⁴ s** (28.278685 vs 28.278285) because the
  two routes are not exactly equal in length. Harmless over 2000 s; quantising
  both routes to a common travel time would remove the drift entirely.
- **`maps/002` / `maps/005`** describe a building that no longer exists.
- **`map_v1` archive has no mesh**, so it cannot restore its own floor.
- **Prop collision meshes are triangle meshes.** Swapping them for `<box>`
  primitives is the untried change that would let dynamic props be affordable
  again, if force-based motion is ever wanted back.
- **`viewer.py` floor constants are hard-coded** (`FLOOR_X0/X1`, `TILE_U/TILE_V`,
  `U0/V0`) and were re-fitted after the rescale; they are not derived at runtime,
  so another rescale needs another fit. The stock values are recorded in the
  comment there.

---

## 8. Addendum — the fully-stocked static variant (2026-09-03)

`small_warehouse_static_01.world` fills **every** bay, at grid capacity: small bays
2×2 = 4, big bays 3×2 = 6, plus one stacked Bucket per bay. 88 props against the
30 the other worlds carry.

Worst-case footprint is 2.16 m against a cell pitch of at least 3.47 m, so ground
props never touch; every cell centre also clears the bay interior at all four
yaws. Both are asserted by the script, not assumed.

**The stacked buckets are invisible to the 2D map, and that is correct.** Each
adds `+ 0 cells` at bake time — all 14 of them. The bake band is z ∈ [0.050, 2.000]
and a bucket sitting at z = 1.115223 does overlap it, but the bucket's footprint
(0.941 × 1.222) lies entirely inside its ClutteringA base's (2.161 × 2.002) at the
same centre, so every cell it would claim is already occupied. Concentric stacking
buys vertical texture for the cameras at zero cost to the occupancy grid.

No RTF concern: these props are `<static>true</static>` with no plugin, and static
bodies never enter the constraint solver (see §3).

---

## 9. The featureless-floor variants (2026-09-10)

Four worlds have a completely blank floor — no concrete texture, no bay
outlines, no walkway lines:

```
worlds/small_warehouse_dynamic/small_warehouse_dynamic_00_nofloortexture.world
worlds/small_warehouse_dynamic/small_warehouse_dynamic_01_nofloortexture.world
worlds/small_warehouse_dynamic/small_warehouse_dynamic_02_nofloortexture.world
worlds/small_warehouse_static/small_warehouse_static_01_nofloortexture.world
```

Built by `script/make_plain_floor.py --world <path> [--world <path> ...]`, which
is idempotent — it regenerates the plain model each run and can be re-pointed at
any number of worlds.

**They could not be made by editing the world files.** Nothing about the floor's
appearance is in a `.world` — every world just includes
`model://aws_robomaker_warehouse_GroundB_01`, and the markings and concrete are
geometry and materials *inside that shared model's visual mesh*. Editing it would
strip the floor out of all twelve worlds at once. So the script generates a
sibling model, `aws_robomaker_warehouse_GroundB_01_plain`, and repoints the
include's `<uri>` and `<name>`. Each world's diff against its textured twin is
exactly those two lines.

What the generator removes from the visual DAE:

| step | effect |
|---|---|
| drop `<triangles count="154" material="#946569">` | every painted line, in one batch |
| drop effect + material `#946569` | they still named a texture image |
| concrete `<diffuse>`: `<texture>` → `<color>` | flat 0.813810 0.811311 0.804215 |
| `<library_images>` → `<library_images/>` | no texture fetched at all |

The flat colour is the **measured mean of `GroundB_01.png`**, not an invented
grey. That texture's per-channel std is only ~0.037, so mean substitution holds
albedo and scene brightness essentially fixed while removing the spatial
variation — the right control if this is a VIO ablation: change the texture, not
the exposure.

**The trap the guard caught.** Emptying `library_images` first looks sufficient,
but the markings' *effect* survives the removal of its triangles and still carries
`<init_from>GroundB_02.png`. Blanking the image library under a live `<texture>`
leaves a dangling reference. The script now asserts `'<texture' not in dae` before
writing, which is what surfaced it.

**Collision geometry is byte-identical** (same md5), so physics, friction and maps
are untouched. Proved rather than assumed: baking
`small_warehouse_static_01_nofloortexture.world` produces a `map.pgm` identical to
`maps/baked_static_01`. **No re-bake is needed for these worlds** — reuse
`baked_static_01` and `baked_dynamic`. The markings were never in the collision
mesh (12 triangles) to begin with.

All four load clean headless; each dynamic one keeps all 60 of its
`KinematicTrajectory` references and its 32 props, and the static one its 88.

**A duplicated XML declaration made `_02` invisible to the tooling (fixed
2026-09-10).** `small_warehouse_dynamic_02.world` carried a second
`<?xml version="1.0" encoding="utf-8"?>` on line 2. Gazebo loads it regardless —
libsdformat's TinyXML2 parser skips the stray declaration — so it looked fine in
simulation. But it is invalid XML, and `bake_map.load_world()` uses ElementTree,
which rejects it outright. Everything downstream of that parse silently lost the
world: `viewer.py` printed `could not parse world ...; obstacles hidden` and drew
**zero** obstacles. Only these two files were affected (`_02` and its
`_nofloortexture` copy); line 2 deleted from both, and both now parse.

The lesson worth keeping: most scripts here are regex-based and tolerated the bad
file, so the defect survived until the one DOM-based reader met it. "It loads in
Gazebo" is not evidence a world file is well-formed.

**`viewer.py` now derives the floor from the world (fixed 2026-09-10).** It used
to hard-code the canonical GroundB_01 mesh and its two textures, so viewing a
`_nofloortexture` world drew 154 painted bay outlines that are not in that world
— the overlay disagreed with what the cameras see, and it failed the quiet way:
the floor still rendered, just wrongly. `ground_assets(world_path)` now reads the
ground model the world actually includes, then reads that mesh's `<diffuse>` to
decide what it is made of:

| world's ground model | markings | concrete |
|---|---|---|
| `GroundB_01` | 154 triangles, `GroundB_02.png` atlas | `GroundB_01.png` |
| `GroundB_01_plain` | **0** | flat `(0.8138, 0.8113, 0.8042)` |

"Is there a texture?" had to become a question rather than an assumption:
`build_floor_array` accepts an `(r,g,b)` tuple and fills, because the plain model
has no image to tile UVs against. `--texture` still overrides when given
explicitly.

**Re-run after reshaping.** `scale_warehouse.py` / `reshape_floor.py` regenerate
the canonical visual mesh; the plain model is derived from it, so it must be
regenerated too or the blank floor keeps the old slab size.

---

## 10. Near-wall objects in the dynamic worlds (2026-09-11)

Six shelving units stand against the walls in all seven
`worlds/small_warehouse_dynamic/*.world`, added by
`script/add_wall_shelves.py --world <path> ...` (idempotent — it strips any
`Shelf?_01_1xx` it previously wrote before re-placing).

**There is almost nowhere to put them.** `reshape_floor.py` grew the bays out to
within 0.30 m of the walls, so three of the four walls have only a sliver — far
short of a 0.880 m shelf. Exactly two wall-adjacent regions are deep enough:

| region | extent | depth |
|---|---|---|
| north wall, east section | x 6.44…20.33, y 16.60…20.64 | 4.0 m |
| south wall, stock section | x −10.25…4.31, y −20.64…−18.11 | 2.5 m |

Both are leftover *zone* floor rather than walkway, so filling them narrows no
lane. The north one already held the trash can (10.0, 20.0) and the robot spawn
(5.25, 20.0); two `ShelfD_01` go east of them. The south pocket is empty and takes
four `ShelfE_01`.

**The south pocket is bounded east by a walkway leg, not by a bay.** An N-S green
strip at x ∈ [4.612, 4.886] runs all the way down to the south wall, and the E-W
strip at y ∈ [−19.429, −19.252] caps the pocket at 1.21 m deep. So the usable run
is x −12.42…4.61 — 16.7 m for four 3.918 m units, which is why their spacing is
0.20 m and not a round number. The checker caught a fifth candidate standing on
that leg; it was moved rather than forced.

Shelves sit at z = 0.004923 (`FLOOR_TOP − 0.0293`), and all three furniture models
already declare `<static>1</static>` in their own `model.sdf`, so the include needs
no override — same as the trash can.

**Margins are not one number.** An early run rejected the second north shelf as
"too close" to the first: the 0.40 m entity margin is about room to drive past the
trash can and the robot, and applying it between two units standing side by side
in one run is simply wrong. Shelves in a run now only have to not touch (0.20 m);
everything pre-existing keeps 0.40 m.

The stock commented-out shelves are deliberately left commented. Their poses are
from the original 13.98 × 20.91 m building — (−16.5, −19.5) and friends land
inside bays after the rescale, which is why they were disabled.

Verified: all seven worlds parse and load clean headless; the `_nofloortexture`
twins still differ from their originals in exactly the two ground lines; the
viewer reports 37 obstacle footprints, up from 31. `maps/baked_dynamic` re-baked —
each shelf contributes ~1400 cells (they are 2.64 m tall, inside the 0.05–2.00 m
band), occupancy 8.8%, and **`unreachable free` stayed at 8424**, so no region was
sealed off.

### 10b. All four walls, emitted as commentable groups

`script/add_wall_objects.py` (replaces `add_wall_shelves.py`) now places **12
objects in 4 groups** across all seven `worlds/small_warehouse_dynamic/*.world`,
inside one delimited region so a whole wall can be switched off by hand.

| wall | pocket | fits |
|---|---|---|
| north | x 6.44…20.33, y 16.60…20.64 (4.05 m deep) | 2 ShelfD + the existing trash can |
| south | x −10.25…4.31, y −20.64…−18.11 (2.53 m) | 4 ShelfE |
| east | y 4.89…8.94 between bays 12/13 (4.05 m tall) | 2 DeskC, + 1 trash can in the NE corner |
| west | x −20.66…−12.11, y 18.94…20.64 (1.70 m tall) | 2 PalletJackB |

**Neither new wall takes a shelf, and the geometry says why.** The east band
between bays 12 and 13 is 4.049 m and a shelf is 3.918 m long — 0.065 m a side,
under the bay margin. The west wall is worse: bays 0–4 run its entire length at
0.30 m standoff, and the four inter-bay gaps are 0.444 m, narrower than the
smallest model's 0.540 m short side, so *nothing* fits in them. Only the
north-west corner pocket is usable, and its 1.70 m of run takes two pallet jacks
and nothing larger.

The loose `TrashCanC_01_002` was moved into the north group at its original pose
so the group is complete — commenting out "north" now hides everything on that
wall.

**Two XML rules bit in sequence here, both silent under Gazebo.**

1. **A comment may not contain `--`.** The first markers were
   `<!-- ---- north wall ---- -->`. ElementTree rejected every world; Gazebo would
   have loaded them. Markers now use `====`.
2. **The removal regex anchored `<!--[^\n]*BEGIN`, but the token sat on the
   marker's second line.** It matched nothing, so re-running *stacked another full
   copy* of all 12 objects rather than replacing them — three times over before it
   showed up, and the only symptom was `too close to DeskC_01 at (20.09, 5.90)`,
   i.e. each object colliding with its own duplicate. Region removal is now
   line-based (`strip_region`) and shape-agnostic, so it cannot miss again.

Verified: all 7 parse; each group can be wrapped in a comment individually and all
four together (46 → 43/42/43/44, and → 34 with everything hidden); twins still
differ in exactly the two ground lines; viewer reports 42 obstacle footprints (31
before any of this). `maps/baked_dynamic` re-baked at 9.1% occupied, with
`unreachable free` still 8424 — nothing sealed off.

**Cosmetic, pre-existing:** `aws_robomaker_warehouse_DeskC_01`'s visual DAE
declares a fifth `<image>` pointing at a leftover Windows path
(`file://C:\Users\Administrator\Desktop\...\aws_Desk_01.png`). Gazebo logs a red
`Could not resolve file` for it on every load. It is an **orphan** — 0 of the
model's 677 triangles bind it; all four materials use the local DeskC_01..04
textures — so the desks render correctly and only the log is noisy. Deleting that
one `<image>` element would silence it.

---

## 11. `small_warehouse_static_nofloortexture_objonwallonly` (2026-09-11)

The three ablations stacked into one world: **blank floor, empty bays, furniture
only against the walls.**

```
ground   GroundB_01_plain        no concrete texture, no painted lines
bays     empty                   30 props removed
walls    12 objects in 4 groups  2 ShelfD, 4 ShelfE, 2 DeskC, 2 PalletJackB, 2 bins
```

Built by `script/clear_bays.py --world <path>`, which strips every live Bucket /
ClutteringA / ClutteringC / ClutteringD include — the four models
`reshuffle_bays.py` and `fill_bays.py` place inside bays — and touches nothing
else.

**It refuses to guess what a "bay prop" is.** Every candidate is checked to lie
inside a bay rectangle first, and one outside every bay aborts the run instead of
being deleted. Here all 30 were in bays and none were inside the near-wall region,
so the two sets are cleanly separable; that is a property of this world, not an
assumption baked into the script. It also comment-blanks before scanning, so a
commented-out block cannot be matched and a live block following one cannot be
swallowed.

Map baked to `maps/baked_static_objonwallonly`: **4.6% occupied**, against 8.8%
for `baked_dynamic` and 15.4% for `baked_static_01`. `unreachable free` is 8424,
the same figure every bake of this floor produces, so nothing is sealed off.

This is the visually sparsest world in the set — blank floor, empty middle, texture
only at the perimeter — which makes it the natural worst case for a VIO front-end
that depends on floor and mid-field features.

---

## 12. Perimeter filled out on all four walls (2026-09-11)

`script/add_wall_objects.py` now places **20 objects** (was 12) in the same four
commentable groups, across the 7 dynamic worlds **and**
`small_warehouse_static_nofloortexture_objonwallonly.world`.

| wall | before | after | why that number |
|---|---|---|---|
| north | 3 | **8** | 5.20 m free east of the strip + 7.29 m in the west section |
| south | 4 | **5** | one shelf traded for two desks; see below |
| east | 3 | **5** | NE pocket had 1.16 m below the bin, 1.99 m above it |
| west | 2 | **2** | geometrically capped |

**Two walls were already full, and the arithmetic says why.**

*South*: the run is x −12.42…4.61 = 17.03 m and four shelves used 15.672 m,
leaving 0.41 m — less than the smallest model's 0.882 m. The only way to add an
object was to trade one `ShelfE` for two `DeskC`, which is what the group now
does: 3 shelves + 2 desks = 5 objects in 15.664 m.

*West*: bays 0–4 stand 0.30 m off it for its entire length, and the four
inter-bay gaps are 0.444 m — thinner than any model's 0.540 m short side, so
nothing fits in them at all. The only pocket is the north-west corner at 1.703 m
of run, and `0.540n + 0.2(n−1) ≤ 1.453` caps it at **n = 2**. Two pallet jacks is
the whole west wall, and it stays two until a bay moves.

**Three bugs, all from placing things at exactly the limit.**

1. Three units were rejected as "too close" at a gap of *precisely* `RUN_GAP`.
   The comparison was deciding it on float noise (`-1.886 + 1.959` is
   `0.07300000000000004`, not `0.073`). `hits()` now carries a 1e-9 slack, and
   the runs were re-spaced to leave real clearance rather than ride the limit.
2. Wall-standoff was being computed per group by hand. It is now `back_to(wall,
   model, yaw)`, which takes the rotated depth — needed because a desk against
   the north wall must lie at yaw 90° (at yaw 0 it is 1.555 m deep and reaches
   into the walkway strip 0.048 m short of the margin).
3. **The insert anchor assumed the world had bay props.** It inserted before the
   first Bucket/Cluttering include and fell back to `len(text)` — so in
   `*_objonwallonly`, which has none, the entire region landed *after* `</sdf>`:
   `junk after document element`. Gazebo would have loaded it. The fallback is
   now `</world>`, and a world with neither aborts rather than producing a
   malformed file.

Verified: all 8 parse, one region each, 20 objects each; twins still differ in
exactly the two ground lines; both worlds load clean headless. Maps re-baked —
`baked_dynamic` 9.1% → **9.6%**, `baked_static_objonwallonly` 4.6% → **5.1%**.

`unreachable free` moved 8424 → **8425**: one 0.05 m cell, almost certainly a
corner pocket behind a new unit. Worth recording rather than rounding away, but
it is one cell, not a sealed region.

---

## 13. The GUI camera was parked inside the roof (2026-09-11)

`headless:=False` opened a window that showed nothing. Not a rendering failure —
a camera inside geometry.

Every world shipped with:

```xml
<camera_pose>0.0 0.0 10.0 0.0 1.5708 -1.5708</camera_pose>
```

That is **z = 10.0 m, pitch +1.5708 rad — straight down**. `RoofB_01`'s collision
mesh occupies **z 9.020 … 10.124**, so the start-up camera sits *within the roof
slab*, looking into it. The viewport fills with one flat surface and the scene
appears empty. Raising it above the roof does not help: the roof is opaque, so the
camera has to be underneath.

Replaced by an interior corner view computed by `script/set_gui_camera.py`:

| world family | pose | why |
|---|---|---|
| rescaled 41 m warehouse | `-18 -18 8.5`, looking at (0,0,1) | high in the SW corner, under the 9.020 m roof |
| `*_tinymap` | `-5.5 -8.5 6.0`, same target | `model_tiny` keeps the pristine 13.98 × 20.91 m shell, so the big pose is *outside* its walls |

`<camera_pose>` is GUI-only: physics, sensors, the robot's own cameras and every
baked map are unaffected.

**Two things I clobbered by not checking first**, now guarded in the script:

- the archived `map_v1` / `map_v2` snapshots, which exist to stay byte-identical
  to what they recorded;
- `no_roof_small_warehouse.world`, which has **no roof** and therefore already had
  a good pose (`-4.70385 10.895 16.2659 …`) looking in from outside and above.

The script now skips any path under `map_v*` and any camera whose z is not inside
`[9.020, 10.124]`, which makes it idempotent — a second run reports "left alone"
for all 18 worlds. Reverting was clean because the change was one line per file
(`git diff --numstat` showed exactly 2 changed lines each).

**Separately, and still unresolved:** the log also carries
`libEGL warning: egl: failed to create dri2 screen` four times. That is the host's
GL/EGL setup (hybrid graphics on this laptop), not the world. If the window is
still black after this fix, that is the next thing to chase —
`IGN_GUI_RENDER_ENGINE=ogre` or a PRIME offload environment would be the test.

---

## 14. Perimeter packed properly: 20 → 47 objects (2026-09-11)

Seen in the GUI, the walls still read as empty: the hand-listed placements were
**one row deep**, and the pockets behind them were left bare. The perimeter holds
147 m² of usable floor and 20 objects were sitting on it.

`script/add_wall_objects.py` no longer carries coordinates. It carries *regions*,
and a packer lays rows parallel to the wall from the wall inward, cycling a model
list and taking whatever fits:

| pocket | size | rows | result |
|---|---|---|---|
| north-east | 14.24 × 3.75 m | 3 | wall lined 3.33 m deep |
| north-west arm | 6.94 × 1.45 m | 1 | 1.45 m only takes one row |
| south | 16.68 × 0.96 m | 1 | depth is 0.96 m; one row is the maximum |
| east, bays 12↔13 | 14.24 × 3.70 m | 3 | rows along the LONG axis |
| west corner | 1.22 × 1.45 m | 1 | the entire west wall |

**north 20, south 7, east 18, west 2 = 47, with zero skips.**

Row depth is the deepest model in the cycle (0.909 m, the bin) and every unit's
back aligns to the row's near edge, so a mixed row still lines up against the
wall.

**Orientation is worth three times the furniture.** Packed against the east wall,
that pocket's rows are 3.70 m long and hold two units each. Run along its *long*
axis and a row holds six: 6 → 18 objects from the same 53 m², same margins.

**Three failures, all of them the verifier earning its keep.**

1. *15 units rejected as "on a walkway strip".* The region bounds were written as
   exactly `strip_edge + LANE_MARGIN`, so the whole first row landed precisely on
   the threshold. A 2 cm `INSET` on every region fixed it — the same
   exact-boundary class as the `RUN_GAP` epsilon in §12.
2. *The west region packed **empty**.* Its column was 1.161 m — the pallet jack's
   own width — and the 2 cm inset left 1.121 m for a 1.161 m unit. A region must
   be the object plus the inset, not the object alone.
3. *The south "second band" is not free floor.* 1.14 m looked available behind the
   shelving, but a **second** green line runs at y −18.584…−18.412: what lies
   between the two lines *is* the walkway, 0.668 m wide with 0.368 m left after
   margins. All 12 candidates were refused. Filling it would have blocked the
   lane — the region is gone, not forced.

Verified: all 8 worlds parse, 47 objects in 4 groups each, twins still differ in
exactly the two ground lines, both load headless with only the known orphan
`DeskC` texture error. Maps re-baked: `baked_dynamic` 9.6% → **11.9%**,
`baked_static_objonwallonly` 5.1% → **7.5%**. `unreachable free` 8425 → **8439**,
14 cells of 0.05 m — pockets behind furniture, not a sealed region.

---

## 15. The long east and west walls, and the camera reverted (2026-09-11)

### The walls

Bays 0–4 and 10–13 stop 0.30 m short of the east and west walls, which is why
those walls looked bare — nothing fits in 0.30 m. But the bays' **prop grid**
stops much further short of the bay *outline*:

| wall | bay outline | grid + widest prop | free band |
|---|---|---|---|
| west | x −20.356 | reaches x −19.344 | **1.012 m** |
| east | x 20.327 | reaches x 18.948 | **1.379 m** |

So that strip is bay-marked floor that no prop ever stands on, and it runs almost
the full 41 m of each wall. Two new region types stand a row there and are
exempted from the bay-rectangle test — for them, clearance from props replaces it.

**The exemption would have been a disaster without one more measurement.** Sizing
the band off the prop *grid* is wrong: in the dynamic worlds the kinematic plugin
drives props between bays and `route_all --edge` puts destinations at the bay's far
edge, so props actually reach **x = 20.059**, a metre past where the grid ends. A
24 m row of shelving would have been parked in their path. `existing()` now sweeps
each prop's `<waypoint>` list into its AABB, so the check is against where a prop
*goes*, not where it starts.

The result differs per world, decided by measurement rather than by a flag:

| world | north | south | east | west | total |
|---|---|---|---|---|---|
| `*_objonwallonly` (no props) | 20 | 7 | 31 | 19 | **77** |
| `dynamic_00` | 20 | 7 | 26 | 14 | **67** |
| `dynamic_01` / `_02` | 20 | 7 | 30 | 9 | **66** |
| `dynamic` (canonical) | 20 | 4 | 8 | 8 | **40** |

Every rejection in a dynamic world names the prop whose swept path caused it. The
empty world takes all 77 with zero skips.

Maps: `baked_dynamic` **10.9%**, `baked_static_objonwallonly` **10.3%** (was 7.5%
with 47 objects). `unreachable free` 8438 / 8446.

### The camera

Reverted to the stock `<camera_pose>0.0 0.0 10.0 0.0 1.5708 -1.5708</camera_pose>`
on request, in every live world. `no_roof_small_warehouse.world` keeps its own pose
and the `map_v*` archives were never touched.

Worth recording against §13: that pose puts the camera *inside* the roof slab
(z 9.020–10.124) looking down, and the §13 entry called that the cause of the blank
viewport. With backface culling the roof's underside is not drawn from inside, so
the top-down view can work after all — the diagnosis in §13 was not proven, only
plausible, and the user preferred this pose once the scene was visible. If it ever
does render blank, `-18 -18 8.5 0 0.2865 0.7854` is the interior corner view that
definitely works.

---

## 16. Why no top-down view exists in this building (2026-09-11)

Restoring the stock pose (§15) put the ceiling back in the viewport, which settles
§13's open question: the stock `0 0 10` **is** unusable here. But the cause is not
only the roof slab — it is the aspect ratio the rescale created.

The building is **41.28 m wide and 9.020 m tall**. A straight-down camera must sit
below the roof, and at the ign-gui default 60° HFOV:

| camera z | floor visible | |
|---|---|---|
| 8.9 m (just under the roof) | 10.3 m — **25%** of the span | the best a plan view can do |
| 10.0 m (stock) | 11.5 m | inside the roof slab, z 9.020–10.124 |
| 25 m | 28.9 m | above the roof, and the roof is opaque |
| 45 m | 52.0 m | would frame the whole floor, roof permitting |

In the **original** 13.713 × 20.642 m warehouse that same 10.3 m covered 75% of
the width, which is exactly why AWS shipped a top-down pose and why it stopped
working: `scale_warehouse.py` multiplied the floor 6× in area and left the ceiling
at 9.020 m. Nothing about the map ratio changed in this session — the mismatch was
created by the rescale and only became visible once the GUI was used.

So a plan view of this warehouse needs the roof gone (the package's own
`no_roof_small_warehouse.world` is the precedent, and it carries a high angled
pose for that reason). Removing the roof also changes scene lighting, so it is not
free for stereo/VIO runs.

Chosen instead: the interior corner view, `-18 -18 8.5 0 0.2865 0.7854`, on every
live world. `viewer.py` remains the tool for an actual plan view — it draws the
baked map and live poses in 2D and has no ceiling to fight.

`set_gui_camera.py` gained `--force`: the guard added in §13 only recognises a
camera inside the roof band, and the world being run had been hand-set to z = 2.0
pitched straight down (2.3 m of floor in frame), which the guard correctly but
unhelpfully skipped.

---

## 17. Stacking removed — one layer everywhere (2026-09-11)

Reverses the two-layer piles from §8. `fill_bays.py` no longer stacks: the
`BASE`/`TOP` logic is gone and `Bucket_01` joined `GROUND_CYCLE`, so buckets are
still placed, just on the floor like everything else.

A sweep of every live world found stacking in exactly two files —
`small_warehouse_static_01.world` and its `_nofloortexture` twin, 14 buckets at
z = 1.1152 (1.086 m up, one per bay). Both regenerated. The check is now an
assertion in the script (`assert not any(p['stacked'] ...)`) and was re-run across
all worlds afterwards: **nothing sits more than 0.05 m above its `sit_z`.**

**The count drops 88 → 74, and that is unavoidable without a second change.** Bays
were already at grid capacity (small 2×2 = 4, big 3×2 = 6); the 14 stacked buckets
were the only props not occupying a cell of their own, so removing the layer
removes them. Going to 4×2 in the big bays would give 92 flat props — the pitch
supports it (14.130 m interior / 4 columns = 3.53 m against a 2.16 m worst-case
footprint) — but that is a density change nobody asked for, so it was not done.

`maps/baked_static_01` re-baked: 13.0% occupied, `unreachable free` 8424.

Note the stacked buckets never appeared in any baked map anyway (§8): their
footprint sat entirely inside their ClutteringA base's at the same centre, so each
contributed `+ 0 cells`. Removing them changes the 3D scene and the camera imagery,
not the occupancy grid.

---

## 18. One row per wall — the blocks removed (2026-09-11)

Seen in plan view, two regions read as blocks of furniture standing in the floor
rather than as a lined wall. They were the only ones packed more than one row
deep, and both are now single-row:

| region | was | now |
|---|---|---|
| north-east pocket (14.24 × 3.75 m) | 3 rows, ~15 units | 1 row along the north wall |
| gap between bays 12 and 13 | 3 rows, 18 units | 1 row along the **east wall** |

The second is the more useful change. Packed as a pocket, those 18 units were
anchored to a *bay edge*, not to a wall — free-standing furniture in the middle of
the floor. Re-anchored to the east wall it becomes one row filling the y gap
between the two wall-band segments, so the east wall reads as continuously lined
except where walkway D crosses it.

`max(r[7] for r in REGIONS) == 1` — no region packs more than one row anywhere.
Within a row the units' *backs* align to the wall, so their centres differ by half
the depth difference (0.540 to 0.909 m); that is still one row, not two.

| world | north | south | east | west | total |
|---|---|---|---|---|---|
| `*_objonwallonly` | 8 | 7 | 15 | 19 | **49** (was 77) |
| `dynamic_00` | 8 | 7 | 10 | 14 | **39** |
| `dynamic_01` / `_02` | 8 | 7 | 14 | 9 | **38** |
| `dynamic` (canonical) | 8 | 4 | 8 | 8 | **28** |

Maps: `baked_dynamic` 9.8%, `baked_static_objonwallonly` 7.6%. `unreachable free`
8434 / 8436. Twins still differ in exactly the two ground lines.

Together with §17 this settles the shape of the environment: **one layer
vertically, one row horizontally, against the walls only.**

---

## 19. Gazebo and the 2D viewer disagreed — install/ had stopped being symlinks

Symptom: the Gazebo scene did not match `viewer.py`'s plan of the same world.
Neither tool was wrong; they were reading **different files**.

`ros2 launch` resolves the world through `get_package_share_directory`, i.e.
`install/`. `viewer.py` is given a path and reads the source tree. Those had been
the same file — the workspace was built with `--symlink-install` — until a plain
`colcon build` at 14:11 replaced every symlink with a real copy. From that moment
every edit to the source was invisible to anything launched:

| copy | north | south | east | west | total |
|---|---|---|---|---|---|
| source (`viewer.py`) | 8 | 7 | 15 | 19 | **49** |
| `install/` (Gazebo) | 8 | 5 | 5 | 2 | **20** |

Twenty objects is the §12 layout — three rounds of edits behind.

The tell was `stat` disagreeing with itself: the install path reported an mtime of
14:11 while the source said 15:33. `stat` follows symlinks, so a symlink could not
have reported anything but the source's own mtime. `ls -l` confirmed a plain file.

Fixed with `colcon build --packages-select aws_robomaker_small_warehouse_world
--symlink-install`; all 18 installed worlds are symlinks again, and source and
install now agree object-for-object. The maps are symlinked too, so re-baking is
picked up without a rebuild.

**Worth knowing for the report:** a plain `colcon build` in this workspace silently
freezes the worlds, meshes and maps that `ros2 launch` sees. Nothing errors — the
simulation just keeps running an older environment, which for recorded VIO runs
means the bag no longer matches the world it is nominally from. Always
`--symlink-install` here, or rebuild after every world edit.

---

## 20. Random-occupancy variants `_02` / `_03` / `_04` (2026-09-17)

*Superseded by §22 — the range changed from 3–7 to 7–9. Kept for the selector
documentation and the reason the draw is seeded.*

Three copies of `small_warehouse_static_01_nofloortexture.world` with a random
subset of bays emptied, for scene-variation runs. `_01` is the full reference and
was not touched (md5 verified unchanged before and after).

`clear_bays.py` gained selectors instead of a second near-identical script:

```
--bays 3,7,11        empty exactly these
--random 3-7         pick a count uniformly in [3,7], then that many bays
--seed N             make the draw reproducible
(none)               empty every bay -- the *_objonwallonly behaviour
```

Seeds 2 / 3 / 4 for `_02` / `_03` / `_04`, so each world is regenerable from its
own name rather than from a note.

| bay | zone | `_01` | `_02` | `_03` | `_04` |
|---|---|---|---|---|---|
| 0 | west | 4 | 4 | 4 | 4 |
| 1 | west | 4 | — | 4 | — |
| 2 | west | 4 | 4 | — | 4 |
| 3 | west | 4 | 4 | 4 | 4 |
| 4 | west | 4 | 4 | 4 | — |
| 5 | stock | 6 | — | — | 6 |
| 6 | stock | 6 | 6 | 6 | — |
| 7 | stock | 6 | 6 | 6 | 6 |
| 8 | stock | 6 | 6 | — | 6 |
| 9 | stock | 6 | 6 | — | 6 |
| 10 | east_s | 6 | 6 | 6 | 6 |
| 11 | east_s | 6 | 6 | 6 | — |
| 12 | east_s | 6 | 6 | 6 | 6 |
| 13 | east_n | 6 | — | 6 | 6 |
| **props** | | **74** | **58** | **52** | **54** |
| **bays emptied** | | 0 | 3 | 4 | 4 |

Bay 7, 10 and 12 happen to be stocked in all four, and bay 0 and 3 likewise —
that is the draw, not a constraint. The counts came out 3 / 4 / 4 against a
uniform [3,7]; an honest draw, but the low end of it, so the three variants differ
less than the range suggests.

Maps baked to `maps/baked_static_02` … `_04` (10.8% / 10.3% / 10.2% occupied,
against 13.0% for `_01`). Rebuilt with `--symlink-install` so `ros2 launch` sees
them — see §19 for why that matters.

---

## 21. Wall objects added to `_01` … `_04` (2026-09-17)

`add_wall_objects.py` run on the four stocked `*_nofloortexture` worlds, giving
them the same perimeter as `*_objonwallonly`.

| world | bay props | north | south | east | west | wall total |
|---|---|---|---|---|---|---|
| `_01` | 74 | 8 | 7 | 15 | 13 | **43** |
| `_02` | 58 | 8 | 7 | 15 | 14 | **44** |
| `_03` | 52 | 8 | 7 | 15 | 15 | **45** |
| `_04` | 54 | 8 | 7 | 15 | 16 | **46** |
| `objonwallonly` | 0 | 8 | 7 | 15 | 19 | **49** |

**North, south and east are identical in all five; only west varies, and it varies
with bay occupancy.** The west band (x −20.556 … −19.544) runs alongside the west
bays, whose first prop column reaches x = −19.344 — a 0.20 m gap against a 0.40 m
entity margin. So each stocked west bay culls a unit or two, and the count rises
monotonically as bays empty: 13 → 14 → 15 → 16 → 19. Every skip names the prop
responsible (all `ClutteringA/C_01` at x = −18.26). The east band is unaffected
because east bays stop 1.379 m short of their outline.

Bay props are untouched (74 / 58 / 52 / 54 before and after) — the wall script only
adds its own delimited region.

Maps re-baked: 16.6% / 14.6% / 14.2% / 14.2% occupied, `unreachable free` 8436 in
all four. Rebuilt with `--symlink-install`.

**Divergence to be aware of:** `small_warehouse_static_01.world` (the *textured*
twin of `_01_nofloortexture`) was not named in the request and so was left alone.
The pair now differs by 240 lines rather than the usual 2 — the twin has no wall
objects. `_02` … `_04` have no textured counterpart, so nothing to keep in step
there.

---

## 22. Re-rolled to 7–9 empty bays (2026-09-17)

Replaces §20's 3–7. **Re-rolling required going back to full occupancy first** —
running `clear_bays.py` again on an already-thinned world empties 7–9 *more* bays,
so the union would have overshot. Each world was refilled to 74 props with
`fill_bays.py` (default seed 3, the same layout `_01` carries) and then drawn on.

| bay | zone | `_01` | `_02` | `_03` | `_04` |
|---|---|---|---|---|---|
| 0 | west | 4 | 4 | 4 | 4 |
| 1 | west | 4 | — | — | — |
| 2 | west | 4 | — | — | — |
| 3 | west | 4 | — | 4 | 4 |
| 4 | west | 4 | — | 4 | — |
| 5 | stock | 6 | — | — | 6 |
| 6 | stock | 6 | 6 | 6 | — |
| 7 | stock | 6 | 6 | — | — |
| 8 | stock | 6 | 6 | — | 6 |
| 9 | stock | 6 | — | — | 6 |
| 10 | east_s | 6 | 6 | 6 | 6 |
| 11 | east_s | 6 | 6 | 6 | — |
| 12 | east_s | 6 | 6 | 6 | — |
| 13 | east_n | 6 | — | — | 6 |
| **bay props** | | **74** | **40** | **36** | **38** |
| **bays empty** | | 0 | **7** | **7** | **7** |
| **wall objects** | | 43 | **48** | **46** | **48** |

**All three drew 7.** `random.Random(s).randint(7, 9)` returns 7 for s = 2, 3 and
4 — a legitimate uniform sample, and the same thing happened at 3–7 (3, 4, 4).
Three draws from a three-wide range landing identically is unremarkable; picking
different seeds *because* they produce 7/8/9 would be choosing the outcome, so it
was not done. `--bays` takes an explicit set if a guaranteed spread is wanted.

**The wall objects had to be re-run, and that is a real coupling.** The west band
(x −20.556 … −19.544) is culled where a west bay's props sit within the 0.40 m
entity margin, so emptying more west bays frees more band. Left alone, the
perimeter would have been sized against the *previous* occupancy: 44/45/46 against
the 48/46/48 the new geometry allows. Bay occupancy and perimeter density are not
independent in this environment — change one and re-run the other.

`_01` untouched again (md5 verified). Maps re-baked: 13.0% / 12.0% / 12.2%.
Rebuilt with `--symlink-install`.

---

## 23. Wheel odometry: bridged, and noisy (2026-09-17)

Two additions, because the wheel odometry was neither on ROS nor corruptible.

**Bridged.** `small_warehouse.launch.py` gained `bridge_wheel_odom` (default
False), publishing AckermannSteering's own odometry as
`/model/<robot_name>/odometry`. It is deliberately a separate node from
`ground_truth_bridge`: one is dead reckoning, the other is the reference, and you
may want either without the other.

**Noise had to go downstream, and that is not a preference.** Fortress's
AckermannSteering exposes 16 SDF parameters — checked against the library's own
symbol table, not the docs — and **none is noise**:

```
left_joint  right_joint  left_steering_joint  right_steering_joint
wheel_base  wheel_radius  wheel_separation  steering_limit
min/max_velocity  min/max_acceleration
odom_topic  tf_topic  child_frame_id  odom_publish_frequency
```

So `script/wheel_odom_noise.py` republishes with error added.

### The model, and why jitter alone would be wrong

Real dead reckoning is wrong because errors **integrate**. Per-message Gaussian
jitter averages out and never drifts, which is the opposite of the failure being
modelled. The node corrupts the twist and then integrates the corrupted twist into
its own pose:

```
v' = v * scale_v + N(0, sigma_v)      scale_v 1.02  sigma_v 0.02 m/s
w' = w * scale_w + N(0, sigma_w)      scale_w 0.99  sigma_w 0.01 rad/s
th' += w' dt ;  x' += v' cos(th') dt ;  y' += v' sin(th') dt
```

`scale_*` is systematic (wheel radius long, track wide) and gives steady drift;
`sigma_*` gives a random walk. Covariance is filled in — twist from the sigmas,
pose growing with elapsed time — so a consumer reading covariance sees uncertainty
grow rather than trusting a drifting pose.

**Measured end to end** against a synthetic 1 m/s straight line at 50 Hz:

| | |
|---|---|
| true distance after 499 msgs | 9.980 m |
| reported | **10.200 m** |
| error | **+0.220 m = +2.20%** — exactly `scale_v` 1.02 |
| lateral / heading drift | −0.002 m, −0.2° — the `sigma_w` walk |

### What it does not touch

`/ground_truth/odometry` is unaffected: different system (`OdometryPublisher`),
reading the model's true pose from the ECM. Nothing downstream of the wheel
joints can reach it.

Keyboard control in `viewer.py` is also unaffected — it is **open-loop**, a
keypress publishes a `Twist` on `/cmd_vel` and nothing reads odometry back.

**The trap worth knowing:** `viewer.py`'s `_find_odom()` falls back to *any*
`nav_msgs/Odometry` topic not in its small skip set, so a stray odometry feed gets
silently adopted and drawn as a VIO estimate. The node therefore defaults to
`/wheel_odom_noisy` rather than something the viewer would grab, and the launch
argument's own help text says so. Check the `<label> topic: <name>` INFO lines at
viewer startup if an unexpected trail appears.

---

## 24. Drivetrain and encoder realism (2026-09-17)

Three pieces, all opt-in, none of which touches `/ground_truth/odometry`.

### Encoders exist now

`model.sdf` gained `JointStatePublisher`, naming its six joints explicitly rather
than defaulting to "all" — the default would silently change the message layout if
a joint were ever added. Launch argument `bridge_joint_states` (default False)
puts it on ROS as `sensor_msgs/JointState`; the `ignition.msgs.Model` ↔
`JointState` mapping is present in this `ros_gz_bridge`.

### `script/encoder_sim.py` — the resolution limit

The raw feed is the solver's continuous float angle. A real incremental encoder
knows the angle only to `2π/N`. That is not a rounding detail: velocity derived
from two quantised positions has a **floor of `(2π/N)/dt`**.

Measured, 1024 CPR at 50 Hz, wheel turning at a true constant **0.2 rad/s**:

| | |
|---|---|
| tick | 0.006136 rad = **0.36 mm** at the rim |
| velocity floor | 0.30680 rad/s = **17.9 mm/s** |
| reported velocity | **0.30680 rad/s** — one tick per sample |

The encoder cannot say 0.2. It says 0 or one whole tick, and nothing between. A
controller tuned on the exact feed will not behave the same way on this one.

Velocity is **re-derived** from the quantised positions rather than passed
through. Passing the true velocity beside a quantised position is the trap worth
naming: it looks quantised while still carrying the exact answer.

### `script/drivetrain_sim.py` — deadband and lag

`AckermannSteering` already clamps acceleration (3 m/s²). It has no notion of a
deadband or of motor bandwidth. This node sits between teleop and the bridge —
`cmd_vel_bridge_topic:=/cmd_vel_exec` reroutes the bridge, and teleop keeps
publishing `/cmd_vel` unchanged.

Measured against a step command:

| command | result |
|---|---|
| 0.02 m/s (below the 0.05 deadband) | **0.0000 m/s** — never moves |
| 1.00 m/s step, t = 0.15 s | 0.633 (first order predicts 0.632) |
| t = 0.30 s | 0.883 (0.865) |
| t = 0.45 s | 0.957 (0.950) |

Deadband is applied to the **input, before the lag**. After the lag it would chop
the tail off every deceleration and make the robot stop abruptly — the opposite of
the effect being modelled. Steering is **rate limited** rather than lagged, because
a rack has a maximum speed and a first-order lag would let tiny corrections arrive
instantly. A command watchdog stops the robot if the publisher goes quiet.

### What is NOT reachable in Fortress, established by inspection

- **Fisheye**: `wide_angle_camera` is a gz-sensors**7** (Garden) library and is
  absent here. `<sensor type='wideanglecamera'>` parses — sdformat12 accepts the
  tag — and is then never created. Matching the real rig's KANNALA_BRANDT lens
  needs a Gazebo upgrade.
- **Image distortion**: `OgreDistortionPass` exists in
  `libignition-rendering6-ogre.so` but **not in `ogre2`**, which is the default
  engine. On ogre2, `<distortion>` fills the `camera_info` D vector and leaves the
  image pinhole — so a VIO front-end would undistort an undistorted image and
  score worse for a fake reason.

That leaves a real sim/real gap on the camera side worth stating in the report:
sim is PINHOLE 1280×720 @ 30 Hz with zero distortion; the rig is KANNALA_BRANDT
1920 @ 20 Hz. Some of the sim's 26 cm ATE advantage is the lens, not the algorithm.

---

## 25. The launch files are 16 copies, and that bit again (2026-09-17)

`bridge_wheel_odom:=True` produced no topic, and `bridge_joint_states` did not
exist. Neither the plugin nor the bridge was at fault: the gz side was verified
publishing `/model/ackermann_robot_001/joint_state` with per-joint position and
velocity, and the library registers both `ignition::gazebo::` and `gz::sim::`
plugin names, so the SDF name was fine.

**The per-world launch files are independent COPIES of
`small_warehouse.launch.py`, not includes.** §23 and §24 edited the base only, so
15 siblings knew nothing about the new arguments — and `ros2 launch` accepts an
undeclared `key:=value` on a top-level file without complaint, so the flag was
silently ignored.

This is the second time this duplication has caused a bug. The first was
`bridge_model_poses` not reaching `nav2_dynamic.launch.py`; the symptom then was
also "the flag does nothing".

All 15 patched by anchor (they are *not* a strict subset of the base — they carry
15 lines of their own, reworded comments and an un-parameterised `cmd_vel`
remap). Verified: every marker appears exactly once in all 16, and all 16 build a
valid launch description.

**A patch bug worth recording, because the first attempt looked like it worked.**
The three new `DeclareLaunchArgument` blocks sit *adjacent with no blank line
between them* in the base, so extracting one with "start marker to the next
`\n\n`" captured all three. Appending the other two then duplicated them:
`declare_cmd_vel_bridge_topic_cmd` ended up defined **three times**, and the file
still parsed and still ran, because a later definition simply rebinds the name.
The count check (`grep -c` per marker, expect exactly 1) is what caught it — the
files were restored from backup and re-patched with the blocks split on
`\n(?=    declare_)`.

**Still outstanding:** sixteen near-identical 430-line launch files differing only
in a default world path. Every future change to the launch surface has to be
applied sixteen times or it silently does not apply. The fix is to make the
per-world files thin wrappers that `IncludeLaunchDescription` the base with a
different `world` default — the nav2 and localization launchers already work that
way, which is why they needed no patch here.

---

## 26. The viewer labelled wheel odometry as VINS (2026-09-17)

With VINS not running at all, the viewer showed **`VINS err 0.567 m`** and drew an
orange trail. The trail was `/model/ackermann_robot_001/odometry` — the
AckermannSteering dead reckoning bridged in §23.

`_find_odom()` tries each estimator's preferred topics and then falls back to
**any** `nav_msgs/Odometry` topic on the graph. Its skip set was only
`{gt_topic, '/odom', '/odometry_filtered'}`, so the wheel odometry was eligible,
adopted, drawn, and **scored** — producing an error figure for an estimator that
was never launched. Nothing in the UI distinguished it from a real VINS run.

This is the failure mode predicted when the bridge was added, and it took one
session to arrive.

Two changes:

**A never-an-estimate filter.** `/model/<name>/odometry`,
`/model/<name>/odometry_with_covariance`, and anything containing `wheel_odom` are
excluded from the fallback. These are simulator dead reckoning by construction.

**The fallback now warns.** It was logged at INFO in the same words as an explicit
topic, so the guess and the certainty were indistinguishable in the console. It
now says which preferred topics were empty, what it adopted instead, and which
`--<key>-topic` flag to pass if that is wrong.

A regex detail worth keeping: the pattern is applied with `.search()`, not
`.match()`. The first version used `.match()`, which anchors at position 0, so the
`wheel_odom` alternative could never fire inside a namespaced name — a check
against `/robot/wheel_odometry` showed it still "eligible".

**The first verification of this was worthless, and that is worth recording.**
The check was "start the viewer, grep the log for an adoption message, find none".
It found none because `NOT_AN_ESTIMATE` was added at module scope while `re` was
never imported in `viewer.py` at all — the viewer died with `NameError` before it
reached any estimator code. An absence-of-output test passes just as happily when
the program never ran. The test now asserts the viewer actually started (its
`[viewer]` banner lines) before drawing any conclusion from what is missing.

Verified properly, both directions:

| graph | result |
|---|---|
| only `/model/ackermann_robot_001/odometry` | viewer starts (117 obstacle footprints), **adopts nothing** |
| that plus `/vins_estimator/odometry` | `VINS topic: /vins_estimator/odometry` — real estimators still resolve |

**For the report:** any VINS or ORB-SLAM3 number recorded from the viewer while a
wheel-odometry topic was live needs re-checking. `--vins-topic ""` disables an
overlay outright, and passing the topic explicitly removes all guessing.

### 26b. A dedicated ODOM overlay

Excluding wheel odometry from auto-adoption (§26) left no way to *look* at it.
Pinning it with `--vins-topic` would have worked and would have relabelled it
VINS, recreating the exact confusion just removed.

`viewer.py` gained a third overlay instead — same machinery, its own identity:

```
--wheel-odom-topic /model/ackermann_robot_001/odometry
```

| | |
|---|---|
| key / label | `odom` / **ODOM** |
| colour | `#8d99ae` grey, dash-dot — visually not a result |
| default | **empty, i.e. off** |

No `'auto'` mode for this one, deliberately. Guessing a wheel-odometry topic is
precisely what produced a VINS error figure with VINS not running; naming it is
the point.

It still gets the rigid fit to ground truth and an error figure, which is the
useful part: the panel's ODOM row is then dead-reckoning drift against truth,
labelled as such. Verified both ways — with the topic live and no flag, zero
overlays resolve; with the flag, `ODOM topic: /model/ackermann_robot_001/odometry`.

---

## 27. The `*_bayline` worlds: bay outlines on a blank floor (2026-09-18)

Eight worlds carrying `bayline` in their name were byte-identical copies of their
`_nofloortexture` twins, i.e. the completely blank `GroundB_01_plain` floor. They
now use a third ground model, **`aws_robomaker_warehouse_GroundB_01_bayline`**:
flat concrete, bay outlines, no walkway lines.

That completes a three-point floor ablation:

| model | concrete | bay outlines | walkway lines |
|---|---|---|---|
| `GroundB_01` | 1024² texture | yes | yes |
| `GroundB_01_bayline` | flat colour | **yes (112 faces)** | no |
| `GroundB_01_plain` | flat colour | no | no |

### Why this needed new machinery

All 154 painted faces are **one `<triangles>` element sharing one material**, so
keeping the bay outlines while dropping the walkway lines is not a matter of
deleting an element. It means rewriting the `<p>` index list face by face, judging
each face by the atlas band its UV centroid falls in. The split is exactly clean:

```
hazard  112 faces   the bay OUTLINES
green    42 faces   the walkway lines
```

`make_plain_floor.py` gained `--keep hazard` for this (and `--keep` accepts any
comma-separated band list).

**The failure this had to avoid is a stale `count` attribute.** COLLADA readers
trust it, and a count longer than the rewritten `<p>` walks off the end of the
array. Verified explicitly rather than by "it loaded": `len(p) == count * 3 *
stride` for both batches (108/108 and 1008/1008), and the largest vertex and UV
indices stay inside their arrays (283 < 284, 123 < 164).

**The atlas has to travel with the model.** Unlike `_plain`, which drops every
image, the surviving stripes still reference `GroundB_02.png` — and a `model://`
mesh resolves `<init_from>` relative to **its own** directory. The generator copies
the atlas into `models/..._bayline/materials/textures/`. Confirmed at load: zero
mentions of `GroundB_02` in the error stream, so the stripes resolve. The
concrete's own image is dropped, being unreferenced once the diffuse is flat.

### State

All 8 parse and load headless with only the known orphan `DeskC` texture error.
Collision geometry is untouched, so **the existing maps still apply**: baking
`_01_bayline` and `_02_bayline` produced files byte-identical to
`maps/baked_static_01` and `_02`. No new maps needed. Rebuilt with
`--symlink-install`.

**Loose end:** two of the bayline launch files are named `*_bayline.py` rather than
`*_bayline.launch.py` (`small_warehouse_static_01_bayline.py`,
`small_warehouse_static_02_bayline.py`). `ros2 launch` accepts either, but the
inconsistency is the kind that makes a later glob miss them.
