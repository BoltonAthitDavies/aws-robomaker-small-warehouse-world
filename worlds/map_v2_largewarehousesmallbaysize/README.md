# map_v2 — large warehouse, small bays

The floor layout as it stood **before** the lanes were halved and the bays grown
(`reshape_floor.py`). Snapshot of: 41.28 m square warehouse, 14 bays at the
original sizes, full-width walkways.

|            | this snapshot (v2) | current (v3) |
|------------|--------------------|--------------|
| lane A / B | 2.584 / 2.510 m    | 1.292 / 1.255 m |
| west bay   | 6.00 x 6.00 m      | 7.94 x 7.24 m |
| stock bay  | 9.03 x 6.00 m      | 14.57 x 7.34 m |
| east bay   | 9.03 x 6.00 m      | 13.89 x 7.66 m |
| bay area   | 36 / 54 m²         | 57 / 107 m²  |

## Why the mesh is in here

A `.world` file does not contain the floor. The bay outlines and walkway lines are
geometry inside `aws_robomaker_warehouse_GroundB_01`'s **visual mesh**, which every
world shares via `model://`. Archiving the worlds alone would leave them pointing at
whatever the shared model happens to be — so `meshes/` carries the matching ground
mesh. `map_v1_tinywarehouseOriginal/` has this same gap and only holds world files.

## Restoring

    cp meshes/aws_robomaker_warehouse_GroundB_01_visual.DAE \
       ../../models/aws_robomaker_warehouse_GroundB_01/meshes/
    cp small_warehouse_dynamic/*.world ../small_warehouse_dynamic/
    cp small_warehouse_static/*.world  ../small_warehouse_static/
    cd ../../.. && python3 bake_map.py --world .../small_warehouse_dynamic.world \
        --out .../maps/baked_dynamic --origin -21.1 -21.1 --size 844 844

`reshuffle_bays.py` reads the bay rectangles from the mesh at import, so it adapts
to whichever floor is installed — no constants to change.

## Rebuilding from scratch instead

    git -C ../.. checkout -- models/          # HEAD is this floor
    python3 reshuffle_bays.py --seed 5 --speed 2.5 --cross 2
