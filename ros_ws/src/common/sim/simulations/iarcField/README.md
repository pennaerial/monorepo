# iarcField

IARC mine field: an 80 x 300 ft (24.384 x 91.44 m) field split into 2 ft cells
(40 columns x 150 rows), with at most one PFM-1 mine per cell, lying flat face up or
face down.

- World: `gz-models/worlds/iarcField.sdf` (ground, sun, GPS origin)
- Mine model: `gz-models/models/PFM1` (meshes built by `gz-models/tools/prepare_pfm1_mesh.py`)
- Node: `sim/world_gen/iarc_field_node.py`, which uses the ROS-free core in `sim/minefield/`

## Frame

gz world, ENU, metres. The origin (UAV spawn) is the centre of the field's start edge,
and the field runs along +x. Rows are numbered along x (row 0 at the start edge), and
columns along y (col 0 at y = -12.192). Cell `(row, col)` is centred at
`x = (row + 0.5) * 0.6096`, `y = (col + 0.5) * 0.6096 - 12.192`.

## Stages

- `base.yaml`: random layout of 300 mines (`count` or `density`, seed, face-down
  probability, keep-out around the spawn).
- `fixed.yaml`: loads `layouts/example.yaml`.

```bash
ros2 launch uav uav_sitl.launch.py world:=iarcField           # base stage
ros2 launch sim sim2.launch.py world:=iarcField stage:=fixed  # uav_sitl does not forward stage yet
```

Every run writes the resolved layout (the ground truth) as YAML and CSV to
`~/.ros/iarcField/<stage>_<utc time>_seed<seed>.{yaml,csv}`. To rerun the same field,
pass that YAML back as `layout:`, or set `random.seed`.

## Layout files

```yaml
version: 1
field: {rows: 150, cols: 40, cell_size: 0.6096}
seed: 1                  # set when the layout was generated randomly
obstacles: []            # reserved for iarcFieldObstacles
mines:
  - {row: 0, col: 23, dx: 0.1238, dy: 0.2062, yaw: 0.3321, face: down}
```

`dx`/`dy` are metres from the cell centre (|dx|, |dy| <= 0.3048), and `yaw` is in
radians. Loading rejects a mine outside the field, outside its cell, or sharing a cell.

Make or check layouts without ROS:

```bash
iarc_field_gen --count 300 --seed 7 -o my_layout.yaml --csv truth.csv
iarc_field_gen --layout my_layout.yaml --sdf field.sdf --grid
```

All mines are drawn as visuals of one static, collision-free model (`iarc_field`) and
spawned in a single request, because spawning thousands of separate models through the
bridge is too slow. The 800-triangle `low` mesh is the default; both meshes are
120 x 66 x 21.2 mm. Mine shadows (`style.cast_shadows`) and the cell grid
(`style.markings.grid`) are off by default.

When the field is in, the node publishes the ground-truth YAML path on the latched topic
`/iarc_field/ready` (`std_msgs/String`); wait on it before recording camera data.
