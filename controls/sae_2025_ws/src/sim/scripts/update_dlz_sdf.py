#!/usr/bin/env python3
# Regenerate border/hash block: flat on ground (no roll). Pink pose = bottom-left of 8ft×8ft pad.
# Hardcoded 8 ft × 8 ft: half_8ft = 1.2192 m, inner square half = 3 ft = 0.9144 m.
import sys

sys.path.insert(0, "/home/ubuntu/monorepo/controls/sae_2025_ws/src/sim/sim/world_gen/models/dlz")
import os

os.chdir("/home/ubuntu/monorepo/controls/sae_2025_ws/src/sim/sim/world_gen/models/dlz")

# Pink visual pose in model.sdf = bottom-left corner of 8 ft × 8 ft pad
pink_bottom_left_x, pink_bottom_left_y, pz = 3.8, 1.2, 0.11
# 8 ft = 2.4384 m, half = 4 ft = 1.2192 m
half_8ft = 1.2192
# Inner square: 6 ft × 6 ft, half = 3 ft = 0.9144 m
half = 0.9144
eps_y = 0.002
step = 0.0508  # 2 in
start = round(-half + step, 4)

# Pad center from bottom-left
px = pink_bottom_left_x + half_8ft
py = pink_bottom_left_y - half_8ft


def to_link(lx, ly):
    return (round(px + lx, 4), round(py + ly, 4), round(pz + eps_y, 4))


borders = [
    ("inner_border_top", (0, half), (1.8288, 0.02, 0.001)),
    ("inner_border_bottom", (0, -half), (1.8288, 0.02, 0.001)),
    ("inner_border_left", (-half, 0), (0.02, 1.8288, 0.001)),
    ("inner_border_right", (half, 0), (0.02, 1.8288, 0.001)),
]
hash_len, hash_wid, hash_h = 0.0508, 0.127, 0.001
colors = [(0.0, 0.6, 0.0, 1), (0.0, 0.0, 0.8, 1), (1.0, 0.5, 0.0, 1)]
rot = "0 0 0"

lines = []
for name, (lx, ly), size in borders:
    x, y, z = to_link(lx, ly)
    sx, sy, sz = size
    lines.append(f'      <visual name="{name}">')
    lines.append("        <cast_shadows>false</cast_shadows>")
    lines.append(f"        <pose>{x} {y} {z} {rot}</pose>")
    lines.append(f"        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>")
    lines.append(
        "        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>"
    )
    lines.append("      </visual>")

for side, lx_val, ly_val, size_x, size_y in [
    ("top", None, half, hash_len, hash_wid),
    ("bottom", None, -half, hash_len, hash_wid),
    ("left", -half, None, hash_wid, hash_len),
    ("right", half, None, hash_wid, hash_len),
]:
    for i in range(35):
        if lx_val is not None:
            lx, ly = lx_val, start + i * step
        else:
            lx, ly = start + i * step, ly_val
        x, y, z = to_link(lx, ly)
        c = colors[i % 3]
        crgb = f"{c[0]} {c[1]} {c[2]} {c[3]}"
        lines.append(f'      <visual name="hash_{side}_{i:02d}">')
        lines.append("        <cast_shadows>false</cast_shadows>")
        lines.append(f"        <pose>{x} {y} {z} {rot}</pose>")
        lines.append(
            f"        <geometry><box><size>{size_x} {size_y} {hash_h}</size></box></geometry>"
        )
        lines.append(
            f"        <material><ambient>{crgb}</ambient><diffuse>{crgb}</diffuse></material>"
        )
        lines.append("      </visual>")

new_block = (
    "      <!-- Inner square: 6 ft × 6 ft inside 8 ft × 8 ft pad. Flat on ground, rot=0 0 0. -->\n"
    + "\n".join(lines)
)

with open("model.sdf") as f:
    content = f.read()
i0 = content.find("      <!-- Inner square:")
i1 = content.find("    </link>")
if i0 == -1 or i1 == -1:
    raise SystemExit("Markers not found")
new_content = content[:i0] + new_block + "\n    " + content[i1:]
with open("model.sdf", "w") as f:
    f.write(new_content)
print("SDF updated: 8ft×8ft pad, 6ft×6ft inner, flat on ground, rot=0 0 0")
