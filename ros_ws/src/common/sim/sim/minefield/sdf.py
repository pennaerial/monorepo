"""Turn a layout into one static SDF model: field markings plus one visual per mine.

Spawning thousands of mines as separate models through the ros_gz create bridge takes
minutes, so the whole field is a single model sent in one request.
"""

from __future__ import annotations

import math

from pydantic import BaseModel
from pydantic import Field as PydanticField

from sim.minefield.layout import Layout, mine_xy

RGBA = tuple[float, float, float, float]

FIELD_MODEL = "iarc_field"
MESH_URIS = {"low": "model://PFM1/meshes/pfm1_low.stl", "full": "model://PFM1/meshes/pfm1.stl"}
# Both meshes are fitted to 120 x 66 x 21.2 mm; a face-down mine is rolled by pi and
# lifted by its height so it rests on its top.
MINE_HEIGHT = 0.0212


class Material(BaseModel):
    """SDF <material> colours, each RGBA in 0-1."""

    ambient: RGBA
    diffuse: RGBA
    specular: RGBA = (0.05, 0.05, 0.05, 1.0)
    emissive: RGBA = (0.0, 0.0, 0.0, 1.0)


OLIVE_DRAB = Material(ambient=(0.149, 0.384, 0.275, 1.0), diffuse=(0.149, 0.384, 0.275, 1.0))
WHITE_LINE = Material(ambient=(0.9, 0.9, 0.9, 1.0), diffuse=(0.9, 0.9, 0.9, 1.0))


class Markings(BaseModel):
    """Painted lines on the field. The grid is off by default: the real field has none."""

    border: bool = True
    grid: bool = False
    line_width: float = PydanticField(default=0.05, gt=0.0)
    height: float = PydanticField(default=0.004, gt=0.0)
    material: Material = PydanticField(default_factory=lambda: WHITE_LINE.model_copy())


class FieldStyle(BaseModel):
    mesh: str = "low"  # key of MESH_URIS
    material: Material = PydanticField(default_factory=lambda: OLIVE_DRAB.model_copy())
    cast_shadows: bool = False  # thousands of 2 cm shadow casters cost more than they show
    markings: Markings = PydanticField(default_factory=Markings)


def _material_xml(material: Material) -> str:
    inner = "".join(
        f"<{tag}>{' '.join(f'{c:g}' for c in rgba)}</{tag}>"
        for tag, rgba in material.model_dump().items()
    )
    return f"<material>{inner}</material>"


def _strip(
    name: str, center: tuple[float, float], size: tuple[float, float], height: float, material: str
) -> str:
    return (
        f'<visual name="{name}">'
        f"<pose>{center[0]:g} {center[1]:g} {height / 2:g} 0 0 0</pose>"
        "<cast_shadows>false</cast_shadows>"
        f"<geometry><box><size>{size[0]:g} {size[1]:g} {height:g}</size></box></geometry>"
        f"{material}</visual>"
    )


def _markings_xml(layout: Layout, markings: Markings) -> list[str]:
    field = layout.field
    length, width, t, h = field.length, field.width, markings.line_width, markings.height
    material = _material_xml(markings.material)
    # each list entry: (name, x of the line, True if it runs along y)
    lines: list[tuple[str, float, bool]] = []
    if markings.grid:
        lines += [(f"grid_x_{i}", i * field.cell_size, True) for i in range(1, field.rows)]
        lines += [
            (f"grid_y_{j}", j * field.cell_size - width / 2, False) for j in range(1, field.cols)
        ]
    if markings.border:
        lines += [("border_start", 0.0, True), ("border_end", length, True)]
        lines += [("border_right", -width / 2, False), ("border_left", width / 2, False)]

    visuals = []
    for name, offset, along_y in lines:
        if along_y:
            visuals.append(_strip(name, (offset, 0.0), (t, width + t), h, material))
        else:
            visuals.append(_strip(name, (length / 2, offset), (length + t, t), h, material))
    return visuals


def _mine_xml(layout: Layout, style: FieldStyle, material: str) -> list[str]:
    uri = MESH_URIS[style.mesh]
    shadows = str(style.cast_shadows).lower()
    visuals = []
    for mine in layout.mines:
        x, y = mine_xy(layout.field, mine)
        down = mine.face == "down"
        z = MINE_HEIGHT if down else 0.0
        roll = math.pi if down else 0.0
        visuals.append(
            f'<visual name="mine_r{mine.row}_c{mine.col}">'
            f"<pose>{x:.4f} {y:.4f} {z:g} {roll:.6f} 0 {mine.yaw:.4f}</pose>"
            f"<cast_shadows>{shadows}</cast_shadows>"
            f"<geometry><mesh><uri>{uri}</uri></mesh></geometry>"
            f"{material}</visual>"
        )
    return visuals


def field_model_sdf(layout: Layout, style: FieldStyle | None = None) -> str:
    """One static, collision-free model holding the markings and every mine."""
    style = style or FieldStyle()
    if style.mesh not in MESH_URIS:
        raise ValueError(f"Unknown mine mesh '{style.mesh}'; use one of {sorted(MESH_URIS)}")
    visuals = _markings_xml(layout, style.markings) + _mine_xml(
        layout, style, _material_xml(style.material)
    )
    return (
        '<?xml version="1.0"?>'
        '<sdf version="1.9">'
        f'<model name="{FIELD_MODEL}">'
        "<static>true</static>"
        f'<link name="link">{"".join(visuals)}</link>'
        "</model>"
        "</sdf>"
    )
