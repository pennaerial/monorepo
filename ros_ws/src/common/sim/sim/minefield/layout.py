"""Mine-field layout schema, random generation and file I/O.

Frame (gz world, ENU, metres): the origin is the centre of the field's start edge and
the field runs along +x. Rows are numbered along x (0 at the start edge) and columns
along y (0 at y = -width / 2). Cell (row, col) has its centre at

    x = (row + 0.5) * cell_size
    y = (col + 0.5) * cell_size - width / 2

A mine is stored as its cell plus an offset (dx, dy) from that centre, so "at most one
mine per cell" holds by construction. The resolved layout file doubles as ground truth.
"""

from __future__ import annotations

import csv
import math
import random
import secrets
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, model_validator
from pydantic import Field as PydanticField

FT = 0.3048
XYR = tuple[float, float, float]


class Field(BaseModel):
    """Field grid. Defaults are the IARC 80 x 300 ft field in 2 ft cells."""

    rows: int = PydanticField(default=150, ge=1)  # along +x (300 ft)
    cols: int = PydanticField(default=40, ge=1)  # along y (80 ft)
    cell_size: float = PydanticField(default=2 * FT, gt=0.0)  # metres

    @property
    def length(self) -> float:
        return self.rows * self.cell_size

    @property
    def width(self) -> float:
        return self.cols * self.cell_size


class Mine(BaseModel):
    row: int = PydanticField(ge=0)
    col: int = PydanticField(ge=0)
    dx: float = 0.0  # offset from the cell centre, metres
    dy: float = 0.0
    yaw: float = 0.0  # radians, about world z
    face: Literal["up", "down"] = "up"


class Layout(BaseModel):
    version: Literal[1] = 1
    field: Field = PydanticField(default_factory=Field)
    seed: int | None = None  # the seed a random layout was generated from
    mines: list[Mine] = PydanticField(default_factory=list)
    obstacles: list[dict] = PydanticField(default_factory=list)  # reserved for iarcFieldObstacles

    @model_validator(mode="after")
    def check_mines(self) -> Layout:
        half = self.field.cell_size / 2
        seen: set[tuple[int, int]] = set()
        for mine in self.mines:
            cell = (mine.row, mine.col)
            if mine.row >= self.field.rows or mine.col >= self.field.cols:
                raise ValueError(
                    f"Mine at cell {cell} is outside the {self.field.rows}x{self.field.cols} field"
                )
            if cell in seen:
                raise ValueError(f"More than one mine in cell {cell}")
            if abs(mine.dx) > half or abs(mine.dy) > half:
                raise ValueError(f"Mine at cell {cell} has an offset outside its cell")
            seen.add(cell)
        return self


class RandomConfig(BaseModel):
    """How to draw a random layout. Give exactly one of `count` or `density`."""

    seed: int | None = None  # None draws a fresh seed, which is recorded in the layout
    count: int | None = PydanticField(default=None, ge=0)  # exact number of mines
    density: float | None = PydanticField(default=None, ge=0.0, le=1.0)  # per-cell probability
    face_down_prob: float = PydanticField(default=0.5, ge=0.0, le=1.0)
    random_yaw: bool = True
    # Gap between the mine centre and the cell edge; 0.07 m keeps the whole 120 mm mine
    # inside its cell at any yaw.
    edge_margin: float = PydanticField(default=0.07, ge=0.0)
    keep_out: list[XYR] = PydanticField(
        default_factory=list
    )  # (x, y, radius), no mines in cells it touches

    @model_validator(mode="after")
    def one_amount(self) -> RandomConfig:
        if (self.count is None) == (self.density is None):
            raise ValueError("Set exactly one of random.count or random.density")
        return self


def cell_center(field: Field, row: int, col: int) -> tuple[float, float]:
    return (row + 0.5) * field.cell_size, (col + 0.5) * field.cell_size - field.width / 2


def mine_xy(field: Field, mine: Mine) -> tuple[float, float]:
    cx, cy = cell_center(field, mine.row, mine.col)
    return cx + mine.dx, cy + mine.dy


def _cell_hits_circle(field: Field, row: int, col: int, circle: XYR) -> bool:
    """True if the cell's square overlaps the (x, y, radius) circle."""
    x, y, r = circle
    cx, cy = cell_center(field, row, col)
    half = field.cell_size / 2
    nearest_x = min(max(x, cx - half), cx + half)
    nearest_y = min(max(y, cy - half), cy + half)
    return (x - nearest_x) ** 2 + (y - nearest_y) ** 2 < r**2


def generate_random(field: Field, config: RandomConfig) -> Layout:
    """Draw a layout: pick cells, then an offset, yaw and face for each mine."""
    seed = config.seed if config.seed is not None else secrets.randbelow(2**31)
    rng = random.Random(seed)

    cells = [
        (row, col)
        for row in range(field.rows)
        for col in range(field.cols)
        if not any(_cell_hits_circle(field, row, col, zone) for zone in config.keep_out)
    ]
    if config.count is not None:
        if config.count > len(cells):
            raise ValueError(f"random.count={config.count} but only {len(cells)} cells are free")
        chosen = sorted(rng.sample(cells, config.count))
    else:
        chosen = [cell for cell in cells if rng.random() < config.density]

    reach = max(field.cell_size / 2 - config.edge_margin, 0.0)
    mines = [
        Mine(
            row=row,
            col=col,
            dx=round(rng.uniform(-reach, reach), 4),
            dy=round(rng.uniform(-reach, reach), 4),
            yaw=round(rng.uniform(-math.pi, math.pi), 4) if config.random_yaw else 0.0,
            face="down" if rng.random() < config.face_down_prob else "up",
        )
        for row, col in chosen
    ]
    return Layout(field=field, seed=seed, mines=mines)


def load_layout(path: str | Path) -> Layout:
    with open(path) as f:
        return Layout.model_validate(yaml.safe_load(f))


def dump_layout(layout: Layout, path: str | Path) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    data = layout.model_dump(mode="json")
    # one flow-style line per mine keeps a 6000-mine file readable
    mines = data.pop("mines")
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
        if not mines:
            f.write("mines: []\n")
            return
        f.write("mines:\n")
        for mine in mines:
            f.write(
                f"  - {yaml.safe_dump(mine, default_flow_style=True, sort_keys=False).strip()}\n"
            )


def write_csv(layout: Layout, path: str | Path) -> None:
    """Ground truth with world coordinates, one row per mine."""
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id", "row", "col", "x", "y", "yaw", "face"])
        for mine in layout.mines:
            x, y = mine_xy(layout.field, mine)
            writer.writerow(
                [
                    mine.row * layout.field.cols + mine.col,
                    mine.row,
                    mine.col,
                    f"{x:.4f}",
                    f"{y:.4f}",
                    f"{mine.yaw:.4f}",
                    mine.face,
                ]
            )
