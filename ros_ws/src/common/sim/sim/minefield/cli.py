"""iarc_field_gen: write an iarcField mine layout (and optionally CSV / SDF) without ROS.

iarc_field_gen --count 300 --seed 7 -o layout.yaml --csv truth.csv
iarc_field_gen --layout layout.yaml --sdf field.sdf   # validate and render a layout
"""

from __future__ import annotations

import argparse
from pathlib import Path

from sim.minefield.layout import (
    Field,
    RandomConfig,
    dump_layout,
    generate_random,
    load_layout,
    write_csv,
)
from sim.minefield.sdf import FieldStyle, Markings, field_model_sdf


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate or check an iarcField mine layout.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--layout", type=Path, help="existing layout YAML to validate/export")
    source.add_argument("--count", type=int, help="random layout with exactly this many mines")
    source.add_argument("--density", type=float, help="random layout, per-cell mine probability")
    parser.add_argument("--seed", type=int, help="random seed (default: fresh, recorded in output)")
    parser.add_argument("--face-down-prob", type=float, default=0.5)
    parser.add_argument("--no-yaw", action="store_true", help="all mines at yaw 0")
    parser.add_argument(
        "--keep-out",
        type=float,
        nargs=3,
        action="append",
        metavar=("X", "Y", "R"),
        help="no mines in cells touching this circle (default: 0 0 1.5, the UAV spawn)",
    )
    parser.add_argument("-o", "--output", type=Path, help="write the resolved layout YAML here")
    parser.add_argument("--csv", type=Path, help="write ground truth CSV here")
    parser.add_argument("--sdf", type=Path, help="write the merged field model SDF here")
    parser.add_argument("--mesh", choices=["low", "full"], default="low")
    parser.add_argument("--grid", action="store_true", help="draw 2 ft cell lines in the SDF")
    args = parser.parse_args(argv)

    if args.layout:
        layout = load_layout(args.layout)
    else:
        config = RandomConfig(
            seed=args.seed,
            count=args.count,
            density=args.density,
            face_down_prob=args.face_down_prob,
            random_yaw=not args.no_yaw,
            keep_out=args.keep_out or [(0.0, 0.0, 1.5)],
        )
        layout = generate_random(Field(), config)

    if args.output:
        dump_layout(layout, args.output)
    if args.csv:
        write_csv(layout, args.csv)
    if args.sdf:
        style = FieldStyle(mesh=args.mesh, markings=Markings(grid=args.grid))
        args.sdf.write_text(field_model_sdf(layout, style))

    down = sum(m.face == "down" for m in layout.mines)
    print(f"{len(layout.mines)} mines ({down} face down), seed={layout.seed}")


if __name__ == "__main__":
    main()
