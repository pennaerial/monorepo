"""Build the PFM1 model meshes from the IARC dataset generator's mine.stl.

The source STL is in millimetres with an off-centre origin. This writes metre-scale
copies whose footprint (bounding box XY) is centred on the origin and whose flat
underside sits at z = 0, plus a decimated copy for fields with thousands of mines.

    pip install trimesh fast_simplification
    python3 gz-models/tools/prepare_pfm1_mesh.py path/to/mine.stl
"""

import argparse
from pathlib import Path

import trimesh

MESH_DIR = Path(__file__).resolve().parents[1] / "models" / "PFM1" / "meshes"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("source", type=Path, help="mine.stl (millimetres)")
    parser.add_argument("--low-faces", type=int, default=800, help="decimated triangle target")
    args = parser.parse_args()

    mesh = trimesh.load_mesh(args.source)
    mesh.apply_scale(0.001)
    (x_min, y_min, z_min), (x_max, y_max, _) = mesh.bounds
    mesh.apply_translation((-(x_min + x_max) / 2, -(y_min + y_max) / 2, -z_min))

    # Decimation shifts the outline by a few mm (and adds ~2 mm of height), so fit the
    # low mesh back onto the full mesh's bounding box to keep both true to size.
    low = mesh.simplify_quadric_decimation(face_count=args.low_faces)
    low.apply_translation(-low.bounds[0])
    low.apply_scale(mesh.extents / low.extents)
    low.apply_translation(mesh.bounds[0])

    MESH_DIR.mkdir(parents=True, exist_ok=True)
    mesh.export(MESH_DIR / "pfm1.stl")
    low.export(MESH_DIR / "pfm1_low.stl")
    for name, m in (("pfm1.stl", mesh), ("pfm1_low.stl", low)):
        print(f"{name}: {len(m.faces)} faces, bounds {m.bounds.round(4).tolist()} m")


if __name__ == "__main__":
    main()
