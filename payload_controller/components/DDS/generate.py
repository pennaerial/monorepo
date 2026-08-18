#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import subprocess


PC_PATH = os.environ["PENNAIR_PAYLOAD_CONTROLLER_PATH"]
XRCE_GEN_PATH = os.environ["PENNAIR_XRCE_DDS_GEN_PATH"]

script_dir = Path(__file__).parent
xrce_gen_exe = Path(XRCE_GEN_PATH) / "scripts" / "microxrceddsgen"


def main(args) -> None:

    idl_path = args.idl_path.resolve()

    for idl_file in args.idl_files:
        idl_file = idl_file.resolve()

        relative_path = idl_file.relative_to(idl_path)
        print(f"Generating IDL: {idl_file.name}")
        dst_dir = args.output_root.resolve() / relative_path.parent
        print(f"dst dir: {dst_dir}")

        dst_dir.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            [
                str(xrce_gen_exe),
                "-I",
                str(args.idl_path),
                "-d",
                str(dst_dir),
                str(idl_file),
            ],
            check=True,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate Micro XRCE-DDS interfaces from IDL files."
    )
    parser.add_argument(
        "--idl-path",
        type=Path,
        required=True,
        help="Path to the directory containing IDL files.",
    )
    parser.add_argument(
        "--idl-files",
        type=Path,
        nargs="+",
        required=True,
        help="List of IDL files to generate.",
    )
    parser.add_argument(
        "-o",
        "--output-root",
        type=Path,
        default=script_dir,
        help="Path to the output directory.",
    )

    args = parser.parse_args()
    main(args)
