"""Regression tests; run in the ROS Python environment with Cargo available.

python3 -m unittest discover -s experimental/gcs/test -p test_generate_rs.py
"""

import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

# Resolve the generator relative to this file, independent of the working directory.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import generate_rs


class NamespaceTests(unittest.TestCase):
    def test_shared_package_types(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            idl_root = root / "idl"
            fixtures = {
                "builtin_interfaces/msg/Time.idl": """
                    module builtin_interfaces { module msg {
                        struct Time { int32 sec; uint32 nanosec; };
                    }; };
                """,
                "std_msgs/msg/Header.idl": """
                    #include "builtin_interfaces/msg/Time.idl"
                    module std_msgs { module msg {
                        struct Header { builtin_interfaces::msg::Time stamp; string frame_id; };
                    }; };
                """,
                "std_msgs/msg/Int32.idl": """
                    module std_msgs { module msg { struct Int32 { int32 data; }; }; };
                """,
                "sensor_msgs/msg/Image.idl": """
                    #include "std_msgs/msg/Header.idl"
                    module sensor_msgs { module msg {
                        struct Image {
                            std_msgs::msg::Header header;
                            sequence<std_msgs::msg::Header> history;
                            std_msgs::msg::Header pair[2];
                        };
                    }; };
                """,
            }
            for relative, content in fixtures.items():
                path = idl_root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)

            # Exercise the real CLI from a directory unrelated to the script.
            script_dir = root / "gcs"
            script_dir.mkdir()
            script = script_dir / "generate_rs.py"
            shutil.copyfile(generate_rs.__file__, script)
            shutil.copytree(idl_root, script_dir / "idl")
            elsewhere = root / "elsewhere"
            elsewhere.mkdir()
            subprocess.run(
                [
                    sys.executable,
                    str(script),
                ],
                cwd=elsewhere,
                check=True,
            )
            default_crate = script_dir / "crates/ros_interfaces"
            self.assertTrue((default_crate / "Cargo.toml").is_file())
            self.assertTrue((default_crate / "src/lib.rs").is_file())
            self.assertFalse((default_crate / "rust").exists())
            self.assertFalse((elsewhere / "crates").exists())

            # Empty non-package directories are ignored during discovery.
            (idl_root / "notes").mkdir()
            output = root / "generated"
            generate_rs.main(
                [
                    "--idl-root",
                    str(idl_root),
                    "--output-dir",
                    str(output),
                ]
            )
            crate = output
            source = (crate / "src/lib.rs").read_text()
            self.assertEqual(source, (default_crate / "src/lib.rs").read_text())
            for name in ["Header", "Time", "Image", "Int32"]:
                self.assertEqual(source.count(f"pub struct {name} {{"), 1)
            self.assertIn("pub header: crate::std_msgs::msg::Header", source)
            self.assertIn("Vec<crate::std_msgs::msg::Header>", source)
            self.assertIn("[crate::std_msgs::msg::Header; 2]", source)

            # Package order must not affect the generated crate.
            generate_rs.generate_rs(["std_msgs", "sensor_msgs", "std_msgs"], idl_root, output)
            self.assertEqual(source, (crate / "src/lib.rs").read_text())
            generate_rs.generate_rs(idl_root=idl_root, output_dir=output)
            self.assertEqual(source, (crate / "src/lib.rs").read_text())
            empty = root / "empty"
            empty.mkdir()
            with self.assertRaisesRegex(ValueError, "No ROS interface packages"):
                generate_rs.generate_rs(idl_root=empty, output_dir=output)
            self.assertEqual(source, (crate / "src/lib.rs").read_text())
            tests = crate / "tests"
            tests.mkdir()
            (tests / "shared.rs").write_text("""
                use ros_interfaces::{builtin_interfaces, sensor_msgs, std_msgs};
                #[test]
                fn shared_types() {
                    let header = std_msgs::msg::Header {
                        stamp: builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
                        frame_id: String::new(),
                    };
                    let image = sensor_msgs::msg::Image {
                        header: header.clone(), history: vec![header.clone()],
                        pair: [header.clone(), header.clone()],
                    };
                    let shared: std_msgs::msg::Header = image.header;
                    assert_eq!(shared, header);
                    assert_eq!(std_msgs::msg::Int32 { data: 42 }.data, 42);
                }
            """)
            subprocess.run(
                [
                    "cargo",
                    "test",
                    "--offline",
                    "--manifest-path",
                    str(crate / "Cargo.toml"),
                ],
                check=True,
            )

            # A single selected message still discovers shared dependencies.
            generate_rs.generate_rs(
                "sensor_msgs",
                idl_root,
                output,
                idl_files=["msg/Image.idl", "msg/Image.idl"],
            )
            source = (crate / "src/lib.rs").read_text()
            for name in ["Header", "Time", "Image"]:
                self.assertEqual(source.count(f"pub struct {name} {{"), 1)
            with self.assertRaisesRegex(ValueError, "exactly one input package"):
                generate_rs.generate_rs(
                    ["std_msgs", "sensor_msgs"],
                    idl_root,
                    output,
                    idl_files=["msg/Image.idl"],
                )


if __name__ == "__main__":
    unittest.main()
