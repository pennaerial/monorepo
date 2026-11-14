"""Regression tests; run in the ROS Python environment with Cargo available.

python3 -m unittest discover -s gcs/tests -p test_generate_rs.py
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
                use ros_interfaces::{builtin_interfaces, sensor_msgs, std_msgs, RosMessage};
                #[test]
                fn shared_types() {
                    assert_eq!(std_msgs::msg::Int32::TYPE_NAME, "std_msgs/msg/Int32");
                    assert_eq!(builtin_interfaces::msg::Time::TYPE_NAME, "builtin_interfaces/msg/Time");
                    let header = std_msgs::msg::Header {
                        stamp: builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
                        frame_id: String::new(),
                    };
                    let image = sensor_msgs::msg::Image {
                        header: header.clone(), history: vec![header.clone()],
                        pair: [header.clone(), header.clone()],
                    };
                    fn decode<T: RosMessage>(bytes: &[u8]) -> Result<T, cdr::Error> {
                        T::deserialize(bytes)
                    }
                    let bytes = image.serialize().unwrap();
                    assert_eq!(decode::<sensor_msgs::msg::Image>(&bytes).unwrap(), image);
                    let bytes = cdr::serialize::<_, _, cdr::CdrBe>(&image, cdr::Infinite).unwrap();
                    assert_eq!(decode::<sensor_msgs::msg::Image>(&bytes).unwrap(), image);
                    assert!(decode::<std_msgs::msg::Int32>(&[]).is_err());
                    assert!(decode::<sensor_msgs::msg::Image>(&bytes[..4]).is_err());
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


class ServiceTests(unittest.TestCase):
    def test_service_generates_request_response_and_marker(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            idl_root = root / "idl"
            fixtures = {
                "builtin_interfaces/msg/Time.idl": """
                    module builtin_interfaces { module msg {
                        struct Time { int32 sec; uint32 nanosec; };
                    }; };
                """,
                "std_srvs/srv/Trigger.idl": """
                    module std_srvs { module srv {
                        struct Trigger_Request { uint8 structure_needs_at_least_one_member; };
                        struct Trigger_Response { boolean success; string message; };
                    }; };
                """,
                "demo_srvs/srv/Stamp.idl": """
                    #include "builtin_interfaces/msg/Time.idl"
                    module demo_srvs { module srv {
                        struct Stamp_Request { int32 id; };
                        struct Stamp_Response { builtin_interfaces::msg::Time stamp; };
                    }; };
                """,
            }
            for relative, content in fixtures.items():
                path = idl_root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)

            output = root / "generated"
            generate_rs.generate_rs(idl_root=idl_root, output_dir=output)
            source = (output / "src/lib.rs").read_text()
            for name in ["Trigger", "Stamp"]:
                self.assertEqual(source.count(f"pub struct {name};"), 1)
                self.assertEqual(source.count(f"pub struct {name}_Request {{"), 1)
                self.assertEqual(source.count(f"pub struct {name}_Response {{"), 1)
            self.assertIn('const TYPE_NAME: &\'static str = "std_srvs/srv/Trigger";', source)
            # Service dependencies resolve like message dependencies.
            self.assertIn("pub stamp: crate::builtin_interfaces::msg::Time", source)

            tests = output / "tests"
            tests.mkdir()
            (tests / "services.rs").write_text("""
                use ros_interfaces::{std_srvs::srv, RosMessage, RosService};
                fn round_trip<M: RosMessage>(message: &M) -> M {
                    let bytes = message.serialize().unwrap();
                    M::deserialize(&bytes).unwrap()
                }
                fn type_name<S: RosService>() -> &'static str { S::TYPE_NAME }
                fn service_round_trip<S: RosService>(request: &S::Request, response: &S::Response) {
                    round_trip(request);
                    round_trip(response);
                }
                #[test]
                fn trigger() {
                    let response = srv::Trigger_Response { success: true, message: "ok".into() };
                    assert_eq!(round_trip(&response), response);
                    let request = srv::Trigger_Request { structure_needs_at_least_one_member: 0 };
                    assert_eq!(round_trip(&request), request);
                    service_round_trip::<srv::Trigger>(&request, &response);
                    assert_eq!(type_name::<srv::Trigger>(), "std_srvs/srv/Trigger");
                    assert_eq!(srv::Trigger_Request::TYPE_NAME, "std_srvs/srv/Trigger_Request");
                    assert_eq!(srv::Trigger_Response::TYPE_NAME, "std_srvs/srv/Trigger_Response");
                }
            """)
            subprocess.run(
                ["cargo", "test", "--offline", "--manifest-path", str(output / "Cargo.toml")],
                check=True,
            )


if __name__ == "__main__":
    unittest.main()
