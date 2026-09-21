# ROS interface definitions

Keep IDLs under `<package>/<namespace>/<Type>.idl` so that generated Rust
modules preserve the ROS namespaces.

`std_msgs/msg/Int32.idl` is copied from ROS Jazzy std_msgs 5.3.8 (Apache-2.0).
It contains one `int32 data` field and has no interface dependencies.

From the GCS directory, run:

```bash
python3 generate_rs.py
```

The default input (`idl/`) and output (`crates/ros_interfaces/`) paths are
relative to the script, so it can also be invoked from another directory.
Generation requires the ROS Python parser environment.

By default, all immediate package directories containing `.idl` files are
combined into one crate, each in its own package module. Empty directories
are ignored. Pass package names to generate a subset, or `--idl-root PATH`
to use another interface tree. Referenced types are shared across modules.
