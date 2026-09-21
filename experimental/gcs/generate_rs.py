# Copyright 2016-2017 Esteve Fernandez <esteve@apache.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ruff: noqa

import argparse
import os
import pathlib
import shutil

from pathlib import Path

DEFAULT_IDL_ROOT = Path(__file__).resolve().parent / "idl"
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "crates" / "ros_interfaces"

if os.environ["ROS_DISTRO"] <= "humble":
    import rosidl_cmake as rosidl_pycommon
else:
    import rosidl_pycommon

from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import Action
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import BoundedString
from rosidl_parser.definition import BoundedWString
from rosidl_parser.definition import IdlContent
from rosidl_parser.definition import IdlLocator
from rosidl_parser.definition import Message
from rosidl_parser.definition import NamedType
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import Service
from rosidl_parser.definition import UnboundedSequence
from rosidl_parser.definition import UnboundedString
from rosidl_parser.definition import UnboundedWString

from rosidl_parser.parser import parse_idl_file

import sys

# Workaround for RHEL 8 which ships Python 3.6 that lacks str.removesuffix()
# (added in Python 3.9, PEP 616). On Python 3.9+, the native implementation
# is used.
# TODO(esteve): Remove this workaround when RHEL 8 is no longer supported.
if sys.version_info >= (3, 9):

    def _removesuffix(s, suffix):
        return s.removesuffix(suffix)
else:

    def _removesuffix(s, suffix):
        return s[: -len(suffix)] if suffix and s.endswith(suffix) else s


package_name = ""


# Taken from http://stackoverflow.com/a/6425628
def convert_lower_case_underscore_to_camel_case(word):
    return "".join(x.capitalize() or "_" for x in word.split("_"))


def _namespace_from_namespaced_type(namespaced_type):
    namespaces = list(namespaced_type.namespaces)
    if not namespaces or namespaces[0] != package_name:
        raise ValueError(
            f"Expected namespace to start with package '{package_name}', got {namespaces}"
        )
    if len(namespaces) != 2:
        raise ValueError(
            f"Expected exactly one namespace component after package '{package_name}', got {namespaces}"
        )
    namespace = namespaces[1]
    if not namespace.isidentifier() or get_rs_name(namespace) != namespace:
        raise ValueError(f"Namespace '{namespace}' cannot be emitted as a Rust module name")
    return namespace


def _namespace_from_message(message):
    return _namespace_from_namespaced_type(message.structure.namespaced_type)


def _namespace_from_service(service):
    return _namespace_from_namespaced_type(service.namespaced_type)


def _namespace_from_action(action):
    return _namespace_from_namespaced_type(action.namespaced_type)


def _group_specs_by_namespace(specs, namespace_getter):
    grouped = {}
    for spec in specs:
        namespace = namespace_getter(spec)
        grouped.setdefault(namespace, []).append(spec)
    return grouped


def _validate_single_kind_per_namespace(namespace_to_kinds):
    for namespace, kinds in namespace_to_kinds.items():
        direct_kinds = {kind for kind, _ in kinds}
        if len(direct_kinds) > 1:
            raise ValueError(
                f"Namespace '{namespace}' contains multiple top-level kinds: {sorted(direct_kinds)}"
            )


def _expand_namespace_templates(
    template_dir, output_dir, namespace, spec_kind, specs, latest_target_timestamp, data
):
    if not specs:
        return

    if spec_kind == "msg":
        mappings = {
            os.path.join(template_dir, "msg.rs.em"): [f"rust/src/{namespace}.rs"],
            os.path.join(template_dir, "msg/rmw.rs.em"): [f"rust/src/{namespace}/rmw.rs"],
        }
        template_specs = "msg_specs"
    elif spec_kind == "srv":
        mappings = {
            os.path.join(template_dir, "srv.rs.em"): [f"rust/src/{namespace}.rs"],
            os.path.join(template_dir, "srv/rmw.rs.em"): [f"rust/src/{namespace}/rmw.rs"],
        }
        template_specs = "srv_specs"
    elif spec_kind == "action":
        mappings = {
            os.path.join(template_dir, "action.rs.em"): [f"rust/src/{namespace}.rs"],
            os.path.join(template_dir, "action/rmw.rs.em"): [f"rust/src/{namespace}/rmw.rs"],
        }
        template_specs = "action_specs"
    else:
        raise ValueError(f"Unknown spec kind {spec_kind}")

    namespace_data = data.copy()
    namespace_data[template_specs] = [(namespace, spec) for spec in specs]

    for template_file, generated_filenames in mappings.items():
        for generated_filename in generated_filenames:
            generated_file = os.path.join(output_dir, generated_filename)
            rosidl_pycommon.expand_template(
                os.path.join(template_dir, template_file),
                namespace_data.copy(),
                generated_file,
                minimum_timestamp=latest_target_timestamp,
            )


def _idl_locator(idl_tuple, package, idl_root=None):
    idl_parts = idl_tuple.rsplit(":", 1)
    if len(idl_parts) != 2:
        raise ValueError(
            f"IDL tuple must have the form '<base path>:<relative path>', got {idl_tuple!r}"
        )

    if idl_root is None:
        return IdlLocator(*idl_parts), pathlib.Path(idl_parts[1])

    root = pathlib.Path(idl_root).expanduser().resolve()
    if not root.is_dir():
        raise ValueError(f"IDL root is not a directory: {root}")

    # An IDL root is the directory containing package directories, e.g.
    # <root>/std_msgs, <root>/geometry_msgs, etc. Keeping the package name in
    # the relative path also lets rosidl_parser resolve cross-package includes
    # such as "builtin_interfaces/msg/Time.idl" from the same root.
    idl_rel_path = pathlib.Path(idl_parts[1])
    rooted_rel_path = pathlib.Path(package) / idl_rel_path
    idl_path = root / rooted_rel_path
    if not idl_path.is_file():
        raise ValueError(
            f"IDL file not found under IDL root: {idl_path} "
            f"(expected layout: <idl root>/{package}/{idl_rel_path})"
        )

    return IdlLocator(str(root), str(rooted_rel_path)), idl_rel_path


def _raw_rs_type(type_):
    basic_types = {
        "boolean": "bool",
        "byte": "u8",
        "octet": "u8",
        "char": "u8",
        "wchar": "u16",
        "float": "f32",
        "double": "f64",
        "int8": "i8",
        "uint8": "u8",
        "int16": "i16",
        "uint16": "u16",
        "int32": "i32",
        "uint32": "u32",
        "int64": "i64",
        "uint64": "u64",
    }
    if isinstance(type_, BasicType):
        return basic_types[type_.typename]
    if isinstance(type_, AbstractGenericString):
        return "std::string::String"
    if isinstance(type_, Array):
        return f"[{_raw_rs_type(type_.value_type)}; {type_.size}]"
    if isinstance(type_, AbstractSequence):
        return f"Vec<{_raw_rs_type(type_.value_type)}>"
    if isinstance(type_, NamespacedType):
        parts = list(type_.namespaces) + [type_.name]
        return "crate::" + "::".join(get_rs_name(part) for part in parts)
    if isinstance(type_, NamedType):
        return get_rs_name(type_.name)
    raise ValueError(f"Unsupported IDL type for raw Rust output: {type_}")


def _namespaced_types(type_):
    if isinstance(type_, NamespacedType):
        yield type_
    elif isinstance(type_, AbstractNestedType):
        yield from _namespaced_types(type_.value_type)


def _load_referenced_messages(messages, idl_root):
    result = list(messages)
    known = {tuple(message.structure.namespaced_type.namespaced_name()) for message in result}
    index = 0
    while index < len(result):
        message = result[index]
        index += 1
        for member in message.structure.members:
            for named_type in _namespaced_types(member.type):
                key = tuple(named_type.namespaced_name())
                if key in known:
                    continue
                idl_path = pathlib.Path(*key[:-1]) / f"{key[-1]}.idl"
                full_path = idl_root / idl_path
                if not full_path.is_file():
                    raise ValueError(
                        f"IDL for referenced type {'::'.join(key)} not found: {full_path}"
                    )
                parsed = parse_idl_file(IdlLocator(str(idl_root), str(idl_path)))
                for dependency in parsed.content.get_elements_of_type(Message):
                    dependency_key = tuple(dependency.structure.namespaced_type.namespaced_name())
                    if dependency_key not in known:
                        known.add(dependency_key)
                        result.append(dependency)
    return result


def _render_raw_struct(message):
    structure = message.structure
    name = get_rs_name(structure.namespaced_type.name)
    lines = [
        "#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]",
        f"pub struct {name} {{",
    ]
    for member in structure.members:
        lines.append(
            f"    pub {get_rs_name(member.name)}: {_raw_rs_type(member.type)},"
        )
    lines.append("}")
    if message.constants:
        lines.extend(["", f"impl {name} {{"])
        for constant in message.constants:
            constant_type = _raw_rs_type(constant.type)
            if isinstance(constant.type, AbstractGenericString):
                constant_type = "&'static str"
            lines.append(
                f"    pub const {get_rs_name(constant.name)}: {constant_type} = "
                f"{constant_value_to_rs(constant.type, constant.value)};"
            )
        lines.append("}")
    return "\n".join(lines)


def _write_raw_crate(messages, crate_name, output_dir, package_version):
    grouped = {}
    for message in messages:
        namespaced_type = message.structure.namespaced_type
        package, namespace = namespaced_type.namespaces
        grouped.setdefault(package, {}).setdefault(namespace, []).append(message)

    source = ["// Generated raw Rust structs. No ROS runtime is required.", ""]
    for package in sorted(grouped):
        source.extend([f"pub mod {get_rs_name(package)} {{", ""])
        indent = "    "
        for namespace in sorted(grouped[package]):
            source.append(f"{indent}pub mod {get_rs_name(namespace)} {{")
            for message in sorted(
                grouped[package][namespace], key=lambda item: item.structure.namespaced_type.name
            ):
                rendered = _render_raw_struct(message)
                source.extend(
                    indent + "    " + line if line else "" for line in rendered.splitlines()
                )
                source.append("")
            source.append(f"{indent}}}")
            source.append("")
        source.extend(["}", ""])

    crate_dir = pathlib.Path(output_dir)
    source_dir = crate_dir / "src"
    if source_dir.exists():
        shutil.rmtree(source_dir)
    build_script = crate_dir / "build.rs"
    if build_script.exists():
        build_script.unlink()
    source_dir.mkdir(parents=True, exist_ok=True)
    (crate_dir / "src/lib.rs").write_text("\n".join(source), encoding="utf-8")
    cargo_toml = (
        "[package]\n"
        f'name = "{crate_name}"\n'
        f'version = "{package_version}"\n'
        'edition = "2021"\n\n'
        "[dependencies]\n"
        'serde = { version = "1", features = ["derive"] }\n'
    )
    (crate_dir / "Cargo.toml").write_text(cargo_toml, encoding="utf-8")


def generate_rs(
    package=None, idl_root=DEFAULT_IDL_ROOT, output_dir=DEFAULT_OUTPUT_DIR, package_version="0.0.0", idl_files=None,
    crate_name="ros_interfaces",
):
    """Generate one Serde-based crate from one or more ROS IDL packages.

    All types retain their package/namespace modules, including transitively
    referenced types. Omit package to discover all package directories with
    IDLs under idl_root. Explicit idl_files apply to a single input package.
    """
    idl_root = pathlib.Path(idl_root).expanduser().resolve()
    if not idl_root.is_dir():
        raise ValueError(f"IDL root is not a directory: {idl_root}")

    packages = [package] if isinstance(package, str) else list(package or [])
    if not packages:
        packages = [
            path.name for path in idl_root.iterdir()
            if path.is_dir() and any(path.rglob("*.idl"))
        ]
    packages = sorted(set(packages))
    if not packages:
        raise ValueError(f"No ROS interface packages containing IDL files found under {idl_root}")
    if idl_files is not None and len(packages) != 1:
        raise ValueError("--idl requires exactly one input package")
    if (not crate_name or not crate_name.isascii()
            or not crate_name.replace("-", "_").isidentifier()):
        raise ValueError(f"Invalid Rust crate name: {crate_name!r}")

    messages = {}
    for package in packages:
        if not package.isascii() or not package.isidentifier():
            raise ValueError(f"Invalid ROS package name: {package!r}")
        package_root = idl_root / package
        if not package_root.is_dir():
            raise ValueError(f"IDL package directory not found: {package_root}")
        paths = (
            sorted(path.relative_to(package_root) for path in package_root.rglob("*.idl"))
            if idl_files is None else [pathlib.Path(path) for path in idl_files]
        )
        if not paths:
            raise ValueError(f"No IDL files found for package {package!r}")
        for path in sorted(set(paths)):
            if path.is_absolute() or ".." in path.parts:
                raise ValueError(f"IDL path must be relative to {package_root}: {path}")
            locator, _ = _idl_locator(f"{package_root}:{path}", package, idl_root)
            parsed = parse_idl_file(locator)
            for message in parsed.content.get_elements_of_type(Message):
                key = tuple(message.structure.namespaced_type.namespaced_name())
                messages[key] = message

    messages = _load_referenced_messages(messages.values(), idl_root)
    _write_raw_crate(messages, crate_name, output_dir, package_version)
    return 0


def get_rs_name(name):
    keywords = [
        # strict keywords
        "as",
        "break",
        "const",
        "continue",
        "crate",
        "else",
        "enum",
        "extern",
        "false",
        "fn",
        "for",
        "if",
        "for",
        "impl",
        "in",
        "let",
        "loop",
        "match",
        "mod",
        "move",
        "mut",
        "pub",
        "ref",
        "return",
        "self",
        "Self",
        "static",
        "struct",
        "super",
        "trait",
        "true",
        "type",
        "unsafe",
        "use",
        "where",
        "while",
        # Edition 2024+
        "gen",
        # Edition 2018+
        "async",
        "await",
        "dyn",
        # Reserved
        "abstract",
        "become",
        "box",
        "do",
        "final",
        "macro",
        "override",
        "priv",
        "typeof",
        "unsized",
        "virtual",
        "yield",
        "try",
    ]
    # If the field name is a reserved keyword in Rust append an underscore
    return name if not name in keywords else name + "_"


def escape_string(s):
    s = s.replace("\\", "\\\\")
    s = s.replace("'", "\\'")
    return s


def value_to_rs(type_, value):
    assert type_.is_primitive_type()
    assert value is not None

    if not type_.is_array:
        return primitive_value_to_rs(type_, value)

    rs_values = []
    for single_value in value:
        rs_value = primitive_value_to_rs(type_, single_value)
        rs_values.append(rs_value)
    return "{%s}" % ", ".join(rs_values)


def primitive_value_to_rs(type_, value):
    assert type_.is_primitive_type()
    assert value is not None

    if type_.type == "bool":
        return "true" if value else "false"

    if type_.type in [
        "byte",
        "char",
        "wchar",
        "int8",
        "uint8",
        "int16",
        "uint16",
        "int32",
        "uint32",
        "int64",
        "uint64",
        "float64",
    ]:
        return str(value)

    if type_.type == "float32":
        return "%sf" % value

    if type_.type == "string":
        return '"%s"' % escape_string(value)

    assert False, "unknown primitive type '%s'" % type_


def constant_value_to_rs(type_, value):
    assert value is not None

    if isinstance(type_, BasicType):
        if type_.typename == "boolean":
            return "true" if value else "false"
        elif type_.typename == "float32":
            return "%sf" % value
        return str(value)

    if isinstance(type_, AbstractGenericString):
        return '"%s"' % escape_string(value)

    assert False, "unknown constant type '%s'" % type_


# Type hierarchy:
#
# AbstractType
# - AbstractNestableType
#   - AbstractGenericString
#     - AbstractString
#       - BoundedString
#       - UnboundedString
#     - AbstractWString
#       - BoundedWString
#       - UnboundedWString
#   - BasicType
#   - NamedType
#   - NamespacedType
# - AbstractNestedType
#   - Array
#   - AbstractSequence
#     - BoundedSequence
#     - UnboundedSequence


def pre_field_serde(type_):
    if isinstance(type_, Array) and type_.size > 32:
        return '#[cfg_attr(feature = "serde", serde(with = "serde_big_array::BigArray"))]\n    '
    else:
        return ""


def make_get_rs_type(idiomatic):
    def get_rs_type(type_, current_idiomatic, desired_idiomatic):
        if isinstance(type_, BasicType):
            if type_.typename == "boolean":
                return "bool"
            elif type_.typename in ["byte", "octet"]:
                return "u8"
            elif type_.typename == "char":
                return "u8"
            elif type_.typename == "wchar":
                return "u16"
            elif type_.typename == "float":
                return "f32"
            elif type_.typename == "double":
                return "f64"
            elif type_.typename == "int8":
                return "i8"
            elif type_.typename == "uint8":
                return "u8"
            elif type_.typename == "int16":
                return "i16"
            elif type_.typename == "uint16":
                return "u16"
            elif type_.typename == "int32":
                return "i32"
            elif type_.typename == "uint32":
                return "u32"
            elif type_.typename == "int64":
                return "i64"
            elif type_.typename == "uint64":
                return "u64"
        elif isinstance(type_, BoundedString):
            return "rosidl_runtime_rs::BoundedString<{}>".format(type_.maximum_size)
        elif isinstance(type_, BoundedWString):
            return "rosidl_runtime_rs::BoundedWString<{}>".format(type_.maximum_size)
        elif isinstance(type_, UnboundedString):
            return (
                "std::string::String"
                if current_idiomatic and desired_idiomatic
                else "rosidl_runtime_rs::String"
            )
        elif isinstance(type_, UnboundedWString):
            return (
                "std::string::String"
                if current_idiomatic and desired_idiomatic
                else "rosidl_runtime_rs::WString"
            )
        elif isinstance(type_, Array):
            return f"[{get_rs_type(type_.value_type, current_idiomatic, desired_idiomatic)}; {type_.size}]"
        elif isinstance(type_, UnboundedSequence):
            container_type = (
                "Vec" if current_idiomatic and desired_idiomatic else "rosidl_runtime_rs::Sequence"
            )
            return f"{container_type}<{get_rs_type(type_.value_type, current_idiomatic, desired_idiomatic)}>"
        elif isinstance(type_, BoundedSequence):
            # BoundedSequences can be in the idiomatic API, but the containing type cannot be from the
            # idiomatic API because we do not implement SequenceAlloc for idiomatic types.
            return f"rosidl_runtime_rs::BoundedSequence<{get_rs_type(type_.value_type, current_idiomatic, False)}, {type_.maximum_size}>"
        elif isinstance(type_, NamespacedType):
            # All types should be referencable like this
            # `super::msg::rmw::Foo` (From idiomatic modules)
            # `super::super::msg::rmw::Foo` (From non-idiomatic modules)
            # `<other_package>::msg::rmw::Foo` (From external packages)
            prefix = "super::" if current_idiomatic else "super::super::"

            symbol = f"{prefix}{'::'.join(type_.namespaced_name()[1:])}"

            # This symbol is coming from an external crate (or needs a `use` statement).
            # So it should not be relative (i.e., no `super::`) and should have the top level
            # package name (i.e., `builtin_interfaces::`)
            top_level_package = type_.namespaces[0]
            if top_level_package != package_name:
                symbol = "::".join(type_.namespaced_name())

            if not desired_idiomatic and "::rmw::" not in symbol:
                parts = symbol.split("::")
                parts.insert(-1, "rmw")
                symbol = "::".join(parts)

            return symbol

        assert False, "unknown type '%s'" % type_.typename

    # Start out by assuming all calls have matching current and desired idiomatic values.
    # (i.e. symbols within the `...::rmw` scope want other values in the `...::rmw` scope).
    return lambda _type: get_rs_type(_type, idiomatic, idiomatic)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Generate one Serde-based Rust crate from ROS IDL packages."
    )
    parser.add_argument(
        "package", nargs="*",
        help="Optional package subset; default: all packages containing IDLs under --idl-root."
    )
    parser.add_argument(
        "--crate-name", default="ros_interfaces",
        help="Generated crate name (default: ros_interfaces).",
    )
    parser.add_argument(
        "--idl-root",
        type=pathlib.Path,
        default=DEFAULT_IDL_ROOT,
        help="IDL package root (default: idl/ beside this script).",
    )
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Crate directory (default: crates/ros_interfaces beside this script).",
    )
    parser.add_argument(
        "--package-version", default="0.0.0", help="Version written to Cargo.toml (default: 0.0.0)."
    )
    parser.add_argument(
        "--idl",
        dest="idl_files",
        action="append",
        type=pathlib.Path,
        help="Relative IDL path (single package only); omit to discover all package IDLs.",
    )
    args = parser.parse_args(argv)

    return generate_rs(
        args.package,
        args.idl_root,
        args.output_dir,
        package_version=args.package_version,
        idl_files=args.idl_files,
        crate_name=args.crate_name,
    )


if __name__ == "__main__":
    sys.exit(main())
