import random as r
import xml.etree.ElementTree as ET

from xml.dom import minidom
from pathlib import Path
from courses.ascent import AscentCourse
from courses.descent import DescentCourse
from courses.slalom import SlalomCourse

def add_hoops(input_file, output_file, hoop_positions):
    # Expand ~, make absolute, and validate paths
    in_path = Path(input_file).expanduser().resolve()
    out_path = Path(output_file).expanduser().resolve()
    if not in_path.exists():
        raise FileNotFoundError(
            f"Input SDF not found: {in_path}\nCWD: {Path.cwd().resolve()}"
        )
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Parse
    tree = ET.parse(str(in_path))  # str() for safety on older libs
    root = tree.getroot()

    # handle namespace if present (root.tag may be like "{...}sdf")
    ns = ""
    if root.tag.startswith("{"):
        ns = root.tag.split("}")[0] + "}"

    # try to find <world> with/without namespace
    world = root.find(f"{ns}world") or root.find("world") or root.find(".//world")
    if world is None:
        raise RuntimeError("No <world> tag found in the SDF!")

    # Add new hoops with given positions
    for i, pos in enumerate(hoop_positions, start=1):
        x, y, z, roll, pitch, yaw = pos
        inc = ET.Element("include")
        ET.SubElement(inc, "uri").text = "model://hoop"
        ET.SubElement(inc, "pose").text = f"{x} {y} {z} {roll} {pitch} {yaw}"
        ET.SubElement(inc, "name").text = f"hoop_{i}"
        world.append(inc)

    # --- remove whitespace-only text/tail nodes to avoid minidom producing extra blank lines ---
    def strip_whitespace(elem):
        if elem.text is not None and elem.text.strip() == "":
            elem.text = None
        for child in list(elem):
            strip_whitespace(child)
            if child.tail is not None and child.tail.strip() == "":
                child.tail = None
    strip_whitespace(root)

    # Pretty print and write
    rough_bytes = ET.tostring(root, encoding="utf-8")
    pretty = minidom.parseString(rough_bytes.decode("utf-8")).toprettyxml(indent="  ")
    lines = [ln for ln in pretty.splitlines() if ln.strip() != ""]
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

def generate_world(gen_style, course_params):
    courses = ['ascent', 'descent', 'slalom']

    if gen_style.lower() == 'random':
        course_id = r.uniform(0, len(courses) - 1)
    elif gen_style.lower() == "previous":
        pass
    elif gen_style.lower() == "ascent":
        pass
    elif gen_style.lower() == "descent":
        pass
    elif gen_style.lower() == "slalom":
        pass

    ip_file = '/home/avaniko/penn-air/monorepo/controls/sae_2025_ws/src/sim/sim/worlds/template.sdf'
    op_file = '/home/avaniko/penn-air/monorepo/controls/sae_2025_ws/src/sim/sim/worlds/custom.sdf'