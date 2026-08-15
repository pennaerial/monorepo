from __future__ import annotations

import sys
from pathlib import Path

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from sim.utils import template_world_reference_candidates  # noqa: E402


def test_template_world_reference_candidates_prefix_worlds_for_bare_filename():
    assert template_world_reference_candidates("template.sdf") == [Path("worlds/template.sdf")]


def test_template_world_reference_candidates_reject_nested_relative_paths():
    try:
        template_world_reference_candidates("worlds/template.sdf")
    except ValueError as exc:
        assert "bare filename" in str(exc)
    else:
        raise AssertionError("Expected nested relative template paths to be rejected.")
