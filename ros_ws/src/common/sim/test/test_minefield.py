import math
import xml.etree.ElementTree as ET

import pytest
from pydantic import ValidationError
from sim.minefield import (
    Field,
    FieldStyle,
    Layout,
    Markings,
    Mine,
    RandomConfig,
    cell_center,
    dump_layout,
    field_model_sdf,
    generate_random,
    load_layout,
    mine_xy,
    write_csv,
)


def test_default_field_is_80_by_300_ft():
    field = Field()
    assert (field.cols, field.rows) == (40, 150)
    assert field.width == pytest.approx(80 * 0.3048)
    assert field.length == pytest.approx(300 * 0.3048)


def test_cell_centres_start_at_the_start_edge_centre():
    field = Field()
    assert cell_center(field, 0, 0) == pytest.approx((0.3048, -12.192 + 0.3048))
    assert cell_center(field, 149, 39) == pytest.approx((91.44 - 0.3048, 12.192 - 0.3048))


def test_random_layout_is_reproducible_and_one_mine_per_cell():
    config = RandomConfig(seed=42, count=500)
    a = generate_random(Field(), config)
    b = generate_random(Field(), config)
    assert a == b
    assert len(a.mines) == 500
    assert len({(m.row, m.col) for m in a.mines}) == 500


def test_random_offsets_keep_the_mine_inside_its_cell():
    field = Field()
    config = RandomConfig(seed=1, count=2000, edge_margin=0.07)
    reach = field.cell_size / 2 - 0.07
    for mine in generate_random(field, config).mines:
        assert abs(mine.dx) <= reach and abs(mine.dy) <= reach
        assert -math.pi <= mine.yaw <= math.pi


def test_unseeded_layout_records_its_seed():
    layout = generate_random(Field(), RandomConfig(count=10))
    assert layout.seed is not None
    assert generate_random(Field(), RandomConfig(seed=layout.seed, count=10)) == layout


def test_density_and_face_down_prob():
    layout = generate_random(Field(), RandomConfig(seed=3, density=0.5, face_down_prob=1.0))
    assert 2700 < len(layout.mines) < 3300
    assert all(m.face == "down" for m in layout.mines)


def test_keep_out_clears_cells_near_the_spawn():
    field = Field()
    layout = generate_random(field, RandomConfig(seed=0, density=1.0, keep_out=[(0, 0, 1.5)]))
    for mine in layout.mines:
        cx, cy = cell_center(field, mine.row, mine.col)
        assert math.hypot(cx, cy) > 1.5 - field.cell_size


def test_random_config_needs_exactly_one_amount():
    with pytest.raises(ValidationError):
        RandomConfig(seed=1)
    with pytest.raises(ValidationError):
        RandomConfig(seed=1, count=5, density=0.1)


def test_count_larger_than_free_cells_fails():
    with pytest.raises(ValueError):
        generate_random(Field(rows=2, cols=2), RandomConfig(count=5))


@pytest.mark.parametrize(
    "mine",
    [
        {"row": 0, "col": 0, "dx": 0.31},  # offset outside the cell
        {"row": 150, "col": 0},  # outside the field
    ],
)
def test_invalid_mines_are_rejected(mine):
    with pytest.raises(ValidationError):
        Layout(mines=[mine])


def test_two_mines_in_one_cell_are_rejected():
    with pytest.raises(ValidationError):
        Layout(mines=[{"row": 3, "col": 4}, {"row": 3, "col": 4, "dx": 0.1}])


def test_layout_round_trips_through_yaml_and_csv(tmp_path):
    layout = generate_random(Field(), RandomConfig(seed=9, count=25))
    dump_layout(layout, tmp_path / "layout.yaml")
    assert load_layout(tmp_path / "layout.yaml") == layout

    write_csv(layout, tmp_path / "truth.csv")
    rows = (tmp_path / "truth.csv").read_text().splitlines()
    assert rows[0] == "id,row,col,x,y,yaw,face"
    assert len(rows) == 26


def test_empty_layout_round_trips(tmp_path):
    dump_layout(Layout(), tmp_path / "empty.yaml")
    assert load_layout(tmp_path / "empty.yaml") == Layout()


def test_field_sdf_has_one_visual_per_mine_and_marking():
    layout = Layout(
        mines=[Mine(row=0, col=0, face="up"), Mine(row=10, col=20, dx=0.1, yaw=1.0, face="down")]
    )
    root = ET.fromstring(field_model_sdf(layout, FieldStyle(markings=Markings(grid=True))))
    model = root.find("model")
    assert model.findtext("static") == "true"
    assert model.find(".//collision") is None
    names = [v.get("name") for v in model.iter("visual")]
    assert names.count("mine_r0_c0") == 1 and names.count("mine_r10_c20") == 1
    assert len(names) == 2 + 4 + (150 - 1) + (40 - 1)

    down = model.find(".//visual[@name='mine_r10_c20']")
    x, y, z, roll, pitch, yaw = (float(v) for v in down.findtext("pose").split())
    assert (x, y) == pytest.approx(mine_xy(layout.field, layout.mines[1]), abs=1e-4)
    assert z > 0 and roll == pytest.approx(math.pi) and yaw == pytest.approx(1.0)


def test_unknown_mesh_is_rejected():
    with pytest.raises(ValueError):
        field_model_sdf(Layout(), FieldStyle(mesh="huge"))


def test_full_field_sdf_is_one_model():
    layout = generate_random(Field(), RandomConfig(seed=5, density=1.0))
    sdf = field_model_sdf(layout)
    assert len(layout.mines) == 6000
    assert len(ET.fromstring(sdf).findall("model")) == 1
