import pytest
import os
from pathlib import Path

from sim.world_gen.entity import Entity

# some tests require valid gz-models to be updated


def test_entity_requires_model_or_path_to_sdf():
    with pytest.raises(ValueError):
        Entity(
            name="test",
            position=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
            world="world",
        )


MODELS_PATH = Path(os.environ["PENNAIR_GZ_MODELS_PATH"]) / "models"


def test_only_path_to_sdf():
    model_dir = MODELS_PATH / "x500"
    sdf_path = model_dir / "model.sdf"
    sdf_path.touch()

    entity = Entity(
        name="x500_0",
        path_to_sdf=str(sdf_path),
        position=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
        world="world",
    )

    assert entity.model == str(model_dir)
    assert entity.path_to_sdf == str(sdf_path)


def test_only_model():
    model_dir = MODELS_PATH / "x500"
    sdf_path = model_dir / "model.sdf"

    entity = Entity(
        name="x500_0",
        model="x500",
        position=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
        world="world",
    )

    assert entity.model == "x500"
    assert entity.path_to_sdf == str(sdf_path)


def test_both_path_to_sdf_and_model():
    model_dir = MODELS_PATH / "x500"

    entity = Entity(
        name="x500_0",
        model="x500",
        path_to_sdf="/wrong/path/model.sdf",
        position=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
        world="world",
    )

    expected_sdf_path = model_dir / "model.sdf"
    assert entity.path_to_sdf == str(expected_sdf_path)


TMP = Path("/tmp")


def test_entity_rejects_non_sdf_path():
    sdf_path = TMP / "nonsdf.txt"
    sdf_path.touch()

    with pytest.raises(ValueError):
        Entity(
            name="test",
            path_to_sdf=str(sdf_path),
            position=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
            world="world",
        )


def test_entity_rejects_missing_sdf(tmp_path: Path):
    sdf_path = TMP / "model.sdf"

    with pytest.raises(ValueError, match="SDF file does not exist"):
        Entity(
            name="test",
            path_to_sdf=str(sdf_path),
            position=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
            world="world",
        )
