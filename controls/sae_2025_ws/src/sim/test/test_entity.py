import os
from pathlib import Path
from unittest.mock import patch

import pytest

from sim.world_gen.entity import Entity


@pytest.fixture
def models_path(tmp_path: Path):
    models_path = tmp_path / "models"
    models_path.mkdir()

    with patch.dict(os.environ, {"PENNAIR_GZ_MODELS_PATH": str(tmp_path)}):
        yield models_path


def test_entity_requires_model_or_path_to_model():
    with pytest.raises(ValueError):
        Entity(
            name="test",
            position=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
            world="world",
        )


@pytest.mark.parametrize("suffix", [".sdf", ".urdf"])
def test_only_path_to_model(tmp_path: Path, suffix: str):
    model_dir = tmp_path / "payload"
    model_dir.mkdir()
    model_path = model_dir / f"model{suffix}"
    model_path.touch()

    entity = Entity(
        name="payload_0",
        path_to_model=str(model_path),
        position=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
        world="world",
    )

    assert entity.model == model_dir.name
    assert entity.path_to_model == str(model_path)


def test_model_prefers_urdf(models_path: Path):
    model_dir = models_path / "payload"
    model_dir.mkdir()
    urdf_path = model_dir / "model.urdf"
    urdf_path.touch()
    (model_dir / "model.sdf").touch()

    entity = Entity(
        name="payload_0",
        model="payload",
        position=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
        world="world",
    )

    assert entity.model == model_dir.name
    assert entity.path_to_model == str(urdf_path)


def test_model_falls_back_to_sdf(models_path: Path):
    model_dir = models_path / "x500"
    model_dir.mkdir()
    sdf_path = model_dir / "model.sdf"
    sdf_path.touch()

    entity = Entity(
        name="x500_0",
        model="x500",
        position=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
        world="world",
    )

    assert entity.model == "x500"
    assert entity.path_to_model == str(sdf_path)


def test_path_to_model_takes_precedence_over_model(
    tmp_path: Path, models_path: Path
):
    named_model_dir = models_path / "payload"
    named_model_dir.mkdir()
    (named_model_dir / "model.urdf").touch()

    explicit_model_dir = tmp_path / "explicit"
    explicit_model_dir.mkdir()
    explicit_model_path = explicit_model_dir / "model.sdf"
    explicit_model_path.touch()

    entity = Entity(
        name="payload_0",
        model="payload",
        path_to_model=str(explicit_model_path),
        position=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
        world="world",
    )

    assert entity.model == "payload"
    assert entity.path_to_model == str(explicit_model_path)


def test_entity_factory_uses_urdf_path(tmp_path: Path):
    urdf_path = tmp_path / "model.urdf"
    urdf_path.touch()

    entity = Entity(
        name="payload_0",
        path_to_model=str(urdf_path),
        position=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
        world="world",
    )

    assert entity.to_entity_factory_msg().sdf_filename == str(urdf_path)


def test_entity_rejects_non_model_path(tmp_path: Path):
    model_path = tmp_path / "nonmodel.txt"
    model_path.touch()

    with pytest.raises(
        ValueError, match="Model path must point to a .urdf or .sdf file"
    ):
        Entity(
            name="test",
            path_to_model=str(model_path),
            position=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
            world="world",
        )


def test_entity_rejects_missing_model(tmp_path: Path):
    model_path = tmp_path / "model.sdf"

    with pytest.raises(ValueError, match="Model file does not exist"):
        Entity(
            name="test",
            path_to_model=str(model_path),
            position=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
            world="world",
        )


def test_entity_rejects_unknown_model(models_path: Path):
    with pytest.raises(ValueError, match="has neither"):
        Entity(
            name="test",
            model="missing",
            position=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
            world="world",
        )
