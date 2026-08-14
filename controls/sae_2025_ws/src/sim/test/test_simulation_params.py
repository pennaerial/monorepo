import yaml
from sim.simulation_params import SimulationParams
from sim.world_gen.entity import Entity

minimum_world_params = """
world:
  name: custom
  node: custom_world_node
"""


def test_minimum_world_params():
    data = yaml.safe_load(minimum_world_params)

    params = SimulationParams.model_validate(data)

    assert params.world.name == "custom"
    assert params.world.node == "custom_world_node"
    assert params.world.physics is None
    assert params.world.controllables == []
    assert params.world.entities == []
    assert params.scoring is None

with_physics = """
world:
  name: custom
  node: custom_world_node
  physics:
    friction: 0.8
"""

def test_with_physics_sim_params():
    data = yaml.safe_load(with_physics)
    params = SimulationParams.model_validate(data)

    assert params.world.name == "custom"
    assert params.world.node == "custom_world_node"
    assert params.world.physics is not None
    assert params.world.physics.friction == 0.8
    assert params.world.controllables == []
    assert params.world.entities == []
    assert params.scoring is None

sae_params = """
world:
  name: sae
  node: sae_world_node
  physics:
    friction: 0.6 # friction coefficient for the ground plane
  entities:
    - name: dlz
      model: dlz
      position: [0.0, 0.0, -0.1]
      rpy: [0.0, 0.0, 0.0]
  controllables:
    - name: payload_0
      model: payload
      position: [5.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
    - name: payload_1
      model: payload
      position: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
"""

def test_sae_params():
    data = yaml.safe_load(sae_params)
    params = SimulationParams.model_validate(data)

    assert params.world.name == "sae"
    assert params.world.node == "sae_world_node"
    assert params.world.physics is not None
    assert params.world.physics.friction == 0.6

    expected_entities = [
        Entity(
            name="dlz",
            model = "dlz",
            position = (0.0, 0.0, -0.1),
            rpy = (0.0, 0.0, 0.0),
            world="sae"
        )
    ]

    expected_controllables = [
        Entity(
            name="payload_0",
            model="payload",
            position = (5.0, 0.0, 0.0),
            rpy = (0.0, 0.0, 0.0),
            world="sae"
        ),
        Entity(
            name="payload_1",
            model="payload",
            position = (0.0, 0.0, 0.0),
            rpy = (0.0, 0.0, 0.0),
            world="sae"
        ),
    ]

    assert params.world.entities == expected_entities
    assert params.world.controllables == expected_controllables
