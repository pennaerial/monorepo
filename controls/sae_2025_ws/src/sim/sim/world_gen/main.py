# sim/world_gen/main.py
import yaml
from pathlib import Path
from types import SimpleNamespace

#import all the generators you will need
from sim.world_gen.generators.hoop_world_generator import HoopWorldGenerator

# used to map strings from YAML to class. Must add to map when we make a new generator
GENERATOR_MAP = {
    "hoop": HoopWorldGenerator,
}

# used to expand file paths with ~ to be absolute paths on local machine
def _expand(path: str):
    return str(Path(path).expanduser().resolve())

def main():
    # assuming we always will update the worldgen.yaml file
    cfg_path = Path(__file__).parent / "configs" / "worldgen.yaml"
    cfg = yaml.safe_load(cfg_path.read_text())

    # make sure the important fields exist
    for k in ("input_sdf", "output_sdf", "generatorName", "style", "params"):
        if k not in cfg:
            raise ValueError(f"Missing required key '{k}' in {cfg_path}")

    input_sdf  = _expand(cfg["input_sdf"])
    output_sdf = _expand(cfg["output_sdf"])
    gen_name   = cfg["generatorName"].lower()
    style      = cfg["style"]
    params     = cfg.get("params", {})


    # turns the dict into an object as required by the worldgen.py file
    params_obj = SimpleNamespace(**params)

    if gen_name not in GENERATOR_MAP:
        available = ", ".join(sorted(GENERATOR_MAP))
        raise ValueError(f"Unknown generator '{gen_name}'. Available: {available}")
    GenClass = GENERATOR_MAP[gen_name]

    generator = GenClass(input_file=input_sdf, output_file=output_sdf)
    generator.generate_world(style=style, params=params_obj)
    print(f"Generated world written to {output_sdf}")

if __name__ == "__main__":
    main()
