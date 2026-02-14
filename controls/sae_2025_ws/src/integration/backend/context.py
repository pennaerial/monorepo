from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .config import RuntimeConfig
from .ssh import SSHExecutor
from .state import MissionStateMachine


@dataclass(slots=True)
class AppContext:
    base_dir: Path
    config: RuntimeConfig
    ssh: SSHExecutor
    mission_state: MissionStateMachine


def create_context(base_dir: Path) -> AppContext:
    config = RuntimeConfig.from_env(base_dir)
    ssh = SSHExecutor(config)
    mission_state = MissionStateMachine.create()
    return AppContext(base_dir=base_dir, config=config, ssh=ssh, mission_state=mission_state)
