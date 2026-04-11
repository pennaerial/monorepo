from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from .config import (
    BuildSourceStore,
    InventoryStore,
    OperatorConfig,
    TargetRecord,
    build_source_store_paths,
    default_fleet_file,
    seed_target_from_env,
)
from .ssh import SSHExecutor
from .state import MissionStateMachine


@dataclass(slots=True)
class TargetContext:
    target: TargetRecord
    ssh: SSHExecutor
    mission_state: MissionStateMachine


@dataclass(slots=True)
class AppContext:
    base_dir: Path
    operator_config: OperatorConfig
    inventory: InventoryStore
    build_source_store: BuildSourceStore
    _target_cache: dict[str, TargetContext] = field(default_factory=dict)

    def resolve_target(self, target_id: str | None = None) -> TargetContext:
        target = self.inventory.get_target(target_id)
        cached = self._target_cache.get(target.target_id)
        if cached is not None:
            if cached.target.to_store_dict() == target.to_store_dict():
                return cached

        context = TargetContext(
            target=target,
            ssh=SSHExecutor(target),
            mission_state=MissionStateMachine.create(),
        )
        self._target_cache[target.target_id] = context
        return context

    def list_targets(self) -> list[TargetRecord]:
        return self.inventory.list_targets()


def create_context(base_dir: Path) -> AppContext:
    operator = OperatorConfig.from_env(base_dir)
    seed_target = seed_target_from_env(base_dir, operator)
    inventory = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
        default_fleet_file=default_fleet_file(base_dir),
        seed_target=seed_target,
    )
    build_source_path, build_source_cache_dir = build_source_store_paths(
        operator.inventory_path
    )
    build_source_store = BuildSourceStore(
        build_source_path,
        cache_dir=build_source_cache_dir,
        default_fleet_file=default_fleet_file(base_dir),
    )
    return AppContext(
        base_dir=base_dir,
        operator_config=operator,
        inventory=inventory,
        build_source_store=build_source_store,
    )
