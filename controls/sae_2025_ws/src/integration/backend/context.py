from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from .config import (
    BuildSourceStore,
    InventoryStore,
    OperatorConfig,
    TargetRecord,
    build_source_store_paths,
    _normalize_host_identity,
)
from .ssh import SSHExecutor
from .state import MissionStateMachine


def _normalize_host_alias(value: str | None) -> str:
    normalized = _normalize_host_identity(value)
    if normalized.endswith(".local"):
        return normalized[:-6]
    return normalized


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

    def require_deploy_context(self, *, require_build_source: bool = True) -> None:
        current = self.build_source_store.current()
        if require_build_source and current.kind == "none":
            raise ValueError(
                "Select a build source before opening per-device deploy controls."
            )
        if not current.fleet_file.strip():
            raise ValueError(
                "Select a fleet file before opening per-device deploy controls."
            )

    def _cache_key(self, target: TargetRecord) -> str:
        return _normalize_host_alias(target.pi_host) or target.target_id

    def _resolve_inventory_target(
        self, target_id: str | None, hostname: str | None
    ) -> TargetRecord | None:
        if not (target_id or "").strip() and not (hostname or "").strip():
            try:
                return self.inventory.get_target(None)
            except KeyError:
                return None

        resolved = (target_id or "").strip() or ""
        if resolved:
            try:
                return self.inventory.get_target(resolved)
            except KeyError:
                pass

        normalized_host = _normalize_host_alias(hostname or resolved)
        if not normalized_host:
            return None

        for target in self.inventory.list_targets():
            if normalized_host == _normalize_host_alias(target.pi_host):
                return target
        return None

    def _build_transient_target(
        self,
        *,
        target_id: str | None,
        hostname: str | None,
        vehicle_name: str | None,
        label: str | None,
        pi_user: str | None,
        deploy_root: str | None,
        ssh_key: str | None,
        ssh_pass: str | None,
        service_unit: str | None,
        enabled: bool | None,
    ) -> TargetRecord:
        normalized_host = _normalize_host_identity(hostname or target_id)
        return TargetRecord.for_host(
            hostname=normalized_host or (target_id or hostname or ""),
            target_id=target_id or normalized_host,
            label=label,
            pi_user=pi_user or self.operator_config.default_pi_user,
            deploy_root=deploy_root or self.operator_config.default_deploy_root,
            ssh_key=ssh_key
            if ssh_key is not None
            else self.operator_config.default_ssh_key,
            ssh_pass=ssh_pass
            if ssh_pass is not None
            else self.operator_config.default_ssh_pass,
            vehicle_name=vehicle_name or "",
            service_unit=service_unit or "pennair-autonomy.service",
            enabled=True if enabled is None else enabled,
            default_deploy_root=self.operator_config.default_deploy_root,
        )

    def resolve_target(
        self,
        target_id: str | None = None,
        *,
        hostname: str | None = None,
        vehicle_name: str | None = None,
        label: str | None = None,
        pi_user: str | None = None,
        deploy_root: str | None = None,
        ssh_key: str | None = None,
        ssh_pass: str | None = None,
        service_unit: str | None = None,
        enabled: bool | None = None,
    ) -> TargetContext:
        normalized_hostname = (hostname or "").strip()
        if normalized_hostname:
            target = self._build_transient_target(
                target_id=target_id,
                hostname=normalized_hostname,
                vehicle_name=vehicle_name,
                label=label,
                pi_user=pi_user,
                deploy_root=deploy_root,
                ssh_key=ssh_key,
                ssh_pass=ssh_pass,
                service_unit=service_unit,
                enabled=enabled,
            )
        else:
            target = self._resolve_inventory_target(target_id, hostname)
            if target is None:
                if not (target_id or "").strip():
                    raise KeyError("No saved targets in inventory.")
                target = self._build_transient_target(
                    target_id=target_id,
                    hostname=hostname,
                    vehicle_name=vehicle_name,
                    label=label,
                    pi_user=pi_user,
                    deploy_root=deploy_root,
                    ssh_key=ssh_key,
                    ssh_pass=ssh_pass,
                    service_unit=service_unit,
                    enabled=enabled,
                )
        if (
            vehicle_name
            and vehicle_name.strip()
            and target.vehicle_name != vehicle_name.strip()
        ):
            target = TargetRecord.from_dict(
                {
                    **target.to_store_dict(),
                    "vehicle_name": vehicle_name.strip(),
                },
                default_deploy_root=self.operator_config.default_deploy_root,
            )

        cached = self._target_cache.get(self._cache_key(target))
        if cached is not None:
            if cached.target.to_store_dict() == target.to_store_dict():
                return cached

        context = TargetContext(
            target=target,
            ssh=SSHExecutor(target),
            mission_state=MissionStateMachine.create(),
        )
        self._target_cache[self._cache_key(target)] = context
        return context

    def resolve_live_target(
        self,
        *,
        hostname: str | None,
        vehicle_name: str | None = None,
        label: str | None = None,
        pi_user: str | None = None,
        deploy_root: str | None = None,
        ssh_key: str | None = None,
        ssh_pass: str | None = None,
        service_unit: str | None = None,
        enabled: bool | None = None,
    ) -> TargetContext:
        normalized_host = _normalize_host_identity(hostname)
        if not normalized_host:
            raise KeyError("Hostname is required.")

        target = self._build_transient_target(
            target_id=normalized_host,
            hostname=normalized_host,
            vehicle_name=vehicle_name,
            label=label,
            pi_user=pi_user,
            deploy_root=deploy_root,
            ssh_key=ssh_key,
            ssh_pass=ssh_pass,
            service_unit=service_unit,
            enabled=enabled,
        )

        cached = self._target_cache.get(self._cache_key(target))
        if (
            cached is not None
            and cached.target.to_store_dict() == target.to_store_dict()
        ):
            return cached

        context = TargetContext(
            target=target,
            ssh=SSHExecutor(target),
            mission_state=MissionStateMachine.create(),
        )
        self._target_cache[self._cache_key(target)] = context
        return context

    def list_targets(self) -> list[TargetRecord]:
        return self.inventory.list_targets()


def create_context(base_dir: Path) -> AppContext:
    operator = OperatorConfig.from_env(base_dir)
    inventory = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
    )
    build_source_path, build_source_cache_dir = build_source_store_paths(
        operator.inventory_path
    )
    build_source_store = BuildSourceStore(
        build_source_path,
        cache_dir=build_source_cache_dir,
    )
    return AppContext(
        base_dir=base_dir,
        operator_config=operator,
        inventory=inventory,
        build_source_store=build_source_store,
    )
