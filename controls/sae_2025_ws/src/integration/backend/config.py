from __future__ import annotations

import json
import os
import re
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import yaml


_TARGET_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")
_PI_USER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_-]*$")
_HOST_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]*$")
_SERVICE_UNIT_RE = re.compile(r"^[A-Za-z0-9@_.-]+\.service$")
_BUILD_SOURCE_KINDS = {"none", "github", "local_artifact", "local_codebase"}
_GITHUB_BUILD_SOURCES = {"release", "actions"}


@dataclass(slots=True)
class OperatorConfig:
    github_repo: str
    github_token: str
    hotspot_name: str
    inventory_path: Path
    default_deploy_root: str
    default_pi_user: str
    default_ssh_key: str
    default_ssh_pass: str

    @classmethod
    def from_env(cls, base_dir: Path) -> "OperatorConfig":
        github_repo = os.environ.get("GITHUB_REPO", "")
        if not github_repo:
            github_repo = _detect_github_repo(base_dir)
        return cls(
            github_repo=github_repo,
            github_token=os.environ.get("GITHUB_TOKEN", ""),
            hotspot_name=os.environ.get("HOTSPOT_CON_NAME", "penn-desktop"),
            # v1 stores operator/target secrets in a local JSON inventory file.
            # Keep the path explicit so we can swap this for a real secret store later.
            inventory_path=Path(
                os.environ.get(
                    "INVENTORY_PATH",
                    str(base_dir / ".integration_inventory.json"),
                )
            ),
            default_deploy_root=os.environ.get(
                "DEPLOY_ROOT", "/home/penn/pennair-deploy"
            ),
            default_pi_user=os.environ.get("DEFAULT_PI_USER", "penn"),
            default_ssh_key=os.environ.get("DEFAULT_SSH_KEY", ""),
            default_ssh_pass=os.environ.get("DEFAULT_SSH_PASS", ""),
        )

    def to_safe_dict(self) -> dict[str, str]:
        return {
            "github_repo": self.github_repo,
            "github_token": "••••" if self.github_token else "",
            "hotspot_name": self.hotspot_name,
            "inventory_path": str(self.inventory_path),
            "default_deploy_root": self.default_deploy_root,
            "default_pi_user": self.default_pi_user,
            "default_ssh_key": self.default_ssh_key,
            "default_ssh_pass": "••••" if self.default_ssh_pass else "",
        }

    def to_store_dict(self) -> dict[str, str]:
        return {
            "github_repo": self.github_repo,
            "github_token": self.github_token,
            "hotspot_name": self.hotspot_name,
            "default_deploy_root": self.default_deploy_root,
            "default_pi_user": self.default_pi_user,
            "default_ssh_key": self.default_ssh_key,
            "default_ssh_pass": self.default_ssh_pass,
        }

    def update_from_form(self, updates: dict[str, str | None]) -> dict[str, str]:
        changed: dict[str, str] = {}

        def _set(field: str, value: str | None) -> None:
            if value is None:
                return
            setattr(self, field, value)
            changed[field] = value

        _set("github_repo", updates.get("github_repo"))
        _set("hotspot_name", updates.get("hotspot_name"))
        _set("default_deploy_root", updates.get("default_deploy_root"))
        _set("default_pi_user", updates.get("default_pi_user"))
        _set("default_ssh_key", updates.get("default_ssh_key"))

        github_token = updates.get("github_token")
        if github_token is not None and github_token != "••••":
            self.github_token = github_token
            changed["github_token"] = "(set)"

        default_ssh_pass = updates.get("default_ssh_pass")
        if default_ssh_pass is not None and default_ssh_pass != "••••":
            self.default_ssh_pass = default_ssh_pass
            changed["default_ssh_pass"] = "(set)"

        return changed


@dataclass(slots=True)
class TargetRecord:
    target_id: str
    label: str
    pi_user: str
    pi_host: str
    deploy_root: str
    ssh_key: str
    ssh_pass: str
    fleet_file: str
    vehicle_name: str
    overlay_yaml: str
    service_unit: str
    enabled: bool = True

    @classmethod
    def from_dict(
        cls,
        data: dict[str, object],
        *,
        default_deploy_root: str,
        default_fleet_file: str,
    ) -> "TargetRecord":
        target_id = str(data.get("target_id", "")).strip()
        if not _TARGET_ID_RE.fullmatch(target_id):
            raise ValueError(
                f"Invalid target_id '{target_id}'. Use letters, numbers, dots, dashes, or underscores."
            )
        pi_user = str(data.get("pi_user", "penn")).strip() or "penn"
        if not _PI_USER_RE.fullmatch(pi_user):
            raise ValueError(f"Target '{target_id}' has invalid pi_user '{pi_user}'.")
        pi_host = str(data.get("pi_host", "")).strip() or "penn-desktop.local"
        if not _HOST_RE.fullmatch(pi_host):
            raise ValueError(f"Target '{target_id}' has invalid pi_host '{pi_host}'.")
        deploy_root = str(data.get("deploy_root", default_deploy_root)).strip()
        if not deploy_root:
            raise ValueError(f"Target '{target_id}' requires a non-empty deploy_root.")
        if not (
            deploy_root.startswith("/")
            or deploy_root.startswith("~")
            or deploy_root.startswith("$HOME")
        ):
            raise ValueError(
                f"Target '{target_id}' deploy_root '{deploy_root}' must be absolute or home-relative."
            )
        fleet_file = str(data.get("fleet_file", default_fleet_file)).strip()
        if not fleet_file:
            raise ValueError(f"Target '{target_id}' requires a non-empty fleet_file.")
        vehicle_name = str(data.get("vehicle_name", "")).strip()
        if not vehicle_name:
            raise ValueError(f"Target '{target_id}' requires a non-empty vehicle_name.")
        service_unit = (
            str(data.get("service_unit", "pennair-autonomy.service")).strip()
            or "pennair-autonomy.service"
        )
        if not _SERVICE_UNIT_RE.fullmatch(service_unit):
            raise ValueError(
                f"Target '{target_id}' has invalid service_unit '{service_unit}'."
            )
        return cls(
            target_id=target_id,
            label=str(data.get("label", target_id)).strip() or target_id,
            pi_user=pi_user,
            pi_host=pi_host,
            deploy_root=deploy_root or default_deploy_root,
            ssh_key=str(data.get("ssh_key", "")).strip(),
            ssh_pass=str(data.get("ssh_pass", "")),
            fleet_file=fleet_file or default_fleet_file,
            vehicle_name=vehicle_name,
            overlay_yaml=str(data.get("overlay_yaml", "") or ""),
            service_unit=service_unit,
            enabled=bool(data.get("enabled", True)),
        )

    def ssh_target(self) -> str:
        return f"{self.pi_user}@{self.pi_host}"

    def deploy_paths(self) -> dict[str, str]:
        root = self.deploy_root.rstrip("/")
        return {
            "deploy_root": root,
            "incoming_dir": f"{root}/incoming",
            "releases_dir": f"{root}/releases",
            "current_link": f"{root}/current",
            "previous_link": f"{root}/previous",
            "config_dir": f"{root}/config",
            "state_dir": f"{root}/state",
            "exports_dir": f"{root}/exports",
            "runtime_fleet": f"{root}/config/runtime_fleet.yaml",
            "fleet_file": f"{root}/config/fleet.yaml",
            "overlay_file": f"{root}/config/overlay.yaml",
            "missions_dir": f"{root}/config/missions",
            "runner_script": f"{root}/config/start-current.sh",
            "service_unit": self.service_unit,
        }

    def workspace_paths(self) -> dict[str, str]:
        paths = self.deploy_paths()
        return {
            "deploy_root": paths["deploy_root"],
            "current_release": paths["current_link"],
            "runtime_fleet": paths["runtime_fleet"],
            "fleet_file": paths["fleet_file"],
            "overlay_file": paths["overlay_file"],
            "missions_dir": paths["missions_dir"],
        }

    def mission_paths(self) -> dict[str, str]:
        paths = self.deploy_paths()
        return {
            "runtime_fleet": paths["runtime_fleet"],
            "overlay_file": paths["overlay_file"],
            "missions_dir": paths["missions_dir"],
            "runner_script": paths["runner_script"],
        }

    def overlay_data(self) -> dict[str, object]:
        if not self.overlay_yaml.strip():
            return {}
        parsed = yaml.safe_load(self.overlay_yaml)
        if parsed is None:
            return {}
        if not isinstance(parsed, dict):
            raise ValueError(
                f"Target '{self.target_id}' overlay must be a YAML mapping."
            )
        return parsed

    def to_store_dict(self) -> dict[str, object]:
        return {
            "target_id": self.target_id,
            "label": self.label,
            "pi_user": self.pi_user,
            "pi_host": self.pi_host,
            "deploy_root": self.deploy_root,
            "ssh_key": self.ssh_key,
            "ssh_pass": self.ssh_pass,
            "fleet_file": self.fleet_file,
            "vehicle_name": self.vehicle_name,
            "overlay_yaml": self.overlay_yaml,
            "service_unit": self.service_unit,
            "enabled": self.enabled,
        }

    def to_safe_dict(self) -> dict[str, object]:
        payload = self.to_store_dict()
        payload["ssh_pass"] = "••••" if self.ssh_pass else ""
        payload["ssh_target"] = self.ssh_target()
        payload["workspace_paths"] = self.workspace_paths()
        return payload

    def update_from_form(
        self, updates: dict[str, str | bool | None]
    ) -> dict[str, str | bool]:
        changed: dict[str, str | bool] = {}

        def _set(field: str, value: str | bool | None) -> None:
            if value is None:
                return
            setattr(self, field, value)
            changed[field] = value

        _set("label", updates.get("label"))
        _set("pi_user", updates.get("pi_user"))
        _set("pi_host", updates.get("pi_host"))
        _set("deploy_root", updates.get("deploy_root"))
        _set("ssh_key", updates.get("ssh_key"))
        _set("fleet_file", updates.get("fleet_file"))
        _set("vehicle_name", updates.get("vehicle_name"))
        _set("overlay_yaml", updates.get("overlay_yaml"))
        _set("service_unit", updates.get("service_unit"))

        enabled = updates.get("enabled")
        if enabled is not None:
            normalized = (
                enabled
                if isinstance(enabled, bool)
                else str(enabled).strip().lower() in {"1", "true", "yes", "on"}
            )
            self.enabled = normalized
            changed["enabled"] = normalized

        ssh_pass = updates.get("ssh_pass")
        if ssh_pass is not None and ssh_pass != "••••":
            self.ssh_pass = str(ssh_pass)
            changed["ssh_pass"] = "(set)"

        return changed


class InventoryStore:
    def __init__(
        self,
        path: Path,
        *,
        operator_config: OperatorConfig,
        default_deploy_root: str,
        default_fleet_file: str,
        seed_target: TargetRecord,
    ):
        self.path = path
        self.operator_config = operator_config
        self.default_deploy_root = default_deploy_root
        self.default_fleet_file = default_fleet_file
        self._targets: dict[str, TargetRecord] = {}
        self._active_target_id: str = seed_target.target_id
        self._load(seed_target)

    def _load(self, seed_target: TargetRecord) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if not self.path.exists():
            self._targets = {seed_target.target_id: seed_target}
            self._active_target_id = seed_target.target_id
            self.save()
            return

        with self.path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle) or {}

        self._apply_operator_store(payload.get("operator", {}))
        loaded = self._parse_targets(payload.get("targets", []))

        if not loaded:
            loaded[seed_target.target_id] = seed_target

        self._targets = loaded
        active_target_id = str(payload.get("active_target_id", "")).strip()
        self._active_target_id = (
            active_target_id
            if active_target_id in self._targets
            else next(iter(self._targets))
        )

    def save(self) -> None:
        payload = self.export_payload()
        with self.path.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
            handle.write("\n")

    def export_payload(self) -> dict[str, object]:
        return {
            "version": 1,
            "operator": self.operator_config.to_store_dict(),
            "active_target_id": self._active_target_id,
            "targets": [
                self._targets[target_id].to_store_dict()
                for target_id in sorted(self._targets)
            ],
        }

    def import_payload(
        self, payload: dict[str, object], *, replace_existing: bool = False
    ) -> dict[str, object]:
        if not isinstance(payload, dict):
            raise ValueError("Inventory import must be a JSON mapping.")

        imported_targets = self._parse_targets(payload.get("targets", []))
        if not imported_targets:
            raise ValueError("Inventory import must define a non-empty targets list.")

        self._apply_operator_store(payload.get("operator", {}))

        if replace_existing:
            self._targets = imported_targets
        else:
            merged = dict(self._targets)
            merged.update(imported_targets)
            self._targets = merged

        active_target_id = str(payload.get("active_target_id", "")).strip()
        if active_target_id in self._targets:
            self._active_target_id = active_target_id
        elif replace_existing:
            self._active_target_id = next(iter(sorted(self._targets)))

        self.save()
        return {
            "targets_imported": len(imported_targets),
            "active_target_id": self._active_target_id,
            "target_ids": sorted(imported_targets),
            "replace_existing": replace_existing,
        }

    def active_target_id(self) -> str:
        return self._active_target_id

    def list_targets(self) -> list[TargetRecord]:
        return [self._targets[target_id] for target_id in sorted(self._targets)]

    def get_target(self, target_id: str | None) -> TargetRecord:
        resolved = (
            target_id.strip()
            if target_id and target_id.strip()
            else self._active_target_id
        )
        try:
            return self._targets[resolved]
        except KeyError as exc:
            raise KeyError(f"Unknown target '{resolved}'.") from exc

    def set_active_target(self, target_id: str) -> TargetRecord:
        target = self.get_target(target_id)
        self._active_target_id = target.target_id
        self.save()
        return target

    def upsert_target(
        self, data: dict[str, str | bool | None]
    ) -> tuple[TargetRecord, bool]:
        target_id = str(data.get("target_id", "") or "").strip()
        created = False
        if target_id:
            if not _TARGET_ID_RE.fullmatch(target_id):
                raise ValueError(
                    "target_id is required and may contain letters, numbers, dots, dashes, and underscores only."
                )
        else:
            raise ValueError("target_id is required.")

        if target_id in self._targets:
            current = self._targets[target_id]
            candidate = current.to_store_dict()
            for key, value in data.items():
                if value is None:
                    continue
                if key == "ssh_pass" and value == "••••":
                    continue
                candidate[key] = value
            target = TargetRecord.from_dict(
                candidate,
                default_deploy_root=self.default_deploy_root,
                default_fleet_file=self.default_fleet_file,
            )
            self._targets[target_id] = target
        else:
            target = TargetRecord.from_dict(
                {
                    "target_id": target_id,
                    "label": data.get("label", target_id),
                    "pi_user": data.get(
                        "pi_user", self.operator_config.default_pi_user
                    ),
                    "pi_host": data.get("pi_host", ""),
                    "deploy_root": data.get("deploy_root", self.default_deploy_root),
                    "ssh_key": data.get(
                        "ssh_key", self.operator_config.default_ssh_key
                    ),
                    "ssh_pass": data.get(
                        "ssh_pass", self.operator_config.default_ssh_pass
                    ),
                    "fleet_file": data.get("fleet_file", self.default_fleet_file),
                    "vehicle_name": data.get("vehicle_name", ""),
                    "overlay_yaml": data.get("overlay_yaml", ""),
                    "service_unit": data.get(
                        "service_unit", "pennair-autonomy.service"
                    ),
                    "enabled": data.get("enabled", True),
                },
                default_deploy_root=self.default_deploy_root,
                default_fleet_file=self.default_fleet_file,
            )
            self._targets[target_id] = target
            created = True

        if len(self._targets) == 1:
            self._active_target_id = target.target_id
        self.save()
        return target, created

    def delete_target(self, target_id: str) -> None:
        if target_id not in self._targets:
            raise KeyError(f"Unknown target '{target_id}'.")
        if len(self._targets) == 1:
            raise ValueError("Cannot delete the last target.")
        del self._targets[target_id]
        if self._active_target_id == target_id:
            self._active_target_id = next(iter(sorted(self._targets)))
        self.save()

    def _apply_operator_store(self, operator: object) -> None:
        if not isinstance(operator, dict):
            return
        if operator.get("github_repo"):
            self.operator_config.github_repo = str(operator["github_repo"])
        if "github_token" in operator:
            self.operator_config.github_token = str(operator.get("github_token", ""))
        if operator.get("hotspot_name"):
            self.operator_config.hotspot_name = str(operator["hotspot_name"])
        if operator.get("default_deploy_root"):
            self.operator_config.default_deploy_root = str(
                operator["default_deploy_root"]
            )
        if operator.get("default_pi_user"):
            self.operator_config.default_pi_user = str(operator["default_pi_user"])
        if "default_ssh_key" in operator:
            self.operator_config.default_ssh_key = str(
                operator.get("default_ssh_key", "")
            )
        if "default_ssh_pass" in operator:
            self.operator_config.default_ssh_pass = str(
                operator.get("default_ssh_pass", "")
            )

    def _parse_targets(self, targets: object) -> dict[str, TargetRecord]:
        if not isinstance(targets, list):
            raise ValueError("Inventory file is invalid: 'targets' must be a list.")

        loaded: dict[str, TargetRecord] = {}
        for item in targets:
            if not isinstance(item, dict):
                raise ValueError(
                    "Inventory file is invalid: each target must be a mapping."
                )
            record = TargetRecord.from_dict(
                item,
                default_deploy_root=self.default_deploy_root,
                default_fleet_file=self.default_fleet_file,
            )
            loaded[record.target_id] = record
        return loaded


@dataclass(slots=True)
class BuildSourceRecord:
    kind: str = "none"
    github_source: str = ""
    tag: str = ""
    artifact_id: str = ""
    run_id: str = ""
    sha: str = ""
    name: str = ""
    date: str = ""
    download_url: str = ""
    size_mb: float | None = None
    branch: str = ""
    artifact_name: str = ""
    local_artifact_path: str = ""
    local_artifact_size_bytes: int | None = None
    codebase_root: str = ""
    updated_at: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, object] | None) -> "BuildSourceRecord":
        payload = data or {}
        kind = str(payload.get("kind", "none")).strip() or "none"
        if kind not in _BUILD_SOURCE_KINDS:
            raise ValueError(f"Unsupported build source kind '{kind}'.")

        record = cls(
            kind=kind,
            github_source=str(payload.get("github_source", "")).strip(),
            tag=str(payload.get("tag", "")).strip(),
            artifact_id=str(payload.get("artifact_id", "")).strip(),
            run_id=str(payload.get("run_id", "")).strip(),
            sha=str(payload.get("sha", "")).strip(),
            name=str(payload.get("name", "")).strip(),
            date=str(payload.get("date", "")).strip(),
            download_url=str(payload.get("download_url", "")).strip(),
            size_mb=(
                float(payload["size_mb"])
                if payload.get("size_mb") not in (None, "")
                else None
            ),
            branch=str(payload.get("branch", "")).strip(),
            artifact_name=str(payload.get("artifact_name", "")).strip(),
            local_artifact_path=str(payload.get("local_artifact_path", "")).strip(),
            local_artifact_size_bytes=(
                int(payload["local_artifact_size_bytes"])
                if payload.get("local_artifact_size_bytes") not in (None, "")
                else None
            ),
            codebase_root=str(payload.get("codebase_root", "")).strip(),
            updated_at=str(payload.get("updated_at", "")).strip(),
        )
        record.validate()
        return record

    def validate(self) -> None:
        if self.kind == "github":
            if self.github_source not in _GITHUB_BUILD_SOURCES:
                raise ValueError(
                    "GitHub build source requires github_source to be 'release' or 'actions'."
                )
            if self.github_source == "release" and not self.tag:
                raise ValueError("GitHub release source requires tag.")
            if self.github_source == "actions" and not self.artifact_id:
                raise ValueError("GitHub Actions source requires artifact_id.")
        elif self.kind == "local_artifact":
            if not self.artifact_name:
                raise ValueError("Local artifact source requires artifact_name.")
            if not self.local_artifact_path:
                raise ValueError("Local artifact source requires local_artifact_path.")
        elif self.kind == "local_codebase" and not self.codebase_root:
            raise ValueError("Local codebase source requires codebase_root.")

    def to_store_dict(self) -> dict[str, object]:
        return {
            "kind": self.kind,
            "github_source": self.github_source,
            "tag": self.tag,
            "artifact_id": self.artifact_id,
            "run_id": self.run_id,
            "sha": self.sha,
            "name": self.name,
            "date": self.date,
            "download_url": self.download_url,
            "size_mb": self.size_mb,
            "branch": self.branch,
            "artifact_name": self.artifact_name,
            "local_artifact_path": self.local_artifact_path,
            "local_artifact_size_bytes": self.local_artifact_size_bytes,
            "codebase_root": self.codebase_root,
            "updated_at": self.updated_at,
        }

    def summary(self) -> str:
        if self.kind == "github":
            if self.github_source == "release":
                return self.name or self.tag or "GitHub release"
            label = self.name or self.artifact_name or self.artifact_id or "artifact"
            return f"GitHub Actions {label}"
        if self.kind == "local_artifact":
            return self.artifact_name or "Local artifact"
        if self.kind == "local_codebase":
            return self.codebase_root or "Local codebase"
        return "No build source selected"

    def to_payload_dict(self) -> dict[str, object]:
        local_exists = None
        if self.kind == "local_artifact":
            local_exists = Path(self.local_artifact_path).exists()
        return {
            "kind": self.kind,
            "summary": self.summary(),
            "github_source": self.github_source or None,
            "tag": self.tag or None,
            "artifact_id": self.artifact_id or None,
            "run_id": self.run_id or None,
            "sha": self.sha or None,
            "name": self.name or None,
            "date": self.date or None,
            "download_url": self.download_url or None,
            "size_mb": self.size_mb,
            "branch": self.branch or None,
            "artifact_name": self.artifact_name or None,
            "local_artifact_path": self.local_artifact_path or None,
            "local_artifact_exists": local_exists,
            "local_artifact_size_bytes": self.local_artifact_size_bytes,
            "codebase_root": self.codebase_root or None,
            "updated_at": self.updated_at or None,
        }


class BuildSourceStore:
    def __init__(self, path: Path, *, cache_dir: Path):
        self.path = path
        self.cache_dir = cache_dir
        self._current = BuildSourceRecord()
        self._load()

    def _load(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        if not self.path.exists():
            self.save()
            return

        with self.path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle) or {}

        try:
            self._current = BuildSourceRecord.from_dict(payload)
        except ValueError:
            self._current = BuildSourceRecord()
            self.save()

    def save(self) -> None:
        with self.path.open("w", encoding="utf-8") as handle:
            json.dump(self._current.to_store_dict(), handle, indent=2)
            handle.write("\n")

    def current(self) -> BuildSourceRecord:
        return self._current

    def _replace(self, record: BuildSourceRecord) -> BuildSourceRecord:
        previous = self._current
        self._current = record
        self.save()
        if previous.kind == "local_artifact":
            self._remove_cached_artifact(previous)
        return self._current

    def _remove_cached_artifact(self, record: BuildSourceRecord | None = None) -> None:
        source = record or self._current
        if source.kind != "local_artifact" or not source.local_artifact_path:
            return
        try:
            artifact_path = Path(source.local_artifact_path)
            if artifact_path.exists():
                artifact_path.unlink()
        except Exception:
            pass

    def set_github(
        self,
        *,
        github_source: str,
        tag: str = "",
        artifact_id: str = "",
        run_id: str = "",
        sha: str = "",
        name: str = "",
        date: str = "",
        download_url: str = "",
        size_mb: float | None = None,
        branch: str = "",
        artifact_name: str = "",
    ) -> BuildSourceRecord:
        record = BuildSourceRecord(
            kind="github",
            github_source=github_source.strip(),
            tag=tag.strip(),
            artifact_id=artifact_id.strip(),
            run_id=run_id.strip(),
            sha=sha.strip(),
            name=name.strip(),
            date=date.strip(),
            download_url=download_url.strip(),
            size_mb=size_mb,
            branch=branch.strip(),
            artifact_name=artifact_name.strip(),
            updated_at=_utc_now(),
        )
        record.validate()
        return self._replace(record)

    def set_local_artifact(
        self, *, artifact_name: str, file_bytes: bytes
    ) -> BuildSourceRecord:
        cached_name = f"{datetime.now(timezone.utc):%Y%m%d-%H%M%S}-{artifact_name}"
        cached_path = self.cache_dir / cached_name
        cached_path.write_bytes(file_bytes)
        record = BuildSourceRecord(
            kind="local_artifact",
            artifact_name=artifact_name,
            local_artifact_path=str(cached_path),
            local_artifact_size_bytes=len(file_bytes),
            updated_at=_utc_now(),
        )
        record.validate()
        return self._replace(record)

    def set_local_codebase(self, *, codebase_root: str) -> BuildSourceRecord:
        record = BuildSourceRecord(
            kind="local_codebase",
            codebase_root=codebase_root.strip(),
            updated_at=_utc_now(),
        )
        record.validate()
        return self._replace(record)

    def clear(self) -> BuildSourceRecord:
        previous = self._current
        self._current = BuildSourceRecord(updated_at=_utc_now())
        self.save()
        if previous.kind == "local_artifact":
            self._remove_cached_artifact(previous)
        return self._current


def build_source_store_paths(inventory_path: Path) -> tuple[Path, Path]:
    parent = inventory_path.parent
    return (
        parent / ".integration_build_source.json",
        parent / ".integration_build_source_cache",
    )


def _utc_now() -> str:
    return (
        datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace("+00:00", "Z")
    )


def _detect_github_repo(base_dir: Path) -> str:
    try:
        result = subprocess.run(
            ["git", "remote", "get-url", "origin"],
            capture_output=True,
            text=True,
            cwd=base_dir,
            timeout=3,
        )
        match = re.search(r"github\.com[:/]([^/]+/[^/.]+)", result.stdout)
        if match:
            return match.group(1).strip()
    except Exception:
        pass
    return ""


def default_fleet_file(base_dir: Path) -> str:
    candidate = (
        base_dir.parent / "uav" / "uav" / "fleets" / "example_fleet.yaml"
    ).resolve()
    return str(candidate)


def seed_target_from_env(base_dir: Path, operator: OperatorConfig) -> TargetRecord:
    return TargetRecord.from_dict(
        {
            "target_id": os.environ.get("TARGET_ID", "default"),
            "label": os.environ.get("TARGET_LABEL", "Default Pi"),
            "pi_user": os.environ.get("PI_USER", "penn"),
            "pi_host": os.environ.get("PI_HOST", "penn-desktop.local"),
            "deploy_root": os.environ.get("DEPLOY_ROOT", operator.default_deploy_root),
            "ssh_key": os.environ.get("SSH_KEY", operator.default_ssh_key),
            "ssh_pass": os.environ.get("SSH_PASS", operator.default_ssh_pass),
            "fleet_file": os.environ.get("FLEET_FILE", default_fleet_file(base_dir)),
            "vehicle_name": os.environ.get("VEHICLE_NAME", "uav"),
            "overlay_yaml": os.environ.get(
                "TARGET_OVERLAY_YAML",
                "mission: basic\nauto_launch: false\npx4_airframe_id: 4004\npayload_controller: GPIOController\n",
            ),
            "service_unit": os.environ.get(
                "TARGET_SYSTEMD_UNIT", "pennair-autonomy.service"
            ),
            "enabled": True,
        },
        default_deploy_root=operator.default_deploy_root,
        default_fleet_file=default_fleet_file(base_dir),
    )
