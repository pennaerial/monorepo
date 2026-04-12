from __future__ import annotations

import hashlib
from datetime import datetime, timezone
import os
import re
import shutil
import subprocess
import tarfile
import tempfile
from time import monotonic
import xml.etree.ElementTree as ET
import zipfile
from copy import deepcopy
from pathlib import Path

import yaml

from ..context import AppContext, TargetContext
from uav.runtime.mission_spec import MissionSpec

_ARTIFACT_NAME_RE = re.compile(r"^[A-Za-z0-9._-]+$")
_SOURCE_BUNDLE_EXTENSIONS = (".tar.gz", ".tgz", ".zip")
_RELEASE_METADATA_NAME = "RELEASE_METADATA.yaml"
_LOCAL_SOURCE_BASE_PACKAGES = [
    "actuator_msgs",
    "payload_interfaces",
    "px4_msgs",
    "uav_interfaces",
    "uav",
]
_SHARED_OVERLAY_KEYS = {
    "kind",
    "mission",
    "mission_path",
    "auto_launch",
    "debug",
    "vision_debug",
    "save_vision_milliseconds",
    "servo_only",
    "camera_mount_offsets",
    "camera_input_transport",
    "camera_rotate_degrees",
    "camera_preprocess_hook",
}
_UAV_OVERLAY_KEYS = _SHARED_OVERLAY_KEYS | {"px4_airframe_id", "px4_namespace"}
_PAYLOAD_OVERLAY_KEYS = _SHARED_OVERLAY_KEYS | {"payload_controller"}
_BUILD_LIST_CACHE_TTL_SECONDS = 60.0
_COMMIT_SUBJECT_CACHE_TTL_SECONDS = 1800.0
_BUILD_LIST_CACHE: dict[tuple[object, ...], tuple[float, dict[str, object]]] = {}
_COMMIT_SUBJECT_CACHE: dict[str, tuple[float, str | None]] = {}


def _selected_fleet_file(ctx: AppContext) -> str:
    return str(ctx.build_source_store.current().fleet_file or "").strip()


def _source_catalog_key(current) -> str:
    payload = "|".join(
        str(value)
        for value in (
            current.kind,
            current.github_source,
            current.tag,
            current.artifact_id,
            current.run_id,
            current.sha,
            current.download_url,
            current.name,
            current.artifact_name,
            current.local_artifact_path,
            current.codebase_root,
            current.updated_at,
        )
    )
    return hashlib.sha1(payload.encode("utf-8")).hexdigest()[:12]


def _catalog_archive_extension(current) -> str:
    if current.kind == "github" and current.github_source == "actions":
        return ".zip"
    return ".tar.gz"


def _build_archive_url(ctx: AppContext) -> str | None:
    current = ctx.build_source_store.current()
    if current.kind != "github":
        return None
    if current.download_url:
        return current.download_url
    repo = ctx.operator_config.github_repo
    if not repo:
        return None
    if current.github_source == "actions" and current.artifact_id:
        return (
            f"https://api.github.com/repos/{repo}/actions/artifacts/"
            f"{current.artifact_id}/zip"
        )
    if current.github_source == "release" and current.tag and current.artifact_name:
        return f"https://github.com/{repo}/releases/download/{current.tag}/{current.artifact_name}"
    return None


def _fleet_catalog_dir(ctx: AppContext) -> Path:
    current = ctx.build_source_store.current()
    return ctx.build_source_store.cache_dir / f"fleet-catalog-{_source_catalog_key(current)}"


def _catalog_yaml_paths(root: Path) -> list[Path]:
    if not root.exists():
        return []
    paths = {
        path.resolve()
        for pattern in ("*.yaml", "*.yml")
        for path in root.rglob(pattern)
        if "fleets" in {part.lower() for part in path.parts}
    }
    return sorted(paths)


def _path_endswith_parts(path: Path, suffix_parts: tuple[str, ...]) -> bool:
    parts = tuple(part.lower() for part in path.parts)
    return len(parts) >= len(suffix_parts) and parts[-len(suffix_parts) :] == suffix_parts


def _fleet_catalog_candidate_key(path: Path) -> tuple[int, int, int, str]:
    fleet_file = path.name.lower()
    if _path_endswith_parts(
        path, ("install", "uav", "share", "uav", "fleets", fleet_file)
    ):
        rank = 0
    elif _path_endswith_parts(path, ("src", "uav", "uav", "fleets", fleet_file)):
        rank = 1
    elif _path_endswith_parts(path, ("uav", "fleets", fleet_file)):
        rank = 2
    else:
        rank = 3
    nested_penalty = sum(1 for part in path.parts if part.lower() == "__nested__")
    return (rank, nested_penalty, len(path.parts), str(path))


def _fleet_catalog_lookup_from_paths(paths: list[Path]) -> dict[str, Path]:
    candidates: dict[str, set[Path]] = {}
    for path in paths:
        fleet_file = path.name.strip()
        if not fleet_file:
            continue
        # Real bundles can mirror the same fleet into both install/ and src/
        # trees. Collapse those copies to one operator-facing filename.
        candidates.setdefault(fleet_file, set()).add(path.resolve())
    catalog: dict[str, Path] = {}
    for fleet_file, fleet_paths in candidates.items():
        catalog[fleet_file] = sorted(fleet_paths, key=_fleet_catalog_candidate_key)[0]
    return {fleet_file: catalog[fleet_file] for fleet_file in sorted(catalog)}


def _safe_extract_path(base_dir: Path, member_name: str) -> Path | None:
    candidate = (base_dir / member_name).resolve()
    base_resolved = base_dir.resolve()
    try:
        candidate.relative_to(base_resolved)
    except ValueError:
        return None
    return candidate


def _archive_member_names(archive_path: Path) -> list[str]:
    if archive_path.name.endswith(".zip"):
        with zipfile.ZipFile(archive_path) as archive:
            return archive.namelist()
    if archive_path.name.endswith(".tar.gz") or archive_path.name.endswith(".tgz"):
        with tarfile.open(archive_path, "r:gz") as archive:
            return [member.name for member in archive.getmembers() if member.isfile()]
    raise ValueError(
        f"Unsupported source bundle '{archive_path.name}'. Use .tar.gz, .tgz, or .zip."
    )


def _is_fleet_yaml_member(member_name: str) -> bool:
    normalized = member_name.replace("\\", "/").lower()
    return "/fleets/" in normalized and normalized.endswith((".yaml", ".yml"))


def _is_supported_archive_member(member_name: str) -> bool:
    normalized = member_name.replace("\\", "/").lower()
    return normalized.endswith(_SOURCE_BUNDLE_EXTENSIONS)


def _nested_archive_destination(extract_dir: Path, member_name: str) -> Path:
    base_name = Path(member_name.replace("\\", "/")).name or "nested-archive"
    digest = hashlib.sha1(member_name.encode("utf-8")).hexdigest()[:8]
    nested_dir = extract_dir / "__nested__"
    nested_dir.mkdir(parents=True, exist_ok=True)
    return nested_dir / f"{digest}-{base_name}"


def _extract_catalog_from_archive(
    archive_path: Path, extract_dir: Path, *, _depth: int = 0
) -> list[Path]:
    extract_dir.mkdir(parents=True, exist_ok=True)
    member_names = _archive_member_names(archive_path)
    catalog_paths: list[Path] = []

    if archive_path.name.endswith(".zip"):
        with zipfile.ZipFile(archive_path) as archive:
            for member_name in member_names:
                if not _is_fleet_yaml_member(member_name):
                    continue
                destination = _safe_extract_path(extract_dir, member_name)
                if destination is None:
                    continue
                destination.parent.mkdir(parents=True, exist_ok=True)
                archive.extract(member_name, extract_dir)
                catalog_paths.append(destination)
            if not catalog_paths and _depth == 0:
                for member_name in member_names:
                    if not _is_supported_archive_member(member_name):
                        continue
                    nested_archive_path = _nested_archive_destination(
                        extract_dir, member_name
                    )
                    with archive.open(member_name) as nested_archive_stream:
                        with nested_archive_path.open("wb") as nested_archive_file:
                            shutil.copyfileobj(
                                nested_archive_stream, nested_archive_file
                            )
                    nested_extract_dir = nested_archive_path.parent / (
                        f"{nested_archive_path.name}.extract"
                    )
                    catalog_paths.extend(
                        _extract_catalog_from_archive(
                            nested_archive_path,
                            nested_extract_dir,
                            _depth=_depth + 1,
                        )
                    )
    else:
        with tarfile.open(archive_path, "r:gz") as archive:
            for member in archive.getmembers():
                if not member.isfile() or not _is_fleet_yaml_member(member.name):
                    continue
                destination = _safe_extract_path(extract_dir, member.name)
                if destination is None:
                    continue
                destination.parent.mkdir(parents=True, exist_ok=True)
                archive.extract(member, extract_dir)
                catalog_paths.append(destination)

    return sorted({path.resolve() for path in catalog_paths})


def _fleet_catalog_lookup_from_local_codebase(
    ctx: AppContext,
) -> tuple[dict[str, Path], str | None]:
    root = _repo_root(ctx) / "src" / "uav" / "uav" / "fleets"
    if not root.exists():
        return {}, str(root)
    paths = _catalog_yaml_paths(root)
    return _fleet_catalog_lookup_from_paths(paths), str(root)


def _fleet_catalog_lookup_from_current_source(
    ctx: AppContext,
) -> tuple[dict[str, Path], str | None, str | None]:
    current = ctx.build_source_store.current()
    if current.kind == "none":
        return {}, None, None

    if current.kind == "local_codebase":
        try:
            catalog, catalog_source = _fleet_catalog_lookup_from_local_codebase(ctx)
            return catalog, catalog_source, None
        except Exception as exc:
            return {}, "local_codebase", str(exc)

    if current.kind == "local_artifact":
        artifact_path = Path(current.local_artifact_path)
        if not artifact_path.exists():
            return {}, str(artifact_path), "Selected local artifact no longer exists."
        catalog_dir = _fleet_catalog_dir(ctx)
        extract_dir = catalog_dir / "extract"
        try:
            if not extract_dir.exists():
                _extract_catalog_from_archive(artifact_path, extract_dir)
            catalog = _fleet_catalog_lookup_from_paths(_catalog_yaml_paths(extract_dir))
            return catalog, str(artifact_path), None
        except Exception as exc:
            return {}, str(artifact_path), str(exc)

    archive_url = _build_archive_url(ctx)
    if not archive_url:
        return (
            {},
            "github",
            "Selected GitHub build source does not expose a download URL.",
        )

    catalog_dir = _fleet_catalog_dir(ctx)
    extract_dir = catalog_dir / "extract"
    if extract_dir.exists():
        try:
            catalog = _fleet_catalog_lookup_from_paths(_catalog_yaml_paths(extract_dir))
            if catalog:
                return catalog, archive_url, None
        except Exception as exc:
            return {}, archive_url, str(exc)

    httpx = _require_httpx()
    archive_ext = _catalog_archive_extension(current)
    archive_path = catalog_dir / f"archive{archive_ext}"
    catalog_dir.mkdir(parents=True, exist_ok=True)
    try:
        with httpx.Client(timeout=300, follow_redirects=True) as client:
            with client.stream("GET", archive_url, headers=_github_headers(ctx)) as resp:
                resp.raise_for_status()
                with archive_path.open("wb") as out:
                    for chunk in resp.iter_bytes():
                        out.write(chunk)
        _extract_catalog_from_archive(archive_path, extract_dir)
        catalog = _fleet_catalog_lookup_from_paths(_catalog_yaml_paths(extract_dir))
        return catalog, archive_url, None
    except Exception as exc:
        return {}, archive_url, str(exc)


def _fleet_catalog_from_current_source(
    ctx: AppContext,
) -> tuple[list[str], str | None, str | None]:
    catalog, catalog_source, catalog_error = _fleet_catalog_lookup_from_current_source(
        ctx
    )
    return sorted(catalog), catalog_source, catalog_error


def _resolve_catalog_fleet_path(ctx: AppContext, fleet_file: str) -> tuple[str, Path]:
    selected = Path(str(fleet_file or "").strip().replace("\\", "/")).name.strip()
    if not selected:
        raise ValueError("No fleet file selected.")

    current_kind = ctx.build_source_store.current().kind
    if current_kind == "none":
        raise ValueError("Select a build source before selecting a fleet.")

    catalog, _, catalog_error = _fleet_catalog_lookup_from_current_source(ctx)
    if catalog_error:
        raise ValueError(catalog_error)
    if not catalog:
        raise ValueError("The selected build source does not expose any fleet YAML files.")

    fleet_path = catalog.get(selected)
    if fleet_path is None:
        raise ValueError(f"Fleet file not found: {selected}")

    return selected, fleet_path


def _reconcile_selected_fleet_with_current_source(
    ctx: AppContext, previous_fleet_file: str
) -> None:
    selected = Path(str(previous_fleet_file or "").strip().replace("\\", "/")).name.strip()
    if not selected:
        if _selected_fleet_file(ctx):
            ctx.build_source_store.set_fleet_file(fleet_file="")
        return

    catalog, _, catalog_error = _fleet_catalog_from_current_source(ctx)
    if catalog_error or selected not in set(catalog):
        ctx.build_source_store.set_fleet_file(fleet_file="")
        return

    if _selected_fleet_file(ctx) != selected:
        ctx.build_source_store.set_fleet_file(fleet_file=selected)


def _require_httpx():
    try:
        import httpx
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Missing Python dependency `httpx`. "
            "If using conda/pip, run: pip install httpx. "
            "If using uv, rerun with `uv run app.py`."
        ) from exc
    return httpx


def _github_headers(ctx: AppContext) -> dict:
    headers = {"Accept": "application/vnd.github+json"}
    if ctx.operator_config.github_token:
        headers["Authorization"] = f"Bearer {ctx.operator_config.github_token}"
    return headers


def _build_list_cache_key(
    ctx: AppContext,
    *,
    artifact_page: int,
    artifact_page_size: int,
    q: str,
    sha: str,
    commit_subject: str,
    branch: str,
    include_fallback: bool,
) -> tuple[object, ...]:
    return (
        ctx.operator_config.github_repo,
        artifact_page,
        artifact_page_size,
        q.strip(),
        sha.strip(),
        commit_subject.strip(),
        branch.strip(),
        include_fallback,
    )


def _build_list_cache_get(
    cache_key: tuple[object, ...], *, allow_stale: bool = False
) -> dict[str, object] | None:
    cached = _BUILD_LIST_CACHE.get(cache_key)
    if cached is None:
        return None
    cached_at, payload = cached
    age = monotonic() - cached_at
    if not allow_stale and age > _BUILD_LIST_CACHE_TTL_SECONDS:
        return None
    return deepcopy(payload)


def _build_list_cache_set(
    cache_key: tuple[object, ...], payload: dict[str, object]
) -> dict[str, object]:
    cached_payload = deepcopy(payload)
    _BUILD_LIST_CACHE[cache_key] = (monotonic(), cached_payload)
    return deepcopy(cached_payload)


def _commit_subject_cache_get(commit_sha: str) -> str | None | object:
    cached = _COMMIT_SUBJECT_CACHE.get(commit_sha)
    if cached is None:
        return ...
    cached_at, subject = cached
    if monotonic() - cached_at > _COMMIT_SUBJECT_CACHE_TTL_SECONDS:
        _COMMIT_SUBJECT_CACHE.pop(commit_sha, None)
        return ...
    return subject


def _commit_subject_cache_set(commit_sha: str, subject: str | None) -> None:
    _COMMIT_SUBJECT_CACHE[commit_sha] = (monotonic(), subject)


def _github_error_message(
    exc: Exception,
    *,
    token_configured: bool,
    using_cached_response: bool = False,
    during_commit_lookup: bool = False,
) -> str | None:
    response = getattr(exc, "response", None)
    status_code = getattr(response, "status_code", None)
    if status_code != 403:
        return None

    message = ""
    headers = getattr(response, "headers", {}) or {}
    try:
        payload = response.json() or {}
        if isinstance(payload, dict):
            message = str(payload.get("message", "")).strip()
    except Exception:
        message = str(getattr(response, "text", "") or "").strip()

    lowered = message.lower()
    if "rate limit" not in lowered:
        return None

    retry_hint = ""
    retry_after = str(headers.get("retry-after", "")).strip()
    if retry_after.isdigit():
        retry_hint = f" Retry after about {retry_after} seconds."
    else:
        reset_at = str(headers.get("x-ratelimit-reset", "")).strip()
        if reset_at.isdigit():
            reset_time = datetime.fromtimestamp(int(reset_at), tz=timezone.utc)
            retry_hint = f" GitHub resets at {reset_time:%Y-%m-%d %H:%M:%S UTC}."

    auth_hint = (
        " Configure `GITHUB_TOKEN` in Operator Defaults under the Inventory tab or the backend environment for higher limits."
        if not token_configured
        else ""
    )

    if during_commit_lookup:
        return (
            "GitHub commit-title lookups were rate-limited. "
            "Showing available builds with fallback titles."
            f"{retry_hint}{auth_hint}"
        ).strip()

    if using_cached_response:
        return (
            "GitHub API rate limit exceeded. Showing a cached build list."
            f"{retry_hint}{auth_hint}"
        ).strip()

    return (
        "GitHub API rate limit exceeded while loading deployable builds."
        f"{retry_hint}{auth_hint}"
    ).strip()


def sanitize_artifact_name(filename: str) -> str:
    name = Path(filename or "").name
    if not name:
        raise ValueError("Artifact filename is empty.")
    if not _ARTIFACT_NAME_RE.match(name):
        raise ValueError(
            "Artifact filename contains unsupported characters. "
            "Use letters, numbers, dots, underscores, and dashes only."
        )
    return name


def sanitize_source_bundle_name(filename: str) -> str:
    name = sanitize_artifact_name(filename)
    if not name.endswith(_SOURCE_BUNDLE_EXTENSIONS):
        raise ValueError("Source bundle must be one of: .tar.gz, .tgz, or .zip.")
    return name


def _build_source_payload(ctx: AppContext) -> dict[str, object]:
    current = ctx.build_source_store.current()
    payload = current.to_payload_dict()
    fleet_file = _selected_fleet_file(ctx) if current.kind != "none" else ""
    payload["fleet_file"] = fleet_file or None
    payload.update(_fleet_preview(ctx, fleet_file))
    try:
        available_fleets, fleet_catalog_source, fleet_catalog_error = (
            _fleet_catalog_from_current_source(ctx)
        )
    except Exception as exc:
        available_fleets, fleet_catalog_source, fleet_catalog_error = (
            [],
            None,
            str(exc),
        )
    payload["available_fleets"] = available_fleets
    payload["fleet_catalog_source"] = fleet_catalog_source
    payload["fleet_catalog_error"] = fleet_catalog_error
    return payload


def _build_source_response(
    ctx: AppContext,
    *,
    success: bool,
    output: str | None = None,
    error: str | None = None,
) -> dict[str, object]:
    return {
        "success": success,
        "source": _build_source_payload(ctx),
        "output": output,
        "error": error,
    }


def _repo_root(ctx: AppContext) -> Path:
    return ctx.base_dir.parent.parent.resolve()


def _missions_root(ctx: AppContext) -> Path:
    return _repo_root(ctx) / "src" / "uav" / "uav" / "missions"


def _resolve_local_path(ctx: AppContext, raw_path: str) -> Path:
    candidate = Path(os.path.expanduser(raw_path))
    if candidate.is_absolute():
        return candidate.resolve()
    repo_root = _repo_root(ctx)
    if (repo_root / candidate).exists():
        return (repo_root / candidate).resolve()
    return (ctx.base_dir / candidate).resolve()


def _load_local_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle) or {}
    if not isinstance(payload, dict):
        raise ValueError(f"YAML file '{path}' must define a mapping.")
    return payload


def _load_selected_fleet(ctx: AppContext) -> tuple[str, Path, dict, dict[str, object]]:
    fleet_file = _selected_fleet_file(ctx)
    if not fleet_file:
        raise ValueError("No fleet file selected.")

    fleet_file, fleet_path = _resolve_catalog_fleet_path(ctx, fleet_file)

    shared_fleet = _load_local_yaml(fleet_path)
    defaults = deepcopy(shared_fleet.get("defaults", {})) or {}
    if not isinstance(defaults, dict):
        raise ValueError("Fleet defaults must be a mapping.")

    vehicles = shared_fleet.get("vehicles", [])
    if not isinstance(vehicles, list):
        raise ValueError("Fleet vehicles must be a list.")

    return fleet_file, fleet_path, shared_fleet, defaults


def _runtime_vehicle_from_config(
    ctx: AppContext,
    *,
    merged_vehicle: dict[str, object],
    vehicle_name: str,
    mission_path_override: str | None = None,
) -> tuple[dict[str, object], Path, str]:
    mission_name, local_mission_path, mission_spec = _mission_source_for(
        ctx, merged_vehicle
    )
    mission_target = mission_spec.target
    runtime_vehicle = {
        "name": vehicle_name,
        "kind": mission_target,
        "mission": mission_name,
        "mission_path": mission_path_override or str(local_mission_path),
        "auto_launch": _bool_value(merged_vehicle.get("auto_launch"), default=False),
        "debug": _bool_value(merged_vehicle.get("debug"), default=False),
        "vision_debug": _bool_value(
            merged_vehicle.get("vision_debug"), default=False
        ),
        "save_vision_milliseconds": int(
            merged_vehicle.get("save_vision_milliseconds", 0) or 0
        ),
        "servo_only": _bool_value(merged_vehicle.get("servo_only"), default=False),
        "camera_mount_offsets": _normalized_camera_offsets(
            merged_vehicle.get("camera_mount_offsets")
        ),
        "camera_input_transport": str(
            merged_vehicle.get("camera_input_transport", "compressed")
        ).strip(),
        "camera_rotate_degrees": float(
            merged_vehicle.get(
                "camera_rotate_degrees", 180.0 if mission_target == "payload" else 0.0
            )
            or 0.0
        ),
        "camera_preprocess_hook": str(
            merged_vehicle.get("camera_preprocess_hook", "")
        ).strip(),
    }

    if mission_target == "uav":
        px4_airframe_id = merged_vehicle.get("px4_airframe_id")
        runtime_vehicle["px4_airframe_id"] = (
            int(px4_airframe_id) if px4_airframe_id is not None else None
        )
        runtime_vehicle["px4_namespace"] = (
            str(merged_vehicle.get("px4_namespace", vehicle_name)).strip()
            or vehicle_name
        )
    else:
        runtime_vehicle["payload_controller"] = (
            str(merged_vehicle.get("payload_controller", "GPIOController")).strip()
            or "GPIOController"
        )

    _validate_runtime_vehicle(runtime_vehicle)
    return runtime_vehicle, local_mission_path, mission_target


def _fleet_preview(ctx: AppContext, fleet_file: str) -> dict[str, object]:
    preview = {
        "fleet_exists": False,
        "fleet_error": None,
        "fleet_vehicles": [],
    }
    raw = str(fleet_file or "").strip()
    if not raw:
        preview["fleet_error"] = "No fleet file selected."
        return preview

    try:
        _, _, shared_fleet, defaults = _load_selected_fleet(ctx)
        vehicles = shared_fleet.get("vehicles", [])
        preview["fleet_exists"] = True
        fleet_vehicles: list[dict[str, object]] = []
        for vehicle in vehicles:
            if not isinstance(vehicle, dict):
                continue
            merged = deepcopy(defaults)
            merged.update(vehicle)
            try:
                runtime_vehicle, local_mission_path, _ = _runtime_vehicle_from_config(
                    ctx,
                    merged_vehicle=merged,
                    vehicle_name=str(vehicle.get("name", "")).strip(),
                )
            except Exception:
                runtime_vehicle = {
                    "name": str(vehicle.get("name", "")).strip(),
                    "kind": str(merged.get("kind", "")).strip() or None,
                    "mission": str(merged.get("mission", "")).strip() or None,
                    "mission_path": str(merged.get("mission_path", "")).strip()
                    or None,
                }
                local_mission_path = None
            fleet_vehicles.append(
                {
                    "name": runtime_vehicle["name"],
                    "kind": runtime_vehicle.get("kind"),
                    "mission": runtime_vehicle.get("mission"),
                    "mission_path": str(local_mission_path)
                    if local_mission_path is not None
                    else runtime_vehicle.get("mission_path"),
                    "auto_launch": runtime_vehicle.get("auto_launch"),
                    "debug": runtime_vehicle.get("debug"),
                    "vision_debug": runtime_vehicle.get("vision_debug"),
                    "save_vision_milliseconds": runtime_vehicle.get(
                        "save_vision_milliseconds"
                    ),
                    "servo_only": runtime_vehicle.get("servo_only"),
                    "camera_mount_offsets": runtime_vehicle.get(
                        "camera_mount_offsets", []
                    ),
                    "camera_input_transport": runtime_vehicle.get(
                        "camera_input_transport"
                    ),
                    "camera_rotate_degrees": runtime_vehicle.get(
                        "camera_rotate_degrees"
                    ),
                    "camera_preprocess_hook": runtime_vehicle.get(
                        "camera_preprocess_hook"
                    ),
                    "px4_airframe_id": runtime_vehicle.get("px4_airframe_id"),
                    "px4_namespace": runtime_vehicle.get("px4_namespace"),
                    "payload_controller": runtime_vehicle.get("payload_controller"),
                }
            )
        preview["fleet_vehicles"] = fleet_vehicles
        return preview
    except Exception as exc:
        preview["fleet_error"] = str(exc)
        return preview


def _vehicle_from_fleet(fleet: dict, vehicle_name: str) -> dict:
    vehicles = fleet.get("vehicles", [])
    if not isinstance(vehicles, list):
        raise ValueError("Fleet file vehicles must be a list.")
    for vehicle in vehicles:
        if (
            isinstance(vehicle, dict)
            and str(vehicle.get("name", "")).strip() == vehicle_name
        ):
            return deepcopy(vehicle)
    raise ValueError(f"Fleet file does not define a vehicle named '{vehicle_name}'.")


def _mission_source_for(
    ctx: AppContext, vehicle: dict
) -> tuple[str, Path, MissionSpec]:
    mission_path = str(vehicle.get("mission_path", "")).strip()
    if mission_path:
        local_path = _resolve_local_path(ctx, mission_path)
        mission_name = Path(local_path).stem
        return mission_name, local_path, MissionSpec.load(local_path)

    mission_name = str(vehicle.get("mission", "")).strip()
    if not mission_name:
        raise ValueError("Selected vehicle is missing mission or mission_path.")
    local_path = (_missions_root(ctx) / f"{mission_name}.yaml").resolve()
    if not local_path.exists():
        raise ValueError(f"Mission '{mission_name}' was not found at '{local_path}'.")
    return mission_name, local_path, MissionSpec.load(local_path)


def _normalized_camera_offsets(value: object) -> list[float]:
    if isinstance(value, list) and len(value) == 3:
        return [float(component) for component in value]
    return [0.0, 0.0, 0.0]


def _bool_value(value: object, *, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _validate_overlay_data(
    target_ctx: TargetContext,
    overlay: dict[str, object],
    *,
    mission_target: str,
) -> None:
    allowed = _UAV_OVERLAY_KEYS if mission_target == "uav" else _PAYLOAD_OVERLAY_KEYS
    unknown = sorted(set(overlay) - allowed)
    if unknown:
        raise ValueError(
            f"Target '{target_ctx.target.target_id}' overlay has unsupported fields: {unknown}."
        )

    declared_kind = str(overlay.get("kind", "")).strip()
    if declared_kind and declared_kind != mission_target:
        raise ValueError(
            f"Target '{target_ctx.target.target_id}' overlay declares kind '{declared_kind}', "
            f"but mission target is '{mission_target}'."
        )

    if mission_target == "uav" and overlay.get("px4_airframe_id") is None:
        raise ValueError(
            f"Target '{target_ctx.target.target_id}' must define px4_airframe_id in its overlay for UAV hardware launch."
        )


def _validate_runtime_vehicle(runtime_vehicle: dict[str, object]) -> None:
    name = str(runtime_vehicle.get("name", "")).strip() or "<unnamed>"
    mission_target = str(runtime_vehicle.get("kind", "")).strip()
    if mission_target not in {"uav", "payload"}:
        raise ValueError(
            f"Runtime vehicle '{name}' has unsupported kind '{mission_target}'."
        )
    if not str(runtime_vehicle.get("mission_path", "")).strip():
        raise ValueError(f"Runtime vehicle '{name}' requires a mission_path.")
    if not isinstance(runtime_vehicle.get("camera_mount_offsets"), list):
        raise ValueError(
            f"Runtime vehicle '{name}' camera_mount_offsets must be a 3-element list."
        )
    if mission_target == "uav" and runtime_vehicle.get("px4_airframe_id") is None:
        raise ValueError(
            f"Runtime vehicle '{name}' requires px4_airframe_id for UAV hardware launch."
        )
    if (
        mission_target == "payload"
        and not str(runtime_vehicle.get("payload_controller", "")).strip()
    ):
        raise ValueError(
            f"Runtime vehicle '{name}' requires payload_controller for payload hardware launch."
        )


def _release_metadata_payload(
    target_ctx: TargetContext,
    *,
    release_id: str,
    source_type: str,
    source_label: str,
    fleet_file: str,
    package_names: list[str] | None = None,
) -> dict[str, object]:
    payload: dict[str, object] = {
        "release_id": release_id,
        "source_type": source_type,
        "source_label": source_label,
        "target_id": target_ctx.target.target_id,
        "vehicle_name": target_ctx.target.vehicle_name,
        "fleet_file": fleet_file,
        "deployed_at_utc": datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace("+00:00", "Z"),
    }
    if package_names:
        payload["packages"] = list(package_names)
    return payload


def _release_metadata_summary(metadata: dict[str, object]) -> str:
    rows = []
    for label, key in (
        ("Release", "release_id"),
        ("Source", "source_type"),
        ("Source label", "source_label"),
        ("Target", "target_id"),
        ("Vehicle", "vehicle_name"),
        ("Deployed", "deployed_at_utc"),
    ):
        value = metadata.get(key)
        if value:
            rows.append(f"{label}: {value}")
    packages = metadata.get("packages")
    if isinstance(packages, list) and packages:
        rows.append(f"Packages: {', '.join(str(pkg) for pkg in packages)}")
    return "\n".join(rows)


def _extract_archive_local(archive_path: Path, destination: Path) -> None:
    if archive_path.name.endswith(".zip"):
        with zipfile.ZipFile(archive_path) as bundle:
            bundle.extractall(destination)
        return
    if archive_path.name.endswith(".tar.gz") or archive_path.name.endswith(".tgz"):
        with tarfile.open(archive_path, "r:gz") as bundle:
            bundle.extractall(destination)
        return
    raise ValueError(
        f"Unsupported source bundle '{archive_path.name}'. Use .tar.gz, .tgz, or .zip."
    )


def _package_name_for(package_dir: Path) -> str:
    root = ET.parse(package_dir / "package.xml").getroot()
    name = root.findtext("name", "").strip()
    if not name:
        raise ValueError(f"Package '{package_dir}' is missing a <name> in package.xml.")
    return name


def _normalize_source_bundle(
    bundle_path: str, bundle_name: str
) -> tuple[str, list[str], str]:
    sanitized_name = sanitize_source_bundle_name(bundle_name)
    source_bundle = Path(bundle_path)
    package_roots: list[Path] = []
    package_names: list[str] = []
    temp_root = Path(tempfile.mkdtemp(prefix="integration-source-bundle-"))
    extract_dir = temp_root / "extract"
    normalized_root = temp_root / "normalized"
    extract_dir.mkdir(parents=True, exist_ok=True)
    normalized_root.mkdir(parents=True, exist_ok=True)

    try:
        _extract_archive_local(source_bundle, extract_dir)
        seen_dirs: set[Path] = set()
        for package_xml in sorted(extract_dir.rglob("package.xml")):
            package_dir = package_xml.parent
            if package_dir in seen_dirs:
                continue
            seen_dirs.add(package_dir)
            package_roots.append(package_dir)

        if not package_roots:
            raise ValueError(
                "Source bundle does not contain any ROS packages with package.xml."
            )

        dest_src = normalized_root / "src"
        dest_src.mkdir(parents=True, exist_ok=True)
        seen_package_names: set[str] = set()
        seen_dest_dirs: set[str] = set()
        for package_dir in package_roots:
            package_name = _package_name_for(package_dir)
            if package_name in seen_package_names:
                raise ValueError(
                    f"Source bundle contains duplicate ROS package '{package_name}'."
                )
            seen_package_names.add(package_name)
            dest_name = package_dir.name
            if dest_name in seen_dest_dirs:
                raise ValueError(
                    "Source bundle contains multiple packages with the same directory "
                    f"name '{dest_name}'. Rename one of them before bundling."
                )
            seen_dest_dirs.add(dest_name)
            shutil.copytree(package_dir, dest_src / dest_name)
            package_names.append(package_name)

        normalized_tarball = temp_root / "normalized-source-bundle.tar.gz"
        with tarfile.open(normalized_tarball, "w:gz") as archive:
            archive.add(dest_src, arcname="src")
        return str(normalized_tarball), sorted(package_names), sanitized_name
    except Exception:
        shutil.rmtree(temp_root, ignore_errors=True)
        raise


def _local_source_package_names(
    ctx: AppContext, target_ctx: TargetContext
) -> list[str]:
    _, _, shared_fleet, _ = _load_selected_fleet(ctx)
    selected_vehicle = _vehicle_from_fleet(shared_fleet, target_ctx.target.vehicle_name)
    overlay = target_ctx.target.overlay_data()
    merged = deepcopy(selected_vehicle)
    merged.update(overlay)
    _, _, mission_spec = _mission_source_for(ctx, merged)
    packages = list(_LOCAL_SOURCE_BASE_PACKAGES)
    if mission_spec.target == "payload":
        packages.append("payload")
    return packages


def _build_local_source_bundle(
    ctx: AppContext, target_ctx: TargetContext
) -> tuple[str, list[str], str]:
    repo_root = _repo_root(ctx)
    src_root = repo_root / "src"
    package_names = _local_source_package_names(ctx, target_ctx)
    temp_root = Path(tempfile.mkdtemp(prefix="integration-local-source-"))
    staged_src = temp_root / "src"
    staged_src.mkdir(parents=True, exist_ok=True)
    short_sha = "workspace"
    try:
        git_result = subprocess.run(
            ["git", "-C", str(repo_root), "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            check=False,
            timeout=3,
        )
        if git_result.returncode == 0:
            candidate = (git_result.stdout or "").strip()
            if candidate:
                short_sha = candidate
    except Exception:
        pass

    for package_name in package_names:
        package_dir = src_root / package_name
        if not package_dir.exists():
            shutil.rmtree(temp_root, ignore_errors=True)
            raise ValueError(
                f"Local codebase source deploy requires '{package_name}' under '{src_root}'."
            )
        shutil.copytree(package_dir, staged_src / package_name)

    bundle_name = f"local-codebase-{short_sha}.tar.gz"
    bundle_path = temp_root / bundle_name
    with tarfile.open(bundle_path, "w:gz") as archive:
        archive.add(staged_src, arcname="src")
    return str(bundle_path), package_names, bundle_name


def _render_runtime_fleet(
    ctx: AppContext, target_ctx: TargetContext
) -> tuple[str, str, str]:
    ctx.require_deploy_context()
    target = target_ctx.target
    if not target.vehicle_name:
        raise ValueError(
            f"Target '{target.target_id}' must be assigned to a controllable before deploy."
        )
    _, _, shared_fleet, fleet_defaults = _load_selected_fleet(ctx)

    selected_vehicle = _vehicle_from_fleet(shared_fleet, target.vehicle_name)
    overlay = target.overlay_data()

    merged = deepcopy(fleet_defaults)
    merged.update(selected_vehicle)
    merged.update(overlay)

    _, local_mission_path, mission_target = _runtime_vehicle_from_config(
        ctx,
        merged_vehicle=merged,
        vehicle_name=target.vehicle_name,
    )
    _validate_overlay_data(target_ctx, overlay, mission_target=mission_target)
    remote_mission_path = (
        f"{target.deploy_paths()['missions_dir']}/{Path(local_mission_path).name}"
    )
    runtime_vehicle, _, _ = _runtime_vehicle_from_config(
        ctx,
        merged_vehicle=merged,
        vehicle_name=target.vehicle_name,
        mission_path_override=remote_mission_path,
    )

    runtime_fleet = {
        "backend": {"kind": "hardware"},
        "vehicles": [runtime_vehicle],
    }
    return (
        yaml.safe_dump(runtime_fleet, sort_keys=False),
        yaml.safe_dump(shared_fleet, sort_keys=False),
        str(local_mission_path),
    )


def validate_overlay_preview(
    ctx: AppContext, target_ctx: TargetContext, content: str
) -> tuple[dict, str]:
    original = target_ctx.target.overlay_yaml
    try:
        target_ctx.target.overlay_yaml = content
        parsed = target_ctx.target.overlay_data()
        _render_runtime_fleet(ctx, target_ctx)
        return parsed, target_ctx.target.overlay_yaml
    finally:
        target_ctx.target.overlay_yaml = original


async def _copy_file_to_pi(
    target_ctx: TargetContext, *, local_path: str, remote_name: str
) -> None:
    remote_root = target_ctx.target.deploy_paths()["incoming_dir"]
    mkdir_result = await target_ctx.ssh.run(
        f"mkdir -p {target_ctx.ssh.q(remote_root)}", timeout=10
    )
    if mkdir_result.returncode != 0:
        raise RuntimeError(
            target_ctx.ssh.format_remote_error(
                mkdir_result.stderr, "Prepare remote incoming directory failed"
            )
        )

    remote_path = f"{remote_root}/{remote_name}"
    copy_result = await target_ctx.ssh.scp(local_path, remote_path, timeout=300)
    if copy_result.returncode != 0:
        raise RuntimeError(
            target_ctx.ssh.format_remote_error(copy_result.stderr, "SCP failed")
        )


async def _copy_artifact_to_pi(
    target_ctx: TargetContext, local_path: str, artifact_name: str
) -> None:
    await _copy_file_to_pi(target_ctx, local_path=local_path, remote_name=artifact_name)


async def _copy_text_to_pi(
    target_ctx: TargetContext, *, remote_path: str, content: str, suffix: str = ".yaml"
) -> None:
    tmp_path = ""
    try:
        with tempfile.NamedTemporaryFile(mode="w", suffix=suffix, delete=False) as tmp:
            tmp.write(content)
            tmp_path = tmp.name
        result = await target_ctx.ssh.scp(tmp_path, remote_path, timeout=60)
        if result.returncode != 0:
            raise RuntimeError(
                target_ctx.ssh.format_remote_error(result.stderr, "SCP failed")
            )
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


async def _ensure_remote_layout(target_ctx: TargetContext) -> None:
    paths = target_ctx.target.deploy_paths()
    result = await target_ctx.ssh.run(
        "mkdir -p "
        + " ".join(
            target_ctx.ssh.q(paths[key])
            for key in (
                "incoming_dir",
                "releases_dir",
                "config_dir",
                "state_dir",
                "exports_dir",
                "missions_dir",
            )
        ),
        timeout=15,
    )
    if result.returncode != 0:
        raise RuntimeError(
            target_ctx.ssh.format_remote_error(
                result.stderr, "Prepare deploy root failed"
            )
        )


async def _seed_repo_missions(ctx: AppContext, target_ctx: TargetContext) -> None:
    missions_root = _missions_root(ctx)
    if not missions_root.exists():
        return

    tmp_archive = ""
    try:
        with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tmp:
            tmp_archive = tmp.name
        import tarfile

        with tarfile.open(tmp_archive, "w:gz") as archive:
            for mission_path in sorted(missions_root.glob("*.yaml")):
                archive.add(mission_path, arcname=mission_path.name)

        remote_archive = (
            f"{target_ctx.target.deploy_paths()['incoming_dir']}/mission-seed.tar.gz"
        )
        copy_result = await target_ctx.ssh.scp(tmp_archive, remote_archive, timeout=120)
        if copy_result.returncode != 0:
            raise RuntimeError(
                target_ctx.ssh.format_remote_error(
                    copy_result.stderr, "Seed mission upload failed"
                )
            )

        cmd = f"""
            set -e
            missions_dir={target_ctx.ssh.q(target_ctx.target.deploy_paths()["missions_dir"])}
            archive={target_ctx.ssh.q(remote_archive)}
            temp_dir="$(mktemp -d)"
            mkdir -p "$missions_dir"
            tar -xzf "$archive" -C "$temp_dir"
            find "$temp_dir" -maxdepth 1 -type f \\( -name '*.yaml' -o -name '*.yml' \\) -print0 | \
                while IFS= read -r -d '' src; do
                    base="$(basename "$src")"
                    if [ ! -e "$missions_dir/$base" ]; then
                        cp "$src" "$missions_dir/$base"
                    fi
                done
            rm -rf "$temp_dir" "$archive"
        """
        result = await target_ctx.ssh.run(
            f"bash -lc {target_ctx.ssh.q(cmd)}", timeout=60
        )
        if result.returncode != 0:
            raise RuntimeError(
                target_ctx.ssh.format_remote_error(
                    result.stderr or result.stdout, "Seed missions failed"
                )
            )
    finally:
        if tmp_archive and os.path.exists(tmp_archive):
            os.unlink(tmp_archive)


def _parse_stage_output(stdout: str, *, error_prefix: str) -> dict[str, str]:
    markers: dict[str, str] = {}
    for line in (stdout or "").splitlines():
        if line.startswith("__RELEASE_ID__:"):
            markers["release_id"] = line.split(":", 1)[1].strip()
        elif line.startswith("__RELEASE_DIR__:"):
            markers["release_dir"] = line.split(":", 1)[1].strip()
        elif line.startswith("__PREVIOUS_TARGET__:"):
            markers["previous_target"] = line.split(":", 1)[1].strip()

    if not markers.get("release_id") or not markers.get("release_dir"):
        raise RuntimeError(f"{error_prefix}: remote deploy output was incomplete.")
    markers.setdefault("previous_target", "")
    return markers


async def _extract_release_on_pi(
    target_ctx: TargetContext, artifact_name: str
) -> dict[str, str]:
    paths = target_ctx.target.deploy_paths()
    cmd = f"""
        set -e
        incoming_dir={target_ctx.ssh.q(paths["incoming_dir"])}
        releases_dir={target_ctx.ssh.q(paths["releases_dir"])}
        current_link={target_ctx.ssh.q(paths["current_link"])}
        previous_link={target_ctx.ssh.q(paths["previous_link"])}
        artifact_name={target_ctx.ssh.q(artifact_name)}
        artifact_path="$incoming_dir/$artifact_name"
        if [ ! -f "$artifact_path" ]; then
            echo "__ERR__:missing_artifact"
            exit 1
        fi

        release_slug="${{artifact_name%.tar.gz}}"
        release_slug="${{release_slug%.tgz}}"
        release_id="$(date -u +%Y%m%d-%H%M%S)-$release_slug"
        release_dir="$releases_dir/$release_id"
        mkdir -p "$release_dir"
        tar -xzf "$artifact_path" -C "$release_dir"
        rm -f "$artifact_path"

        previous_target=""
        if [ -L "$current_link" ]; then
            previous_target="$(readlink -f "$current_link" || true)"
        elif [ -d "$current_link" ]; then
            previous_target="$current_link"
        fi

        if [ ! -d "$release_dir/install" ]; then
            echo "__ERR__:missing_install"
            exit 1
        fi

        ln -sfn "$release_dir" "$current_link"
        if [ -n "$previous_target" ] && [ -d "$previous_target" ]; then
            ln -sfn "$previous_target" "$previous_link"
        else
            rm -f "$previous_link"
        fi

        find "$releases_dir" -mindepth 1 -maxdepth 1 -type d | while read -r candidate; do
            if [ "$candidate" != "$release_dir" ] && [ "$candidate" != "$previous_target" ]; then
                rm -rf "$candidate"
            fi
        done

        echo "__RELEASE_ID__:$release_id"
        echo "__RELEASE_DIR__:$release_dir"
        echo "__PREVIOUS_TARGET__:$previous_target"
    """
    result = await target_ctx.ssh.run(f"bash -lc {target_ctx.ssh.q(cmd)}", timeout=180)
    if result.returncode != 0:
        raise RuntimeError(
            target_ctx.ssh.format_remote_error(
                result.stderr or result.stdout, "Extract artifact failed"
            )
        )
    return _parse_stage_output(
        result.stdout or "", error_prefix="Extract artifact failed"
    )


def _runner_script(target_ctx: TargetContext) -> str:
    paths = target_ctx.target.deploy_paths()
    return f"""#!/usr/bin/env bash
set -euo pipefail
DEPLOY_ROOT={target_ctx.ssh.q(paths["deploy_root"])}
cd "$DEPLOY_ROOT"
mkdir -p "$DEPLOY_ROOT/state/logs"
export ROS_LOG_DIR="$DEPLOY_ROOT/state/logs"
source /opt/ros/humble/setup.bash
source "$DEPLOY_ROOT/current/install/setup.bash"
exec ros2 launch uav fleet.launch.py fleet_file:="$DEPLOY_ROOT/config/runtime_fleet.yaml"
"""


def _systemd_unit_text(target_ctx: TargetContext) -> str:
    target = target_ctx.target
    paths = target.deploy_paths()
    return f"""[Unit]
Description=PennAiR autonomy runtime
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User={target.pi_user}
WorkingDirectory={paths["deploy_root"]}
ExecStart={paths["runner_script"]}
Restart=on-failure
RestartSec=2
KillSignal=SIGINT
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
"""


async def _ensure_runtime_service(target_ctx: TargetContext) -> None:
    paths = target_ctx.target.deploy_paths()
    runner_script = _runner_script(target_ctx)
    await _copy_text_to_pi(
        target_ctx,
        remote_path=paths["runner_script"],
        content=runner_script,
        suffix=".sh",
    )
    chmod_result = await target_ctx.ssh.run(
        f"chmod +x {target_ctx.ssh.q(paths['runner_script'])}", timeout=10
    )
    if chmod_result.returncode != 0:
        raise RuntimeError(
            target_ctx.ssh.format_remote_error(
                chmod_result.stderr, "Mark runner script executable failed"
            )
        )

    unit_tmp = ""
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".service", delete=False
        ) as tmp:
            tmp.write(_systemd_unit_text(target_ctx))
            unit_tmp = tmp.name
        remote_unit_tmp = (
            f"{paths['incoming_dir']}/{target_ctx.target.service_unit}.tmp"
        )
        scp_result = await target_ctx.ssh.scp(unit_tmp, remote_unit_tmp, timeout=60)
        if scp_result.returncode != 0:
            raise RuntimeError(
                target_ctx.ssh.format_remote_error(
                    scp_result.stderr, "Upload service unit failed"
                )
            )

        install_cmd = f"""
            set -e
            remote_unit={target_ctx.ssh.q(remote_unit_tmp)}
            sudo -n install -D -m 644 "$remote_unit" /etc/systemd/system/{target_ctx.target.service_unit}
            rm -f "$remote_unit"
            sudo -n systemctl daemon-reload
            sudo -n systemctl enable {target_ctx.target.service_unit}
        """
        result = await target_ctx.ssh.run(
            f"bash -lc {target_ctx.ssh.q(install_cmd)}", timeout=30
        )
        if result.returncode != 0:
            raise RuntimeError(
                target_ctx.ssh.format_remote_error(
                    result.stderr or result.stdout,
                    "Install systemd unit failed. Bootstrap the Pi or grant passwordless sudo for systemctl/install.",
                )
            )
    finally:
        if unit_tmp and os.path.exists(unit_tmp):
            os.unlink(unit_tmp)


def _deploy_lib_local_path(ctx: AppContext) -> Path:
    return _repo_root(ctx) / "scripts" / "hardware" / "deploy-lib.sh"


async def _run_remote_deploy_lib(
    ctx: AppContext,
    target_ctx: TargetContext,
    *,
    helper_invocation: str,
    timeout: int,
    error_prefix: str,
) -> None:
    helper_path = _deploy_lib_local_path(ctx)
    if not helper_path.exists():
        raise RuntimeError(f"Missing deploy helper library: {helper_path}")

    await _copy_file_to_pi(
        target_ctx, local_path=str(helper_path), remote_name="deploy-lib.sh"
    )

    remote_helper = f"{target_ctx.target.deploy_paths()['incoming_dir']}/deploy-lib.sh"
    cmd = f"""
        set -euo pipefail
        remote_helper={target_ctx.ssh.q(remote_helper)}
        run_root() {{
            if [ "$(id -u)" -eq 0 ]; then
                "$@"
            else
                sudo -n "$@"
            fi
        }}
        source "$remote_helper"
        {helper_invocation}
    """
    result = await target_ctx.ssh.run(f"bash -lc {target_ctx.ssh.q(cmd)}", timeout=timeout)
    if result.returncode != 0:
        raise RuntimeError(
            target_ctx.ssh.format_remote_error(
                result.stderr or result.stdout,
                error_prefix,
            )
        )


async def _ensure_runtime_prereqs(ctx: AppContext, target_ctx: TargetContext) -> None:
    await _run_remote_deploy_lib(
        ctx,
        target_ctx,
        helper_invocation=(
            "deploy_ensure_uav_python_runtime run_root "
            + target_ctx.ssh.q(target_ctx.target.pi_user)
        ),
        timeout=300,
        error_prefix="Install UAV runtime Python dependencies failed",
    )


async def _ensure_source_build_prereqs(
    ctx: AppContext, target_ctx: TargetContext
) -> None:
    await _run_remote_deploy_lib(
        ctx,
        target_ctx,
        helper_invocation=(
            "deploy_ensure_source_build_prereqs run_root "
            + target_ctx.ssh.q(target_ctx.target.pi_user)
        ),
        timeout=300,
        error_prefix="Install source-build prerequisites failed",
    )


async def _write_release_metadata(
    target_ctx: TargetContext, *, release_dir: str, metadata: dict[str, object]
) -> None:
    await _copy_text_to_pi(
        target_ctx,
        remote_path=f"{release_dir}/{_RELEASE_METADATA_NAME}",
        content=yaml.safe_dump(metadata, sort_keys=False),
        suffix=".yaml",
    )


async def _write_build_info(
    target_ctx: TargetContext, *, release_dir: str, content: str
) -> None:
    await _copy_text_to_pi(
        target_ctx,
        remote_path=f"{release_dir}/install/BUILD_INFO.txt",
        content=content,
        suffix=".txt",
    )


def _source_build_info_text(
    *, target_ctx: TargetContext, source_label: str, package_names: list[str]
) -> str:
    package_block = "\n".join(f"- {name}" for name in package_names)
    return (
        "Build Information\n"
        "=================\n"
        "Source: local package bundle\n"
        f"Bundle: {source_label}\n"
        f"Target: {target_ctx.target.target_id}\n"
        f"Vehicle: {target_ctx.target.vehicle_name}\n"
        "Build Type: source-build on target Pi\n"
        "Packages:\n"
        f"{package_block}\n"
    )


async def _activate_release(
    target_ctx: TargetContext, *, release_dir: str, previous_target: str = ""
) -> None:
    paths = target_ctx.target.deploy_paths()
    restart_cmd = f"""
        set -e
        current_link={target_ctx.ssh.q(paths["current_link"])}
        runtime_fleet={target_ctx.ssh.q(paths["runtime_fleet"])}
        release_dir={target_ctx.ssh.q(release_dir)}
        if [ ! -d "$current_link" ] || [ ! -f "$runtime_fleet" ]; then
            echo "__ERR__:runtime_missing"
            exit 1
        fi
        if [ ! -d "$release_dir/install" ]; then
            echo "__ERR__:install_missing"
            exit 1
        fi
        sudo -n systemctl restart {target_ctx.target.service_unit}
        state="$(sudo -n systemctl is-active {target_ctx.target.service_unit} 2>/dev/null || true)"
        if [ "$state" != "active" ]; then
            echo "__ERR__:inactive:$state"
            exit 1
        fi
        echo "ACTIVE"
    """
    result = await target_ctx.ssh.run(
        f"bash -lc {target_ctx.ssh.q(restart_cmd)}", timeout=40
    )
    if result.returncode == 0 and "ACTIVE" in (result.stdout or ""):
        return

    error = target_ctx.ssh.format_remote_error(
        result.stderr or result.stdout,
        "Restart runtime service failed",
    )
    if previous_target:
        rollback_cmd = f"""
            set -e
            current_link={target_ctx.ssh.q(paths["current_link"])}
            previous_link={target_ctx.ssh.q(paths["previous_link"])}
            previous_target={target_ctx.ssh.q(previous_target)}
            failed_release={target_ctx.ssh.q(release_dir)}
            if [ -d "$previous_target" ]; then
                ln -sfn "$previous_target" "$current_link"
                ln -sfn "$failed_release" "$previous_link"
                sudo -n systemctl restart {target_ctx.target.service_unit} >/dev/null 2>&1 || true
            fi
        """
        await target_ctx.ssh.run(
            f"bash -lc {target_ctx.ssh.q(rollback_cmd)}", timeout=30
        )
        raise RuntimeError(f"{error} Reverted to the previous release.")

    raise RuntimeError(error)


async def _stage_source_build_on_pi(
    target_ctx: TargetContext, *, bundle_name: str, package_names: list[str]
) -> dict[str, str]:
    paths = target_ctx.target.deploy_paths()
    package_list = " ".join(target_ctx.ssh.q(name) for name in package_names)
    cmd = f"""
        set -e
        incoming_dir={target_ctx.ssh.q(paths["incoming_dir"])}
        releases_dir={target_ctx.ssh.q(paths["releases_dir"])}
        current_link={target_ctx.ssh.q(paths["current_link"])}
        previous_link={target_ctx.ssh.q(paths["previous_link"])}
        bundle_name={target_ctx.ssh.q(bundle_name)}
        bundle_path="$incoming_dir/$bundle_name"
        if [ ! -f "$bundle_path" ]; then
            echo "__ERR__:missing_bundle"
            exit 1
        fi

        current_target=""
        if [ -L "$current_link" ]; then
            current_target="$(readlink -f "$current_link" || true)"
        elif [ -d "$current_link" ]; then
            current_target="$current_link"
        fi
        if [ -z "$current_target" ] || [ ! -d "$current_target/install" ]; then
            echo "__ERR__:missing_current"
            exit 1
        fi

        release_slug="${{bundle_name%.tar.gz}}"
        release_slug="${{release_slug%.tgz}}"
        release_slug="${{release_slug%.zip}}"
        release_id="$(date -u +%Y%m%d-%H%M%S)-source-$release_slug"
        release_dir="$releases_dir/$release_id"
        workspace_dir="$release_dir/source_workspace"
        build_dir="$release_dir/build"
        mkdir -p "$release_dir/install" "$workspace_dir"
        cp -a "$current_target/install/." "$release_dir/install/"

        if [[ "$bundle_name" == *.zip ]]; then
            python3 - "$bundle_path" "$workspace_dir" <<'PY'
from pathlib import Path
import sys, zipfile

bundle = Path(sys.argv[1])
dest = Path(sys.argv[2])
with zipfile.ZipFile(bundle) as archive:
    archive.extractall(dest)
PY
        else
            tar -xzf "$bundle_path" -C "$workspace_dir"
        fi
        rm -f "$bundle_path"

        if [ ! -d "$workspace_dir/src" ]; then
            echo "__ERR__:missing_src"
            exit 1
        fi

        packages=({package_list})
        if [ "${{#packages[@]}}" -eq 0 ]; then
            echo "__ERR__:missing_packages"
            exit 1
        fi

        cd "$workspace_dir"
        source /opt/ros/humble/setup.bash
        source "$current_target/install/setup.bash"
        colcon build \
            --build-base "$build_dir" \
            --install-base "$release_dir/install" \
            --packages-select "${{packages[@]}}" \
            --cmake-args -DBUILD_SIM=OFF
        rm -rf "$workspace_dir" "$build_dir"

        ln -sfn "$release_dir" "$current_link"
        if [ -n "$current_target" ] && [ -d "$current_target" ]; then
            ln -sfn "$current_target" "$previous_link"
        else
            rm -f "$previous_link"
        fi

        find "$releases_dir" -mindepth 1 -maxdepth 1 -type d | while read -r candidate; do
            if [ "$candidate" != "$release_dir" ] && [ "$candidate" != "$current_target" ]; then
                rm -rf "$candidate"
            fi
        done

        echo "__RELEASE_ID__:$release_id"
        echo "__RELEASE_DIR__:$release_dir"
        echo "__PREVIOUS_TARGET__:$current_target"
    """
    result = await target_ctx.ssh.run(f"bash -lc {target_ctx.ssh.q(cmd)}", timeout=900)
    if result.returncode != 0:
        combined = result.stderr or result.stdout
        if "__ERR__:missing_current" in combined:
            raise RuntimeError(
                "Source build requires an installed base release on the target Pi. "
                "Deploy a build artifact first."
            )
        raise RuntimeError(
            target_ctx.ssh.format_remote_error(
                combined, "Build source bundle on target failed"
            )
        )
    return _parse_stage_output(
        result.stdout or "", error_prefix="Build source bundle on target failed"
    )


async def _deploy_artifact_path(
    ctx: AppContext,
    *,
    target_id: str | None,
    local_path: str,
    artifact_name: str,
    source_type: str,
    source_label: str,
) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    await _ensure_remote_layout(target_ctx)
    await _copy_artifact_to_pi(target_ctx, local_path, artifact_name)

    runtime_fleet_yaml, shared_fleet_yaml, local_mission_path = _render_runtime_fleet(
        ctx, target_ctx
    )
    paths = target_ctx.target.deploy_paths()
    await _copy_text_to_pi(
        target_ctx,
        remote_path=paths["fleet_file"],
        content=shared_fleet_yaml,
    )
    await _copy_text_to_pi(
        target_ctx,
        remote_path=paths["overlay_file"],
        content=target_ctx.target.overlay_yaml or "{}\n",
    )
    await _copy_text_to_pi(
        target_ctx,
        remote_path=paths["runtime_fleet"],
        content=runtime_fleet_yaml,
    )
    await _seed_repo_missions(ctx, target_ctx)

    mission_name = Path(local_mission_path).name
    mission_copy = await target_ctx.ssh.scp(
        local_mission_path,
        f"{paths['missions_dir']}/{mission_name}",
        timeout=60,
    )
    if mission_copy.returncode != 0:
        raise RuntimeError(
            target_ctx.ssh.format_remote_error(
                mission_copy.stderr, "Upload selected mission failed"
            )
        )

    release = await _extract_release_on_pi(target_ctx, artifact_name)
    await _write_release_metadata(
        target_ctx,
        release_dir=release["release_dir"],
        metadata=_release_metadata_payload(
            target_ctx,
            release_id=release["release_id"],
            source_type=source_type,
            source_label=source_label,
            fleet_file=_selected_fleet_file(ctx),
        ),
    )
    await _ensure_runtime_service(target_ctx)
    await _ensure_runtime_prereqs(ctx, target_ctx)
    await _activate_release(
        target_ctx,
        release_dir=release["release_dir"],
        previous_target=release["previous_target"],
    )

    return {
        "success": True,
        "output": (
            f"Deployed {artifact_name} to {target_ctx.target.target_id} "
            f"({release['release_id']})"
        ),
    }


async def _deploy_source_bundle_path(
    ctx: AppContext,
    *,
    target_id: str | None,
    local_bundle_path: str,
    bundle_name: str,
    package_names: list[str],
    source_type: str,
    source_label: str,
) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    await _ensure_remote_layout(target_ctx)
    await _ensure_source_build_prereqs(ctx, target_ctx)
    await _copy_file_to_pi(
        target_ctx, local_path=local_bundle_path, remote_name=bundle_name
    )

    runtime_fleet_yaml, shared_fleet_yaml, local_mission_path = _render_runtime_fleet(
        ctx, target_ctx
    )
    paths = target_ctx.target.deploy_paths()
    await _copy_text_to_pi(
        target_ctx,
        remote_path=paths["fleet_file"],
        content=shared_fleet_yaml,
    )
    await _copy_text_to_pi(
        target_ctx,
        remote_path=paths["overlay_file"],
        content=target_ctx.target.overlay_yaml or "{}\n",
    )
    await _copy_text_to_pi(
        target_ctx,
        remote_path=paths["runtime_fleet"],
        content=runtime_fleet_yaml,
    )
    await _seed_repo_missions(ctx, target_ctx)

    mission_name = Path(local_mission_path).name
    mission_copy = await target_ctx.ssh.scp(
        local_mission_path,
        f"{paths['missions_dir']}/{mission_name}",
        timeout=60,
    )
    if mission_copy.returncode != 0:
        raise RuntimeError(
            target_ctx.ssh.format_remote_error(
                mission_copy.stderr, "Upload selected mission failed"
            )
        )

    release = await _stage_source_build_on_pi(
        target_ctx,
        bundle_name=bundle_name,
        package_names=package_names,
    )
    await _write_release_metadata(
        target_ctx,
        release_dir=release["release_dir"],
        metadata=_release_metadata_payload(
            target_ctx,
            release_id=release["release_id"],
            source_type=source_type,
            source_label=source_label,
            fleet_file=_selected_fleet_file(ctx),
            package_names=package_names,
        ),
    )
    await _write_build_info(
        target_ctx,
        release_dir=release["release_dir"],
        content=_source_build_info_text(
            target_ctx=target_ctx,
            source_label=source_label,
            package_names=package_names,
        ),
    )
    await _ensure_runtime_service(target_ctx)
    await _ensure_runtime_prereqs(ctx, target_ctx)
    await _activate_release(
        target_ctx,
        release_dir=release["release_dir"],
        previous_target=release["previous_target"],
    )

    return {
        "success": True,
        "output": (
            f"Built and deployed source bundle {bundle_name} to "
            f"{target_ctx.target.target_id} ({release['release_id']})"
        ),
    }


async def upload_source_bundle(
    ctx: AppContext,
    *,
    target_id: str | None,
    filename: str,
    file_bytes: bytes,
) -> dict:
    upload_path = ""
    normalized_bundle = ""
    try:
        bundle_name = sanitize_source_bundle_name(filename)
        bundle_suffix = (
            ".zip"
            if bundle_name.endswith(".zip")
            else ".tgz"
            if bundle_name.endswith(".tgz")
            else ".tar.gz"
        )
        with tempfile.NamedTemporaryFile(suffix=bundle_suffix, delete=False) as tmp:
            tmp.write(file_bytes)
            upload_path = tmp.name

        normalized_bundle, package_names, sanitized_name = _normalize_source_bundle(
            upload_path, bundle_name
        )
        return await _deploy_source_bundle_path(
            ctx,
            target_id=target_id,
            local_bundle_path=normalized_bundle,
            bundle_name=sanitized_name,
            package_names=package_names,
            source_type="source-build",
            source_label=sanitized_name,
        )
    except RuntimeError as exc:
        return {"success": False, "error": str(exc)}
    except Exception as exc:
        return {"success": False, "error": str(exc)}
    finally:
        if upload_path and os.path.exists(upload_path):
            os.unlink(upload_path)
        if normalized_bundle:
            normalized_root = Path(normalized_bundle).resolve().parent
            shutil.rmtree(normalized_root, ignore_errors=True)


async def deploy_local_codebase(
    ctx: AppContext,
    *,
    target_id: str | None,
) -> dict:
    local_bundle = ""
    try:
        target_ctx = ctx.resolve_target(target_id)
        local_bundle, package_names, bundle_name = _build_local_source_bundle(
            ctx, target_ctx
        )
        return await _deploy_source_bundle_path(
            ctx,
            target_id=target_id,
            local_bundle_path=local_bundle,
            bundle_name=bundle_name,
            package_names=package_names,
            source_type="local-codebase",
            source_label=bundle_name,
        )
    except RuntimeError as exc:
        return {"success": False, "error": str(exc)}
    except Exception as exc:
        return {"success": False, "error": str(exc)}
    finally:
        if local_bundle:
            shutil.rmtree(Path(local_bundle).resolve().parent, ignore_errors=True)


async def get_build_source(ctx: AppContext) -> dict[str, object]:
    return _build_source_response(ctx, success=True)


async def get_fleet_catalog(ctx: AppContext) -> dict[str, object]:
    current = ctx.build_source_store.current()
    available_fleets, catalog_source, catalog_error = _fleet_catalog_from_current_source(
        ctx
    )
    selected_fleet_file = (
        _selected_fleet_file(ctx) if current.kind != "none" else ""
    ) or None
    fleet_preview = _fleet_preview(ctx, selected_fleet_file or "")
    return {
        "success": True,
        "source_kind": current.kind,
        "source_label": current.summary(),
        "selected_fleet_file": selected_fleet_file,
        "available_fleets": available_fleets,
        "fleet_catalog_source": catalog_source,
        "fleet_catalog_error": catalog_error,
        "fleet_exists": fleet_preview.get("fleet_exists"),
        "fleet_error": fleet_preview.get("fleet_error"),
        "fleet_vehicles": fleet_preview.get("fleet_vehicles", []),
        "error": None,
    }


async def set_github_build_source(
    ctx: AppContext,
    *,
    source: str,
    tag: str = "",
    artifact_id: str = "",
    run_id: str = "",
    sha: str = "",
    commit_subject: str = "",
    name: str = "",
    date: str = "",
    download_url: str = "",
    size_mb: float | None = None,
    branch: str = "",
    artifact_name: str = "",
) -> dict[str, object]:
    try:
        previous_fleet_file = (
            _selected_fleet_file(ctx)
            if ctx.build_source_store.current().kind != "none"
            else ""
        )
        ctx.build_source_store.set_github(
            github_source=source,
            tag=tag,
            artifact_id=artifact_id,
            run_id=run_id,
            sha=sha,
            commit_subject=commit_subject,
            name=name,
            date=date,
            download_url=download_url,
            size_mb=size_mb,
            branch=branch,
            artifact_name=artifact_name,
        )
        _reconcile_selected_fleet_with_current_source(ctx, previous_fleet_file)
        return _build_source_response(
            ctx,
            success=True,
            output="GitHub build source selected",
        )
    except Exception as exc:
        return _build_source_response(
            ctx,
            success=False,
            error=str(exc),
        )


async def set_local_artifact_build_source(
    ctx: AppContext,
    *,
    filename: str,
    file_bytes: bytes,
) -> dict[str, object]:
    try:
        artifact_name = sanitize_artifact_name(filename)
        previous_fleet_file = (
            _selected_fleet_file(ctx)
            if ctx.build_source_store.current().kind != "none"
            else ""
        )
        ctx.build_source_store.set_local_artifact(
            artifact_name=artifact_name,
            file_bytes=file_bytes,
        )
        _reconcile_selected_fleet_with_current_source(ctx, previous_fleet_file)
        return _build_source_response(
            ctx,
            success=True,
            output=f"Cached local artifact {artifact_name}",
        )
    except Exception as exc:
        return _build_source_response(
            ctx,
            success=False,
            error=str(exc),
        )


async def set_local_codebase_build_source(ctx: AppContext) -> dict[str, object]:
    try:
        previous_fleet_file = (
            _selected_fleet_file(ctx)
            if ctx.build_source_store.current().kind != "none"
            else ""
        )
        ctx.build_source_store.set_local_codebase(
            codebase_root=str(_repo_root(ctx)),
        )
        _reconcile_selected_fleet_with_current_source(ctx, previous_fleet_file)
        return _build_source_response(
            ctx,
            success=True,
            output="Local codebase selected",
        )
    except Exception as exc:
        return _build_source_response(
            ctx,
            success=False,
            error=str(exc),
        )


async def set_global_fleet_file(
    ctx: AppContext,
    *,
    fleet_file: str,
) -> dict[str, object]:
    try:
        selected = Path(str(fleet_file or "").strip().replace("\\", "/")).name.strip()
        if not selected:
            raise ValueError("fleet_file is required.")
        current_kind = ctx.build_source_store.current().kind
        if current_kind == "none":
            raise ValueError("Select a build source before selecting a fleet.")
        catalog_lookup, catalog_source, catalog_error = (
            _fleet_catalog_lookup_from_current_source(ctx)
        )
        if catalog_error:
            raise ValueError(catalog_error)
        if not catalog_lookup:
            raise ValueError(
                "The selected build source does not expose any fleet YAML files."
            )
        fleet_path = catalog_lookup.get(selected)
        if fleet_path is None:
            raise ValueError(
                "fleet_file must be one of the fleet YAML files from the selected build source."
            )
        fleet_payload = _load_local_yaml(fleet_path)
        if not isinstance(fleet_payload.get("vehicles", []), list):
            raise ValueError("Fleet file must define vehicles as a list.")
        ctx.build_source_store.set_fleet_file(fleet_file=selected)
        response = _build_source_response(
            ctx,
            success=True,
            output=f"Fleet selected: {selected}",
        )
        if response.get("source"):
            response["source"]["fleet_catalog_source"] = catalog_source
            response["source"]["available_fleets"] = sorted(catalog_lookup)
        return response
    except Exception as exc:
        return _build_source_response(
            ctx,
            success=False,
            error=str(exc),
        )


async def clear_build_source(ctx: AppContext) -> dict[str, object]:
    try:
        ctx.build_source_store.clear()
        return _build_source_response(
            ctx,
            success=True,
            output="Cleared build source selection",
        )
    except Exception as exc:
        return _build_source_response(
            ctx,
            success=False,
            error=str(exc),
        )


async def deploy_selected_source(
    ctx: AppContext,
    *,
    target_id: str | None,
) -> dict:
    current = ctx.build_source_store.current()
    if current.kind == "none":
        return {"success": False, "error": "No build source selected"}

    if current.kind == "github":
        return await download_build(
            ctx,
            target_id=target_id,
            tag=current.tag,
            source=current.github_source,
            artifact_id=current.artifact_id,
        )

    if current.kind == "local_artifact":
        artifact_path = Path(current.local_artifact_path)
        if not artifact_path.exists():
            return {
                "success": False,
                "error": (
                    "Selected local artifact is no longer available on the backend. "
                    "Upload it again before deploying."
                ),
            }
        return await _deploy_artifact_path(
            ctx,
            target_id=target_id,
            local_path=str(artifact_path),
            artifact_name=current.artifact_name,
            source_type="local-artifact",
            source_label=current.artifact_name,
        )

    if current.kind == "local_codebase":
        return await deploy_local_codebase(ctx, target_id=target_id)

    return {
        "success": False,
        "error": f"Unsupported build source kind '{current.kind}'.",
    }


async def current_build(ctx: AppContext, *, target_id: str | None = None) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    try:
        ctx.require_deploy_context()
        paths = target_ctx.target.deploy_paths()
        cmd = f"""
            current_link={target_ctx.ssh.q(paths["current_link"])}
            if [ -L "$current_link" ] || [ -d "$current_link" ]; then
                release_id="$(basename "$(readlink -f "$current_link" 2>/dev/null || echo "$current_link")")"
                metadata="$current_link/{_RELEASE_METADATA_NAME}"
                build_info="$current_link/install/BUILD_INFO.txt"
                if [ -f "$metadata" ]; then
                    echo "__META_BEGIN__"
                    cat "$metadata"
                    echo "__META_END__"
                fi
                if [ -f "$build_info" ]; then
                    echo "__INFO_BEGIN__"
                    cat "$build_info"
                    echo "__INFO_END__"
                    echo "__RELEASE__:$release_id"
                    exit 0
                fi
                echo "__INSTALLED__:$release_id"
                exit 0
            fi
            echo "__NONE__"
        """
        result = await target_ctx.ssh.run(
            f"bash -lc {target_ctx.ssh.q(cmd)}", timeout=10
        )
        if result.returncode != 0:
            return {
                "success": True,
                "installed": False,
                "info": target_ctx.ssh.friendly_error(result.stderr),
            }

        lines = [line for line in (result.stdout or "").splitlines() if line.strip()]
        if not lines or lines == ["__NONE__"]:
            return {"success": True, "installed": False, "info": "No build deployed"}

        metadata_lines: list[str] = []
        build_info_lines: list[str] = []
        section = None
        trailing_markers: list[str] = []
        for line in lines:
            if line == "__META_BEGIN__":
                section = "meta"
                continue
            if line == "__META_END__":
                section = None
                continue
            if line == "__INFO_BEGIN__":
                section = "info"
                continue
            if line == "__INFO_END__":
                section = None
                continue
            if line.startswith("__RELEASE__:") or line.startswith("__INSTALLED__:"):
                trailing_markers.append(line)
                continue
            if section == "meta":
                metadata_lines.append(line)
            elif section == "info":
                build_info_lines.append(line)

        info_parts: list[str] = []
        if metadata_lines:
            metadata = yaml.safe_load("\n".join(metadata_lines)) or {}
            if isinstance(metadata, dict):
                summary = _release_metadata_summary(metadata)
                if summary:
                    info_parts.append(summary)
        if build_info_lines:
            info_parts.append("\n".join(build_info_lines).strip())

        release_id = None
        marker = trailing_markers[-1] if trailing_markers else ""
        if marker.startswith("__RELEASE__:"):
            release_id = marker.split(":", 1)[1]
            info = "\n\n".join(part for part in info_parts if part).strip()
            return {
                "success": True,
                "installed": True,
                "info": info or "Build installed",
                "release_id": release_id,
            }
        if marker.startswith("__INSTALLED__:"):
            release_id = marker.split(":", 1)[1]
            info = "\n\n".join(part for part in info_parts if part).strip()
            return {
                "success": True,
                "installed": True,
                "info": info or "Build installed (no BUILD_INFO.txt)",
                "release_id": release_id,
            }
        return {
            "success": True,
            "installed": True,
            "info": "\n".join(lines).strip(),
            "release_id": release_id,
        }
    except Exception as exc:
        return {
            "success": True,
            "installed": False,
            "info": target_ctx.ssh.friendly_error(str(exc)),
        }


async def upload_build(
    ctx: AppContext,
    *,
    target_id: str | None,
    filename: str,
    file_bytes: bytes,
) -> dict:
    tmp_path = ""
    try:
        artifact_name = sanitize_artifact_name(filename)
        with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tmp:
            tmp.write(file_bytes)
            tmp_path = tmp.name
        return await _deploy_artifact_path(
            ctx,
            target_id=target_id,
            local_path=tmp_path,
            artifact_name=artifact_name,
            source_type="upload",
            source_label=artifact_name,
        )
    except RuntimeError as exc:
        return {"success": False, "error": str(exc)}
    except Exception as exc:
        return {"success": False, "error": str(exc)}
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


def _release_body_value(release: dict, label: str) -> str:
    body = str(release.get("body", "") or "")
    prefix = f"{label}:"
    for raw_line in body.splitlines():
        line = raw_line.strip()
        if line.startswith(prefix):
            return line.split(":", 1)[1].strip()
    return ""


def _release_commit_sha(release: dict) -> str:
    commit_sha = _release_body_value(release, "Commit")
    if commit_sha:
        return commit_sha
    return str(release.get("tag_name", "")).replace("build-", "").strip()


def _release_timestamp(release: dict) -> str:
    built_at = _release_body_value(release, "Built")
    if built_at:
        return built_at
    return str(release.get("published_at", "")).strip()


def _release_branch(release: dict) -> str | None:
    branch = _release_body_value(release, "Branch")
    return branch or None


def _artifact_name(artifact: dict) -> str:
    return str(artifact.get("name", "")).strip()


def _artifact_workflow_name(artifact: dict) -> str:
    workflow_run = artifact.get("workflow_run") or {}
    if not isinstance(workflow_run, dict):
        return ""
    return str(workflow_run.get("name", "")).strip()


def _artifact_is_deployable(artifact: dict) -> bool:
    artifact_name = _artifact_name(artifact).lower()
    workflow_name = _artifact_workflow_name(artifact).lower()
    return artifact_name.startswith("ros2-build-") or workflow_name in {
        "arm artifact",
        "arm fallback",
    }


def _artifact_is_fallback(artifact: dict) -> bool:
    artifact_name = _artifact_name(artifact).lower()
    workflow_name = _artifact_workflow_name(artifact).lower()
    return "fallback" in artifact_name or "fallback" in workflow_name


def _matches_build_filters(
    item: dict[str, object],
    *,
    q: str = "",
    sha: str = "",
    commit_subject: str = "",
    branch: str = "",
    include_fallback: bool = False,
) -> bool:
    if not include_fallback and bool(item.get("fallback")):
        return False

    q_value = q.strip().lower()
    sha_value = sha.strip().lower()
    subject_value = commit_subject.strip().lower()
    branch_value = branch.strip().lower()

    if sha_value:
        haystack = " ".join(
            value
            for value in (
                str(item.get("sha", "") or "").lower(),
                str(item.get("commit_sha", "") or "").lower(),
            )
            if value
        )
        if sha_value not in haystack:
            return False

    if subject_value:
        if subject_value not in str(item.get("commit_subject", "") or "").lower():
            return False

    if branch_value:
        if branch_value not in str(item.get("branch", "") or "").lower():
            return False

    if q_value:
        searchable = " ".join(
            str(value or "").lower()
            for value in (
                item.get("sha"),
                item.get("commit_sha"),
                item.get("commit_subject"),
                item.get("name"),
                item.get("tag"),
                item.get("branch"),
                item.get("workflow_name"),
            )
        )
        if q_value not in searchable:
            return False

    return True


async def list_builds(
    ctx: AppContext,
    *,
    artifact_page: int = 1,
    artifact_page_size: int = 20,
    q: str = "",
    sha: str = "",
    commit_subject: str = "",
    branch: str = "",
    include_fallback: bool = False,
) -> dict:
    artifact_page = max(int(artifact_page or 1), 1)
    artifact_page_size = max(1, min(int(artifact_page_size or 20), 50))
    cache_key = _build_list_cache_key(
        ctx,
        artifact_page=artifact_page,
        artifact_page_size=artifact_page_size,
        q=q,
        sha=sha,
        commit_subject=commit_subject,
        branch=branch,
        include_fallback=include_fallback,
    )
    if not ctx.operator_config.github_repo:
        return {
            "success": False,
            "error": "GITHUB_REPO not configured",
            "builds": [],
            "releases": [],
            "artifacts": [],
            "artifact_page": artifact_page,
            "artifact_page_size": artifact_page_size,
            "artifact_has_more": False,
        }
    cached_response = _build_list_cache_get(cache_key)
    if cached_response is not None:
        return cached_response
    try:
        httpx = _require_httpx()
        token_configured = bool(ctx.operator_config.github_token)
        releases_builds: list[dict[str, object]] = []
        artifact_builds: list[dict[str, object]] = []
        commit_lookup_warning: str | None = None
        commit_lookup_rate_limited = False

        async def _collect_releases(client) -> list[dict[str, object]]:
            collected: list[dict[str, object]] = []
            page = 1
            while True:
                response = await client.get(
                    f"https://api.github.com/repos/{ctx.operator_config.github_repo}/releases?per_page=100&page={page}",
                    headers=_github_headers(ctx),
                )
                response.raise_for_status()
                page_payload = response.json()
                if not page_payload:
                    break
                if not isinstance(page_payload, list):
                    raise ValueError("Unexpected releases payload from GitHub.")
                collected.extend(page_payload)
                if len(page_payload) < 100:
                    break
                page += 1
            return collected

        async def _collect_artifacts(client) -> list[dict[str, object]]:
            collected: list[dict[str, object]] = []
            page = 1
            while True:
                response = await client.get(
                    "https://api.github.com/repos/"
                    f"{ctx.operator_config.github_repo}/actions/artifacts"
                    f"?per_page={artifact_page_size}&page={page}",
                    headers=_github_headers(ctx),
                )
                response.raise_for_status()
                payload = response.json() or {}
                rows = payload.get("artifacts", [])
                if not isinstance(rows, list):
                    raise ValueError("Unexpected artifacts payload from GitHub.")
                collected.extend(rows)
                if not rows:
                    break
                if len(rows) < artifact_page_size:
                    break
                if not include_fallback and page >= artifact_page and not q and not sha and not commit_subject and not branch:
                    break
                page += 1
            return collected

        async def _commit_subject_for(client, commit_sha: str) -> str | None:
            nonlocal commit_lookup_warning, commit_lookup_rate_limited
            if not commit_sha:
                return None
            cached_subject = _commit_subject_cache_get(commit_sha)
            if cached_subject is not ...:
                return cached_subject
            if commit_lookup_rate_limited:
                return None
            try:
                response = await client.get(
                    f"https://api.github.com/repos/{ctx.operator_config.github_repo}/commits/{commit_sha}",
                    headers=_github_headers(ctx),
                )
                response.raise_for_status()
                payload = response.json() or {}
                message = (
                    payload.get("commit", {}).get("message", "").splitlines()[0].strip()
                )
                subject = message or None
                _commit_subject_cache_set(commit_sha, subject)
                return subject
            except Exception as exc:
                warning = _github_error_message(
                    exc,
                    token_configured=token_configured,
                    during_commit_lookup=True,
                )
                if warning:
                    commit_lookup_rate_limited = True
                    commit_lookup_warning = warning
                    return None
                return None

        async with httpx.AsyncClient(timeout=20) as client:
            releases = await _collect_releases(client)
            artifact_payload_rows = await _collect_artifacts(client)

            release_commit_shas: list[str] = []
            artifact_commit_shas: list[str] = []
            release_rows: list[tuple[dict, str]] = []
            artifact_rows: list[tuple[dict, str]] = []

            for rel in releases:
                tag_name = str(rel.get("tag_name", "")).strip()
                if not tag_name.startswith("build-"):
                    continue
                commit_sha = _release_commit_sha(rel)
                release_rows.append((rel, commit_sha))
                if commit_sha:
                    release_commit_shas.append(commit_sha)

            for artifact in artifact_payload_rows:
                if not _artifact_is_deployable(artifact):
                    continue
                if not include_fallback and _artifact_is_fallback(artifact):
                    continue
                workflow_run = artifact.get("workflow_run") or {}
                if not isinstance(workflow_run, dict):
                    workflow_run = {}
                commit_sha = str(workflow_run.get("head_sha", "")).strip()
                artifact_rows.append((artifact, commit_sha))
                if commit_sha:
                    artifact_commit_shas.append(commit_sha)

            unique_commit_shas = {
                sha: None for sha in (release_commit_shas + artifact_commit_shas) if sha
            }
            for commit_sha in unique_commit_shas:
                unique_commit_shas[commit_sha] = await _commit_subject_for(
                    client, commit_sha
                )

        for rel, commit_sha in release_rows:
            assets = rel.get("assets", []) or []
            asset = assets[0] if assets else {}
            item = {
                "source": "release",
                "tag": rel.get("tag_name", ""),
                "sha": commit_sha[:7] if len(commit_sha) > 7 else commit_sha,
                "commit_sha": commit_sha or None,
                "commit_subject": unique_commit_shas.get(commit_sha),
                "name": unique_commit_shas.get(commit_sha)
                or rel.get("name", rel.get("tag_name", "")),
                "date": _release_timestamp(rel),
                "download_url": asset.get("browser_download_url"),
                "size_mb": round((asset.get("size", 0) or 0) / 1_000_000, 1)
                if assets
                else None,
                "branch": _release_branch(rel),
                "workflow_name": "ARM Artifact Release",
                "workflow_event": "release",
                "workflow_conclusion": None,
                "deployable": True,
                "fallback": False,
            }
            if _matches_build_filters(
                item,
                q=q,
                sha=sha,
                commit_subject=commit_subject,
                branch=branch,
                include_fallback=include_fallback,
            ):
                releases_builds.append(item)

        for artifact, commit_sha in artifact_rows:
            workflow_run = artifact.get("workflow_run") or {}
            if not isinstance(workflow_run, dict):
                workflow_run = {}
            item = {
                "source": "actions",
                "tag": f"run-{workflow_run.get('id', artifact.get('id'))}",
                "sha": commit_sha[:7] if len(commit_sha) > 7 else commit_sha,
                "commit_sha": commit_sha or None,
                "commit_subject": unique_commit_shas.get(commit_sha),
                "name": unique_commit_shas.get(commit_sha)
                or artifact.get("name", f"artifact-{artifact.get('id')}"),
                "date": str(artifact.get("updated_at", "")).strip(),
                "download_url": artifact.get("archive_download_url")
                or (
                    f"https://api.github.com/repos/{ctx.operator_config.github_repo}"
                    f"/actions/artifacts/{artifact.get('id')}/zip"
                ),
                "size_mb": round((artifact.get("size_in_bytes", 0) or 0) / 1_000_000, 1),
                "run_id": str(workflow_run.get("id", "")) or None,
                "artifact_id": str(artifact.get("id", "")) or None,
                "artifact_name": artifact.get("name"),
                "branch": workflow_run.get("head_branch"),
                "workflow_name": workflow_run.get("name"),
                "workflow_event": workflow_run.get("event"),
                "workflow_conclusion": workflow_run.get("conclusion"),
                "deployable": _artifact_is_deployable(artifact),
                "fallback": _artifact_is_fallback(artifact),
            }
            if _matches_build_filters(
                item,
                q=q,
                sha=sha,
                commit_subject=commit_subject,
                branch=branch,
                include_fallback=include_fallback,
            ):
                artifact_builds.append(item)

        combined = releases_builds + artifact_builds
        combined.sort(key=lambda item: str(item.get("date", "")), reverse=True)
        artifact_has_more = False if (q or sha or commit_subject or branch or include_fallback) else len(artifact_rows) >= artifact_page_size
        response_payload = {
            "success": True,
            "builds": combined,
            "releases": releases_builds,
            "artifacts": artifact_builds,
            "artifact_page": 1 if (q or sha or commit_subject or branch or include_fallback) else artifact_page,
            "artifact_page_size": artifact_page_size,
            "artifact_has_more": artifact_has_more,
            "error": commit_lookup_warning,
        }
        return _build_list_cache_set(cache_key, response_payload)
    except Exception as exc:
        cached_stale = _build_list_cache_get(cache_key, allow_stale=True)
        fallback_error = _github_error_message(
            exc,
            token_configured=bool(ctx.operator_config.github_token),
            using_cached_response=bool(cached_stale),
        )
        if cached_stale is not None and fallback_error:
            cached_stale["error"] = fallback_error
            return cached_stale
        return {
            "success": False,
            "error": fallback_error or str(exc),
            "builds": [],
            "releases": [],
            "artifacts": [],
            "artifact_page": artifact_page,
            "artifact_page_size": artifact_page_size,
            "artifact_has_more": False,
        }


async def download_build(
    ctx: AppContext,
    *,
    target_id: str | None,
    tag: str = "",
    source: str = "release",
    artifact_id: str = "",
) -> dict:
    if not ctx.operator_config.github_repo:
        return {"success": False, "error": "GITHUB_REPO not configured"}

    tmp_path = ""
    try:
        httpx = _require_httpx()
        headers = _github_headers(ctx)
        artifact_name = ""
        extract_dir: Path | None = None
        async with httpx.AsyncClient(timeout=300, follow_redirects=True) as client:
            if source == "actions":
                if not artifact_id:
                    return {
                        "success": False,
                        "error": "Downloading a GitHub Actions artifact requires artifact_id.",
                    }
                download_url = f"https://api.github.com/repos/{ctx.operator_config.github_repo}/actions/artifacts/{artifact_id}/zip"
                with tempfile.NamedTemporaryFile(suffix=".zip", delete=False) as tmp:
                    tmp_path = tmp.name
                async with client.stream("GET", download_url, headers=headers) as resp:
                    resp.raise_for_status()
                    with open(tmp_path, "wb") as out:
                        async for chunk in resp.aiter_bytes():
                            out.write(chunk)

                extract_dir = Path(
                    tempfile.mkdtemp(prefix="integration-actions-artifact-")
                )
                with zipfile.ZipFile(tmp_path) as bundle:
                    bundle.extractall(extract_dir)
                tarballs = sorted(extract_dir.rglob("*.tar.gz"))
                if not tarballs:
                    return {
                        "success": False,
                        "error": "GitHub Actions artifact did not contain a .tar.gz deploy bundle.",
                    }
                local_tarball = tarballs[0]
                artifact_name = sanitize_artifact_name(local_tarball.name)
                result = await _deploy_artifact_path(
                    ctx,
                    target_id=target_id,
                    local_path=str(local_tarball),
                    artifact_name=artifact_name,
                    source_type="actions",
                    source_label=f"actions-artifact-{artifact_id}",
                )
                return result

            resp = await client.get(
                f"https://api.github.com/repos/{ctx.operator_config.github_repo}/releases/tags/{tag}",
                headers=headers,
            )
            resp.raise_for_status()
            release = resp.json()
            assets = release.get("assets", [])
            if not assets:
                return {"success": False, "error": f"No assets found for {tag}"}
            download_url = assets[0]["browser_download_url"]
            artifact_name = sanitize_artifact_name(assets[0]["name"])

            with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tmp:
                tmp_path = tmp.name
            async with client.stream("GET", download_url, headers=headers) as resp:
                resp.raise_for_status()
                with open(tmp_path, "wb") as out:
                    async for chunk in resp.aiter_bytes():
                        out.write(chunk)

        return await _deploy_artifact_path(
            ctx,
            target_id=target_id,
            local_path=tmp_path,
            artifact_name=artifact_name,
            source_type="release",
            source_label=tag or artifact_name,
        )
    except RuntimeError as exc:
        return {"success": False, "error": str(exc)}
    except Exception as exc:
        return {"success": False, "error": str(exc)}
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)
        if "extract_dir" in locals() and extract_dir is not None:
            shutil.rmtree(extract_dir, ignore_errors=True)


async def rollback_build(ctx: AppContext, *, target_id: str | None = None) -> dict:
    target_ctx = ctx.resolve_target(target_id)
    try:
        paths = target_ctx.target.deploy_paths()
        rollback_cmd = f"""
            set -e
            current_link={target_ctx.ssh.q(paths["current_link"])}
            previous_link={target_ctx.ssh.q(paths["previous_link"])}
            if [ ! -L "$previous_link" ] && [ ! -d "$previous_link" ]; then
                echo "NO_PREVIOUS"
                exit 1
            fi
            current_target=""
            if [ -L "$current_link" ] || [ -d "$current_link" ]; then
                current_target="$(readlink -f "$current_link" 2>/dev/null || echo "$current_link")"
            fi
            previous_target="$(readlink -f "$previous_link" 2>/dev/null || echo "$previous_link")"
            ln -sfn "$previous_target" "$current_link"
            if [ -n "$current_target" ] && [ -d "$current_target" ]; then
                ln -sfn "$current_target" "$previous_link"
            fi
            echo "__RELEASE_DIR__:$previous_target"
            echo "__PREVIOUS_TARGET__:$current_target"
        """
        result = await target_ctx.ssh.run(
            f"bash -lc {target_ctx.ssh.q(rollback_cmd)}", timeout=20
        )
        if result.returncode != 0:
            if "NO_PREVIOUS" in (result.stdout or ""):
                return {
                    "success": False,
                    "error": "No previous release to roll back to",
                }
            return {
                "success": False,
                "error": target_ctx.ssh.friendly_error(result.stderr or result.stdout),
            }

        release_dir = ""
        previous_target = ""
        for line in (result.stdout or "").splitlines():
            if line.startswith("__RELEASE_DIR__:"):
                release_dir = line.split(":", 1)[1].strip()
            elif line.startswith("__PREVIOUS_TARGET__:"):
                previous_target = line.split(":", 1)[1].strip()
        await _activate_release(
            target_ctx, release_dir=release_dir, previous_target=previous_target
        )
        return {"success": True, "output": "Rolled back to previous release"}
    except Exception as exc:
        return {"success": False, "error": target_ctx.ssh.friendly_error(str(exc))}
