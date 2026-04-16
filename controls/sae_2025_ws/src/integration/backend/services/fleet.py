from __future__ import annotations

import asyncio
import subprocess
from typing import Any

from .. import discovery
from ..context import AppContext, TargetContext
from ..models import (
    BuildSourceResponse,
    FleetActionDeviceResponse,
    FleetActionSummary,
    FleetBoardDeviceResponse,
    FleetBoardSummary,
    FleetCatalogResponse,
    FleetConnectionSummary,
    FleetCurrentBuildSummary,
    FleetDeviceSelection,
    FleetReadinessSummary,
    FleetRuntimeSummary,
    FleetVehiclePreview,
)
from . import deploy as deploy_service
from . import mission as mission_service


def _fleet_vehicle_lookup(
    fleet_vehicles: list[FleetVehiclePreview],
) -> dict[str, FleetVehiclePreview]:
    return {vehicle.name: vehicle for vehicle in fleet_vehicles if vehicle.name.strip()}


def _current_build_info(build: dict[str, Any]) -> FleetCurrentBuildSummary:
    return FleetCurrentBuildSummary.model_validate(
        {
            "success": bool(build.get("success", False)),
            "installed": bool(build.get("installed", False)),
            "info": str(build.get("info", "") or ""),
            "release_id": build.get("release_id"),
        }
    )


def _runtime_info(runtime: dict[str, Any]) -> FleetRuntimeSummary:
    return FleetRuntimeSummary.model_validate(
        {
            "success": bool(runtime.get("success", False)),
            "running": bool(runtime.get("running", False)),
            "state": str(runtime.get("state", "error") or "error"),
            "pid": runtime.get("pid"),
            "error": runtime.get("error"),
        }
    )


def _connection_info(connection: dict[str, Any]) -> FleetConnectionSummary:
    return FleetConnectionSummary.model_validate(
        {
            "success": bool(connection.get("success", False)),
            "connected": bool(connection.get("connected", False)),
            "target": connection.get("target"),
            "info": str(connection.get("info", "") or ""),
            "error": connection.get("error"),
        }
    )


def _readiness_summary(
    *,
    connection: FleetConnectionSummary,
    current_build: FleetCurrentBuildSummary,
    runtime: FleetRuntimeSummary,
    vehicle_assigned: bool,
) -> FleetReadinessSummary:
    notes: list[str] = []
    if not connection.connected:
        notes.append("SSH unavailable")
    if not current_build.installed:
        notes.append("No installed build")
    if not vehicle_assigned:
        notes.append("No vehicle assigned")
    runtime_ready = runtime.success and runtime.state in {"stopped", "not_prepared"}
    if not runtime_ready:
        notes.append("Runtime not ready")
    ready = (
        connection.connected
        and current_build.installed
        and vehicle_assigned
        and runtime_ready
    )
    return FleetReadinessSummary(
        connected=connection.connected,
        build_installed=current_build.installed,
        runtime_ready=runtime_ready,
        vehicle_assigned=vehicle_assigned,
        ready=ready,
        notes=notes,
    )


async def _resolve_device_context(
    ctx: AppContext,
    device: dict[str, Any],
) -> TargetContext:
    hostname = (
        str(
            device.get("hostname")
            or device.get("target_id")
            or device.get("matched_target_id")
            or ""
        ).strip()
        or None
    )
    vehicle_name = str(device.get("vehicle_name", "") or "").strip() or None
    label = str(device.get("label", "") or "").strip() or None
    return ctx.resolve_live_target(
        hostname=hostname,
        vehicle_name=vehicle_name,
        label=label,
    )


async def _summarize_device(
    ctx: AppContext,
    device: dict[str, Any],
    fleet_vehicle_lookup: dict[str, FleetVehiclePreview],
) -> dict[str, Any]:
    target_ctx = await _resolve_device_context(ctx, device)
    vehicle_name = target_ctx.target.vehicle_name.strip() or None
    selected_vehicle = fleet_vehicle_lookup.get(vehicle_name) if vehicle_name else None

    async def _connection() -> dict[str, Any]:
        try:
            result = await target_ctx.ssh.run("echo ok", timeout=5)
            if result.returncode == 0:
                info = await target_ctx.ssh.run("hostname && hostname -I", timeout=5)
                return {
                    "success": True,
                    "connected": True,
                    "target": target_ctx.target.ssh_target(),
                    "info": info.stdout.strip() if info.returncode == 0 else "",
                    "error": None,
                }
            return {
                "success": False,
                "connected": False,
                "target": target_ctx.target.ssh_target(),
                "info": "",
                "error": target_ctx.ssh.friendly_error(result.stderr),
            }
        except subprocess.TimeoutExpired:
            return {
                "success": False,
                "connected": False,
                "target": target_ctx.target.ssh_target(),
                "info": "",
                "error": target_ctx.ssh.friendly_timeout(),
            }
        except Exception as exc:
            return {
                "success": False,
                "connected": False,
                "target": target_ctx.target.ssh_target(),
                "info": "",
                "error": target_ctx.ssh.friendly_error(str(exc)),
            }

    connection_result, current_build_result, runtime_result = await asyncio.gather(
        _connection(),
        deploy_service.current_build(ctx, target_ctx=target_ctx),
        mission_service.launch_status(ctx, target_ctx=target_ctx),
    )

    connection = _connection_info(connection_result)
    current_build = _current_build_info(current_build_result)
    runtime = _runtime_info(runtime_result)
    readiness = _readiness_summary(
        connection=connection,
        current_build=current_build,
        runtime=runtime,
        vehicle_assigned=bool(vehicle_name),
    )

    payload = {
        "hardware_id": str(device.get("hardware_id") or target_ctx.target.target_id),
        "hostname": str(device.get("hostname") or target_ctx.target.pi_host),
        "addresses": list(device.get("addresses", []) or []),
        "service_name": device.get("service_name"),
        "service_type": device.get("service_type"),
        "last_seen_at": device.get("last_seen_at"),
        "discovery_stale": bool(device.get("discovery_stale", False)),
        "target_id": target_ctx.target.target_id,
        "vehicle_name": vehicle_name,
        "fleet_vehicle": (
            selected_vehicle.model_dump() if selected_vehicle is not None else None
        ),
        "connection": connection.model_dump(),
        "current_build": current_build.model_dump(),
        "runtime": runtime.model_dump(),
        "readiness": readiness.model_dump(),
    }
    return payload


def _failed_device_row(
    selection: FleetDeviceSelection,
    *,
    action: str,
    error: str,
) -> dict[str, Any]:
    hostname = str(selection.hostname or selection.target_id or "").strip()
    return {
        **_device_selection_identity(selection),
        "hardware_id": hostname,
        "hostname": hostname,
        "addresses": [],
        "service_name": None,
        "service_type": None,
        "last_seen_at": None,
        "discovery_stale": False,
        "target_id": hostname,
        "vehicle_name": selection.vehicle_name,
        "fleet_vehicle": None,
        "connection": {
            "success": False,
            "connected": False,
            "target": None,
            "info": "",
            "error": error,
        },
        "current_build": {
            "success": False,
            "installed": False,
            "info": error,
            "release_id": None,
        },
        "runtime": {
            "success": False,
            "running": False,
            "state": "error",
            "pid": None,
            "error": error,
        },
        "readiness": FleetReadinessSummary(
            connected=False,
            build_installed=False,
            runtime_ready=False,
            vehicle_assigned=bool(selection.vehicle_name),
            ready=False,
            notes=["Device refresh failed"],
        ).model_dump(),
        "action": action,
        "success": False,
        "attempted_release_id": None,
        "active_release_id": None,
        "rolled_back": False,
        "output": None,
        "error": error,
    }


async def _build_snapshot(ctx: AppContext) -> dict[str, Any]:
    build_source = BuildSourceResponse.model_validate(
        await deploy_service.get_build_source(ctx)
    )
    fleet_catalog = FleetCatalogResponse.model_validate(
        await deploy_service.get_fleet_catalog(ctx)
    )
    fleet_vehicles = [
        FleetVehiclePreview.model_validate(vehicle)
        for vehicle in fleet_catalog.fleet_vehicles
    ]
    return {
        "build_source": build_source,
        "fleet_catalog": fleet_catalog.model_copy(
            update={"fleet_vehicles": fleet_vehicles}
        ),
        "fleet_vehicles": fleet_vehicles,
        "vehicle_lookup": _fleet_vehicle_lookup(fleet_vehicles),
    }


async def get_board(ctx: AppContext) -> dict[str, Any]:
    snapshot = await _build_snapshot(ctx)
    try:
        devices = await discovery.live_hardware_cards(ctx)
        rows = await asyncio.gather(
            *[
                _summarize_device(
                    ctx,
                    device,
                    snapshot["vehicle_lookup"],
                )
                for device in devices
            ],
            return_exceptions=True,
        )
        board_devices = []
        for device, row in zip(devices, rows):
            if isinstance(row, Exception):
                board_devices.append(
                    FleetBoardDeviceResponse.model_validate(
                        {
                            **device,
                            "target_id": device.get("hostname")
                            or device.get("hardware_id"),
                            "last_seen_at": device.get("last_seen_at"),
                            "discovery_stale": bool(
                                device.get("discovery_stale", False)
                            ),
                            "vehicle_name": None,
                            "fleet_vehicle": None,
                            "connection": {
                                "success": False,
                                "connected": False,
                                "target": None,
                                "info": "",
                                "error": str(row),
                            },
                            "current_build": {
                                "success": False,
                                "installed": False,
                                "info": str(row),
                                "release_id": None,
                            },
                            "runtime": {
                                "success": False,
                                "running": False,
                                "state": "error",
                                "pid": None,
                                "error": str(row),
                            },
                            "readiness": FleetReadinessSummary(
                                connected=False,
                                build_installed=False,
                                runtime_ready=False,
                                vehicle_assigned=False,
                                ready=False,
                                notes=["Board row failed to load"],
                            ).model_dump(),
                        }
                    )
                )
            else:
                board_devices.append(FleetBoardDeviceResponse.model_validate(row))
        summary = FleetBoardSummary(
            total_devices=len(board_devices),
            stale_devices=sum(1 for row in board_devices if row.discovery_stale),
            connected_devices=sum(
                1
                for row in board_devices
                if row.connection and row.connection.connected
            ),
            build_installed_devices=sum(
                1
                for row in board_devices
                if row.current_build and row.current_build.installed
            ),
            running_devices=sum(
                1 for row in board_devices if row.runtime and row.runtime.running
            ),
            ready_devices=sum(
                1 for row in board_devices if row.readiness and row.readiness.ready
            ),
        )
        return {
            "success": True,
            "build_source": snapshot["build_source"],
            "fleet_catalog": snapshot["fleet_catalog"],
            "fleet_vehicles": snapshot["fleet_vehicles"],
            "devices": board_devices,
            "summary": summary,
            "error": None,
        }
    except Exception as exc:
        return {
            "success": False,
            "build_source": snapshot["build_source"],
            "fleet_catalog": snapshot["fleet_catalog"],
            "fleet_vehicles": snapshot["fleet_vehicles"],
            "devices": [],
            "summary": FleetBoardSummary(),
            "error": str(exc),
        }


def _device_selection_identity(selection: FleetDeviceSelection) -> dict[str, Any]:
    return {
        "hostname": selection.hostname,
        "vehicle_name": selection.vehicle_name,
    }


async def _resolve_action_devices(
    ctx: AppContext,
    devices: list[FleetDeviceSelection] | None,
) -> list[FleetDeviceSelection]:
    if devices:
        return devices
    board = await get_board(ctx)
    if not board["success"]:
        return []
    return [
        FleetDeviceSelection.model_validate(
            {
                "hostname": device.hostname,
                "vehicle_name": device.vehicle_name,
            }
        )
        for device in board["devices"]
    ]


async def _perform_action(
    ctx: AppContext,
    *,
    action: str,
    selection: FleetDeviceSelection,
    snapshot: dict[str, Any],
    session_assignments: dict[str, str] | None = None,
) -> dict[str, Any]:
    target_ctx = ctx.resolve_live_target(
        hostname=selection.hostname or selection.target_id,
        vehicle_name=selection.vehicle_name,
        label=selection.label,
        pi_user=selection.pi_user,
        deploy_root=selection.deploy_root,
        service_unit=selection.service_unit,
        ssh_key=selection.ssh_key,
        ssh_pass=selection.ssh_pass,
    )

    try:
        if action == "deploy":
            result = await deploy_service.deploy_selected_source(
                ctx,
                target_ctx=target_ctx,
                network_policy_override=selection.network_policy_override(),
                operator_allowed_ap_vehicles=(
                    None
                    if selection.allowed_ap_hosts is not None
                    else selection.allowed_ap_vehicles_override()
                ),
                session_assignments=session_assignments,
            )
        elif action == "prepare":
            result = await mission_service.prepare_mission(ctx, target_ctx=target_ctx)
        elif action == "start":
            result = await mission_service.start_mission(ctx, target_ctx=target_ctx)
        elif action == "stop":
            result = await mission_service.stop_mission(ctx, target_ctx=target_ctx)
        else:
            raise ValueError(f"Unsupported fleet action '{action}'.")
    except Exception as exc:
        result = {
            "success": False,
            "error": target_ctx.ssh.friendly_error(str(exc)),
        }

    refresh_failed = False
    try:
        refreshed = await _summarize_device(
            ctx,
            {
                "hardware_id": target_ctx.target.target_id,
                "hostname": target_ctx.target.pi_host,
                "vehicle_name": target_ctx.target.vehicle_name,
            },
            snapshot["vehicle_lookup"],
        )
    except Exception as exc:
        refresh_failed = True
        refreshed = _failed_device_row(
            selection,
            action=action,
            error=target_ctx.ssh.friendly_error(str(exc)),
        )

    current_build = refreshed.get("current_build") or {}
    active_release_id = (
        result.get("active_release_id") or current_build.get("release_id") or None
    )
    attempted_release_id = result.get("attempted_release_id")
    rolled_back = bool(result.get("rolled_back", False))
    if (
        action == "deploy"
        and attempted_release_id
        and active_release_id
        and active_release_id != attempted_release_id
    ):
        rolled_back = True

    refreshed["action"] = action
    refreshed["attempted_release_id"] = attempted_release_id
    refreshed["active_release_id"] = active_release_id
    refreshed["rolled_back"] = rolled_back if action == "deploy" else False
    refreshed["success"] = bool(result.get("success", False)) and not refresh_failed
    refreshed["output"] = result.get("output") if refreshed["success"] else None
    refreshed["error"] = (
        result.get("error") if result.get("error") else refreshed.get("error")
    )
    if not refreshed["success"] and refresh_failed and not refreshed["error"]:
        refreshed["error"] = target_ctx.ssh.friendly_error("Device refresh failed")
    return refreshed


async def batch_action(
    ctx: AppContext,
    *,
    action: str,
    devices: list[FleetDeviceSelection] | None = None,
    session_assignments: dict[str, str] | None = None,
) -> dict[str, Any]:
    snapshot = await _build_snapshot(ctx)
    selected_devices = await _resolve_action_devices(ctx, devices)
    if not selected_devices:
        return {
            "success": False,
            "action": action,
            "build_source": snapshot["build_source"],
            "fleet_catalog": snapshot["fleet_catalog"],
            "fleet_vehicles": snapshot["fleet_vehicles"],
            "results": [],
            "summary": FleetActionSummary(),
            "error": "No fleet devices available.",
        }

    results = await asyncio.gather(
        *[
            _perform_action(
                ctx,
                action=action,
                selection=selection,
                snapshot=snapshot,
                session_assignments=session_assignments,
            )
            for selection in selected_devices
        ],
        return_exceptions=True,
    )

    rows: list[FleetActionDeviceResponse] = []
    for selection, result in zip(selected_devices, results):
        if isinstance(result, Exception):
            row = FleetActionDeviceResponse.model_validate(
                _failed_device_row(selection, action=action, error=str(result))
            )
        else:
            row = FleetActionDeviceResponse.model_validate(result)
        rows.append(row)

    summary = FleetActionSummary(
        requested_devices=len(rows),
        successful_devices=sum(1 for row in rows if row.success),
        failed_devices=sum(1 for row in rows if not row.success),
    )
    return {
        "success": summary.failed_devices == 0,
        "action": action,
        "build_source": snapshot["build_source"],
        "fleet_catalog": snapshot["fleet_catalog"],
        "fleet_vehicles": snapshot["fleet_vehicles"],
        "results": rows,
        "summary": summary,
        "error": None
        if summary.failed_devices == 0
        else "One or more fleet actions failed.",
    }
