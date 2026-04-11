from __future__ import annotations

import json
from typing import Annotated

from fastapi import APIRouter, File, Form, HTTPException, UploadFile
from fastapi.responses import Response

from ..context import AppContext
from ..models import InventoryMutationResponse, InventoryResponse, TargetPayload


def _target_payloads(ctx: AppContext) -> list[TargetPayload]:
    return [
        TargetPayload.model_validate(target.to_safe_dict())
        for target in ctx.list_targets()
    ]


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/inventory", tags=["inventory"])

    @router.get("", response_model=InventoryResponse)
    async def list_inventory() -> InventoryResponse:
        return InventoryResponse(
            active_target_id=ctx.inventory.active_target_id(),
            targets=_target_payloads(ctx),
        )

    @router.get("/export")
    async def export_inventory() -> Response:
        payload = ctx.inventory.export_payload()
        return Response(
            content=json.dumps(payload, indent=2) + "\n",
            media_type="application/json",
            headers={
                "Content-Disposition": (
                    'attachment; filename="integration-inventory.json"'
                )
            },
        )

    @router.post("", response_model=InventoryMutationResponse)
    async def upsert_target(
        target_id: Annotated[str, Form(...)],
        label: Annotated[str | None, Form()] = None,
        pi_user: Annotated[str | None, Form()] = None,
        pi_host: Annotated[str | None, Form()] = None,
        deploy_root: Annotated[str | None, Form()] = None,
        ssh_key: Annotated[str | None, Form()] = None,
        ssh_pass: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
        overlay_yaml: Annotated[str | None, Form()] = None,
        service_unit: Annotated[str | None, Form()] = None,
        enabled: Annotated[str | None, Form()] = None,
    ) -> InventoryMutationResponse:
        target, created = ctx.inventory.upsert_target(
            {
                "target_id": target_id,
                "label": label,
                "pi_user": pi_user,
                "pi_host": pi_host,
                "deploy_root": deploy_root,
                "ssh_key": ssh_key,
                "ssh_pass": ssh_pass,
                "vehicle_name": vehicle_name,
                "overlay_yaml": overlay_yaml,
                "service_unit": service_unit,
                "enabled": enabled,
            }
        )
        return InventoryMutationResponse(
            output=("Created" if created else "Updated")
            + f" target {target.target_id}",
            target=TargetPayload.model_validate(target.to_safe_dict()),
            active_target_id=ctx.inventory.active_target_id(),
        )

    @router.post("/active", response_model=InventoryMutationResponse)
    async def set_active_target(
        target_id: Annotated[str, Form(...)],
    ) -> InventoryMutationResponse:
        target = ctx.inventory.set_active_target(target_id)
        return InventoryMutationResponse(
            output=f"Active target set to {target.target_id}",
            target=TargetPayload.model_validate(target.to_safe_dict()),
            active_target_id=target.target_id,
        )

    @router.post("/delete", response_model=InventoryMutationResponse)
    async def delete_target(
        target_id: Annotated[str, Form(...)],
    ) -> InventoryMutationResponse:
        ctx.inventory.delete_target(target_id)
        return InventoryMutationResponse(
            output=f"Deleted target {target_id}",
            active_target_id=ctx.inventory.active_target_id(),
        )

    @router.post("/import", response_model=InventoryMutationResponse)
    async def import_inventory(
        file: UploadFile = File(...),
        replace_existing: Annotated[str | None, Form()] = None,
    ) -> InventoryMutationResponse:
        try:
            payload = json.loads((await file.read()).decode("utf-8"))
            summary = ctx.inventory.import_payload(
                payload,
                replace_existing=str(replace_existing or "").strip().lower()
                in {"1", "true", "yes", "on"},
            )
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except json.JSONDecodeError as exc:
            raise HTTPException(
                status_code=400,
                detail="Inventory import file must be valid JSON.",
            ) from exc

        return InventoryMutationResponse(
            output=(
                f"Imported {summary['targets_imported']} target(s): "
                + ", ".join(summary["target_ids"])
            ),
            active_target_id=summary["active_target_id"],
        )

    return router
