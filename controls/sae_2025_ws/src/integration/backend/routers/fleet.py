from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Body, Query

from ..context import AppContext
from ..models import (
    FleetBatchRequest,
    FleetActionResponse,
    FleetBoardResponse,
    FleetDeviceSelection,
)


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/fleet", tags=["fleet"])

    @router.get("/board", response_model=FleetBoardResponse)
    async def fleet_board() -> FleetBoardResponse:
        from ..services import fleet as fleet_service

        return FleetBoardResponse.model_validate(await fleet_service.get_board(ctx))

    @router.post("/deploy", response_model=FleetActionResponse)
    async def deploy_fleet(
        body: Annotated[FleetBatchRequest | None, Body()] = None,
        target_id: str | None = Query(default=None),
        hostname: str | None = Query(default=None),
        vehicle_name: str | None = Query(default=None),
    ) -> FleetActionResponse:
        from ..services import fleet as fleet_service

        devices = body.devices if body else None
        if not devices and (target_id or hostname):
            devices = [
                FleetDeviceSelection(
                    hostname=hostname or target_id,
                    vehicle_name=vehicle_name,
                )
            ]
        return FleetActionResponse.model_validate(
            await fleet_service.batch_action(
                ctx, action="deploy", devices=devices
            )
        )

    @router.post("/prepare", response_model=FleetActionResponse)
    async def prepare_fleet(
        body: Annotated[FleetBatchRequest | None, Body()] = None,
        target_id: str | None = Query(default=None),
        hostname: str | None = Query(default=None),
        vehicle_name: str | None = Query(default=None),
    ) -> FleetActionResponse:
        from ..services import fleet as fleet_service

        devices = body.devices if body else None
        if not devices and (target_id or hostname):
            devices = [
                FleetDeviceSelection(
                    hostname=hostname or target_id,
                    vehicle_name=vehicle_name,
                )
            ]
        return FleetActionResponse.model_validate(
            await fleet_service.batch_action(
                ctx, action="prepare", devices=devices
            )
        )

    @router.post("/start", response_model=FleetActionResponse)
    async def start_fleet(
        body: Annotated[FleetBatchRequest | None, Body()] = None,
        target_id: str | None = Query(default=None),
        hostname: str | None = Query(default=None),
        vehicle_name: str | None = Query(default=None),
    ) -> FleetActionResponse:
        from ..services import fleet as fleet_service

        devices = body.devices if body else None
        if not devices and (target_id or hostname):
            devices = [
                FleetDeviceSelection(
                    hostname=hostname or target_id,
                    vehicle_name=vehicle_name,
                )
            ]
        return FleetActionResponse.model_validate(
            await fleet_service.batch_action(
                ctx, action="start", devices=devices
            )
        )

    @router.post("/stop", response_model=FleetActionResponse)
    async def stop_fleet(
        body: Annotated[FleetBatchRequest | None, Body()] = None,
        target_id: str | None = Query(default=None),
        hostname: str | None = Query(default=None),
        vehicle_name: str | None = Query(default=None),
    ) -> FleetActionResponse:
        from ..services import fleet as fleet_service

        devices = body.devices if body else None
        if not devices and (target_id or hostname):
            devices = [
                FleetDeviceSelection(
                    hostname=hostname or target_id,
                    vehicle_name=vehicle_name,
                )
            ]
        return FleetActionResponse.model_validate(
            await fleet_service.batch_action(
                ctx, action="stop", devices=devices
            )
        )

    return router
