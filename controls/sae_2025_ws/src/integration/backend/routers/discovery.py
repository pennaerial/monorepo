from __future__ import annotations

from fastapi import APIRouter

from ..context import AppContext
from ..models import HardwareDiscoveryResponse


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/hardware", tags=["hardware"])

    @router.get("/live", response_model=HardwareDiscoveryResponse)
    async def live_hardware() -> HardwareDiscoveryResponse:
        from .. import discovery

        try:
            devices = await discovery.live_hardware_cards(ctx)
            return HardwareDiscoveryResponse(success=True, devices=devices)
        except Exception as exc:
            return HardwareDiscoveryResponse(success=False, error=str(exc))

    return router
