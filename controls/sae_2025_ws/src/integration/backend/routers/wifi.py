from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Form, Query

from ..context import AppContext
from ..models import MessageResponse, WifiScanResponse, WifiStatusResponse
from ..services import wifi as wifi_service


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/wifi", tags=["wifi"])

    @router.get("/status", response_model=WifiStatusResponse)
    async def wifi_status(target_id: str = Query(...)) -> WifiStatusResponse:
        return WifiStatusResponse.model_validate(
            await wifi_service.wifi_status(ctx, target_id=target_id)
        )

    @router.get("/scan", response_model=WifiScanResponse)
    async def wifi_scan(target_id: str = Query(...)) -> WifiScanResponse:
        return WifiScanResponse.model_validate(
            await wifi_service.wifi_scan(ctx, target_id=target_id)
        )

    @router.post("/connect", response_model=MessageResponse | WifiScanResponse)
    async def wifi_connect(
        ssid: Annotated[str, Form(...)],
        target_id: Annotated[str, Form(...)],
        password: Annotated[str, Form()] = "",
    ):
        result = await wifi_service.wifi_connect(
            ctx, target_id=target_id, ssid=ssid, password=password
        )
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return WifiScanResponse.model_validate({**result, "networks": []})

    @router.post("/hotspot", response_model=MessageResponse | WifiScanResponse)
    async def wifi_hotspot(target_id: Annotated[str, Form(...)]):
        result = await wifi_service.wifi_hotspot(ctx, target_id=target_id)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return WifiScanResponse.model_validate({**result, "networks": []})

    @router.post("/switch-local", response_model=MessageResponse | WifiScanResponse)
    async def switch_local_wifi(
        ssid: Annotated[str, Form(...)],
        password: Annotated[str, Form()] = "",
    ):
        result = await wifi_service.switch_local_wifi(ssid=ssid, password=password)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return WifiScanResponse.model_validate({**result, "networks": []})

    return router
