from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Form

from ..context import AppContext
from ..models import MessageResponse, WifiScanResponse, WifiStatusResponse
from ..services import wifi as wifi_service


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/wifi", tags=["wifi"])

    @router.get("/status", response_model=WifiStatusResponse)
    async def wifi_status() -> WifiStatusResponse:
        return WifiStatusResponse.model_validate(await wifi_service.wifi_status(ctx))

    @router.get("/scan", response_model=WifiScanResponse)
    async def wifi_scan() -> WifiScanResponse:
        return WifiScanResponse.model_validate(await wifi_service.wifi_scan(ctx))

    @router.post("/connect", response_model=MessageResponse | WifiScanResponse)
    async def wifi_connect(
        ssid: Annotated[str, Form(...)],
        password: Annotated[str, Form()] = "",
    ):
        result = await wifi_service.wifi_connect(ctx, ssid=ssid, password=password)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return WifiScanResponse.model_validate({**result, "networks": []})

    @router.post("/hotspot", response_model=MessageResponse | WifiScanResponse)
    async def wifi_hotspot():
        result = await wifi_service.wifi_hotspot(ctx)
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
