from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Form, HTTPException, Query

from ..context import AppContext
from ..models import MessageResponse, WifiScanResponse, WifiStatusResponse
from ..services import wifi as wifi_service


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/wifi", tags=["wifi"])

    def _resolve_target_context(
        target_id: str | None,
        *,
        hostname: str | None = None,
        vehicle_name: str | None = None,
    ):
        try:
            return ctx.resolve_live_target(
                hostname=hostname or target_id,
                vehicle_name=vehicle_name,
            )
        except (KeyError, ValueError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @router.get("/status", response_model=WifiStatusResponse)
    async def wifi_status(
        target_id: str | None = Query(default=None),
        hostname: str | None = Query(default=None),
        vehicle_name: str | None = Query(default=None),
    ) -> WifiStatusResponse:
        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        return WifiStatusResponse.model_validate(
            await wifi_service.wifi_status(ctx, target_ctx=target_ctx)
        )

    @router.get("/scan", response_model=WifiScanResponse)
    async def wifi_scan(
        target_id: str | None = Query(default=None),
        hostname: str | None = Query(default=None),
        vehicle_name: str | None = Query(default=None),
    ) -> WifiScanResponse:
        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        return WifiScanResponse.model_validate(
            await wifi_service.wifi_scan(ctx, target_ctx=target_ctx)
        )

    @router.post("/connect", response_model=MessageResponse | WifiScanResponse)
    async def wifi_connect(
        ssid: Annotated[str, Form(...)],
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
        password: Annotated[str, Form()] = "",
    ):
        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        result = await wifi_service.wifi_connect(
            ctx,
            target_ctx=target_ctx,
            ssid=ssid,
            password=password,
        )
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return WifiScanResponse.model_validate({**result, "networks": []})

    @router.post("/hotspot", response_model=MessageResponse | WifiScanResponse)
    async def wifi_hotspot(
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
    ):
        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        result = await wifi_service.wifi_hotspot(ctx, target_ctx=target_ctx)
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
