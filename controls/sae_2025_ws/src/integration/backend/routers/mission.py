from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Form, Query

from ..context import AppContext
from ..models import (
    LaunchParamsResponse,
    MessageResponse,
    MissionFileResponse,
    MissionLaunchStatusResponse,
    MissionLogsResponse,
    MissionNameOptionsResponse,
    MissionStateResponse,
)
from ..services import mission as mission_service


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(tags=["mission"])

    @router.get("/api/mission/state", response_model=MissionStateResponse)
    async def mission_state() -> MissionStateResponse:
        return MissionStateResponse.model_validate(
            await mission_service.mission_state(ctx)
        )

    @router.get(
        "/api/mission/launch/status", response_model=MissionLaunchStatusResponse
    )
    async def mission_launch_status() -> MissionLaunchStatusResponse:
        return MissionLaunchStatusResponse.model_validate(
            await mission_service.launch_status(ctx)
        )

    @router.get("/api/mission/launch/logs", response_model=MissionLogsResponse)
    async def mission_launch_logs(
        lines: Annotated[int, Query()] = 200,
        offset: Annotated[int | None, Query()] = None,
        inode: Annotated[int | None, Query()] = None,
    ) -> MissionLogsResponse:
        return MissionLogsResponse.model_validate(
            await mission_service.launch_logs(
                ctx,
                lines=lines,
                offset=offset,
                inode=inode,
            )
        )

    @router.post(
        "/api/mission/prepare", response_model=MessageResponse | MissionLogsResponse
    )
    async def prepare_mission():
        result = await mission_service.prepare_mission(ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate(
            {**result, "running": False, "logs": ""}
        )

    @router.post(
        "/api/mission/stop", response_model=MessageResponse | MissionLogsResponse
    )
    async def stop_mission():
        result = await mission_service.stop_mission(ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate(
            {**result, "running": False, "logs": ""}
        )

    @router.post(
        "/api/mission/start", response_model=MessageResponse | MissionLogsResponse
    )
    async def start_mission():
        result = await mission_service.start_mission(ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate(
            {**result, "running": False, "logs": ""}
        )

    @router.post("/api/failsafe", response_model=MessageResponse | MissionLogsResponse)
    async def trigger_failsafe():
        result = await mission_service.trigger_failsafe(ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate(
            {**result, "running": False, "logs": ""}
        )

    @router.get("/api/mission/launch-params", response_model=LaunchParamsResponse)
    async def get_launch_params() -> LaunchParamsResponse:
        return LaunchParamsResponse.model_validate(
            await mission_service.get_launch_params(ctx)
        )

    @router.get("/api/mission/mission-names", response_model=MissionNameOptionsResponse)
    async def get_mission_names() -> MissionNameOptionsResponse:
        return MissionNameOptionsResponse.model_validate(
            await mission_service.list_mission_names(ctx)
        )

    @router.post("/api/mission/launch-params", response_model=LaunchParamsResponse)
    async def set_launch_params(content: Annotated[str, Form(...)]):
        return LaunchParamsResponse.model_validate(
            await mission_service.set_launch_params(ctx, content=content)
        )

    @router.get("/api/mission/mission-file", response_model=MissionFileResponse)
    async def get_mission_file(name: Annotated[str, Query(...)]) -> MissionFileResponse:
        return MissionFileResponse.model_validate(
            await mission_service.get_mission_file(ctx, name=name)
        )

    @router.post("/api/mission/mission-file", response_model=MissionFileResponse)
    async def set_mission_file(
        name: Annotated[str, Form(...)], content: Annotated[str, Form(...)]
    ) -> MissionFileResponse:
        return MissionFileResponse.model_validate(
            await mission_service.set_mission_file(ctx, name=name, content=content)
        )

    return router
