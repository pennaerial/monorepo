from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Form, HTTPException, Query

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


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(tags=["mission"])

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

    @router.get("/api/mission/state", response_model=MissionStateResponse)
    async def mission_state(
        target_id: Annotated[str | None, Query()] = None,
        hostname: Annotated[str | None, Query()] = None,
        vehicle_name: Annotated[str | None, Query()] = None,
    ) -> MissionStateResponse:
        from ..services import mission as mission_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        return MissionStateResponse.model_validate(
            await mission_service.mission_state(ctx, target_ctx=target_ctx)
        )

    @router.get(
        "/api/mission/launch/status", response_model=MissionLaunchStatusResponse
    )
    async def mission_launch_status(
        target_id: Annotated[str | None, Query()] = None,
        hostname: Annotated[str | None, Query()] = None,
        vehicle_name: Annotated[str | None, Query()] = None,
    ) -> MissionLaunchStatusResponse:
        from ..services import mission as mission_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        return MissionLaunchStatusResponse.model_validate(
            await mission_service.launch_status(ctx, target_ctx=target_ctx)
        )

    @router.get("/api/mission/launch/logs", response_model=MissionLogsResponse)
    async def mission_launch_logs(
        target_id: Annotated[str | None, Query()] = None,
        hostname: Annotated[str | None, Query()] = None,
        vehicle_name: Annotated[str | None, Query()] = None,
        lines: Annotated[int, Query()] = 200,
        offset: Annotated[int | None, Query()] = None,
        inode: Annotated[int | None, Query()] = None,
    ) -> MissionLogsResponse:
        from ..services import mission as mission_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        return MissionLogsResponse.model_validate(
            await mission_service.launch_logs(
                ctx,
                target_ctx=target_ctx,
                lines=lines,
                offset=offset,
                inode=inode,
            )
        )

    @router.post(
        "/api/mission/prepare", response_model=MessageResponse | MissionLogsResponse
    )
    async def prepare_mission(
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
    ):
        from ..services import mission as mission_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        result = await mission_service.prepare_mission(ctx, target_ctx=target_ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate(
            {**result, "running": False, "logs": ""}
        )

    @router.post(
        "/api/mission/stop", response_model=MessageResponse | MissionLogsResponse
    )
    async def stop_mission(
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
    ):
        from ..services import mission as mission_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        result = await mission_service.stop_mission(ctx, target_ctx=target_ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate(
            {**result, "running": False, "logs": ""}
        )

    @router.post(
        "/api/mission/start", response_model=MessageResponse | MissionLogsResponse
    )
    async def start_mission(
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
    ):
        from ..services import mission as mission_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        result = await mission_service.start_mission(ctx, target_ctx=target_ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate(
            {**result, "running": False, "logs": ""}
        )

    @router.post("/api/failsafe", response_model=MessageResponse | MissionLogsResponse)
    async def trigger_failsafe(
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
    ):
        from ..services import mission as mission_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        result = await mission_service.trigger_failsafe(ctx, target_ctx=target_ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate(
            {**result, "running": False, "logs": ""}
        )

    @router.get("/api/mission/launch-params", response_model=LaunchParamsResponse)
    async def get_launch_params(
        target_id: Annotated[str | None, Query()] = None,
        hostname: Annotated[str | None, Query()] = None,
        vehicle_name: Annotated[str | None, Query()] = None,
    ) -> LaunchParamsResponse:
        from ..services import mission as mission_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        return LaunchParamsResponse.model_validate(
            await mission_service.get_launch_params(ctx, target_ctx=target_ctx)
        )

    @router.get("/api/mission/mission-names", response_model=MissionNameOptionsResponse)
    async def get_mission_names(
        target_id: Annotated[str | None, Query()] = None,
        hostname: Annotated[str | None, Query()] = None,
        vehicle_name: Annotated[str | None, Query()] = None,
    ) -> MissionNameOptionsResponse:
        from ..services import mission as mission_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        return MissionNameOptionsResponse.model_validate(
            await mission_service.list_mission_names(ctx, target_ctx=target_ctx)
        )

    @router.post("/api/mission/launch-params", response_model=LaunchParamsResponse)
    async def set_launch_params(
        content: Annotated[str, Form(...)],
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
    ):
        from ..services import mission as mission_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        return LaunchParamsResponse.model_validate(
            await mission_service.set_launch_params(
                ctx, target_ctx=target_ctx, content=content
            )
        )

    @router.get("/api/mission/mission-file", response_model=MissionFileResponse)
    async def get_mission_file(
        name: Annotated[str, Query(...)],
        target_id: Annotated[str | None, Query()] = None,
        hostname: Annotated[str | None, Query()] = None,
        vehicle_name: Annotated[str | None, Query()] = None,
    ) -> MissionFileResponse:
        from ..services import mission as mission_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        return MissionFileResponse.model_validate(
            await mission_service.get_mission_file(
                ctx, target_ctx=target_ctx, name=name
            )
        )

    @router.post("/api/mission/mission-file", response_model=MissionFileResponse)
    async def set_mission_file(
        name: Annotated[str, Form(...)],
        content: Annotated[str, Form(...)],
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
    ) -> MissionFileResponse:
        from ..services import mission as mission_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        return MissionFileResponse.model_validate(
            await mission_service.set_mission_file(
                ctx, target_ctx=target_ctx, name=name, content=content
            )
        )

    return router
