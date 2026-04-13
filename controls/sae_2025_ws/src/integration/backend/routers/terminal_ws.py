from __future__ import annotations

from fastapi import APIRouter, HTTPException, Query, WebSocket

from ..context import AppContext
from ..services import mission as mission_service


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(tags=["terminal"])

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

    @router.websocket("/ws/mission/terminal")
    async def mission_terminal_stream(
        websocket: WebSocket,
        offset: int = Query(default=0),
        inode: int = Query(default=0),
        target_id: str | None = Query(default=None),
        hostname: str | None = Query(default=None),
        vehicle_name: str | None = Query(default=None),
    ):
        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        await mission_service.stream_terminal(
            ctx,
            websocket,
            target_ctx=target_ctx,
            offset=offset,
            inode=inode,
        )

    return router
