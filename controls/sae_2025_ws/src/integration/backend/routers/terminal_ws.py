from __future__ import annotations

from fastapi import APIRouter, Query, WebSocket

from ..context import AppContext
from ..services import mission as mission_service


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(tags=["terminal"])

    @router.websocket("/ws/mission/terminal")
    async def mission_terminal_stream(
        websocket: WebSocket,
        offset: int = Query(default=0),
        inode: int = Query(default=0),
        target_id: str | None = Query(default=None),
    ):
        await mission_service.stream_terminal(
            ctx,
            websocket,
            target_id=target_id,
            offset=offset,
            inode=inode,
        )

    return router
