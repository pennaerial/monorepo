from __future__ import annotations

from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .context import create_context
from .routers import (
    config_router,
    connection_router,
    deploy_router,
    mission_router,
    terminal_ws_router,
    wifi_router,
)


def create_app(base_dir: Path) -> FastAPI:
    app = FastAPI(title="PennAiR Auton Deploy")

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    ctx = create_context(base_dir)

    app.include_router(config_router(ctx))
    app.include_router(connection_router(ctx))
    app.include_router(wifi_router(ctx))
    app.include_router(deploy_router(ctx))
    app.include_router(mission_router(ctx))
    app.include_router(terminal_ws_router(ctx))

    frontend_dist = base_dir / "frontend" / "dist"
    if frontend_dist.exists():
        app.mount("/", StaticFiles(directory=str(frontend_dist), html=True), name="frontend")

    return app
