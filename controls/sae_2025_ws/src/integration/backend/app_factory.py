from __future__ import annotations

from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .context import create_context
from .routers import (
    build_source_router,
    config_router,
    connection_router,
    discovery_router,
    deploy_router,
    fleet_router,
    inventory_router,
    mission_router,
    schema_router,
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
    app.include_router(build_source_router(ctx))
    app.include_router(inventory_router(ctx))
    app.include_router(connection_router(ctx))
    app.include_router(discovery_router(ctx))
    app.include_router(wifi_router(ctx))
    app.include_router(fleet_router(ctx))
    app.include_router(deploy_router(ctx))
    app.include_router(mission_router(ctx))
    app.include_router(schema_router(ctx))
    app.include_router(terminal_ws_router(ctx))

    frontend_dist = base_dir / "frontend" / "dist"
    if frontend_dist.exists():
        app.mount(
            "/", StaticFiles(directory=str(frontend_dist), html=True), name="frontend"
        )

    return app
