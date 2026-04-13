from __future__ import annotations

import subprocess

from fastapi import APIRouter, HTTPException, Query

from ..context import AppContext
from ..models import ConnectionStatusResponse, SSHCommandResponse


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/connection", tags=["connection"])

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

    @router.get("/status", response_model=ConnectionStatusResponse)
    async def connection_status(
        target_id: str | None = Query(default=None),
        hostname: str | None = Query(default=None),
        vehicle_name: str | None = Query(default=None),
    ) -> ConnectionStatusResponse:
        session = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        try:
            result = await session.ssh.run("echo ok", timeout=5)
            if result.returncode == 0:
                info = await session.ssh.run("hostname && hostname -I", timeout=5)
                return ConnectionStatusResponse(
                    connected=True,
                    target=session.target.ssh_target(),
                    info=info.stdout.strip() if info.returncode == 0 else "",
                )
            return ConnectionStatusResponse(
                connected=False,
                error=session.ssh.friendly_error(result.stderr),
            )
        except subprocess.TimeoutExpired:
            return ConnectionStatusResponse(
                connected=False, error=session.ssh.friendly_timeout()
            )
        except Exception as exc:
            return ConnectionStatusResponse(
                connected=False, error=session.ssh.friendly_error(str(exc))
            )

    @router.get("/ssh-command", response_model=SSHCommandResponse)
    async def ssh_command(
        target_id: str | None = Query(default=None),
        hostname: str | None = Query(default=None),
        vehicle_name: str | None = Query(default=None),
    ) -> SSHCommandResponse:
        session = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        cmd = "ssh"
        if session.target.ssh_key:
            cmd += f" -i {session.target.ssh_key}"
        cmd += f" {session.target.ssh_target()}"
        return SSHCommandResponse(command=cmd)

    return router
