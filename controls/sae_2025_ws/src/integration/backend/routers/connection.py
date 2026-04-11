from __future__ import annotations

import subprocess

from fastapi import APIRouter, Query

from ..context import AppContext
from ..models import ConnectionStatusResponse, SSHCommandResponse


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/connection", tags=["connection"])

    @router.get("/status", response_model=ConnectionStatusResponse)
    async def connection_status(
        target_id: str | None = Query(default=None),
    ) -> ConnectionStatusResponse:
        session = ctx.resolve_target(target_id)
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
    ) -> SSHCommandResponse:
        session = ctx.resolve_target(target_id)
        cmd = "ssh"
        if session.target.ssh_key:
            cmd += f" -i {session.target.ssh_key}"
        cmd += f" {session.target.ssh_target()}"
        return SSHCommandResponse(command=cmd)

    return router
