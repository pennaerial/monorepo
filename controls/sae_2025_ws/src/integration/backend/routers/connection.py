from __future__ import annotations

import subprocess

from fastapi import APIRouter

from ..context import AppContext
from ..models import ConnectionStatusResponse, SSHCommandResponse


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/connection", tags=["connection"])

    @router.get("/status", response_model=ConnectionStatusResponse)
    async def connection_status() -> ConnectionStatusResponse:
        try:
            result = await ctx.ssh.run("echo ok", timeout=5)
            if result.returncode == 0:
                info = await ctx.ssh.run("hostname && hostname -I", timeout=5)
                return ConnectionStatusResponse(
                    connected=True,
                    target=ctx.config.ssh_target(),
                    info=info.stdout.strip() if info.returncode == 0 else "",
                )
            return ConnectionStatusResponse(
                connected=False,
                error=ctx.ssh.friendly_error(result.stderr),
            )
        except subprocess.TimeoutExpired:
            return ConnectionStatusResponse(
                connected=False, error=ctx.ssh.friendly_timeout()
            )
        except Exception as exc:
            return ConnectionStatusResponse(
                connected=False, error=ctx.ssh.friendly_error(str(exc))
            )

    @router.get("/ssh-command", response_model=SSHCommandResponse)
    async def ssh_command() -> SSHCommandResponse:
        cmd = "ssh"
        if ctx.config.ssh_key:
            cmd += f" -i {ctx.config.ssh_key}"
        cmd += f" {ctx.config.ssh_target()}"
        return SSHCommandResponse(command=cmd)

    return router
