from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Form

from ..context import AppContext
from ..models import ConfigPayload, ConfigResponse, MessageResponse


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/config", tags=["config"])

    @router.get("", response_model=ConfigResponse)
    async def get_config() -> ConfigResponse:
        return ConfigResponse(config=ConfigPayload.model_validate(ctx.config.to_safe_dict()))

    @router.post("", response_model=MessageResponse)
    async def set_config(
        pi_user: Annotated[str | None, Form()] = None,
        pi_host: Annotated[str | None, Form()] = None,
        remote_dir: Annotated[str | None, Form()] = None,
        ssh_key: Annotated[str | None, Form()] = None,
        ssh_pass: Annotated[str | None, Form()] = None,
        github_repo: Annotated[str | None, Form()] = None,
        hotspot_name: Annotated[str | None, Form()] = None,
    ) -> MessageResponse:
        updates = ctx.config.update_from_form(
            {
                "pi_user": pi_user,
                "pi_host": pi_host,
                "remote_dir": remote_dir,
                "ssh_key": ssh_key,
                "ssh_pass": ssh_pass,
                "github_repo": github_repo,
                "hotspot_name": hotspot_name,
            }
        )
        out = f"Updated: {', '.join(updates.keys())}" if updates else "No changes"
        return MessageResponse(output=out)

    return router
