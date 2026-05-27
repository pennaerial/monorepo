from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Form

from ..context import AppContext
from ..models import (
    ConfigResponse,
    MessageResponse,
    OperatorConfigPayload,
)


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/config", tags=["config"])

    @router.get("", response_model=ConfigResponse)
    async def get_config() -> ConfigResponse:
        return ConfigResponse(
            config=OperatorConfigPayload.model_validate(
                ctx.operator_config.to_safe_dict()
            ),
        )

    @router.post("", response_model=MessageResponse)
    async def set_config(
        github_repo: Annotated[str | None, Form()] = None,
        github_token: Annotated[str | None, Form()] = None,
        hotspot_name: Annotated[str | None, Form()] = None,
        default_deploy_root: Annotated[str | None, Form()] = None,
        default_pi_user: Annotated[str | None, Form()] = None,
        default_ssh_key: Annotated[str | None, Form()] = None,
        default_ssh_pass: Annotated[str | None, Form()] = None,
    ) -> MessageResponse:
        updates = ctx.operator_config.update_from_form(
            {
                "github_repo": github_repo,
                "github_token": github_token,
                "hotspot_name": hotspot_name,
                "default_deploy_root": default_deploy_root,
                "default_pi_user": default_pi_user,
                "default_ssh_key": default_ssh_key,
                "default_ssh_pass": default_ssh_pass,
            }
        )
        if updates:
            ctx.inventory.save()
        out = f"Updated: {', '.join(updates.keys())}" if updates else "No changes"
        return MessageResponse(output=out)

    return router
