from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Form

from ..context import AppContext
from ..models import (
    ConfigResponse,
    MessageResponse,
)


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/config", tags=["config"])

    @router.get("", response_model=ConfigResponse)
    async def get_config() -> ConfigResponse:
        active_target = ctx.resolve_target().target
        return ConfigResponse(
            config=ctx.operator_config.to_safe_dict(),
            active_target_id=ctx.inventory.active_target_id(),
            workspace_paths=active_target.workspace_paths(),
        )

    @router.post("", response_model=MessageResponse)
    async def set_config(
        github_repo: Annotated[str | None, Form()] = None,
        github_token: Annotated[str | None, Form()] = None,
        hotspot_name: Annotated[str | None, Form()] = None,
        default_deploy_root: Annotated[str | None, Form()] = None,
    ) -> MessageResponse:
        updates = ctx.operator_config.update_from_form(
            {
                "github_repo": github_repo,
                "github_token": github_token,
                "hotspot_name": hotspot_name,
                "default_deploy_root": default_deploy_root,
            }
        )
        if updates:
            ctx.inventory.save()
        out = f"Updated: {', '.join(updates.keys())}" if updates else "No changes"
        return MessageResponse(output=out)

    return router
