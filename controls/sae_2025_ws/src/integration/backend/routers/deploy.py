from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, File, Form, UploadFile

from ..context import AppContext
from ..models import BuildCurrentResponse, BuildListResponse, MessageResponse


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/builds", tags=["builds"])

    @router.get("/current", response_model=BuildCurrentResponse)
    async def current_build(target_id: str | None = None) -> BuildCurrentResponse:
        from ..services import deploy as deploy_service

        return BuildCurrentResponse.model_validate(
            await deploy_service.current_build(ctx, target_id=target_id)
        )

    @router.post("/upload", response_model=MessageResponse | BuildListResponse)
    async def upload_build(
        file: UploadFile = File(...),
        target_id: Annotated[str | None, Form()] = None,
    ):
        from ..services import deploy as deploy_service

        result = await deploy_service.upload_build(
            ctx,
            target_id=target_id,
            filename=file.filename or "",
            file_bytes=await file.read(),
        )
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    @router.post("/upload-source", response_model=MessageResponse | BuildListResponse)
    async def upload_source_bundle(
        file: UploadFile = File(...),
        target_id: Annotated[str | None, Form()] = None,
    ):
        from ..services import deploy as deploy_service

        result = await deploy_service.upload_source_bundle(
            ctx,
            target_id=target_id,
            filename=file.filename or "",
            file_bytes=await file.read(),
        )
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    @router.get("/list", response_model=BuildListResponse)
    async def list_builds() -> BuildListResponse:
        from ..services import deploy as deploy_service

        return BuildListResponse.model_validate(await deploy_service.list_builds(ctx))

    @router.post("/download", response_model=MessageResponse | BuildListResponse)
    async def download_build(
        tag: Annotated[str, Form()] = "",
        source: Annotated[str, Form()] = "release",
        artifact_id: Annotated[str, Form()] = "",
        target_id: Annotated[str | None, Form()] = None,
    ):
        from ..services import deploy as deploy_service

        result = await deploy_service.download_build(
            ctx,
            target_id=target_id,
            tag=tag,
            source=source,
            artifact_id=artifact_id,
        )
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    @router.post("/rollback", response_model=MessageResponse | BuildListResponse)
    async def rollback_build(target_id: Annotated[str | None, Form()] = None):
        from ..services import deploy as deploy_service

        result = await deploy_service.rollback_build(ctx, target_id=target_id)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    return router
