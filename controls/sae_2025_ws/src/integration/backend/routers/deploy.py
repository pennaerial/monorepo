from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, File, Form, UploadFile

from ..context import AppContext
from ..models import BuildCurrentResponse, BuildListResponse, MessageResponse
from ..services import deploy as deploy_service


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/builds", tags=["builds"])

    @router.get("/current", response_model=BuildCurrentResponse)
    async def current_build() -> BuildCurrentResponse:
        return BuildCurrentResponse.model_validate(await deploy_service.current_build(ctx))

    @router.post("/upload", response_model=MessageResponse | BuildListResponse)
    async def upload_build(file: UploadFile = File(...)):
        result = await deploy_service.upload_build(
            ctx,
            filename=file.filename or "",
            file_bytes=await file.read(),
        )
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    @router.get("/list", response_model=BuildListResponse)
    async def list_builds() -> BuildListResponse:
        return BuildListResponse.model_validate(await deploy_service.list_builds(ctx))

    @router.post("/download", response_model=MessageResponse | BuildListResponse)
    async def download_build(tag: Annotated[str, Form(...)]):
        result = await deploy_service.download_build(ctx, tag=tag)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    @router.post("/rollback", response_model=MessageResponse | BuildListResponse)
    async def rollback_build():
        result = await deploy_service.rollback_build(ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    return router
