from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, File, Form, HTTPException, Query, UploadFile

from ..context import AppContext
from ..models import BuildCurrentResponse, BuildListResponse, MessageResponse


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/builds", tags=["builds"])

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

    @router.get("/current", response_model=BuildCurrentResponse)
    async def current_build(
        target_id: str | None = None,
        hostname: str | None = None,
        vehicle_name: str | None = None,
    ) -> BuildCurrentResponse:
        from ..services import deploy as deploy_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        return BuildCurrentResponse.model_validate(
            await deploy_service.current_build(ctx, target_ctx=target_ctx)
        )

    @router.post("/deploy-selected", response_model=MessageResponse | BuildListResponse)
    async def deploy_selected_source(
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
    ):
        from ..services import deploy as deploy_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        result = await deploy_service.deploy_selected_source(ctx, target_ctx=target_ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    @router.post("/upload", response_model=MessageResponse | BuildListResponse)
    async def upload_build(
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
        file: UploadFile = File(...),
    ):
        from ..services import deploy as deploy_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        source_result = await deploy_service.set_local_artifact_build_source(
            ctx,
            filename=file.filename or "",
            file_bytes=await file.read(),
        )
        if not source_result.get("success"):
            return BuildListResponse.model_validate({**source_result, "builds": []})
        result = await deploy_service.deploy_selected_source(ctx, target_ctx=target_ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    @router.post("/upload-source", response_model=MessageResponse | BuildListResponse)
    async def upload_source_bundle(
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
        file: UploadFile = File(...),
    ):
        from ..services import deploy as deploy_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        result = await deploy_service.upload_source_bundle(
            ctx,
            target_ctx=target_ctx,
            filename=file.filename or "",
            file_bytes=await file.read(),
        )
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    @router.get("/list", response_model=BuildListResponse)
    async def list_builds(
        artifact_page: Annotated[int, Query()] = 1,
        artifact_page_size: Annotated[int, Query()] = 20,
        q: Annotated[str, Query()] = "",
        sha: Annotated[str, Query()] = "",
        commit_subject: Annotated[str, Query()] = "",
        branch: Annotated[str, Query()] = "",
        include_fallback: Annotated[bool, Query()] = False,
    ) -> BuildListResponse:
        from ..services import deploy as deploy_service

        return BuildListResponse.model_validate(
            await deploy_service.list_builds(
                ctx,
                artifact_page=artifact_page,
                artifact_page_size=artifact_page_size,
                q=q,
                sha=sha,
                commit_subject=commit_subject,
                branch=branch,
                include_fallback=include_fallback,
            )
        )

    @router.post("/download", response_model=MessageResponse | BuildListResponse)
    async def download_build(
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
        tag: Annotated[str, Form()] = "",
        source: Annotated[str, Form()] = "release",
        artifact_id: Annotated[str, Form()] = "",
        run_id: Annotated[str, Form()] = "",
        sha: Annotated[str, Form()] = "",
        commit_subject: Annotated[str, Form()] = "",
        name: Annotated[str, Form()] = "",
        date: Annotated[str, Form()] = "",
        download_url: Annotated[str, Form()] = "",
        size_mb: Annotated[float | None, Form()] = None,
        branch: Annotated[str, Form()] = "",
        artifact_name: Annotated[str, Form()] = "",
    ):
        from ..services import deploy as deploy_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        source_result = await deploy_service.set_github_build_source(
            ctx,
            source=source,
            tag=tag,
            artifact_id=artifact_id,
            run_id=run_id,
            sha=sha,
            commit_subject=commit_subject,
            name=name,
            date=date,
            download_url=download_url,
            size_mb=size_mb,
            branch=branch,
            artifact_name=artifact_name,
        )
        if not source_result.get("success"):
            return BuildListResponse.model_validate({**source_result, "builds": []})
        result = await deploy_service.deploy_selected_source(ctx, target_ctx=target_ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    @router.post("/rollback", response_model=MessageResponse | BuildListResponse)
    async def rollback_build(
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
    ):
        from ..services import deploy as deploy_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        result = await deploy_service.rollback_build(ctx, target_ctx=target_ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    @router.post("/deploy-local", response_model=MessageResponse | BuildListResponse)
    async def deploy_local_code(
        target_id: Annotated[str | None, Form()] = None,
        hostname: Annotated[str | None, Form()] = None,
        vehicle_name: Annotated[str | None, Form()] = None,
    ):
        from ..services import deploy as deploy_service

        target_ctx = _resolve_target_context(
            target_id,
            hostname=hostname,
            vehicle_name=vehicle_name,
        )
        source_result = await deploy_service.set_local_codebase_build_source(ctx)
        if not source_result.get("success"):
            return BuildListResponse.model_validate({**source_result, "builds": []})
        result = await deploy_service.deploy_selected_source(ctx, target_ctx=target_ctx)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return BuildListResponse.model_validate({**result, "builds": []})

    return router
