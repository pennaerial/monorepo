from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, File, Form, UploadFile

from ..context import AppContext
from ..models import BuildSourceResponse, FleetCatalogResponse


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/build-source", tags=["build-source"])

    @router.get("", response_model=BuildSourceResponse)
    async def get_build_source() -> BuildSourceResponse:
        from ..services import deploy as deploy_service

        return BuildSourceResponse.model_validate(
            await deploy_service.get_build_source(ctx)
        )

    @router.get("/fleet-catalog", response_model=FleetCatalogResponse)
    async def get_fleet_catalog() -> FleetCatalogResponse:
        from ..services import deploy as deploy_service

        return FleetCatalogResponse.model_validate(
            await deploy_service.get_fleet_catalog(ctx)
        )

    @router.post("/github", response_model=BuildSourceResponse)
    async def set_github_build_source(
        source: Annotated[str, Form(...)],
        tag: Annotated[str, Form()] = "",
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
    ) -> BuildSourceResponse:
        from ..services import deploy as deploy_service

        return BuildSourceResponse.model_validate(
            await deploy_service.set_github_build_source(
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
        )

    @router.post("/local-artifact", response_model=BuildSourceResponse)
    async def set_local_artifact_build_source(
        file: UploadFile = File(...),
    ) -> BuildSourceResponse:
        from ..services import deploy as deploy_service

        return BuildSourceResponse.model_validate(
            await deploy_service.set_local_artifact_build_source(
                ctx,
                filename=file.filename or "",
                file_bytes=await file.read(),
            )
        )

    @router.post("/local-codebase", response_model=BuildSourceResponse)
    async def set_local_codebase_build_source() -> BuildSourceResponse:
        from ..services import deploy as deploy_service

        return BuildSourceResponse.model_validate(
            await deploy_service.set_local_codebase_build_source(ctx)
        )

    @router.post("/fleet", response_model=BuildSourceResponse)
    async def set_global_fleet_file(
        fleet_file: Annotated[str, Form(...)],
    ) -> BuildSourceResponse:
        from ..services import deploy as deploy_service

        return BuildSourceResponse.model_validate(
            await deploy_service.set_global_fleet_file(
                ctx,
                fleet_file=fleet_file,
            )
        )

    @router.post("/clear", response_model=BuildSourceResponse)
    async def clear_build_source() -> BuildSourceResponse:
        from ..services import deploy as deploy_service

        return BuildSourceResponse.model_validate(
            await deploy_service.clear_build_source(ctx)
        )

    return router
