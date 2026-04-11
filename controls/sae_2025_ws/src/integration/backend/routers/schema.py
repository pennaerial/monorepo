from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, HTTPException, Query

from ..context import AppContext
from ..models import (
    FleetSchemaResponse,
    MissionCatalogResponse,
    MissionSchemaResponse,
    ModeRegistryResponse,
    SchemaIndexResponse,
)


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(prefix="/api/schema", tags=["schema"])

    @router.get("", response_model=SchemaIndexResponse)
    async def schema_index() -> SchemaIndexResponse:
        from ..schema import fleet_schema, mode_registry, mission_catalog

        return SchemaIndexResponse(
            missions=mission_catalog(),
            fleet=fleet_schema(),
            modes=mode_registry(),
        )

    @router.get("/modes", response_model=ModeRegistryResponse)
    async def get_mode_registry(
        target: Annotated[str | None, Query()] = None,
    ) -> ModeRegistryResponse:
        from ..schema import mode_registry

        return mode_registry(target=target)

    @router.get("/missions", response_model=MissionCatalogResponse)
    async def get_missions(
        target: Annotated[str | None, Query()] = None,
    ) -> MissionCatalogResponse:
        from ..schema import mission_catalog

        return mission_catalog(target=target)

    @router.get("/mission", response_model=MissionSchemaResponse)
    async def get_mission(
        name: Annotated[str | None, Query()] = None,
        path: Annotated[str | None, Query()] = None,
    ) -> MissionSchemaResponse:
        from ..schema import mission_schema_for_name, mission_schema_for_path

        if path:
            return mission_schema_for_path(path)
        if not name:
            raise HTTPException(
                status_code=400,
                detail="schema mission endpoint requires name or path.",
            )
        return mission_schema_for_name(name)

    @router.get("/fleet", response_model=FleetSchemaResponse)
    async def get_fleet_schema() -> FleetSchemaResponse:
        from ..schema import fleet_schema

        return fleet_schema()

    return router
