from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass

from .models import MissionRuntimeState


@dataclass(slots=True)
class MissionStateMachine:
    _state: MissionRuntimeState
    _lock: asyncio.Lock

    @classmethod
    def create(cls) -> "MissionStateMachine":
        return cls(
            _state=MissionRuntimeState(
                phase="idle",
                launch_state="not_prepared",
                running=False,
                updated_at=time.time(),
            ),
            _lock=asyncio.Lock(),
        )

    async def snapshot(self) -> MissionRuntimeState:
        async with self._lock:
            return MissionRuntimeState.model_validate(self._state.model_dump())

    async def set(
        self,
        *,
        phase: str | None = None,
        launch_state: str | None = None,
        running: bool | None = None,
        pid: str | None = None,
        message: str | None = None,
        error: str | None = None,
    ) -> MissionRuntimeState:
        async with self._lock:
            update = self._state.model_dump()
            if phase is not None:
                update["phase"] = phase
            if launch_state is not None:
                update["launch_state"] = launch_state
            if running is not None:
                update["running"] = running
            if pid is not None:
                update["pid"] = pid
            if message is not None:
                update["message"] = message
            if error is not None:
                update["error"] = error
            if phase != "error" and error is None:
                update["error"] = None
            update["updated_at"] = time.time()
            self._state = MissionRuntimeState.model_validate(update)
            return MissionRuntimeState.model_validate(self._state.model_dump())

    async def apply_launch_status(
        self,
        *,
        success: bool,
        state: str,
        running: bool,
        pid: str | None = None,
        error: str | None = None,
    ) -> MissionRuntimeState:
        if not success:
            phase = "offline" if state == "offline" else "error"
            return await self.set(
                phase=phase,
                launch_state=state if state in {"offline", "error"} else "error",
                running=False,
                pid=None,
                error=error,
            )

        if state == "running":
            return await self.set(
                phase="running",
                launch_state="running",
                running=True,
                pid=pid,
                message="Mission launch running",
                error=None,
            )

        if state in {"stopped", "not_prepared"}:
            return await self.set(
                phase="idle",
                launch_state=state,
                running=False,
                pid=None,
                message="Mission launch idle",
                error=None,
            )

        return await self.set(
            phase="error",
            launch_state="error",
            running=False,
            pid=None,
            error=error or "Unknown launch status",
        )
