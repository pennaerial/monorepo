from __future__ import annotations


class ManagedEntity:
    def __init__(self, spec):
        self.spec = spec
        self._entity = None
        self._destroyed = False

    @property
    def destroyed(self) -> bool:
        return self._destroyed

    def attach(self, entity: object) -> None:
        self._entity = entity

    def detach(self):
        entity = self._entity
        self._entity = None
        return entity

    def mark_destroyed(self) -> None:
        self._destroyed = True
        self._entity = None

    def get_underlying(self):
        return self._entity

    def __getattr__(self, name: str):
        entity = self._entity
        if entity is None:
            raise AttributeError(name)
        return getattr(entity, name)


class ManagedPublisher(ManagedEntity):
    def publish(self, message: object) -> None:
        entity = self.get_underlying()
        if entity is None or self.destroyed:
            return None
        publish = getattr(entity, "publish", None)
        if not callable(publish):
            return None
        publish(message)
        return None


class ManagedClient(ManagedEntity):
    def wait_for_service(self, timeout_sec: float = 0.0) -> bool:
        entity = self.get_underlying()
        if entity is None or self.destroyed:
            return False
        wait_for_service = getattr(entity, "wait_for_service", None)
        if not callable(wait_for_service):
            return False
        return bool(wait_for_service(timeout_sec=timeout_sec))

    def call_async(self, request: object):
        entity = self.get_underlying()
        if entity is None or self.destroyed:
            return None
        if not self.wait_for_service(timeout_sec=0.0):
            return None
        call_async = getattr(entity, "call_async", None)
        if not callable(call_async):
            return None
        return call_async(request)
