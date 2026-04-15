from importlib import import_module

__all__ = [
    "MissionSpec",
    "ModeSpec",
    "load_mission_spec",
    "load_mode_class",
    "mission_path_for_name",
    "MissionDocumentModel",
    "FleetDocumentModel",
    "load_fleet_document",
    "mode_registry_entries",
    "mode_entry_for_mode_id",
    "mode_entry_for_class_path",
    "mission_document_schema",
    "fleet_document_schema",
    "schema_registry_document",
    "ModeManager",
    "UAVModeManager",
    "PayloadModeManager",
]

_EXPORTS = {
    "MissionSpec": ".mission_spec",
    "ModeSpec": ".mission_spec",
    "load_mission_spec": ".mission_spec",
    "load_mode_class": ".mission_spec",
    "mission_path_for_name": ".mission_spec",
    "MissionDocumentModel": ".mission_spec",
    "FleetDocumentModel": ".fleet_spec",
    "load_fleet_document": ".fleet_spec",
    "mode_registry_entries": ".schema",
    "mode_entry_for_mode_id": ".schema",
    "mode_entry_for_class_path": ".schema",
    "mission_document_schema": ".schema",
    "fleet_document_schema": ".schema",
    "schema_registry_document": ".schema",
    "ModeManager": ".ModeManager",
    "UAVModeManager": ".UAVModeManager",
    "PayloadModeManager": ".PayloadModeManager",
}


def __getattr__(name: str):
    module_path = _EXPORTS.get(name)
    if module_path is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(module_path, __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value
