//! The only Rust that knows Leaflet exists. It sends field (x, y) metres to
//! `assets/field_map.js`, which owns the map. Swapping in another map library means
//! replacing this file and that JS file; nothing else changes.

use dioxus::logger::tracing::warn;
use dioxus::prelude::*;
use serde::Serialize;

use crate::map::{FieldFrame, FieldPoint, MineStatus};

/// The div Leaflet takes over.
pub const CONTAINER_ID: &str = "field-map";

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct MineMarker {
    pub id: u32,
    #[serde(flatten)]
    pub at: FieldPoint,
    #[serde(serialize_with = "status_name")]
    pub status: MineStatus,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct DroneMarker {
    pub id: u32,
    #[serde(flatten)]
    pub at: FieldPoint,
}

fn status_name<S: serde::Serializer>(status: &MineStatus, s: S) -> Result<S::Ok, S::Error> {
    s.serialize_str(match status {
        MineStatus::Detected => "detected",
        MineStatus::Confirmed => "confirmed",
        MineStatus::Cleared => "cleared",
    })
}

/// Creates the map in [`CONTAINER_ID`] once Leaflet and the bridge script have loaded.
/// Call after the container mounts; the setters below do nothing until this resolves.
pub async fn init(field: &FieldFrame) -> Result<(), document::EvalError> {
    let js = format!(
        r#"
        // Scripts load asynchronously; wait for both.
        while (!(window.L && window.fieldMap)) await new Promise((r) => setTimeout(r, 20));
        window.fieldMap.init({id:?}, {{ width: {w}, height: {h} }});
        return true;
        "#,
        id = CONTAINER_ID,
        w = field.width_m,
        h = field.height_m,
    );
    document::eval(&js).await.map(|_| ())
}

pub fn set_mines(mines: &[MineMarker]) {
    call("setMines", mines);
}

pub fn set_drones(drones: &[DroneMarker]) {
    call("setDrones", drones);
}

/// The planned route, person first, target last.
pub fn set_path(points: &[FieldPoint]) {
    call("setPath", points);
}

pub fn set_person(at: Option<FieldPoint>) {
    call("setPerson", at);
}

pub fn set_target(at: Option<FieldPoint>) {
    call("setTarget", at);
}

pub fn fit_field() {
    call("fitField", ());
}

/// Zooms to everything on the map.
pub fn fit_all() {
    call("fitAll", ());
}

pub fn pan_to(at: FieldPoint) {
    call("panTo", at);
}

/// Fire-and-forget `fieldMap.<method>(<arg as JSON>)`. `()` serializes to `null`.
fn call(method: &'static str, arg: impl Serialize) {
    let arg = match serde_json::to_string(&arg) {
        Ok(json) => json,
        Err(e) => return warn!("fieldMap.{method}: can't serialize argument: {e}"),
    };
    let js = format!("window.fieldMap.{method}({arg});");
    spawn(async move {
        if let Err(e) = document::eval(&js).await {
            warn!("fieldMap.{method} failed: {e}");
        }
    });
}
