use dioxus::prelude::*;

use super::bridge::{self, DroneMarker, MineMarker, CONTAINER_ID};
use crate::components::Button;
use crate::map::{use_map_state, FieldFrame, FieldPoint, FIELD};

const LEAFLET_JS: Asset = asset!("/assets/leaflet/leaflet.js");
const LEAFLET_CSS: Asset = asset!("/assets/leaflet/leaflet.css");
const FIELD_MAP_JS: Asset = asset!("/assets/field_map.js");

/// Fills its parent; size it with the container (e.g. a div with a fixed height).
/// Shows `field` (default: `crate::map::FIELD`) upright and fitted to the screen.
#[component]
pub fn FieldMap(#[props(default = *FIELD)] field: FieldFrame) -> Element {
    let state = use_map_state();
    let mut ready = use_signal(|| false);

    // One memo per thing on the map, already in field metres. A memo only notifies
    // when its value changes, so e.g. mines aren't re-sent every time the person moves.
    let mines = use_memo(move || {
        state
            .read()
            .mines
            .iter()
            .map(|mine| MineMarker {
                id: mine.id.0,
                at: field.to_field(mine.position),
                status: mine.status,
            })
            .collect::<Vec<_>>()
    });
    let drones = use_memo(move || {
        state
            .read()
            .drones
            .iter()
            .map(|drone| DroneMarker {
                id: drone.id,
                at: field.to_field(drone.position),
            })
            .collect::<Vec<_>>()
    });
    let path = use_memo(move || {
        state
            .read()
            .route
            .iter()
            .map(|&point| field.to_field(point))
            .collect::<Vec<FieldPoint>>()
    });
    let person = use_memo(move || state.read().person.map(|p| field.to_field(p)));
    let target = use_memo(move || state.read().target.map(|p| field.to_field(p)));

    // Each effect re-runs when its memo changes. Until the map is ready it only reads
    // `ready`, then runs again (subscribing to its memo) once init finishes.
    use_effect(move || {
        if ready() {
            bridge::set_mines(&mines.read());
        }
    });
    use_effect(move || {
        if ready() {
            bridge::set_drones(&drones.read());
        }
    });
    use_effect(move || {
        if ready() {
            bridge::set_path(&path.read());
        }
    });
    use_effect(move || {
        if ready() {
            bridge::set_person(person());
        }
    });
    use_effect(move || {
        if ready() {
            bridge::set_target(target());
        }
    });

    rsx! {
        document::Stylesheet { href: LEAFLET_CSS }
        document::Script { src: LEAFLET_JS }
        document::Script { src: FIELD_MAP_JS }

        div { style: "position: relative; width: 100%; height: 100%; overflow: hidden;",
            div {
                id: CONTAINER_ID,
                style: "position: absolute; inset: 0; background: #f4f4f5;",
                onmounted: move |_| async move {
                    match bridge::init(&field).await {
                        Ok(()) => {
                            info!(
                                "field map: {:.1} x {:.1} m, long edge {:.1} deg from east",
                                field.width_m,
                                field.height_m,
                                field.angle_deg(),
                            );
                            ready.set(true);
                        }
                        Err(e) => error!("field map failed to start: {e}"),
                    }
                },
            }

            // Above Leaflet's panes and controls (z-index up to 1000).
            div { style: "position: absolute; bottom: 12px; right: 12px; z-index: 1000; display: flex; flex-direction: column; gap: 8px;",
                Button { label: "Field", onclick: move |_| bridge::fit_field() }
                Button { label: "Fit all", onclick: move |_| bridge::fit_all() }
                Button {
                    label: "Me",
                    onclick: move |_| {
                        if let Some(at) = person() {
                            bridge::pan_to(at);
                        }
                    },
                }
            }
        }
    }
}
