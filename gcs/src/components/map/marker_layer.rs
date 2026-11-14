use dioxus::prelude::*;

use super::LAYER_STYLE;
use crate::map::{use_map_state, MineStatus, Viewport};

/// Keep markers slightly past the edge so they don't pop in while panning.
const CULL_MARGIN_PX: f64 = 32.0;

fn mine_color(status: MineStatus) -> &'static str {
    match status {
        MineStatus::Detected => "#f59e0b",  // amber
        MineStatus::Confirmed => "#ef4444", // red
        MineStatus::Cleared => "#22c55e",   // green
    }
}

/// Mines, the target, and the person.
#[component]
pub fn MarkerLayer(viewport: Viewport) -> Element {
    let map_state = use_map_state();
    let state = map_state.read();
    let mines: Vec<_> = state
        .mines
        .iter()
        .map(|mine| (mine.id, viewport.project(mine.position), mine.status))
        .filter(|(_, point, _)| viewport.contains(*point, CULL_MARGIN_PX))
        .collect();
    let target = state.target.map(|position| viewport.project(position));
    let person = state.person.map(|position| viewport.project(position));

    rsx! {
        svg { style: LAYER_STYLE,
            for (id, point, status) in mines {
                circle {
                    key: "{id.0}",
                    cx: point.x,
                    cy: point.y,
                    r: 7,
                    fill: mine_color(status),
                    stroke: "white",
                    stroke_width: "2",
                }
            }
            // Target: a bullseye.
            if let Some(point) = target {
                circle { cx: point.x, cy: point.y, r: 12, fill: "none", stroke: "#7c3aed", stroke_width: "3" }
                circle { cx: point.x, cy: point.y, r: 4, fill: "#7c3aed" }
            }
            // Person: drawn last so it's on top. Halo + dot, like "you are here" on phone maps.
            if let Some(point) = person {
                circle { cx: point.x, cy: point.y, r: 16, fill: "#2563eb", fill_opacity: "0.2" }
                circle { cx: point.x, cy: point.y, r: 7, fill: "#2563eb", stroke: "white", stroke_width: "3" }
            }
        }
    }
}
