use dioxus::prelude::*;

use super::LAYER_STYLE;
use crate::map::{use_map_state, Viewport};

/// The planned route to the target, drawn dashed (it's where to go, not where we've been).
#[component]
pub fn PathLayer(viewport: Viewport) -> Element {
    let map_state = use_map_state();
    let route: Vec<_> = map_state
        .read()
        .route
        .iter()
        .map(|point| viewport.project(*point))
        .collect();
    let points = route
        .iter()
        .map(|p| format!("{:.1},{:.1}", p.x, p.y))
        .collect::<Vec<_>>()
        .join(" ");
    // Turn points between the start and the target.
    let waypoints = route.len().saturating_sub(1);

    rsx! {
        svg { style: LAYER_STYLE,
            polyline {
                points,
                fill: "none",
                stroke: "#0284c7",
                stroke_width: "3",
                stroke_dasharray: "8 6",
                stroke_linejoin: "round",
                stroke_linecap: "round",
            }
            for point in route.iter().take(waypoints).skip(1) {
                circle { cx: point.x, cy: point.y, r: 3, fill: "#0284c7" }
            }
        }
    }
}
