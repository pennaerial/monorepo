use dioxus::prelude::*;

use super::LAYER_STYLE;
use crate::map::{grid, GeoBounds, GeoPoint, Viewport};

/// Squares should be at least this big on screen; the grid picks 1/2/5/10/... m to match.
const MIN_CELL_PX: f64 = 40.0;

/// Metric grid squares plus the field outline. Replaces map imagery.
#[component]
pub fn GridLayer(viewport: Viewport, field: GeoBounds) -> Element {
    let grid = grid(&viewport, field.min, MIN_CELL_PX);
    // All lines in one <path> ("M x,y L x,y" per line) instead of hundreds of elements.
    let lines: String = grid
        .lines
        .iter()
        .map(|(a, b)| format!("M{:.1},{:.1}L{:.1},{:.1}", a.x, a.y, b.x, b.y))
        .collect();

    let corners = [
        field.min,
        GeoPoint::new(field.min.lat, field.max.lon),
        field.max,
        GeoPoint::new(field.max.lat, field.min.lon),
    ];
    let outline = corners
        .map(|corner| viewport.project(corner))
        .map(|p| format!("{:.1},{:.1}", p.x, p.y))
        .join(" ");

    let label = if grid.spacing_m < 1.0 {
        format!("grid {:.0} cm", grid.spacing_m * 100.0)
    } else {
        format!("grid {:.0} m", grid.spacing_m)
    };

    rsx! {
        svg { style: LAYER_STYLE,
            path { d: lines, stroke: "rgb(113 113 122 / 0.35)", stroke_width: "1", fill: "none" }
            polygon {
                points: outline,
                fill: "rgb(250 204 21 / 0.08)",
                stroke: "#a16207",
                stroke_width: "2",
            }
        }
        div { style: "position: absolute; left: 12px; bottom: 12px; padding: 2px 8px; border-radius: 4px; background: rgb(255 255 255 / 0.8); color: #3f3f46; font: 12px sans-serif; pointer-events: none;",
            "{label}"
        }
    }
}
