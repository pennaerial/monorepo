//! Map renderer. Reads `crate::map` state; never writes mission data.
//!
//! Layout-critical styles are inline rather than Tailwind classes, so the map
//! renders correctly even if `assets/tailwind.css` hasn't been regenerated.

mod gestures;
mod grid_layer;
mod map_view;
mod marker_layer;
mod path_layer;

pub use map_view::MapView;

/// Shared by the SVG layers: cover the map, draw past the edges, ignore the mouse
/// (so pointer coordinates are always relative to the gesture surface).
const LAYER_STYLE: &str = "position: absolute; inset: 0; width: 100%; height: 100%; overflow: visible; pointer-events: none;";
