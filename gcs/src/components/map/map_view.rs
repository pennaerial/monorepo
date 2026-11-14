use dioxus::html::geometry::{ElementPoint, WheelDelta};
use dioxus::prelude::*;

use super::gestures::Gestures;
use super::grid_layer::GridLayer;
use super::marker_layer::MarkerLayer;
use super::path_layer::PathLayer;
use crate::components::Button;
use crate::map::{use_map_state, GeoBounds, ScreenPoint, Viewport, FIELD};

/// Zoom limits, in standard web-map zoom levels. 15 shows ~4 km across a laptop
/// screen; 23 is ~2 cm per pixel.
const MIN_ZOOM: f64 = 15.0;
const MAX_ZOOM: f64 = 23.0;
/// Zoom levels per pixel of wheel scroll.
const WHEEL_ZOOM_PER_PIXEL: f64 = 0.002;
/// Wheel events in "lines" (some mice) are converted at this many pixels per line.
const WHEEL_LINE_PX: f64 = 16.0;
/// Space kept around the field / data when fitting.
const FIT_PADDING_PX: f64 = 24.0;

fn to_screen(point: ElementPoint) -> ScreenPoint {
    ScreenPoint::new(point.x, point.y)
}

/// Vertical scroll in pixels, whatever unit the device reports.
fn wheel_pixels(delta: WheelDelta, page_height: f64) -> f64 {
    match delta {
        WheelDelta::Pixels(v) => v.y,
        WheelDelta::Lines(v) => v.y * WHEEL_LINE_PX,
        WheelDelta::Pages(v) => v.y * page_height,
    }
}

/// Fills its parent; size it with the container (e.g. a div with a fixed height).
/// Opens fitted to `field` (default: `crate::map::FIELD`), which is also outlined and
/// anchors the grid.
#[component]
pub fn MapView(#[props(default = FIELD)] field: GeoBounds) -> Element {
    let map_state = use_map_state();
    // Center/zoom are placeholders until the first resize, which fits `field`.
    let mut viewport = use_signal(|| Viewport::new(field.min, 19.0, MIN_ZOOM, MAX_ZOOM));
    let mut fitted = use_signal(|| false);
    let mut gestures = use_signal(Gestures::default);

    rsx! {
        div { style: "position: relative; width: 100%; height: 100%; overflow: hidden; background: #f4f4f5;",

            // Gesture surface. Layers are pointer-events: none, so element coordinates
            // are always relative to this div.
            div {
                style: "position: absolute; inset: 0; cursor: grab; touch-action: none; user-select: none; -webkit-user-select: none;",
                onresize: move |e| {
                    if let Ok(size) = e.get_border_box_size() {
                        let mut view = viewport.write();
                        view.resize(size.width, size.height);
                        // Fit once, as soon as the map has a real size.
                        if !fitted() && view.fit_bounds(field, FIT_PADDING_PX) {
                            fitted.set(true);
                        }
                    }
                },
                onpointerdown: move |e| gestures.write().press(e.pointer_id(), to_screen(e.element_coordinates())),
                onpointermove: move |e| {
                    let gesture = gestures
                        .write()
                        .move_to(e.pointer_id(), to_screen(e.element_coordinates()));
                    if let Some(gesture) = gesture {
                        gesture.apply(&mut viewport.write());
                    }
                },
                onpointerup: move |e| gestures.write().release(e.pointer_id()),
                onpointercancel: move |e| gestures.write().release(e.pointer_id()),
                onpointerleave: move |e| gestures.write().release(e.pointer_id()),
                onwheel: move |e| {
                    e.prevent_default();
                    let anchor = to_screen(e.element_coordinates());
                    let mut view = viewport.write();
                    let scrolled = wheel_pixels(e.delta(), view.height);
                    let zoom = view.zoom - scrolled * WHEEL_ZOOM_PER_PIXEL;
                    view.zoom_at(anchor, zoom);
                },

                // Bottom to top: grid, planned route, markers.
                GridLayer { viewport: viewport(), field }
                PathLayer { viewport: viewport() }
                MarkerLayer { viewport: viewport() }
            }

            div { style: "position: absolute; bottom: 12px; right: 12px; display: flex; flex-direction: column; gap: 8px;",
                Button { label: "+", onclick: move |_| viewport.write().zoom_by(1.0) }
                Button { label: "−", onclick: move |_| viewport.write().zoom_by(-1.0) }
                Button {
                    label: "Field",
                    onclick: move |_| {
                        viewport.write().fit_bounds(field, FIT_PADDING_PX);
                    },
                }
                Button {
                    label: "Fit all",
                    onclick: move |_| {
                        let bounds = map_state.read().bounds();
                        if let Some(bounds) = bounds {
                            viewport.write().fit_bounds(bounds, FIT_PADDING_PX);
                        }
                    },
                }
                Button {
                    label: "Me",
                    onclick: move |_| {
                        let person = map_state.read().person;
                        if let Some(position) = person {
                            viewport.write().center = position;
                        }
                    },
                }
            }
        }
    }
}
