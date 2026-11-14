use crate::components::{MapView, SideBar};
use dioxus::prelude::*;

#[component]
pub fn MineMap() -> Element {
    // fake drone + mines so the map has something to show without ROS (debug builds only).
    #[cfg(debug_assertions)]
    crate::map::use_mock_feed(crate::map::use_map_state(), crate::map::FIELD);

    rsx! {
        div { class: "app-layout",
            SideBar {}

            // Exactly one screen tall (padding included), stacked as a column: the title
            // takes its natural height and the map gets everything left, so nothing scrolls.
            main {
                class: "main-content",
                style: "height: 100vh; display: flex; flex-direction: column; overflow: hidden;",

                h1 { class: "page-title", "Mine Map" }

                // flex: 1 = take the leftover height. min-height: 0 lets it shrink below
                // its content's size (flex items refuse to by default).
                div { style: "flex: 1; min-height: 0; width: 100%; overflow: hidden; border-radius: 8px;",
                    MapView {}
                }
            }
        }
    }
}
