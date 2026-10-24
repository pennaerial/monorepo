use crate::components::SideBar;
use dioxus::prelude::*;

#[component]
pub fn Dashboard() -> Element {
    rsx! {
        div { class: "app-layout",

            SideBar {}

            main { class: "main-content",

                h1 { class: "page-title", "Dashboard" }

                p { "Welcome to PennAir Control Dashboard" }
            }
        }
    }
}
