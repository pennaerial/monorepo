use dioxus::prelude::*;
use crate::routes::Route;

#[component]
pub fn SideBar() -> Element {
    
    rsx!{
        aside {
            class: "sidebar", 

        div {
            class: "sidebar-header",
            h2 {"PennAiR Control"}
        }

        nav {
            class: "sidebar-nav",

            Link {
                to: Route::Dashboard {},
                class: "sidebar-link",
                span { class: "nav-icon", "-" },
                "Dashboard"
            }

            Link {
                to: Route::MissionControl {},
                class: "sidebar-link",
                span { class: "nav-icon", "-" },
                "Mission Control"
            }
        }
        }
    }
}