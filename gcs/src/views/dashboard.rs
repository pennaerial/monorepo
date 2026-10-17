use dioxus::prelude::*;
use crate::components::{SideBar};

#[component]
pub fn Dashboard() -> Element {
    rsx! {
        div {
            class: "app-layout",

            SideBar{}

            main {
                class: "main-content",

                h1 {class: "page-title",
                "Dashboard"
                }

                p {"Welcome to PennAir Control Dashboard"}
            }
        }
    }
    
}