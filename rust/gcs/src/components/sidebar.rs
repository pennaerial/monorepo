use crate::routes::Route;
use dioxus::prelude::*;

const PANEL_LEFT_ICON: Asset = asset!("/assets/panel-left.svg");
const DASHBOARD_ICON: Asset = asset!("/assets/layout-dashboard.svg");
const PLANE_ICON: Asset = asset!("/assets/plane.svg");

#[derive(Clone)]
pub(crate) struct SidebarContext {
    pub(crate) expanded: Signal<bool>,
}

#[component]
pub fn SideBar() -> Element {
    let mut expanded = use_context::<SidebarContext>().expanded;

    rsx! {
        aside {
            class: if expanded() { "sidebar" } else { "sidebar w-16! px-2.5!" },
            div { class: "flex items-center justify-between",

                if expanded() {
                    h2 { "PennAiR GCS" }
                }
                button {
                    r#type: "button",
                    class: "ms-auto grid size-11 shrink-0 cursor-pointer place-items-center",
                    // aria_label: toggle_label,
                    aria_expanded: if expanded() { "true" } else { "false" },
                    aria_controls: "sidebar-navigation",
                    onclick: move |_| expanded.toggle(),
                    img {
                        src: PANEL_LEFT_ICON,
                        width: "24",
                        height: "24",
                        alt: "panel-left",
                    }
                }
            }

            // version label
            p {
                class: if expanded() {
                    "text-left text-xs leading-4 text-[#64748b]"
                } else {
                    "text-center text-xs leading-4 text-[#64748b]"
                },
                {concat!("v", env!("CARGO_PKG_VERSION"))}
            }

            // horizontal line separator
            hr { class: "border-0 border-t border-solid border-[#cbd5e1]" }

            // sidebar items
            nav { class: "sidebar-nav",
                SidebarItem {
                    to: Route::MissionControl {},
                    label: "Mission Control",
                    icon: PLANE_ICON,
                    expanded: expanded(),
                }
                SidebarItem {
                    to: Route::Dashboard {},
                    label: "Dashboard",
                    icon: DASHBOARD_ICON,
                    expanded: expanded(),
                }
            }
        }
    }
}

#[component]
fn SidebarItem(to: Route, label: String, icon: Asset, expanded: bool) -> Element {
    rsx! {
        Link { to, class: "sidebar-link", active_class: "sidebar-link-active",
            img {
                src: icon,
                width: "18",
                height: "18",
                alt: if expanded { String::new() } else { label.clone() },
            }
            if expanded {
                "{label}"
            }
        }
    }
}
