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
            class: "sidebar",
            style: if !expanded() { "width: 64px; padding-inline: 10px;" },
            div { class: "sidebar-header",
                style: "display: flex; align-items: center; justify-content: space-between;",

                if expanded() {
                    h2 { "PennAiR GCS" }
                }
                button {
                    r#type: "button",
                    style: "display: grid; place-items: center; width: 44px; height: 44px; margin-inline-start: auto; flex-shrink: 0; cursor: pointer;",
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
                style: format!("color: #64748b; font-size: 12px; line-height: 16px; text-align: {};", if expanded() { "left" } else { "center" }),
                {concat!("v", env!("CARGO_PKG_VERSION"))}
            }

            // horizontal line separator
            hr { style: "border: 0; border-top: 1px solid #cbd5e1;" }

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
