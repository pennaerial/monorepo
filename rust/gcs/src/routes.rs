use crate::views::{Dashboard, MissionControl};
use dioxus::prelude::*;

// A shared router layout with `Outlet::<Route>` could keep one sidebar mounted
// across pages. For now, each page renders its own sidebar.

#[derive(Clone, Debug, PartialEq, Routable)]
pub enum Route {
    #[route("/dashboard")]
    Dashboard {},

    #[redirect("/", || Route::MissionControl {})]
    #[route("/mission_control")]
    MissionControl {},
}
