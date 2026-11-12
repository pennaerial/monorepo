use crate::views::{Dashboard, MissionControl};
use dioxus::prelude::*;

#[derive(Clone, Debug, PartialEq, Routable)]
pub enum Route {
    #[route("/")]
    Dashboard {},

    #[route("/mission_control")]
    MissionControl {},
}
