use dioxus::prelude::*;
use crate::views::{Dashboard, MissionControl};

#[derive(Clone, Debug, PartialEq, Routable)]
pub enum Route {
    #[route("/")]
    Dashboard {},

    #[route("/mission_control")]
    MissionControl {},
}