use crate::views::{Dashboard, MineMap, MissionControl};
use dioxus::prelude::*;

#[derive(Clone, Debug, PartialEq, Routable)]
pub enum Route {
    #[route("/")]
    Dashboard {},

    #[route("/mission_control")]
    MissionControl {},

    #[route("/mine_map")]
    MineMap {},
}
