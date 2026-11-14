use dioxus::prelude::*;

use super::model::MineStore;
use super::{GeoBounds, GeoPoint};

/// Everything the map draws. ROS handlers (and the planner) write it; map layers read it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MapState {
    /// Mines reported by the drone.
    pub mines: MineStore,
    /// The person on the ground (operator's GPS). The map's "you are here".
    pub person: Option<GeoPoint>,
    /// Where the person is trying to get to.
    pub target: Option<GeoPoint>,
    /// Planned route from the person to the target, around the mines.
    /// Produced by the planner; the map only draws it. Empty = no plan yet.
    pub route: Vec<GeoPoint>,
}

impl MapState {
    /// Box around everything on the map; `None` before any data arrives.
    pub fn bounds(&self) -> Option<GeoBounds> {
        GeoBounds::from_points(
            self.route
                .iter()
                .copied()
                .chain(self.mines.iter().map(|mine| mine.position))
                .chain(self.person) // an Option iterates over 0 or 1 items
                .chain(self.target),
        )
    }
}

/// Sync so ROS subscription callbacks (tokio tasks) can write to it directly.
pub type MapStateSignal = SyncSignal<MapState>;

/// Call once in `App`.
pub fn provide_map_state() -> MapStateSignal {
    let state = use_signal_sync(MapState::default);
    use_context_provider(|| state)
}

pub fn use_map_state() -> MapStateSignal {
    use_context::<MapStateSignal>()
}
