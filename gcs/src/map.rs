//! Map data layer: coordinates, what's on the map, and shared state.
//! No rendering here and no ROS code. See `components::map` for the UI.

mod geo;
mod grid;
#[cfg(debug_assertions)]
mod mock;
mod model;
mod state;

pub use geo::{GeoBounds, GeoPoint, ScreenPoint, Viewport};
pub use grid::{grid, Grid};
#[cfg(debug_assertions)]
pub use mock::use_mock_feed;
pub use model::{Mine, MineId, MineStatus};
pub use state::{provide_map_state, use_map_state};

/// The minefield. The map opens fitted to it, outlines it, and anchors the grid on its
/// south-west corner; the mock drone surveys it.
// TODO: replace this placeholder (~300 x 100 ft at Penn) with the competition field corners.
pub const FIELD: GeoBounds = GeoBounds {
    min: GeoPoint::new(39.95207, -75.19373), // south-west corner
    max: GeoPoint::new(39.95234, -75.19267), // north-east corner
};
