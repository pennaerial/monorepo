//! Metric grid: square cells of a round size (1 m, 2 m, 5 m, 10 m, ...) anchored to a point.
//! The cell size grows as you zoom out so squares never get too small to read.

use super::{GeoPoint, ScreenPoint, Viewport};

/// Candidate cell sizes, smallest first.
const NICE_SPACINGS_M: [f64; 12] = [
    0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0,
];
/// Safety cap; with sensible zoom limits you never get close.
const MAX_LINES: usize = 400;

/// Grid lines to draw, already in screen coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct Grid {
    /// Side length of one square, in metres.
    pub spacing_m: f64,
    pub lines: Vec<(ScreenPoint, ScreenPoint)>,
}

/// Lines for the part of the grid that's on screen. `anchor` sits on a grid corner,
/// so the squares stay put relative to the field while you pan.
pub fn grid(viewport: &Viewport, anchor: GeoPoint, min_cell_px: f64) -> Grid {
    let meters_per_px = viewport.meters_per_pixel();
    let spacing_m = NICE_SPACINGS_M
        .into_iter()
        .find(|spacing| spacing / meters_per_px >= min_cell_px)
        .unwrap_or(NICE_SPACINGS_M[NICE_SPACINGS_M.len() - 1]);

    // The screen's extent in metres north/east of the anchor.
    let corners = [
        ScreenPoint::new(0.0, 0.0),
        ScreenPoint::new(viewport.width, 0.0),
        ScreenPoint::new(0.0, viewport.height),
        ScreenPoint::new(viewport.width, viewport.height),
    ]
    .map(|corner| viewport.unproject(corner).local_m(anchor));
    let (north_min, north_max) = min_max(corners.iter().map(|&(north, _)| north));
    let (east_min, east_max) = min_max(corners.iter().map(|&(_, east)| east));

    // Grid line indices on screen along one axis.
    let indices = |min: f64, max: f64| {
        let first = (min / spacing_m).ceil() as i64;
        let last = (max / spacing_m).floor() as i64;
        first..=last
    };
    let columns = indices(east_min, east_max);
    let rows = indices(north_min, north_max);
    let count = columns.clone().count() + rows.clone().count();
    if count > MAX_LINES {
        return Grid {
            spacing_m,
            lines: Vec::new(),
        };
    }

    let project = |north: f64, east: f64| viewport.project(anchor.offset_m(north, east));
    let vertical = columns.map(|i| {
        let east = i as f64 * spacing_m;
        (project(north_min, east), project(north_max, east))
    });
    let horizontal = rows.map(|i| {
        let north = i as f64 * spacing_m;
        (project(north, east_min), project(north, east_max))
    });
    Grid {
        spacing_m,
        lines: vertical.chain(horizontal).collect(),
    }
}

fn min_max(values: impl Iterator<Item = f64>) -> (f64, f64) {
    values.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
        (lo.min(v), hi.max(v))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const PENN: GeoPoint = GeoPoint::new(39.9522, -75.1932);

    fn viewport(zoom: f64) -> Viewport {
        let mut viewport = Viewport::new(PENN, zoom, 0.0, 24.0);
        viewport.resize(800.0, 600.0);
        viewport
    }

    #[test]
    fn cells_are_big_enough_but_not_huge() {
        for zoom in [15.0, 17.0, 19.3, 21.0, 23.0] {
            let viewport = viewport(zoom);
            let grid = grid(&viewport, PENN, 40.0);
            let cell_px = grid.spacing_m / viewport.meters_per_pixel();
            // Each step in the 1-2-5 series is at most 2.5x the last.
            assert!(
                (40.0..=100.0).contains(&cell_px),
                "{cell_px}px at zoom {zoom}"
            );
        }
    }

    #[test]
    fn lines_cover_the_screen() {
        let viewport = viewport(19.0);
        let grid = grid(&viewport, PENN, 40.0);
        let cell_px = grid.spacing_m / viewport.meters_per_pixel();
        let xs: Vec<f64> = grid
            .lines
            .iter()
            .filter(|(a, b)| (a.x - b.x).abs() < 1e-6) // vertical lines
            .map(|(a, _)| a.x)
            .collect();
        let min = xs.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(min <= cell_px && max >= 800.0 - cell_px);
    }

    #[test]
    fn anchor_is_on_a_grid_corner() {
        let viewport = viewport(19.0);
        let anchor = viewport.project(PENN);
        let grid = grid(&viewport, PENN, 40.0);
        assert!(grid
            .lines
            .iter()
            .any(|(a, b)| (a.x - anchor.x).abs() < 1e-6 && (b.x - anchor.x).abs() < 1e-6));
        assert!(grid
            .lines
            .iter()
            .any(|(a, b)| (a.y - anchor.y).abs() < 1e-6 && (b.y - anchor.y).abs() < 1e-6));
    }
}
