//! All lat/lon <-> pixel math for the map. Layers only ever see screen points.

use std::f64::consts::PI;
use std::ops::{Add, Sub};

/// World width in pixels at zoom 0. 256 keeps zoom levels matching standard web maps
/// (zoom 17 here means the same scale as zoom 17 in Google/OSM).
const WORLD_SIZE_AT_ZOOM_0: f64 = 256.0;
const EARTH_RADIUS_M: f64 = 6_371_008.8;
/// Web Mercator is undefined at the poles; the projection stops at +/- this latitude.
const MAX_LATITUDE: f64 = 85.051_128_779_806_59;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeoPoint {
    pub lat: f64,
    pub lon: f64,
}

impl GeoPoint {
    pub const fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }

    /// Great-circle distance in metres (haversine).
    pub fn distance_m(self, other: GeoPoint) -> f64 {
        let (lat1, lat2) = (self.lat.to_radians(), other.lat.to_radians());
        let dlat = lat2 - lat1;
        let dlon = (other.lon - self.lon).to_radians();
        let a = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
        2.0 * EARTH_RADIUS_M * a.sqrt().asin()
    }

    /// Point offset by metres north/east. Accurate over field-sized distances.
    pub fn offset_m(self, north_m: f64, east_m: f64) -> GeoPoint {
        let dlat = (north_m / EARTH_RADIUS_M).to_degrees();
        let dlon = (east_m / (EARTH_RADIUS_M * self.lat.to_radians().cos())).to_degrees();
        GeoPoint::new(self.lat + dlat, self.lon + dlon)
    }

    /// Inverse of `offset_m`: how far `self` is (north, east) of `origin`, in metres.
    pub fn local_m(self, origin: GeoPoint) -> (f64, f64) {
        let north = (self.lat - origin.lat).to_radians() * EARTH_RADIUS_M;
        let east =
            (self.lon - origin.lon).to_radians() * EARTH_RADIUS_M * origin.lat.to_radians().cos();
        (north, east)
    }
}

/// Smallest lat/lon box containing a set of points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeoBounds {
    /// South-west corner.
    pub min: GeoPoint,
    /// North-east corner.
    pub max: GeoPoint,
}

impl GeoBounds {
    /// `None` if there are no points.
    pub fn from_points(points: impl IntoIterator<Item = GeoPoint>) -> Option<Self> {
        let mut points = points.into_iter();
        let first = points.next()?;
        Some(points.fold(
            Self {
                min: first,
                max: first,
            },
            |b, p| Self {
                min: GeoPoint::new(b.min.lat.min(p.lat), b.min.lon.min(p.lon)),
                max: GeoPoint::new(b.max.lat.max(p.lat), b.max.lon.max(p.lon)),
            },
        ))
    }

    /// East-west extent in metres, measured along the southern edge.
    pub fn width_m(&self) -> f64 {
        self.min
            .distance_m(GeoPoint::new(self.min.lat, self.max.lon))
    }

    /// North-south extent in metres.
    pub fn height_m(&self) -> f64 {
        self.min
            .distance_m(GeoPoint::new(self.max.lat, self.min.lon))
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ScreenPoint {
    pub x: f64,
    pub y: f64,
}

impl ScreenPoint {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn distance(self, other: ScreenPoint) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }

    pub fn midpoint(self, other: ScreenPoint) -> ScreenPoint {
        ScreenPoint::new((self.x + other.x) / 2.0, (self.y + other.y) / 2.0)
    }
}

impl Add for ScreenPoint {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        ScreenPoint::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for ScreenPoint {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        ScreenPoint::new(self.x - rhs.x, self.y - rhs.y)
    }
}

/// Width/height of the whole world in pixels at `zoom`.
fn world_size(zoom: f64) -> f64 {
    WORLD_SIZE_AT_ZOOM_0 * zoom.exp2()
}

/// Lat/lon -> absolute "world pixel" coordinates at `zoom`.
fn to_world(point: GeoPoint, zoom: f64) -> ScreenPoint {
    let size = world_size(zoom);
    let lat = point.lat.clamp(-MAX_LATITUDE, MAX_LATITUDE).to_radians();
    let x = (point.lon + 180.0) / 360.0 * size;
    let y = (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / PI) / 2.0 * size;
    ScreenPoint::new(x, y)
}

fn from_world(world: ScreenPoint, zoom: f64) -> GeoPoint {
    let size = world_size(zoom);
    let lon = world.x / size * 360.0 - 180.0;
    let lat = (PI * (1.0 - 2.0 * world.y / size))
        .sinh()
        .atan()
        .to_degrees();
    GeoPoint::new(lat, lon)
}

/// What part of the world is on screen. Zoom may be fractional (pinch / wheel).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Viewport {
    pub center: GeoPoint,
    pub zoom: f64,
    pub width: f64,
    pub height: f64,
    min_zoom: f64,
    max_zoom: f64,
}

impl Viewport {
    pub fn new(center: GeoPoint, zoom: f64, min_zoom: f64, max_zoom: f64) -> Self {
        Self {
            center,
            zoom: zoom.clamp(min_zoom, max_zoom),
            width: 0.0,
            height: 0.0,
            min_zoom,
            max_zoom,
        }
    }

    pub fn resize(&mut self, width: f64, height: f64) {
        self.width = width;
        self.height = height;
    }

    fn half_size(&self) -> ScreenPoint {
        ScreenPoint::new(self.width / 2.0, self.height / 2.0)
    }

    /// World-pixel position of the viewport's top-left corner.
    pub fn origin(&self) -> ScreenPoint {
        to_world(self.center, self.zoom) - self.half_size()
    }

    pub fn project(&self, point: GeoPoint) -> ScreenPoint {
        to_world(point, self.zoom) - self.origin()
    }

    pub fn unproject(&self, point: ScreenPoint) -> GeoPoint {
        from_world(point + self.origin(), self.zoom)
    }

    pub fn contains(&self, point: ScreenPoint, margin: f64) -> bool {
        (-margin..=self.width + margin).contains(&point.x)
            && (-margin..=self.height + margin).contains(&point.y)
    }

    /// Moves the map content by `delta` screen pixels (drag direction).
    pub fn pan_by(&mut self, delta: ScreenPoint) {
        self.center = from_world(to_world(self.center, self.zoom) - delta, self.zoom);
    }

    /// Changes zoom while keeping the geo point under `anchor` fixed on screen.
    pub fn zoom_at(&mut self, anchor: ScreenPoint, zoom: f64) {
        let anchored = self.unproject(anchor);
        self.zoom = zoom.clamp(self.min_zoom, self.max_zoom);
        let world = to_world(anchored, self.zoom) - anchor + self.half_size();
        self.center = from_world(world, self.zoom);
    }

    pub fn zoom_by(&mut self, delta: f64) {
        self.zoom_at(self.half_size(), self.zoom + delta);
    }

    /// Centers and zooms so `bounds` fits on screen with `padding_px` on every side.
    /// Returns false (and does nothing) while the viewport has no size yet.
    pub fn fit_bounds(&mut self, bounds: GeoBounds, padding_px: f64) -> bool {
        if self.width <= 0.0 || self.height <= 0.0 {
            return false;
        }
        // Project at zoom 0; every zoom level up doubles these spans.
        let (a, b) = (to_world(bounds.min, 0.0), to_world(bounds.max, 0.0));
        let span_x = (a.x - b.x).abs();
        let span_y = (a.y - b.y).abs();
        let room_x = (self.width - 2.0 * padding_px).max(1.0);
        let room_y = (self.height - 2.0 * padding_px).max(1.0);
        // How much to scale up from zoom 0 so the tighter axis just fits.
        // A single point gives infinity, which the clamp turns into max zoom.
        let scale = (room_x / span_x).min(room_y / span_y);
        self.zoom = scale.log2().clamp(self.min_zoom, self.max_zoom);
        self.center = from_world(a.midpoint(b), 0.0);
        true
    }

    /// Ground metres covered by one screen pixel at the view's center.
    pub fn meters_per_pixel(&self) -> f64 {
        2.0 * PI * EARTH_RADIUS_M * self.center.lat.to_radians().cos() / world_size(self.zoom)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PENN: GeoPoint = GeoPoint::new(39.9522, -75.1932);

    fn assert_close(a: f64, b: f64, tolerance: f64) {
        assert!((a - b).abs() < tolerance, "{a} != {b}");
    }

    fn viewport() -> Viewport {
        let mut viewport = Viewport::new(PENN, 17.0, 0.0, 22.0);
        viewport.resize(800.0, 600.0);
        viewport
    }

    #[test]
    fn world_round_trip() {
        let back = from_world(to_world(PENN, 17.3), 17.3);
        assert_close(back.lat, PENN.lat, 1e-9);
        assert_close(back.lon, PENN.lon, 1e-9);
    }

    #[test]
    fn known_mercator_position_for_london() {
        let world = to_world(GeoPoint::new(51.5074, -0.1278), 10.0);
        assert_eq!((world.x / WORLD_SIZE_AT_ZOOM_0) as u32, 511);
        assert_eq!((world.y / WORLD_SIZE_AT_ZOOM_0) as u32, 340);
    }

    #[test]
    fn center_projects_to_middle_of_screen() {
        let center = viewport().project(PENN);
        assert_close(center.x, 400.0, 1e-6);
        assert_close(center.y, 300.0, 1e-6);
    }

    #[test]
    fn zoom_at_keeps_anchor_fixed() {
        let mut viewport = viewport();
        let anchor = ScreenPoint::new(100.0, 450.0);
        let before = viewport.unproject(anchor);
        viewport.zoom_at(anchor, 18.4);
        let after = viewport.project(before);
        assert_close(after.x, anchor.x, 1e-6);
        assert_close(after.y, anchor.y, 1e-6);
    }

    #[test]
    fn pan_moves_content_with_drag() {
        let mut viewport = viewport();
        viewport.pan_by(ScreenPoint::new(30.0, -20.0));
        let moved = viewport.project(PENN);
        assert_close(moved.x, 430.0, 1e-6);
        assert_close(moved.y, 280.0, 1e-6);
    }

    #[test]
    fn meters_per_pixel_matches_projection() {
        let viewport = viewport();
        let east = viewport.project(PENN.offset_m(0.0, 100.0));
        let px = east.x - viewport.project(PENN).x;
        assert_close(100.0 / px, viewport.meters_per_pixel(), 1e-6);
        // ~0.9145 m/px at zoom 17 at Penn's latitude (using the mean Earth radius).
        assert_close(viewport.meters_per_pixel(), 0.9145, 0.001);
    }

    #[test]
    fn local_m_inverts_offset_m() {
        let (north, east) = PENN.offset_m(-12.5, 40.0).local_m(PENN);
        assert_close(north, -12.5, 1e-9);
        assert_close(east, 40.0, 1e-9);
    }

    #[test]
    fn offset_matches_distance() {
        assert_close(PENN.distance_m(PENN.offset_m(100.0, 0.0)), 100.0, 0.01);
        assert_close(PENN.distance_m(PENN.offset_m(0.0, 100.0)), 100.0, 0.01);
    }

    #[test]
    fn bounds_from_points() {
        assert_eq!(GeoBounds::from_points([]), None);
        let ne = PENN.offset_m(30.0, 91.0);
        let bounds = GeoBounds::from_points([ne, PENN, PENN.offset_m(10.0, 10.0)]).unwrap();
        assert_eq!(bounds, GeoBounds { min: PENN, max: ne });
        assert_close(bounds.width_m(), 91.0, 0.01);
        assert_close(bounds.height_m(), 30.0, 0.01);
    }

    #[test]
    fn fit_bounds_shows_all_corners_snugly() {
        let mut viewport = viewport();
        let bounds = GeoBounds {
            min: PENN,
            max: PENN.offset_m(30.0, 91.0),
        };
        assert!(viewport.fit_bounds(bounds, 20.0));
        let (sw, ne) = (viewport.project(bounds.min), viewport.project(bounds.max));
        for corner in [sw, ne] {
            assert!(
                viewport.contains(corner, -19.9),
                "{corner:?} outside padding"
            );
        }
        // The wide field is limited by width: it should span the padded width exactly.
        assert_close(ne.x - sw.x, 800.0 - 40.0, 1e-6);
    }

    #[test]
    fn fit_bounds_waits_for_size_and_clamps_points() {
        let mut viewport = Viewport::new(PENN, 17.0, 15.0, 21.0);
        let point = GeoBounds {
            min: PENN,
            max: PENN,
        };
        assert!(!viewport.fit_bounds(point, 0.0));
        viewport.resize(800.0, 600.0);
        assert!(viewport.fit_bounds(point, 0.0));
        assert_eq!(viewport.zoom, 21.0);
    }
}
