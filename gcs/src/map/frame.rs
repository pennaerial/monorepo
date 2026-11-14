//! Field coordinates: metres in a frame aligned with the field, not with north.
//!
//! The real field can sit at any angle. Rotating GPS points into this frame before they
//! reach the map means the map always shows an upright `width_m x height_m` rectangle,
//! with no map-rotation plugin. Everything the map draws is a [`FieldPoint`].

use serde::Serialize;

use super::GeoPoint;

/// Metres from the field's origin corner: `x` along the long edge, `y` along the short edge.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize)]
pub struct FieldPoint {
    pub x: f64,
    pub y: f64,
}

impl FieldPoint {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// Three surveyed corners of the field. Measure these once at the competition.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FieldCorners {
    /// Becomes (0, 0): the bottom-left corner on screen.
    pub origin: GeoPoint,
    /// The corner at the other end of the long edge from `origin` (bottom-right on screen).
    pub x_end: GeoPoint,
    /// The corner at the other end of the short edge from `origin` (top-left on screen).
    /// Must be counter-clockwise from `x_end` as seen from above, or the field comes out
    /// mirrored (negative y).
    pub y_end: GeoPoint,
}

/// Converts between GPS and field coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FieldFrame {
    origin: GeoPoint,
    /// cos/sin of the x axis's angle, counter-clockwise from east.
    cos: f64,
    sin: f64,
    pub width_m: f64,
    pub height_m: f64,
}

impl FieldFrame {
    pub fn from_corners(corners: FieldCorners) -> Self {
        let (north, east) = corners.x_end.local_m(corners.origin);
        let width_m = north.hypot(east);
        let mut frame = Self {
            origin: corners.origin,
            cos: east / width_m,
            sin: north / width_m,
            width_m,
            height_m: 0.0,
        };
        // Only the part of the short edge perpendicular to x counts, so slightly
        // non-square surveyed corners still give a rectangle.
        frame.height_m = frame.to_field(corners.y_end).y;
        debug_assert!(
            frame.height_m > 0.0,
            "field corners are clockwise; swap x_end and y_end"
        );
        frame
    }

    /// The x axis's angle from east, counter-clockwise, in degrees.
    pub fn angle_deg(&self) -> f64 {
        self.sin.atan2(self.cos).to_degrees()
    }

    pub fn to_field(&self, point: GeoPoint) -> FieldPoint {
        let (north, east) = point.local_m(self.origin);
        // Rotate by -angle so the field's long edge lies along +x.
        FieldPoint::new(
            east * self.cos + north * self.sin,
            -east * self.sin + north * self.cos,
        )
    }

    pub fn to_geo(&self, point: FieldPoint) -> GeoPoint {
        let east = point.x * self.cos - point.y * self.sin;
        let north = point.x * self.sin + point.y * self.cos;
        self.origin.offset_m(north, east)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PENN: GeoPoint = GeoPoint::new(39.9522, -75.1932);

    fn assert_close(a: FieldPoint, b: FieldPoint) {
        assert!(
            (a.x - b.x).abs() < 1e-6 && (a.y - b.y).abs() < 1e-6,
            "{a:?} != {b:?}"
        );
    }

    /// A 91 x 30 m field whose long edge points `angle_deg` counter-clockwise from east.
    fn field(angle_deg: f64) -> FieldFrame {
        let (s, c) = angle_deg.to_radians().sin_cos();
        let (w, h) = (91.0, 30.0);
        FieldFrame::from_corners(FieldCorners {
            origin: PENN,
            x_end: PENN.offset_m(w * s, w * c),
            y_end: PENN.offset_m(h * c, -h * s),
        })
    }

    #[test]
    fn axis_aligned_field_is_east_north() {
        let frame = field(0.0);
        assert_close(
            frame.to_field(PENN.offset_m(10.0, 20.0)),
            FieldPoint::new(20.0, 10.0),
        );
    }

    #[test]
    fn rotated_corners_become_an_upright_rectangle() {
        for angle in [-120.0, -35.0, 0.0, 27.5, 90.0, 170.0] {
            let frame = field(angle);
            assert!((frame.width_m - 91.0).abs() < 1e-6);
            assert!((frame.height_m - 30.0).abs() < 1e-6);
            assert!((frame.angle_deg() - angle).abs() < 1e-6);
            let far = frame.to_geo(FieldPoint::new(91.0, 30.0));
            assert_close(frame.to_field(far), FieldPoint::new(91.0, 30.0));
        }
    }

    #[test]
    fn round_trip() {
        let frame = field(33.0);
        let point = FieldPoint::new(-4.0, 57.25);
        assert_close(frame.to_field(frame.to_geo(point)), point);
    }
}
