//! Turns raw pointer events into pan/pinch gestures. Mouse and touch share this path.
//! Pure logic, no Dioxus, so it's unit-tested below.

use std::collections::HashMap;

use crate::map::{ScreenPoint, Viewport};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Gesture {
    Pan(ScreenPoint),
    Pinch {
        anchor: ScreenPoint,
        pan: ScreenPoint,
        scale: f64,
    },
}

impl Gesture {
    pub fn apply(self, viewport: &mut Viewport) {
        match self {
            Gesture::Pan(delta) => viewport.pan_by(delta),
            Gesture::Pinch { anchor, pan, scale } => {
                viewport.pan_by(pan);
                viewport.zoom_at(anchor, viewport.zoom + scale.log2());
            }
        }
    }
}

/// Pointers currently pressed, by pointer id.
#[derive(Debug, Default)]
pub struct Gestures {
    pressed: HashMap<i32, ScreenPoint>,
}

impl Gestures {
    pub fn press(&mut self, id: i32, at: ScreenPoint) {
        self.pressed.insert(id, at);
    }

    pub fn release(&mut self, id: i32) {
        self.pressed.remove(&id);
    }

    /// Returns `None` for hover moves (pointer not pressed) and for 3+ finger touches.
    pub fn move_to(&mut self, id: i32, at: ScreenPoint) -> Option<Gesture> {
        // get_mut, not insert: a hovering mouse must not register as a press.
        let previous = std::mem::replace(self.pressed.get_mut(&id)?, at);
        match self.pressed.len() {
            1 => Some(Gesture::Pan(at - previous)),
            2 => {
                let other = *self.pressed.iter().find(|(&key, _)| key != id)?.1;
                let before = previous.distance(other);
                if before <= f64::EPSILON {
                    return None;
                }
                let (mid_before, mid_after) = (previous.midpoint(other), at.midpoint(other));
                Some(Gesture::Pinch {
                    anchor: mid_after,
                    pan: mid_after - mid_before,
                    scale: at.distance(other) / before,
                })
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hover_is_ignored() {
        let mut gestures = Gestures::default();
        assert_eq!(gestures.move_to(1, ScreenPoint::new(5.0, 5.0)), None);
        // ...and doesn't leave a phantom press behind.
        gestures.press(2, ScreenPoint::new(0.0, 0.0));
        assert!(matches!(
            gestures.move_to(2, ScreenPoint::new(1.0, 0.0)),
            Some(Gesture::Pan(_))
        ));
    }

    #[test]
    fn single_pointer_pans() {
        let mut gestures = Gestures::default();
        gestures.press(1, ScreenPoint::new(10.0, 10.0));
        assert_eq!(
            gestures.move_to(1, ScreenPoint::new(15.0, 7.0)),
            Some(Gesture::Pan(ScreenPoint::new(5.0, -3.0)))
        );
    }

    #[test]
    fn released_pointer_stops_panning() {
        let mut gestures = Gestures::default();
        gestures.press(1, ScreenPoint::new(10.0, 10.0));
        gestures.release(1);
        assert_eq!(gestures.move_to(1, ScreenPoint::new(15.0, 7.0)), None);
    }

    #[test]
    fn spreading_fingers_zooms_in() {
        let mut gestures = Gestures::default();
        gestures.press(1, ScreenPoint::new(0.0, 0.0));
        gestures.press(2, ScreenPoint::new(100.0, 0.0));
        let Some(Gesture::Pinch { scale, .. }) = gestures.move_to(2, ScreenPoint::new(200.0, 0.0))
        else {
            panic!("expected pinch");
        };
        assert!((scale - 2.0).abs() < 1e-9);
    }
}
