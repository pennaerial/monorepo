//! Debug-only fake data so the map can be developed without ROS. Not in release builds.
//!
//! - An (invisible) drone flies a lawnmower survey over the field and reports mines.
//! - A person starts south of the field and walks to a target north of it.
//! - A placeholder "planner" re-plans the person's route around known mines every tick.

use std::time::Duration;

use dioxus::prelude::*;

use super::state::MapStateSignal;
use super::{GeoBounds, GeoPoint, Mine, MineId, MineStatus};

const TICK: Duration = Duration::from_millis(200);
const DRONE_M_PER_TICK: f64 = 1.0; // 5 m/s
const WALK_M_PER_TICK: f64 = 0.1; // 0.5 m/s, careful walking pace
const LANE_SPACING_M: f64 = 5.0;
const TICKS_PER_MINE: u32 = 30;
/// How far the mock route keeps from a mine.
const CLEARANCE_M: f64 = 4.0;

pub fn use_mock_feed(mut state: MapStateSignal, field: GeoBounds) {
    use_future(move || async move {
        let survey = Survey::over(field);
        let start = field.min.offset_m(-8.0, field.width_m() * 0.15);
        let target = field
            .min
            .offset_m(field.height_m() + 6.0, field.width_m() * 0.85);

        let mut tick = 0u32;
        let mut person = start;
        loop {
            tokio::time::sleep(TICK).await;
            // The write lock is dropped at the end of each iteration, before the next await.
            let mut map = state.write();

            if person.distance_m(target) < WALK_M_PER_TICK {
                // Arrived: start over.
                *map = Default::default();
                tick = 0;
                person = start;
                continue;
            }

            // Drone: keep surveying and reporting mines until the pattern is done.
            let flown = f64::from(tick) * DRONE_M_PER_TICK;
            if flown < survey.length_m() && tick % TICKS_PER_MINE == 0 {
                let id = tick / TICKS_PER_MINE;
                let position = survey
                    .position(flown)
                    .offset_m(jitter(id, 1) * 2.5, jitter(id, 2) * 3.0);
                map.mines.upsert(Mine {
                    id: MineId(id),
                    position,
                    status: MineStatus::Detected,
                });
                if let Some(previous) = id.checked_sub(1) {
                    map.mines
                        .set_status(MineId(previous), MineStatus::Confirmed);
                }
            }

            // Planner: route from where the person is now, then take one step along it.
            let mines: Vec<GeoPoint> = map.mines.iter().map(|mine| mine.position).collect();
            let route = plan_around(person, target, &mines);
            person = step_toward(person, route[1], WALK_M_PER_TICK);

            map.person = Some(person);
            map.target = Some(target);
            map.route = route;
            tick += 1;
        }
    });
}

/// Placeholder planner, NOT a real one: a straight line with one sidestep per nearby mine.
/// It can still pass close to mines that are clustered together. The real planner
/// replaces this and just writes `MapState::route`.
fn plan_around(from: GeoPoint, to: GeoPoint, mines: &[GeoPoint]) -> Vec<GeoPoint> {
    // Work in metres relative to `from`: `along` the line, and `left` of it.
    let (north, east) = to.local_m(from);
    let length = north.hypot(east);
    if length < 1e-6 {
        return vec![from, to];
    }
    let (dir_n, dir_e) = (north / length, east / length);

    let mut detours: Vec<(f64, f64)> = mines
        .iter()
        .filter_map(|mine| {
            let (n, e) = mine.local_m(from);
            let along = n * dir_n + e * dir_e;
            let left = n * dir_e - e * dir_n; // dot product with the left vector below
            let near_line = (0.0..length).contains(&along) && left.abs() < CLEARANCE_M;
            // Step to whichever side of the mine is closer to the line.
            near_line.then(|| (along, left - left.signum() * CLEARANCE_M))
        })
        // A sidestep onto where we already stand would stall the walk forever.
        .filter(|&(along, left): &(f64, f64)| along.hypot(left) > 0.5)
        .collect();
    detours.sort_by(|a, b| a.0.total_cmp(&b.0));

    // Back from (along, left) to a lat/lon. The "left" unit vector, as (north, east),
    // is (dir_e, -dir_n): e.g. heading north (1, 0), left is west (0, -1).
    let to_geo = |(along, left): (f64, f64)| {
        from.offset_m(along * dir_n + left * dir_e, along * dir_e - left * dir_n)
    };
    std::iter::once(from)
        .chain(detours.into_iter().map(to_geo))
        .chain(std::iter::once(to))
        .collect()
}

/// Moves up to `step_m` from `from` toward `to`, without overshooting.
fn step_toward(from: GeoPoint, to: GeoPoint, step_m: f64) -> GeoPoint {
    let (north, east) = to.local_m(from);
    let distance = north.hypot(east);
    if distance <= step_m {
        return to;
    }
    from.offset_m(north / distance * step_m, east / distance * step_m)
}

/// Deterministic pseudo-random number in [-1, 1) so mines don't sit in a perfect row.
fn jitter(id: u32, salt: u32) -> f64 {
    let hash = (id ^ salt.wrapping_mul(0x9E37_79B9))
        .wrapping_mul(0x85EB_CA6B)
        .rotate_left(13)
        .wrapping_mul(0xC2B2_AE35);
    f64::from(hash >> 8) / f64::from(1u32 << 24) * 2.0 - 1.0
}

/// The drone's back-and-forth east/west lanes, stepping north between them.
struct Survey {
    origin: GeoPoint,
    lane_length_m: f64,
    lane_count: u32,
}

impl Survey {
    fn over(area: GeoBounds) -> Self {
        Self {
            origin: area.min,
            lane_length_m: area.width_m(),
            lane_count: (area.height_m() / LANE_SPACING_M).floor() as u32 + 1,
        }
    }

    /// Distance flown from the first lane's start to the last lane's end.
    fn length_m(&self) -> f64 {
        f64::from(self.lane_count) * (self.lane_length_m + LANE_SPACING_M) - LANE_SPACING_M
    }

    /// Position after flying `distance_m` along the pattern.
    fn position(&self, distance_m: f64) -> GeoPoint {
        let lane_with_turn = self.lane_length_m + LANE_SPACING_M;
        let lane = (distance_m / lane_with_turn).floor();
        let along = distance_m % lane_with_turn;
        let eastbound = lane as u32 % 2 == 0;

        let (north, east) = if along < self.lane_length_m {
            let east = if eastbound {
                along
            } else {
                self.lane_length_m - along
            };
            (lane * LANE_SPACING_M, east)
        } else {
            // Turning: fly north to the next lane at the end we finished on.
            let east = if eastbound { self.lane_length_m } else { 0.0 };
            (lane * LANE_SPACING_M + (along - self.lane_length_m), east)
        };
        self.origin.offset_m(north, east)
    }
}
