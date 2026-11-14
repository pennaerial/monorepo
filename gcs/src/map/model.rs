//! Mines as the drone reports them. Plain Rust: ROS handlers fill this, layers read it.

use std::collections::BTreeMap;

use super::GeoPoint;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MineId(pub u32);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MineStatus {
    #[default]
    Detected,
    Confirmed,
    Cleared,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Mine {
    pub id: MineId,
    pub position: GeoPoint,
    pub status: MineStatus,
}

/// Mines keyed by id, so repeated detections update in place instead of duplicating.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MineStore {
    mines: BTreeMap<MineId, Mine>,
}

impl MineStore {
    /// Inserts or replaces a mine, returning the previous value.
    pub fn upsert(&mut self, mine: Mine) -> Option<Mine> {
        self.mines.insert(mine.id, mine)
    }

    /// Returns false if the mine isn't known.
    pub fn set_status(&mut self, id: MineId, status: MineStatus) -> bool {
        self.mines
            .get_mut(&id)
            .map(|mine| mine.status = status)
            .is_some()
    }

    pub fn remove(&mut self, id: MineId) -> Option<Mine> {
        self.mines.remove(&id)
    }

    pub fn iter(&self) -> impl Iterator<Item = &Mine> {
        self.mines.values()
    }

    pub fn len(&self) -> usize {
        self.mines.len()
    }

    pub fn is_empty(&self) -> bool {
        self.mines.is_empty()
    }

    pub fn clear(&mut self) {
        self.mines.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ORIGIN: GeoPoint = GeoPoint::new(39.9522, -75.1932);

    fn mine(id: u32) -> Mine {
        Mine {
            id: MineId(id),
            position: ORIGIN,
            status: MineStatus::Detected,
        }
    }

    #[test]
    fn upsert_replaces_by_id() {
        let mut store = MineStore::default();
        assert!(store.upsert(mine(1)).is_none());
        assert!(store.upsert(mine(1)).is_some());
        assert_eq!(store.len(), 1);
    }

    #[test]
    fn set_status_and_remove() {
        let mut store = MineStore::default();
        store.upsert(mine(1));
        assert!(store.set_status(MineId(1), MineStatus::Confirmed));
        assert!(!store.set_status(MineId(2), MineStatus::Confirmed));
        assert_eq!(store.iter().next().unwrap().status, MineStatus::Confirmed);
        assert!(store.remove(MineId(1)).is_some());
        assert!(store.is_empty());
    }
}
