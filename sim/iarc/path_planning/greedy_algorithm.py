#####
# High-level design assumptions

# Map is a 2D grid
# Each cell has a clearance value:
#   clearance > 0 -> traversable
#   clearance <= 0 -> unsafe (mine / obstacle)
# Unknown areas are marked separately and not trusted
# Planner runs repeatedly as new data arrives
####

import heapq
import math
from typing import List, Tuple, Optional, Dict


class State:
    """
    Search state used in greedy bottleneck planning.
    """

    def __init__(self, x: int, y: int, bottleneck: float, parent: Optional["State"]):
        self.x = x
        self.y = y
        self.w = bottleneck       # Path bottleneck up to this state
        self.parent = parent

    def __lt__(self, other):
        """
        Comparator for priority queue.
        Python heapq is a min-heap, so we invert bottleneck.
        """
        return self.w > other.w   # higher bottleneck = higher priority


class GreedyBottleneckPlanner:
    """
    Greedy bottleneck-preserving path planner for IARC Mission 10.
    """

    def __init__(
        self,
        clearance_map: List[List[float]],
        known_map: List[List[bool]],
        start_cells: List[Tuple[int, int]],
        goal_cells: List[Tuple[int, int]],
    ):
        """
        Parameters
            clearance_map : 2D array
                Clearance value for each cell (updated online).
            known_map : 2D boolean array
                True if the cell has been observed, False if unknown.
            start_cells : list of (x, y)
                Start boundary cells.
            goal_cells : list of (x, y)
                Goal boundary cells.
        """
        self.clearance = clearance_map
        self.known = known_map
        self.start_cells = start_cells
        self.goal_cells = set(goal_cells)

        self.height = len(clearance_map)
        self.width = len(clearance_map[0])

        # For pruning: best bottleneck achieved at each cell
        self.best_bottleneck = [
            [-math.inf for _ in range(self.width)]
            for _ in range(self.height)
        ]
        
    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def neighbors(self, x: int, y: int):
        """
        4-connected grid neighbors.
        """
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if self.in_bounds(nx, ny):
                yield nx, ny

    def heuristic_distance_to_goal(self, x: int, y: int) -> float:
        """
        Heuristic used only for tie-breaking.
        """
        return min(abs(x - gx) + abs(y - gy) for gx, gy in self.goal_cells)

    def reconstruct_path(self, state: State) -> List[Tuple[int, int]]:
        """
        Backtrack parents to reconstruct path.
        """
        path = []
        while state is not None:
            path.append((state.x, state.y))
            state = state.parent
        return path[::-1]


    def plan(self) -> Optional[Dict]:
        """
        Perform greedy bottleneck-preserving planning.

        Returns: dict or None
            {
                "path": list of (x, y),
                "bottleneck": float,
                "length": int
            }
        """

        open_set = []

        # Initialize from all start boundary cells which are known and safe
        for x, y in self.start_cells:
            if not self.known[y][x]:
                continue
            if self.clearance[y][x] <= 0:
                continue

            s = State(x, y, self.clearance[y][x], None)
            self.best_bottleneck[y][x] = s.w
            # heapq.heappush(open_set, s)
            heapq.heappush(  # lexicographic priority queue
                open_set,
                # maximizes bottleneck first and breaks ties by distance-to-goal
                (-s.w, self.heuristic_distance_to_goal(x, y), s) # (−bottleneck, heuristic_distance, state)
            )

        # Main greedy loop
        while open_set:
            # select the state with the biggest bottleneck (most safe)
            _, _, current = heapq.heappop(open_set)

            # Goal reached
            if (current.x, current.y) in self.goal_cells:
                path = self.reconstruct_path(current)
                return {
                    "path": path,
                    "bottleneck": current.w,
                    "length": len(path),
                }

            # Expand neighbors
            for nx, ny in self.neighbors(current.x, current.y):

                # Unknown area is not trusted for traversal
                if not self.known[ny][nx]:
                    continue
                # Unsafe cell
                if self.clearance[ny][nx] <= 0:
                    continue

                # Bottleneck update (monotone non-increasing)
                new_w = min(current.w, self.clearance[ny][nx])

                # Greedy pruning
                # Only keep the "most safe" path to walk on in the same location
                if new_w <= self.best_bottleneck[ny][nx]:
                    continue

                self.best_bottleneck[ny][nx] = new_w
                new_state = State(nx, ny, new_w, current) # add backpointer
                # heapq.heappush(open_set, new_state)
                heapq.heappush(
                    open_set,
                    (-new_w, self.heuristic_distance_to_goal(nx, ny), new_state)
                )

        # No path found
        return None


    def suggest_exploration_targets(
        self,
        path: List[Tuple[int, int]],
        num_drones: int = 4,
        radius: int = 5,
    ) -> List[Tuple[int, int]]:
        """
        Suggest exploration targets for multiple drones.

        Strategy:
        - Sample points ahead of the current best path
        - Prefer unknown cells near the path with high clearance

        Parameters
            path : list of (x, y)
                Current best path.
            num_drones : int
                Number of drones.
            radius : int
                Neighborhood radius around path points.

        Returns
            list of (x, y)
                Exploration target coordinates.
        """

        candidates = []

        # Sample along the path
        for px, py in path[::max(1, len(path)//10)]:
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = px + dx, py + dy
                    if not self.in_bounds(nx, ny):
                        continue
                    if self.known[ny][nx]:
                        continue
                    candidates.append((nx, ny))

        # Rank candidates by Manhattan distance to last path point −(∣x−xg​∣+∣y−yg​∣)
        def score(cell):
            x, y = cell
            return -abs(x - path[-1][0]) - abs(y - path[-1][1])

        candidates = sorted(set(candidates), key=score)

        return candidates[:num_drones]
