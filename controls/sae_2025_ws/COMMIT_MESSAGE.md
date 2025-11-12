feat(trajectory): add retry logic for trajectory generation

Add retry mechanism to handle collision and constraint violations during
trajectory planning.

- Add retry loop with max 5 attempts in plan_trajectory()
- Track attempts to prevent infinite loops
- Break early on valid trajectory without collisions
- Handle exceptions gracefully
- Remove temporary documentation files

Improves robustness when generating trajectories in complex scenarios
with multiple hoops and obstacles.

