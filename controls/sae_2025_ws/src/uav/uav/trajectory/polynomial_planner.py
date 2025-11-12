import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class TrajectorySegment:
    """Represents a single polynomial segment between two waypoints."""
    coefficients: np.ndarray  # Polynomial coefficients [a0, a1, a2, ..., an] for p(t) = a0 + a1*t + a2*t^2 + ...
    start_time: float  # Start time of this segment
    duration: float  # Duration of this segment
    start_pos: np.ndarray  # Start position [x, y, z]
    end_pos: np.ndarray  # End position [x, y, z]

@dataclass
class Trajectory:
    """Complete trajectory composed of multiple polynomial segments."""
    segments: List[TrajectorySegment]  # List of trajectory segments
    waypoints: List[np.ndarray]  # Waypoint positions
    waypoint_times: List[float]  # Waypoint times (cumulative)
    total_duration: float  # Total trajectory duration
    polynomial_order: int  # Order of polynomials used

class PolynomialTrajectoryPlanner:
    def __init__(self, polynomial_order=7, minimize_derivative=4, 
                 max_velocity=2.0, max_acceleration=1.0, max_jerk=2.0,
                 duration_factor=1.2, min_duration=1.0, max_duration=30.0):
        """
        Args:
            polynomial_order: Order of polynomial (5, 7, or 9)
            minimize_derivative: Which derivative to minimize (3=jerk, 4=snap)
            max_velocity: Maximum allowed velocity (m/s)
            max_acceleration: Maximum allowed acceleration (m/s²)
            max_jerk: Maximum allowed jerk (m/s³)
            duration_factor: Safety factor for duration calculation
            min_duration: Minimum trajectory duration (s)
            max_duration: Maximum trajectory duration (s)
        """
        self.order = polynomial_order
        self.minimize_derivative = minimize_derivative
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_jerk = max_jerk
        self.duration_factor = duration_factor
        self.min_duration = min_duration
        self.max_duration = max_duration
    
    def calculate_trajectory_duration(self, start_pos, end_pos, 
                                     start_vel=None, end_vel=None):
        """
        Calculate optimal trajectory duration based on distance and constraints.
        
        Args:
            start_pos: Starting position [x, y, z]
            end_pos: Ending position [x, y, z]
            start_vel: Starting velocity [vx, vy, vz] (optional)
            end_vel: Ending velocity [vx, vy, vz] (optional)
        
        Returns:
            Duration in seconds
        """
        # Calculate 3D distance
        distance = np.linalg.norm(np.array(end_pos) - np.array(start_pos))
        
        # Calculate speed along trajectory direction
        direction = (np.array(end_pos) - np.array(start_pos)) / distance if distance > 0 else np.array([1, 0, 0])
        start_speed = np.dot(start_vel, direction) if start_vel is not None else 0.0
        end_speed = np.dot(end_vel, direction) if end_vel is not None else 0.0
        

        # Estimate minimum time using trapezoidal velocity profile
        # There are 2 ways we can achieve this
        # 1. Get the average speed (if we have start and end speed), and get the time from that
        # 2. Get the time to accelerate to max velocity, time at max velocity, and the time to decelerate from max velocity to end speed


        # Method 1: Simple estimation
        avg_speed = (start_speed + end_speed) / 2.0
        if avg_speed > 0:
            time_from_speed = distance / avg_speed
        else:
            time_from_speed = np.sqrt(2 * distance / self.max_acceleration)
        
        # Method 2: Account for acceleration limits
        # Time to accelerate from start_speed to max_velocity
        if start_speed < self.max_velocity:
            accel_time = (self.max_velocity - start_speed) / self.max_acceleration
            accel_distance = start_speed * accel_time + 0.5 * self.max_acceleration * accel_time**2
        else:
            accel_time = 0
            accel_distance = 0
        
        # Time to decelerate from max_velocity to end_speed
        if end_speed < self.max_velocity:
            decel_time = (self.max_velocity - end_speed) / self.max_acceleration
            decel_distance = self.max_velocity * decel_time - 0.5 * self.max_acceleration * decel_time**2
        else:
            decel_time = 0
            decel_distance = 0
        
        # Cruise distance
        cruise_distance = distance - accel_distance - decel_distance
        if cruise_distance > 0:
            cruise_time = cruise_distance / self.max_velocity
        else:
            # Not enough distance to reach max velocity
            # Use quadratic formula: d = v₀t + ½at², solve for t
            # Where a changes sign at midpoint
            cruise_time = 0
            # Simplified: assume constant acceleration
            if start_speed != end_speed or distance > 0:
                # Solve: distance = (start_speed + end_speed) * t / 2
                if (start_speed + end_speed) > 0:
                    time_from_speed = 2 * distance / (start_speed + end_speed)
                else:
                    time_from_speed = np.sqrt(2 * distance / self.max_acceleration)
        
        total_time = accel_time + cruise_time + decel_time
        if total_time < time_from_speed:
            total_time = time_from_speed
        
        # Apply safety factor and enforce limits
        duration = total_time * self.duration_factor
        duration = max(self.min_duration, min(duration, self.max_duration))
        
        return duration



    def generate_waypoints_and_times(self, start_pos, end_pos, start_vel=None):
        """
        Generate waypoints and time allocation for trajectory.
        
        For long distances, add intermediate waypoints to ensure smooth trajectories.
        
        Returns:
            waypoints: List of [x, y, z] positions
            waypoint_times: List of times for each waypoint (cumulative)
        """
        distance = np.linalg.norm(np.array(end_pos) - np.array(start_pos))
        
        # Calculate total duration
        total_duration = self.calculate_trajectory_duration(start_pos, end_pos, start_vel)
        
        # For long trajectories, add intermediate waypoints
        waypoint_segment_length = 0.5  # meters (configurable)
        num_segments = max(1, int(distance / waypoint_segment_length))
        
        if num_segments == 1:
            # Single segment
            waypoints = [start_pos, end_pos]
            waypoint_times = [0.0, total_duration]
        else:
            # Multiple segments
            waypoints = [start_pos]
            waypoint_times = [0.0]
            
            direction = (np.array(end_pos) - np.array(start_pos)) / distance
            segment_length = distance / num_segments
            segment_time = total_duration / num_segments
            
            for i in range(1, num_segments):
                waypoint = np.array(start_pos) + direction * (i * segment_length)
                waypoints.append(waypoint.tolist())
                waypoint_times.append(i * segment_time)
            
            waypoints.append(end_pos)
            waypoint_times.append(total_duration)
        
        return waypoints, waypoint_times

    
    def _compute_polynomial_matrix(self, t: float, n: int, derivative: int = 0) -> np.ndarray:
        """
        Compute matrix for evaluating polynomial derivatives at time t.
        
        Args:
            t: Time
            n: Polynomial order
            derivative: Which derivative to compute (0=position, 1=velocity, 2=acceleration, etc.)
        
        Returns:
            Row vector [t^0, t^1, t^2, ..., t^n] scaled by derivative factor
        """
        if derivative == 0:
            # Position: [1, t, t^2, ..., t^n]
            return np.array([t**i for i in range(n + 1)])
        else:
            # Derivative: multiply by factorial coefficients
            result = np.zeros(n + 1)
            for i in range(derivative, n + 1):
                # d/dt (t^i) = i * t^(i-1)
                # d²/dt² (t^i) = i*(i-1) * t^(i-2)
                # etc.
                coeff = 1.0
                for j in range(derivative):
                    coeff *= (i - j)
                if i - derivative >= 0:
                    result[i] = coeff * (t ** (i - derivative))
            return result
    
    def _solve_polynomial_segment(self, start_pos: np.ndarray, end_pos: np.ndarray,
                                  start_vel: Optional[np.ndarray], end_vel: Optional[np.ndarray],
                                  start_acc: Optional[np.ndarray], end_acc: Optional[np.ndarray],
                                  duration: float) -> np.ndarray:
        """
        Solve for polynomial coefficients for a single segment.
        
        For order 7 polynomial (8 coefficients), we can specify:
        - Position at t=0 and t=duration (6 constraints: 3D position)
        - Velocity at t=0 and t=duration (6 constraints: 3D velocity) 
        - Acceleration at t=0 and t=duration (6 constraints: 3D acceleration)
        Total: 18 constraints, but we only have 8 coefficients per axis = 24 total
        
        For simplicity, we'll solve each axis independently with:
        - Position, velocity, acceleration at start (3 constraints)
        - Position, velocity, acceleration at end (3 constraints)
        - Optional: jerk at start = 0, jerk at end = 0 (2 constraints) for smoothness
        Total: 8 constraints for 8 coefficients (perfect!)
        
        Args:
            start_pos: Start position [x, y, z]
            end_pos: End position [x, y, z]
            start_vel: Start velocity [vx, vy, vz] (optional, defaults to [0,0,0])
            end_vel: End velocity [vx, vy, vz] (optional, defaults to [0,0,0])
            start_acc: Start acceleration [ax, ay, az] (optional, defaults to [0,0,0])
            end_acc: End acceleration [ax, ay, az] (optional, defaults to [0,0,0])
            duration: Segment duration
        
        Returns:
            coefficients: Array of shape (3, n+1) where coefficients[i] are coeffs for axis i
        """
        n = self.order
        if start_vel is None:
            start_vel = np.zeros(3)
        if end_vel is None:
            end_vel = np.zeros(3)
        if start_acc is None:
            start_acc = np.zeros(3)
        if end_acc is None:
            end_acc = np.zeros(3)
        
        # Solve for each axis independently
        coefficients = np.zeros((3, n + 1))
        
        for axis in range(3):
            # Build constraint matrix A and constraint vector b
            # We have 8 constraints for 8 coefficients
            
            # Constraints at t=0:
            # 1. Position: p(0) = start_pos
            # 2. Velocity: p'(0) = start_vel
            # 3. Acceleration: p''(0) = start_acc
            # 4. Jerk: p'''(0) = 0 (for smoothness)
            
            # Constraints at t=duration:
            # 5. Position: p(T) = end_pos
            # 6. Velocity: p'(T) = end_vel
            # 7. Acceleration: p''(T) = end_acc
            # 8. Jerk: p'''(T) = 0 (for smoothness)
            
            A = np.zeros((n + 1, n + 1))
            b = np.zeros(n + 1)
            
            # Constraint 1: Position at t=0
            A[0] = self._compute_polynomial_matrix(0.0, n, 0)
            b[0] = start_pos[axis]
            
            # Constraint 2: Velocity at t=0
            A[1] = self._compute_polynomial_matrix(0.0, n, 1)
            b[1] = start_vel[axis]
            
            # Constraint 3: Acceleration at t=0
            A[2] = self._compute_polynomial_matrix(0.0, n, 2)
            b[2] = start_acc[axis]
            
            # Constraint 4: Jerk at t=0 = 0
            A[3] = self._compute_polynomial_matrix(0.0, n, 3)
            b[3] = 0.0
            
            # Constraint 5: Position at t=duration
            A[4] = self._compute_polynomial_matrix(duration, n, 0)
            b[4] = end_pos[axis]
            
            # Constraint 6: Velocity at t=duration
            A[5] = self._compute_polynomial_matrix(duration, n, 1)
            b[5] = end_vel[axis]
            
            # Constraint 7: Acceleration at t=duration
            A[6] = self._compute_polynomial_matrix(duration, n, 2)
            b[6] = end_acc[axis]
            
            # Constraint 8: Jerk at t=duration = 0
            A[7] = self._compute_polynomial_matrix(duration, n, 3)
            b[7] = 0.0
            
            # Solve: A * coefficients = b
            try:
                coeffs = np.linalg.solve(A, b)
                coefficients[axis] = coeffs
            except np.linalg.LinAlgError:
                # If matrix is singular, use least squares
                coeffs = np.linalg.lstsq(A, b, rcond=None)[0]
                coefficients[axis] = coeffs
        
        return coefficients
    
    def generate_trajectory(self, start_pos, end_pos, 
                           start_vel=None, start_acc=None,
                           end_vel=None, end_acc=None):
        """
        Generate polynomial trajectory from start to end position.
        
        Duration is calculated dynamically based on distance and constraints.
        
        Args:
            start_pos: Starting position [x, y, z]
            end_pos: Ending position [x, y, z]
            start_vel: Starting velocity [vx, vy, vz] (optional)
            start_acc: Starting acceleration [ax, ay, az] (optional)
            end_vel: Ending velocity [vx, vy, vz] (optional)
            end_acc: Ending acceleration [ax, ay, az] (optional)
        
        Returns:
            Trajectory object with polynomial coefficients and waypoint times
        """
        # Generate waypoints and times
        waypoints, waypoint_times = self.generate_waypoints_and_times(
            start_pos, end_pos, start_vel
        )
        
        # Convert to numpy arrays
        waypoints = [np.array(wp) for wp in waypoints]
        
        # Pre-compute velocities at intermediate waypoints for smoothness
        # For intermediate waypoints, compute velocity based on direction to next waypoint
        waypoint_velocities = [None] * len(waypoints)
        
        # Set start and end velocities
        waypoint_velocities[0] = np.array(start_vel) if start_vel is not None else np.zeros(3)
        waypoint_velocities[-1] = np.array(end_vel) if end_vel is not None else np.zeros(3)
        
        # Compute velocities at intermediate waypoints based on average direction
        for i in range(1, len(waypoints) - 1):
            # Direction from previous to current waypoint
            dir_prev = waypoints[i] - waypoints[i - 1]
            dist_prev = np.linalg.norm(dir_prev)
            if dist_prev > 1e-6:
                dir_prev = dir_prev / dist_prev
            
            # Direction from current to next waypoint
            dir_next = waypoints[i + 1] - waypoints[i]
            dist_next = np.linalg.norm(dir_next)
            if dist_next > 1e-6:
                dir_next = dir_next / dist_next
            
            # Average direction (weighted by segment durations)
            dt_prev = waypoint_times[i] - waypoint_times[i - 1]
            dt_next = waypoint_times[i + 1] - waypoint_times[i]
            avg_dir = (dir_prev * dt_prev + dir_next * dt_next) / (dt_prev + dt_next)
            avg_dir = avg_dir / (np.linalg.norm(avg_dir) + 1e-6)
            
            # Estimate speed based on average segment length and time
            avg_speed = 0.5 * ((dist_prev / dt_prev) + (dist_next / dt_next)) if (dt_prev > 0 and dt_next > 0) else 1.0
            avg_speed = min(avg_speed, self.max_velocity * 0.8)  # Cap at 80% of max velocity
            
            waypoint_velocities[i] = avg_dir * avg_speed
        
        # Generate polynomial segments for each waypoint pair
        segments = []
        
        for i in range(len(waypoints) - 1):
            seg_start_pos = waypoints[i]
            seg_end_pos = waypoints[i + 1]
            seg_start_time = waypoint_times[i]
            seg_duration = waypoint_times[i + 1] - waypoint_times[i]
            
            # Get velocities at waypoints
            seg_start_vel = waypoint_velocities[i]
            seg_end_vel = waypoint_velocities[i + 1]
            
            # Determine accelerations
            if i == 0:
                # First segment: use provided start_acc
                seg_start_acc = np.array(start_acc) if start_acc is not None else np.zeros(3)
            else:
                # Subsequent segments: ensure continuity with previous segment
                prev_seg = segments[i - 1]
                _, _, seg_start_acc = self._evaluate_segment_derivatives(
                    prev_seg.coefficients, prev_seg.duration
                )
            
            if i == len(waypoints) - 2:
                # Last segment: use provided end_acc
                seg_end_acc = np.array(end_acc) if end_acc is not None else np.zeros(3)
            else:
                # Intermediate segments: zero acceleration for smoothness
                seg_end_acc = np.zeros(3)
            
            # Solve for coefficients
            coefficients = self._solve_polynomial_segment(
                seg_start_pos, seg_end_pos,
                seg_start_vel, seg_end_vel,
                seg_start_acc, seg_end_acc,
                seg_duration
            )
            
            # Create segment
            segment = TrajectorySegment(
                coefficients=coefficients,
                start_time=seg_start_time,
                duration=seg_duration,
                start_pos=seg_start_pos,
                end_pos=seg_end_pos
            )
            segments.append(segment)
        
        # Create trajectory
        trajectory = Trajectory(
            segments=segments,
            waypoints=waypoints,
            waypoint_times=waypoint_times,
            total_duration=waypoint_times[-1],
            polynomial_order=self.order
        )
        
        return trajectory
    
    def _evaluate_segment_derivatives(self, coefficients: np.ndarray, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Evaluate position, velocity, and acceleration of a segment at time t.
        
        Args:
            coefficients: Polynomial coefficients (shape: 3 x n+1)
            t: Time relative to segment start
        
        Returns:
            position, velocity, acceleration (each is [x, y, z])
        """
        n = coefficients.shape[1] - 1
        pos = np.zeros(3)
        vel = np.zeros(3)
        acc = np.zeros(3)
        
        for axis in range(3):
            # Position
            pos_vec = self._compute_polynomial_matrix(t, n, 0)
            pos[axis] = np.dot(coefficients[axis], pos_vec)
            
            # Velocity
            vel_vec = self._compute_polynomial_matrix(t, n, 1)
            vel[axis] = np.dot(coefficients[axis], vel_vec)
            
            # Acceleration
            acc_vec = self._compute_polynomial_matrix(t, n, 2)
            acc[axis] = np.dot(coefficients[axis], acc_vec)
        
        return pos, vel, acc

    def evaluate_trajectory(self, trajectory: Trajectory, time: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Evaluate trajectory at a specific time.
        
        Args:
            trajectory: Trajectory object
            time: Time since trajectory start (in seconds)
        
        Returns:
            position, velocity, acceleration, jerk at time t (each is [x, y, z])
        """
        # Clamp time to trajectory duration
        time = max(0.0, min(time, trajectory.total_duration))
        
        # Find which segment this time belongs to
        segment_idx = 0
        for i, seg in enumerate(trajectory.segments):
            if time <= seg.start_time + seg.duration:
                segment_idx = i
                break
        else:
            # Time is at or beyond last segment, use last segment
            segment_idx = len(trajectory.segments) - 1
        
        # Get segment and relative time
        segment = trajectory.segments[segment_idx]
        t_rel = time - segment.start_time
        
        # Evaluate segment
        pos, vel, acc = self._evaluate_segment_derivatives(segment.coefficients, t_rel)
        
        # Compute jerk
        jerk = np.zeros(3)
        n = segment.coefficients.shape[1] - 1
        for axis in range(3):
            jerk_vec = self._compute_polynomial_matrix(t_rel, n, 3)
            jerk[axis] = np.dot(segment.coefficients[axis], jerk_vec)
        
        return pos, vel, acc, jerk
    
    def check_collision(self, trajectory: Trajectory, hoop_geometries: List, safety_margin: float = 0.5) -> bool:
        """
        Check if trajectory collides with any hoop geometry.
        
        Args:
            trajectory: Trajectory to check
            hoop_geometries: List of hoop geometry objects (each should have:
                - center_position: [x, y, z]
                - normal_vector: [nx, ny, nz] (unit vector)
                - radius: float
                - thickness: float)
            safety_margin: Safety margin in meters
        
        Returns:
            bool: True if collision detected, False otherwise
        """
        # Sample trajectory at regular intervals
        sample_rate = 10.0  # Hz
        dt = 1.0 / sample_rate
        
        for t in np.arange(0.0, trajectory.total_duration, dt):
            pos, _, _, _ = self.evaluate_trajectory(trajectory, t)
            
            # Check collision with each hoop
            for hoop in hoop_geometries:
                if self._point_in_hoop_cylinder(pos, hoop, safety_margin):
                    return True
        
        # Also check end position
        pos, _, _, _ = self.evaluate_trajectory(trajectory, trajectory.total_duration)
        for hoop in hoop_geometries:
            if self._point_in_hoop_cylinder(pos, hoop, safety_margin):
                return True
        
        return False
    
    def _point_in_hoop_cylinder(self, point: np.ndarray, hoop, safety_margin: float) -> bool:
        """
        Check if a point is inside a hoop cylinder.
        
        Hoop is represented as a cylinder:
        - Center: hoop.center_position
        - Axis: hoop.normal_vector (unit vector)
        - Radius: hoop.radius + safety_margin
        - Height: hoop.thickness + safety_margin
        
        Args:
            point: Point to check [x, y, z]
            hoop: Hoop geometry object
            safety_margin: Safety margin in meters
        
        Returns:
            bool: True if point is inside cylinder
        """
        point = np.array(point)
        center = np.array(hoop.center_position)
        axis = np.array(hoop.normal_vector)
        axis = axis / np.linalg.norm(axis)  # Ensure unit vector
        
        # Vector from center to point
        vec_to_point = point - center
        
        # Project point onto cylinder axis
        projection_length = np.dot(vec_to_point, axis)
        
        # Check if point is within cylinder height
        half_height = (hoop.thickness + safety_margin) / 2.0
        if abs(projection_length) > half_height:
            return False
        
        # Calculate distance from point to axis
        point_on_axis = center + axis * projection_length
        distance_to_axis = np.linalg.norm(point - point_on_axis)
        
        # Check if within cylinder radius
        effective_radius = hoop.radius + safety_margin
        return distance_to_axis < effective_radius
    
    def validate_trajectory_constraints(self, trajectory: Trajectory) -> Tuple[bool, List[str]]:
        """
        Check if trajectory satisfies velocity/acceleration/jerk constraints.
        
        Args:
            trajectory: Trajectory to validate
        
        Returns:
            is_valid: True if all constraints satisfied
            violations: List of constraint violation messages (for debugging)
        """
        violations = []
        sample_rate = 20.0  # Hz - higher sampling for better constraint checking
        dt = 1.0 / sample_rate
        
        max_vel_magnitude = 0.0
        max_acc_magnitude = 0.0
        max_jerk_magnitude = 0.0
        
        for t in np.arange(0.0, trajectory.total_duration + dt, dt):
            t = min(t, trajectory.total_duration)  # Clamp to end
            _, vel, acc, jerk = self.evaluate_trajectory(trajectory, t)
            
            # Check velocity magnitude
            vel_mag = np.linalg.norm(vel)
            max_vel_magnitude = max(max_vel_magnitude, vel_mag)
            if vel_mag > self.max_velocity:
                violations.append(f"Velocity violation at t={t:.2f}s: {vel_mag:.2f} > {self.max_velocity:.2f} m/s")
            
            # Check acceleration magnitude
            acc_mag = np.linalg.norm(acc)
            max_acc_magnitude = max(max_acc_magnitude, acc_mag)
            if acc_mag > self.max_acceleration:
                violations.append(f"Acceleration violation at t={t:.2f}s: {acc_mag:.2f} > {self.max_acceleration:.2f} m/s²")
            
            # Check jerk magnitude
            jerk_mag = np.linalg.norm(jerk)
            max_jerk_magnitude = max(max_jerk_magnitude, jerk_mag)
            if jerk_mag > self.max_jerk:
                violations.append(f"Jerk violation at t={t:.2f}s: {jerk_mag:.2f} > {self.max_jerk:.2f} m/s³")
        
        is_valid = len(violations) == 0
        
        # Add summary if there are violations
        if not is_valid:
            violations.insert(0, f"Trajectory constraint violations detected:")
            violations.append(f"Max velocity: {max_vel_magnitude:.2f} m/s (limit: {self.max_velocity:.2f} m/s)")
            violations.append(f"Max acceleration: {max_acc_magnitude:.2f} m/s² (limit: {self.max_acceleration:.2f} m/s²)")
            violations.append(f"Max jerk: {max_jerk_magnitude:.2f} m/s³ (limit: {self.max_jerk:.2f} m/s³)")
        
        return is_valid, violations