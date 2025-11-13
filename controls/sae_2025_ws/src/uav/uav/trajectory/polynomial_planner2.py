import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class TrajectorySegment:
    """Represents a single polynomial segment between two waypoints."""
    coefficients: np.ndarray  # Polynomial coefficients (3 x n+1) for x, y, z axes
    start_time: float  # Start time of this segment
    duration: float  # Duration of this segment

@dataclass
class Trajectory:
    """Complete trajectory composed of polynomial segments."""
    segments: List[TrajectorySegment]  # List of trajectory segments
    total_duration: float  # Total trajectory duration
    polynomial_order: int  # Order of polynomials used

class PolynomialTrajectoryPlanner:
    def __init__(self, polynomial_order=7, 
                 max_velocity=2.0, max_acceleration=1.0):
        """
        Args:
            polynomial_order: Order of polynomial (7 recommended)
            max_velocity: Maximum allowed velocity (m/s) - used for scaling trajectory
            max_acceleration: Maximum allowed acceleration (m/s²) - used for scaling trajectory
        """
        self.order = polynomial_order
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
    
    def _get_trajectory_duration(self, start_pos, end_pos, start_vel=None):
        """
        Get a duration parameter for polynomial solving.
        This is NOT a target time - it's just a scaling parameter.
        The actual time to complete depends on how fast the vehicle can follow the trajectory.
        
        We pick a duration that ensures the polynomial trajectory won't violate
        velocity/acceleration constraints. Longer duration = slower but smoother.
        
        Args:
            start_pos: Starting position [x, y, z]
            end_pos: Ending position [x, y, z]
            start_vel: Starting velocity [vx, vy, vz] (optional)
        
        Returns:
            Duration parameter (seconds) - just for polynomial solving
        """
        # Calculate 3D distance
        distance = np.linalg.norm(np.array(end_pos) - np.array(start_pos))
        
        if distance < 1e-6:
            return 1.0  # Arbitrary small duration for zero distance
        
        # Use a simple heuristic: duration = distance / (fraction of max velocity)
        # This ensures we have enough time to traverse the distance smoothly
        # We use 60% of max velocity as a conservative estimate
        avg_speed = self.max_velocity * 0.6
        
        # Minimum duration = distance / average speed
        duration = distance / avg_speed
        
        # Ensure minimum duration accounts for acceleration if starting from rest
        if start_vel is None or np.linalg.norm(start_vel) < 0.1:
            # If starting from rest, need at least sqrt(2*distance/accel) to accelerate
            min_duration_from_accel = np.sqrt(2.0 * distance / self.max_acceleration)
            duration = max(duration, min_duration_from_accel)
        
        # Clamp to reasonable bounds (0.5s to 60s)
        duration = max(0.5, min(duration, 60.0))
        
        return duration
    
    def _compute_polynomial_matrix(self, t: float, n: int, derivative: int = 0) -> np.ndarray:
        """
        Compute matrix for evaluating polynomial derivatives at time t.
        
        Args:
            t: Time
            n: Polynomial order
            derivative: Which derivative to compute (0=position, 1=velocity, 2=acceleration)
        
        Returns:
            Row vector [t^0, t^1, t^2, ..., t^n] scaled by derivative factor
        """
        if derivative == 0:
            return np.array([t**i for i in range(n + 1)])
        else:
            result = np.zeros(n + 1)
            for i in range(derivative, n + 1):
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
        
        For order 7 polynomial (8 coefficients), we specify:
        - Position, velocity, acceleration at start (3 constraints)
        - Position, velocity, acceleration at end (3 constraints)
        - Jerk at start = 0, jerk at end = 0 (2 constraints) for smoothness
        Total: 8 constraints for 8 coefficients
        
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
            
            # Constraint 4: Jerk at t=0 = 0 (for smoothness)
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
            
            # Constraint 8: Jerk at t=duration = 0 (for smoothness)
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
                           start_vel=None, end_vel=None,
                           start_acc=None, end_acc=None):
        """
        Generate polynomial trajectory from start to end position.
        
        Args:
            start_pos: Starting position [x, y, z]
            end_pos: Ending position [x, y, z]
            start_vel: Starting velocity [vx, vy, vz] (optional)
            end_vel: Ending velocity [vx, vy, vz] (optional)
            start_acc: Starting acceleration [ax, ay, az] (optional)
            end_acc: Ending acceleration [ax, ay, az] (optional)
        
        Returns:
            Trajectory object with polynomial coefficients
        """
        start_pos = np.array(start_pos)
        end_pos = np.array(end_pos)
        
        if start_vel is None:
            start_vel = np.zeros(3)
        else:
            start_vel = np.array(start_vel)
        
        if end_vel is None:
            end_vel = np.zeros(3)
        else:
            end_vel = np.array(end_vel)
        
        if start_acc is None:
            start_acc = np.zeros(3)
        else:
            start_acc = np.array(start_acc)
        
        if end_acc is None:
            end_acc = np.zeros(3)
        else:
            end_acc = np.array(end_acc)
        
        # Get duration parameter (just for polynomial solving, not a target time)
        duration = self._get_trajectory_duration(start_pos, end_pos, start_vel)
        
        # Solve for single segment
        coefficients = self._solve_polynomial_segment(
            start_pos, end_pos,
            start_vel, end_vel,
            start_acc, end_acc,
            duration
        )
        
        # Create segment
        segment = TrajectorySegment(
            coefficients=coefficients,
            start_time=0.0,
            duration=duration
        )
        
        # Create trajectory (single segment for now, can be extended for multi-segment)
        trajectory = Trajectory(
            segments=[segment],
            total_duration=duration,
            polynomial_order=self.order
        )
        
        return trajectory
    
    def evaluate_trajectory(self, trajectory: Trajectory, time: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Evaluate trajectory at a specific time to get position and velocity.
        
        Args:
            trajectory: Trajectory object
            time: Time since trajectory start (in seconds)
        
        Returns:
            position, velocity at time t (each is [x, y, z])
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
        pos, vel = self._evaluate_segment(segment.coefficients, t_rel)
        
        return pos, vel
    
    def _evaluate_segment(self, coefficients: np.ndarray, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Evaluate position and velocity of a segment at time t.
        
        Args:
            coefficients: Polynomial coefficients (shape: 3 x n+1)
            t: Time relative to segment start
        
        Returns:
            position, velocity (each is [x, y, z])
        """
        n = coefficients.shape[1] - 1
        pos = np.zeros(3)
        vel = np.zeros(3)
        
        for axis in range(3):
            # Position
            pos_vec = self._compute_polynomial_matrix(t, n, 0)
            pos[axis] = np.dot(coefficients[axis], pos_vec)
            
            # Velocity
            vel_vec = self._compute_polynomial_matrix(t, n, 1)
            vel[axis] = np.dot(coefficients[axis], vel_vec)
        
        return pos, vel

