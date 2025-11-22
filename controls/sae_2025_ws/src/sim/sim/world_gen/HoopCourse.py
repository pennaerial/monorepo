import random as r
import xml.etree.ElementTree as ET
from sim.world_gen import WorldNode
from xml.dom import minidom
from pathlib import Path
from abc import ABC, abstractmethod
from typing import List, Tuple
from sim_interfaces.srv import HoopList
from sim.world_gen import WorldNode
import math
from sim_interfaces.msg import HoopPose

Pose = Tuple[float, float, float, float, float, float]

class CourseStyle(ABC):
    """Abstract base class for all course styles."""

    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int
                ):
        self.dlz = dlz
        self.uav = uav
        self.num_hoops = num_hoops

        # max_dist represents max distance we want next step for hoop in our course
        self.max_dist = max_dist

    @abstractmethod
    def generate_course(self) -> List[Pose]:
        """Generate a list of (x, y, z, roll, pitch, yaw) for hoop placement."""
        pass

class AscentCourse(CourseStyle):
    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 start_height: int
                 ):
        super().__init__(dlz, uav, num_hoops, max_dist)
        self.start_height = start_height

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        # Direction vector UAV → DLZ
        dx = x_dlz - x_uav
        dy = y_dlz - y_uav
        dz = z_dlz - z_uav

        # Total distance
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist == 0:
            raise ValueError("DLZ and UAV cannot be the same point.")

        # Normalize to get unit vector
        dir_x = dx / dist
        dir_y = dy / dist
        dir_z = dz / dist

        # Distance between hoops
        segment_len = dist / self.num_hoops

        for i in range(self.num_hoops):
            # Progress along line (slightly randomized)
            along = segment_len * (i + r.uniform(0.2, 0.8))

            # Height transition from UAV’s start_height to DLZ altitude
            frac = (i + 1) / self.num_hoops
            z_interp = self.start_height + frac * (z_dlz - self.start_height)
            z_noise = r.uniform(-0.3, 0.3) * (z_dlz - self.start_height) / self.num_hoops

            new_x = x_uav + dir_x * along
            new_y = y_uav + dir_y * along
            new_z = z_interp + z_noise

            # Each hoop faces along the course direction
            yaw_deg = math.degrees(math.atan2(dy, dx))
            hoops.append((new_x, new_y, new_z, 0, yaw_deg, 0))

        return hoops

class DescentCourse(CourseStyle):
    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 start_height: int
                 ):
        super().__init__(dlz, uav, num_hoops, max_dist)
        self.start_height = start_height

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        # Direction vector UAV → DLZ
        dx = x_dlz - x_uav
        dy = y_dlz - y_uav
        dz = z_dlz - z_uav

        # Total distance
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist == 0:
            raise ValueError("DLZ and UAV cannot be the same point.")

        # Normalize to get unit vector
        dir_x = dx / dist
        dir_y = dy / dist
        dir_z = dz / dist

        # Distance between hoops
        segment_len = dist / self.num_hoops

        for i in range(self.num_hoops):
            # Progress along line (slightly randomized)
            along = segment_len * (i + r.uniform(0.2, 0.8))

            # Height transition from start_height down to DLZ
            frac = (i + 1) / self.num_hoops
            z_interp = self.start_height + frac * (z_dlz - self.start_height)  # dz negative if descending
            z_noise = r.uniform(-0.3, 0.3) * abs(z_dlz - self.start_height) / self.num_hoops

            new_x = x_uav + dir_x * along
            new_y = y_uav + dir_y * along
            new_z = z_interp + z_noise

            # Orientation along course
            yaw_deg = math.degrees(math.atan2(dy, dx))
            hoops.append((new_x, new_y, new_z, 0, yaw_deg, 0))

        return hoops

class SlalomCourse(CourseStyle):
    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 width: int,
                 height: int
                 ):
        super().__init__(dlz, uav, num_hoops, max_dist)
        self.width = width
        self.height = height

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        # Direction vector from UAV → DLZ
        dx = x_dlz - x_uav
        dy = y_dlz - y_uav
        dz = z_dlz - z_uav

        # Distance between UAV and DLZ
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist == 0:
            raise ValueError("DLZ and UAV cannot be at the same position.")

        # Normalize to get unit direction vector
        dir_x = dx / dist
        dir_y = dy / dist
        dir_z = dz / dist

        # Get perpendicular vector in XY plane for slalom offset
        # (-dy, dx) is perpendicular to (dx, dy)
        perp_x = -dir_y
        perp_y = dir_x

        zone_len = dist / self.num_hoops

        for i in range(self.num_hoops):
            alt = 1 if i % 2 == 0 else -1

            # Move forward along direction vector
            along_dist = zone_len * (i + r.uniform(0.2, 0.8))  # slight randomness
            offset = alt * self.width * 0.5 * r.uniform(0.5, 1.0)

            # Compute new hoop position
            new_x = x_uav + dir_x * along_dist + perp_x * offset
            new_y = y_uav + dir_y * along_dist + perp_y * offset
            new_z = r.uniform(self.height * 0.3, self.height * 0.7)

            hoops.append((new_x, new_y, new_z, 0, 90, 0))

        return hoops

class StraightCourse(CourseStyle):
    """
    Simple straight course with hoops in a straight line.
    Perfect for testing basic navigation and scoring.
    """
    
    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 height: float = 2.0,
                 spacing: float = 2.0
                ):
        # Call parent initializer
        super().__init__(dlz, uav, num_hoops, max_dist)
        
        # Height of all hoops (same for straight course)
        self.height = height
        
        # Distance between hoops
        self.spacing = spacing
        
    def generate_course(self) -> List[Pose]:
        """
        Generate a straight line of hoops.
        
        Returns:
            List of (x, y, z, roll, pitch, yaw) positions for each hoop
        """
        hoops = []
        x_dlz, y_dlz, z_dlz = self.dlz
        print(self.uav)
        x_uav, y_uav, z_uav = self.uav
        
        # Calculate total course length
        total_length = (self.num_hoops - 1) * self.spacing
        
        # Start position (slightly offset from UAV start)
        start_x = x_uav + 1.0  # 1 meter forward from UAV
        start_y = y_uav
        
        # Generate hoops in a straight line
        for i in range(self.num_hoops):
            # Calculate position
            hoop_x = start_x + (i * self.spacing)
            hoop_y = start_y
            hoop_z = self.height
            
            # Add small random variation (±0.2m) for realism
            variation_x = 0
            variation_y = 0
            variation_z = 0
            
            # Final position with variation
            final_x = hoop_x + variation_x
            final_y = hoop_y + variation_y
            final_z = hoop_z + variation_z
            
            # All hoops face forward (yaw = 0)
            hoop_pose = (final_x, final_y, final_z, 0, 90, 0)
            hoops.append(hoop_pose)
            
        return hoops

class BezierCourse(CourseStyle):
    """
    Smooth cubic Bezier curve between UAV start and DLZ.
    P0 = UAV, P3 = DLZ
    P1/P2 are along the line with randomized lateral offsets to create a more interesting arc.
    """

    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 height: float = 4.0,
                 lateral_offset: float = 4.0):
        super().__init__(dlz, uav, num_hoops, max_dist)
        self.height = height                # used as vertical bump amplitude
        self.lateral_offset = lateral_offset

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        # Direction UAV -> DLZ in XY
        dx = x_dlz - x_uav
        dy = y_dlz - y_uav
        dz = z_dlz - z_uav

        dist_xy = math.hypot(dx, dy)
        dist_3d = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist_xy == 0 or dist_3d == 0:
            raise ValueError("DLZ and UAV cannot be the same point.")

        dir_x = dx / dist_xy
        dir_y = dy / dist_xy

        # Perpendicular vector in XY plane
        perp_x = -dir_y
        perp_y = dir_x

        # Bezier control points in XY
        P0x, P0y = x_uav, y_uav
        P3x, P3y = x_dlz, y_dlz

        # --- Make control points more "interesting" ---
        # Randomize how far along the line P1 and P2 are
        one_third = dist_xy * r.uniform(0.20, 0.40)
        two_third = dist_xy * r.uniform(0.60, 0.85)

        # Base positions on the line
        base1_x = P0x + dir_x * one_third
        base1_y = P0y + dir_y * one_third
        base2_x = P0x + dir_x * two_third
        base2_y = P0y + dir_y * two_third

        # Random lateral offsets (asymmetric, so the arc isn't perfectly mirrored)
        lat1 = self.lateral_offset * r.uniform(0.6, 1.3)
        lat2 = self.lateral_offset * r.uniform(-1.3, -0.6)  # opposite side

        P1x = base1_x + perp_x * lat1
        P1y = base1_y + perp_y * lat1

        P2x = base2_x + perp_x * lat2
        P2y = base2_y + perp_y * lat2

        # Helper: cubic Bezier in 2D
        def bezier_xy(t: float) -> Tuple[float, float]:
            mt = 1.0 - t
            mt2 = mt * mt
            t2 = t * t
            x = (mt2 * mt) * P0x + 3 * (mt2) * t * P1x + 3 * mt * t2 * P2x + (t2 * t) * P3x
            y = (mt2 * mt) * P0y + 3 * (mt2) * t * P1y + 3 * mt * t2 * P2y + (t2 * t) * P3y
            return x, y

        # Helper: derivative for tangent direction
        def bezier_xy_deriv(t: float) -> Tuple[float, float]:
            mt = 1.0 - t
            x = 3 * mt * mt * (P1x - P0x) + 6 * mt * t * (P2x - P1x) + 3 * t * t * (P3x - P2x)
            y = 3 * mt * mt * (P1y - P0y) + 6 * mt * t * (P2y - P1y) + 3 * t * t * (P3y - P2y)
            return x, y

        # --- Place hoops along t in (0, 1) ---
        for i in range(self.num_hoops):
            # keep hoops off the exact endpoints
            t = (i + 1) / (self.num_hoops + 1)

            x, y = bezier_xy(t)

            # Base linear z from UAV -> DLZ
            z_linear = z_uav + t * (z_dlz - z_uav)

            # Vertical "arch" plus small randomness
            # sin(pi * t) gives a single hump peaking mid-course
            z_bump = self.height * math.sin(math.pi * t*4)
            z_noise = r.uniform(-0.2, 0.2)

            z = z_linear + z_bump + z_noise
            z = max(0.5, z)  # keep above ground a bit

            # Orientation from tangent of curve
            dx_dt, dy_dt = bezier_xy_deriv(t)
            if dx_dt == 0 and dy_dt == 0:
                yaw_deg = 0.0
            else:
                yaw = math.atan2(dy_dt, dx_dt)
                yaw_deg = math.degrees(yaw)

            hoops.append((x, y, z, 0.0, yaw_deg, 0.0))

        return hoops


class SplineCourse(CourseStyle):
    """
    Smooth Catmull–Rom spline course between UAV and DLZ.
    XY follows a spline through offset control points.
    Z follows a strong vertical wave for more interesting climbs/descents.
    """

    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 num_ctrl_points: int = 5,
                 lateral_spread: float = 4.0,
                 vertical_spread: float = 1.5,
                 vertical_wave_amplitude: float = 3.0,
                 vertical_waves: int = 2,
                 min_height: float = 0.75):
        super().__init__(dlz, uav, num_hoops, max_dist)
        self.num_ctrl_points = max(4, num_ctrl_points)  # need at least 4
        self.lateral_spread = lateral_spread
        self.vertical_spread = vertical_spread
        self.vertical_wave_amplitude = vertical_wave_amplitude
        self.vertical_waves = max(1, vertical_waves)
        self.min_height = min_height

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        dx = x_dlz - x_uav
        dy = y_dlz - y_uav
        dz = z_dlz - z_uav

        dist_xy = math.hypot(dx, dy)
        dist_3d = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist_xy == 0 or dist_3d == 0:
            raise ValueError("DLZ and UAV cannot be the same point.")

        # Forward and perpendicular directions in XY
        dir_x = dx / dist_xy
        dir_y = dy / dist_xy
        perp_x = -dir_y
        perp_y = dir_x

        # ---- Build control points (start + interior + end) in XY/Z ----
        ctrl: List[Tuple[float, float, float]] = []
        ctrl.append((x_uav, y_uav, z_uav))

        num_interior = self.num_ctrl_points - 2
        for k in range(1, num_interior + 1):
            t = k / (num_interior + 1)

            # Base point on straight line
            base_x = x_uav + t * dx
            base_y = y_uav + t * dy
            base_z = z_uav + t * dz

            # Alternate sides for lateral wiggle
            side = 1 if k % 2 == 1 else -1
            lateral = side * self.lateral_spread * r.uniform(0.5, 1.0)
            vertical_noise = r.uniform(-self.vertical_spread, self.vertical_spread)

            wx = base_x + perp_x * lateral
            wy = base_y + perp_y * lateral
            wz = max(self.min_height, base_z + vertical_noise)

            ctrl.append((wx, wy, wz))

        ctrl.append((x_dlz, y_dlz, z_dlz))

        n = len(ctrl)
        if n < 4:
            # fallback: straight line if something weird happens
            return StraightCourse(self.dlz, self.uav, self.num_hoops, self.max_dist).generate_course()

        # ---- Chord lengths for approx arc-length param (for XY spline) ----
        seg_lengths = []
        cum_lengths = [0.0]
        total_len = 0.0
        for i in range(n - 1):
            x0, y0, z0 = ctrl[i]
            x1, y1, z1 = ctrl[i + 1]
            L = math.sqrt((x1 - x0)**2 + (y1 - y0)**2 + (z1 - z0)**2)
            seg_lengths.append(L)
            total_len += L
            cum_lengths.append(total_len)

        # Extended control points for Catmull–Rom
        P_full = [ctrl[0]] + ctrl + [ctrl[-1]]  # indices 0..n+1

        def catmull_rom_point(i_seg: int, u: float) -> Tuple[float, float, float]:
            """
            Evaluate Catmull–Rom for segment between ctrl[i_seg] and ctrl[i_seg+1],
            with local parameter u in [0, 1].
            """
            base = i_seg + 1  # shift because of leading duplicate
            P0 = P_full[base - 1]
            P1 = P_full[base]
            P2 = P_full[base + 1]
            P3 = P_full[base + 2]

            x0, y0, z0 = P0
            x1, y1, z1 = P1
            x2, y2, z2 = P2
            x3, y3, z3 = P3

            u2 = u * u
            u3 = u2 * u

            def blend(p0, p1, p2, p3):
                # Standard Catmull–Rom, tension = 0.5
                return 0.5 * (
                    (2.0 * p1) +
                    (-p0 + p2) * u +
                    (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u2 +
                    (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u3
                )

            x = blend(x0, x1, x2, x3)
            y = blend(y0, y1, y2, y3)
            z = blend(z0, z1, z2, z3)
            return x, y, z

        def catmull_rom_tangent_xy(i_seg: int, u: float) -> Tuple[float, float]:
            """
            Derivative of Catmull–Rom in XY only, for yaw.
            """
            base = i_seg + 1
            P0 = P_full[base - 1]
            P1 = P_full[base]
            P2 = P_full[base + 1]
            P3 = P_full[base + 2]

            x0, y0, _ = P0
            x1, y1, _ = P1
            x2, y2, _ = P2
            x3, y3, _ = P3

            u2 = u * u

            def blend_deriv(p0, p1, p2, p3):
                return 0.5 * (
                    (-p0 + p2) +
                    2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u +
                    3.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u2
                )

            dx_du = blend_deriv(x0, x1, x2, x3)
            dy_du = blend_deriv(y0, y1, y2, y3)
            return dx_du, dy_du

        # ---- Place hoops along the path ----
        for j in range(self.num_hoops):
            # Global normalized param along path, for vertical wave
            t_global = (j + 1) / (self.num_hoops + 1)

            # Target arc length for this hoop
            target_len = t_global * total_len

            # Find segment in which this arc length lies
            seg_idx = 0
            for i in range(len(seg_lengths)):
                if cum_lengths[i + 1] >= target_len:
                    seg_idx = i
                    break

            seg_len = seg_lengths[seg_idx] if seg_lengths[seg_idx] > 1e-6 else 1e-6
            seg_start = cum_lengths[seg_idx]
            u = (target_len - seg_start) / seg_len
            u = max(0.0, min(1.0, u))

            # XY from spline
            x, y, z_spline = catmull_rom_point(seg_idx, u)

            # Base linear z from UAV -> DLZ
            z_base = z_uav + t_global * dz

            # Strong vertical wave on top of base z
            angle = 2.0 * math.pi * self.vertical_waves * t_global
            z_wave = self.vertical_wave_amplitude * math.sin(angle)

            # Small additional noise to break perfect symmetry
            z_noise = r.uniform(-self.vertical_spread, self.vertical_spread)

            z = z_base + z_wave + z_noise
            z = max(self.min_height, z)

            # Orientation from spline tangent in XY
            dx_du, dy_du = catmull_rom_tangent_xy(seg_idx, u)
            if dx_du == 0.0 and dy_du == 0.0:
                yaw_deg = 0.0
            else:
                yaw_deg = math.degrees(math.atan2(dy_du, dx_du))

            # (roll, pitch, yaw): you’ve been using yaw in the middle slot
            hoops.append((x, y, z, 0.0, yaw_deg, 0.0))

        return hoops
class RollerCoasterCourse(CourseStyle):
    """
    3D 'roller coaster' course:
    - Follows the straight line from UAV to DLZ
    - Adds smooth lateral + vertical waves in a local Frenet-like frame
    """

    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 lateral_amp: float = 4.0,
                 vertical_amp: float = 3.0,
                 lateral_waves: int = 2,
                 vertical_waves: int = 3,
                 min_height: float = 0.75):
        super().__init__(dlz, uav, num_hoops, max_dist)
        self.lateral_amp = lateral_amp
        self.vertical_amp = vertical_amp
        self.lateral_waves = max(0, lateral_waves)
        self.vertical_waves = max(1, vertical_waves)
        self.min_height = min_height

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []

        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        Dx = x_dlz - x_uav
        Dy = y_dlz - y_uav
        Dz = z_dlz - z_uav

        dist = math.sqrt(Dx*Dx + Dy*Dy + Dz*Dz)
        if dist == 0.0:
            raise ValueError("DLZ and UAV cannot be the same point.")

        # Tangent
        Tx = Dx / dist
        Ty = Dy / dist
        Tz = Dz / dist

        # Choose an up vector not parallel to T
        upx, upy, upz = 0.0, 0.0, 1.0
        dot_up_T = Tx*upx + Ty*upy + Tz*upz
        if abs(dot_up_T) > 0.9:  # nearly parallel
            upx, upy, upz = 0.0, 1.0, 0.0

        # N = normalize(up × T)
        Nx = upy*Tz - upz*Ty
        Ny = upz*Tx - upx*Tz
        Nz = upx*Ty - upy*Tx
        n_norm = math.sqrt(Nx*Nx + Ny*Ny + Nz*Nz)
        if n_norm < 1e-6:
            # fallback: arbitrary perpendicular in XY
            Nx, Ny, Nz = -Ty, Tx, 0.0
            n_norm = math.sqrt(Nx*Nx + Ny*Ny + Nz*Nz)
        Nx /= n_norm
        Ny /= n_norm
        Nz /= n_norm

        # B = T × N
        Bx = Ty*Nz - Tz*Ny
        By = Tz*Nx - Tx*Nz
        Bz = Tx*Ny - Ty*Nx

        # Helper to compute position along track
        def pos(s: float) -> Tuple[float, float, float]:
            # Base along the line
            base_x = x_uav + s * Dx
            base_y = y_uav + s * Dy
            base_z = z_uav + s * Dz

            # Smooth lateral + vertical offsets
            angle_lat = 2.0 * math.pi * self.lateral_waves * s if self.lateral_waves > 0 else 0.0
            angle_vert = 2.0 * math.pi * self.vertical_waves * s

            lat = self.lateral_amp * math.sin(angle_lat) if self.lateral_waves > 0 else 0.0
            vert = self.vertical_amp * math.sin(angle_vert)

            # Compose in local frame
            x = base_x + lat * Nx + vert * Bx
            y = base_y + lat * Ny + vert * By
            z = base_z + lat * Nz + vert * Bz

            # Keep above ground
            if z < self.min_height:
                z = self.min_height

            return x, y, z

        # Finite-diff derivative for yaw
        def tangent_xy(s: float) -> Tuple[float, float]:
            eps = 1e-3
            s1 = max(0.0, min(1.0, s - eps))
            s2 = max(0.0, min(1.0, s + eps))
            x1, y1, _ = pos(s1)
            x2, y2, _ = pos(s2)
            return x2 - x1, y2 - y1

        for i in range(self.num_hoops):
            s = (i + 1) / (self.num_hoops + 1)  # (0,1)

            x, y, z = pos(s)
            vx, vy = tangent_xy(s)

            if vx == 0.0 and vy == 0.0:
                yaw_deg = 0.0
            else:
                yaw_deg = math.degrees(math.atan2(vy, vx))

            # (roll, pitch, yaw): you've been using yaw in the middle slot
            hoops.append((x, y, z, 0.0, yaw_deg, 0.0))

        return hoops



class HoopCourseNode(WorldNode):
    """
    ROS node for generating hoop course for in house comp.
    """

    def __init__(self, course: str, dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 world_name: str, output_file: str):
        super().__init__()
        self.course = course
        self.dlz = dlz
        self.uav = uav
        self.num_hoops = num_hoops
        self.max_dist = max_dist
        self.world_name = world_name
        self.output_file = output_file
        print("WORLD GENERATION NODE")
        self.generate_world()
        self.srv = self.create_service(HoopList, "list_hoops", self.hoop_list_req)
    
    def hoop_list_req(self, request, response):
        response.hoop_positions = []
        for (x, y, z, roll, pitch, yaw) in self.hoop_positions:
            hp = HoopPose()
            hp.x = float(x)
            hp.y = float(y)
            hp.z = float(z)
            hp.roll = float(roll)
            hp.pitch = float(pitch)
            hp.yaw = float(yaw)
            response.hoop_positions.append(hp)
        return response


    def add_hoops(self, input_file, output_file, hoop_positions):
        # Expand ~, make absolute, and validate paths
        in_path = Path(input_file).expanduser().resolve()
        out_path = Path(output_file).expanduser().resolve()
        print(in_path)
        print(out_path)
        if not in_path.exists():
            raise FileNotFoundError(
                f"Input SDF not found: {in_path}\nCWD: {Path.cwd().resolve()}"
            )
        out_path.parent.mkdir(parents=True, exist_ok=True)

        # Parse
        tree = ET.parse(str(in_path))  # str() for safety on older libs
        root = tree.getroot()

        # handle namespace if present (root.tag may be like "{...}sdf")
        ns = ""
        if root.tag.startswith("{"):
            ns = root.tag.split("}")[0] + "}"

        # try to find <world> with/without namespace
        world = root.find(f"{ns}world") or root.find("world") or root.find(".//world")
        if world is None:
            raise RuntimeError("No <world> tag found in the SDF!")

        # Add new hoops with given positions
        for i, pos in enumerate(hoop_positions, start=1):
            inc = ET.Element("include")
            x, y, z, roll, pitch, yaw = pos
            z = 1 if z < 1 else z
            ET.SubElement(inc, "uri").text = "model://hoop"
            ET.SubElement(inc, "pose").text = f"{x} {y} {z} {roll} {pitch} {yaw}"
            ET.SubElement(inc, "name").text = f"hoop_{i}"
            world.append(inc)
        
       
        # Use last two hoops to define the approach direction
        if len(hoop_positions) >= 2:
            (prev_x, prev_y, _prev_z, *_), (end_x, end_y, end_z, *_) = hoop_positions[-2], hoop_positions[-1]
        else:
            # Fallback: only one hoop; use UAV->hoop direction
            start_x, start_y, _ = self.uav
            end_x, end_y, end_z, *_, = hoop_positions[-1]
            prev_x, prev_y = start_x, start_y

        # Direction vector in XY from previous hoop to final hoop
        dir_x = end_x - prev_x
        dir_y = end_y - prev_y
        length = math.hypot(dir_x, dir_y)
        if length == 0:
            # Degenerate case: fall back to +X
            dir_x, dir_y = 1.0, 0.0
        else:
            dir_x /= length
            dir_y /= length

        # Perpendicular vector (rotate by +90 degrees)
        perp_x = -dir_y
        perp_y = dir_x

        # How far apart to space the DLZs (meters)
        spacing = 3.0
        num_dlz = 3

        # Center DLZs around the final hoop's XY position
        center_x = end_x
        center_y = end_y

        # DLZs on the ground
        dlz_z = 0.0

        # Yaw aligned with the approach direction
        yaw = math.atan2(dir_y, dir_x)
        roll = 0.0
        pitch = 0.0
# Colors for 3 DLZs
        colors = [
            ("red",   "1 0 0 1"),
            ("green", "0 1 0 1"),
            ("blue",  "0 0 1 1"),
        ]

        for i in range(num_dlz):
            offset = (i - (num_dlz - 1) / 2.0) * spacing  # -s, 0, +s
            dlz_x = center_x + offset * perp_x
            dlz_y = center_y + offset * perp_y

            color_name, rgba = colors[i]

            dlz_inc = ET.Element("include")
            ET.SubElement(dlz_inc, "uri").text = "model://dlz_" + color_name
            ET.SubElement(dlz_inc, "name").text = f"dlz_{i+1}"
            ET.SubElement(dlz_inc, "pose").text = f"{dlz_x} {dlz_y} {dlz_z} {roll} {pitch} {yaw}"

            # --- Material override ---
            # This forces all visuals of the included model to use the chosen color
            model_override = ET.SubElement(dlz_inc, "model")
            link_override = ET.SubElement(model_override, "link", {"name": "link"})  # assumes main link is "link"
            visual_override = ET.SubElement(link_override, "visual", {"name": "visual"})
            material = ET.SubElement(visual_override, "material")
            # ambient = ET.SubElement(material, "ambient")
            diffuse = ET.SubElement(material, "diffuse")

            # ambient.text = rgba
            diffuse.text = rgba

            world.append(dlz_inc)


        # --- remove whitespace-only text/tail nodes to avoid minidom producing extra blank lines ---
        def strip_whitespace(elem):
            if elem.text is not None and elem.text.strip() == "":
                elem.text = None
            for child in list(elem):
                strip_whitespace(child)
                if child.tail is not None and child.tail.strip() == "":
                    child.tail = None
        strip_whitespace(root)

        # Pretty print and write
        rough_bytes = ET.tostring(root, encoding="utf-8")
        pretty = minidom.parseString(rough_bytes.decode("utf-8")).toprettyxml(indent="  ")
        lines = [ln for ln in pretty.splitlines() if ln.strip() != ""]
        out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def generate_world(self):
        courses = ['ascent', 'descent', 'slalom', 'bezier', 'spline', 'coaster']

        if self.course.lower() == 'random':
            course_id = r.uniform(0, len(courses) - 1)
            self.course = courses[course_id]
            
        if self.course.lower() == "previous":
            return

        elif self.course.lower() == "ascent":
            course = AscentCourse(dlz=self.dlz, 
                                uav=self.uav, 
                                num_hoops=self.num_hoops, 
                                max_dist=self.max_dist, 
                                start_height=2)
            hoop_poses = course.generate_course()
        elif self.course.lower() == "descent":
            course = DescentCourse(dlz=self.dlz, 
                                uav=self.uav, 
                                num_hoops=self.num_hoops, 
                                max_dist=self.max_dist, 
                                start_height=4)
            hoop_poses = course.generate_course()
        elif self.course.lower() == "slalom":
            course = SlalomCourse(dlz=self.dlz, 
                                uav=self.uav, 
                                num_hoops=self.num_hoops, 
                                max_dist=self.max_dist,
                                width=4,
                                height=2)
            hoop_poses = course.generate_course()
        elif self.course.lower() == "straight":
            course = StraightCourse(dlz=self.dlz, 
                                uav=self.uav, 
                                num_hoops=self.num_hoops, 
                                max_dist=self.max_dist,
                                height=2,
                                spacing=2)
            hoop_poses = course.generate_course()
        elif self.course.lower() == "bezier":
            course = BezierCourse(dlz=self.dlz,
                                  uav=self.uav,
                                  num_hoops=self.num_hoops,
                                  max_dist=self.max_dist,
                                  height=2.0,
                                  lateral_offset=4.0)
            hoop_poses = course.generate_course()
        elif self.course.lower() == "spline":
            course = SplineCourse(
                dlz=self.dlz,
                uav=self.uav,
                num_hoops=self.num_hoops,
                max_dist=self.max_dist,
                num_ctrl_points=50,
                lateral_spread=4.0,
                vertical_spread=0.5,          # just small noise
                vertical_wave_amplitude=4.0,  # bigger = more up/down
                vertical_waves=3,             # more waves along path
                min_height=0.75,
            )
            hoop_poses = course.generate_course()
        elif self.course.lower() == "coaster":
            course = RollerCoasterCourse(
                dlz=self.dlz,
                uav=self.uav,
                num_hoops=self.num_hoops,
                max_dist=self.max_dist,
                lateral_amp=4.0,
                vertical_amp=3.0,
                lateral_waves=2,
                vertical_waves=3,
                min_height=0.75,
            )
            hoop_poses = course.generate_course()
        self.hoop_positions = hoop_poses
        self.add_hoops(input_file=self.world_name, output_file=self.output_file, hoop_positions=hoop_poses)


