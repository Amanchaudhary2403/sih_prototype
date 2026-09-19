"""
perception_map.py — Multi-Layer Occupancy Grid & Costmap
==========================================================

Builds a detailed 2-D semantic occupancy costmap from scenario state:
  • Raw obstacle footprints (Lethal cost = 1.0)
  • Morphological safety inflation zone (Buffer cost = 0.55)
  • Road boundaries & off-road shoulders (Impassable cost = 0.85)
  • Lane markings & drivable asphalt (Traversable cost = 0.05)

Provides metric world coordinates and rich layers for both planner collision
checking and high-definition visualization for hackathon presentation.

Smart India Hackathon — Adaptive Path Planning POC
"""

import numpy as np
from scipy.ndimage import binary_dilation
from dataclasses import dataclass
from typing import Tuple, List


@dataclass
class OccupancyGrid:
    """
    A multi-layer 2-D occupancy grid aligned with the world coordinate frame.

    Attributes
    ----------
    grid           : np.ndarray, shape (H, W), dtype bool
                     Collision map (True = impassable, False = free).
    raw_obs_grid   : np.ndarray, shape (H, W), dtype bool
                     Exact physical footprints of obstacles (no inflation).
    inflation_grid : np.ndarray, shape (H, W), dtype bool
                     Safety buffer zone around obstacles.
    boundary_grid  : np.ndarray, shape (H, W), dtype bool
                     Off-road / ditch boundaries.
    cost_map       : np.ndarray, shape (H, W), dtype float32
                     Normalized cost values [0.0 = clear asphalt, 1.0 = lethal obstacle].
    resolution     : int
                     Cells per metre (e.g. 2 cells/m = 0.5m grid).
    origin         : np.ndarray, shape (2,)
                     World-frame [x, y] of grid's bottom-left corner.
    width_m        : float
                     Grid extent along x (metres).
    height_m       : float
                     Grid extent along y (metres).
    """
    grid: np.ndarray
    raw_obs_grid: np.ndarray
    inflation_grid: np.ndarray
    boundary_grid: np.ndarray
    cost_map: np.ndarray
    resolution: int
    origin: np.ndarray
    width_m: float
    height_m: float

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world (x, y) in metres -> grid indices (row, col)."""
        col = int(round((x - self.origin[0]) * self.resolution))
        row = int(round((y - self.origin[1]) * self.resolution))
        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Convert grid indices (row, col) -> world (x, y) in metres."""
        x = col / self.resolution + self.origin[0]
        y = row / self.resolution + self.origin[1]
        return x, y

    def is_occupied(self, x: float, y: float) -> bool:
        """Check if a world-frame coordinate is occupied or off-road."""
        row, col = self.world_to_grid(x, y)
        if 0 <= row < self.grid.shape[0] and 0 <= col < self.grid.shape[1]:
            return bool(self.grid[row, col])
        return True  # Treat out-of-bounds as impassable

    def get_rgb_costmap(self) -> np.ndarray:
        """
        Generate a vivid, presentation-quality RGB image of the costmap:
          • Drivable road surface  : Deep slate/blue (#15202b)
          • Center line divider    : Soft gold tint (#403d2b)
          • Off-road shoulder      : Dark earthy charcoal (#2b1f1f)
          • Safety inflation zone  : Glowing amber/orange (#d97706)
          • Hard obstacle core     : Bright warning red (#ef4444)
        """
        H, W = self.grid.shape
        rgb = np.zeros((H, W, 3), dtype=np.float32)

        # 1. Base: Drivable road surface (deep navy/slate)
        rgb[:, :] = [0.08, 0.13, 0.18]

        # 2. Road boundaries / shoulders (dark earthy)
        rgb[self.boundary_grid] = [0.18, 0.14, 0.14]

        # 3. Inflation buffer zone (amber / orange)
        rgb[self.inflation_grid] = [0.85, 0.48, 0.05]

        # 4. Raw hard obstacles (crimson red)
        rgb[self.raw_obs_grid] = [0.95, 0.20, 0.20]

        return rgb


def build_occupancy_grid(
    scenario,
    width_m: float = 120.0,
    height_m: float = 20.0,
    resolution: int = 2,
    inflate_radius: float = 1.8,
) -> OccupancyGrid:
    """
    Build a multi-layer occupancy grid and continuous costmap from scenario state.

    Parameters
    ----------
    scenario       : Scenario object (from scenario_builder.py)
    width_m        : Longitudinal grid span in metres (default 120m).
    height_m       : Lateral grid span in metres (default 20m).
    resolution     : Cells per metre (default 2 -> 0.5m cell size).
    inflate_radius : Safety buffer around obstacles in metres (default 1.0m).

    Returns
    -------
    OccupancyGrid with collision, inflation, boundary, and costmap layers.
    """
    road = scenario.road
    road_half = road.half_width

    origin = np.array([-10.0, -height_m / 2.0])
    n_cols = int(width_m * resolution)
    n_rows = int(height_m * resolution)

    boundary_grid = np.zeros((n_rows, n_cols), dtype=bool)
    raw_obs_grid = np.zeros((n_rows, n_cols), dtype=bool)
    cost_map = np.full((n_rows, n_cols), 0.05, dtype=np.float32)  # Base drivable cost

    # ── 1. Road Boundaries & Shoulder ─────────────────────────────────────────
    for row in range(n_rows):
        y_world = row / resolution + origin[1]
        if abs(y_world) > road_half:
            boundary_grid[row, :] = True
            cost_map[row, :] = 0.85  # Non-drivable shoulder cost
        elif abs(y_world) < 0.25:
            # Faint cost ridge along road centerline to favor lane discipline
            cost_map[row, :] = 0.15

    # ── 2. Raw Obstacle Footprints ────────────────────────────────────────────
    for obs in scenario.obstacles:
        obs_x, obs_y = obs.position
        half_l = obs.length / 2.0
        half_w = obs.width / 2.0

        col_min = max(0, int((obs_x - half_l - origin[0]) * resolution))
        col_max = min(n_cols - 1, int((obs_x + half_l - origin[0]) * resolution))
        row_min = max(0, int((obs_y - half_w - origin[1]) * resolution))
        row_max = min(n_rows - 1, int((obs_y + half_w - origin[1]) * resolution))

        raw_obs_grid[row_min:row_max + 1, col_min:col_max + 1] = True

    # ── 3. Morphological Safety Inflation ─────────────────────────────────────
    inflate_cells = int(round(inflate_radius * resolution))
    if inflate_cells > 0:
        diam = 2 * inflate_cells + 1
        yy, xx = np.ogrid[-inflate_cells:inflate_cells + 1,
                          -inflate_cells:inflate_cells + 1]
        kernel = (xx**2 + yy**2) <= inflate_cells**2
        dilated_obs = binary_dilation(raw_obs_grid, structure=kernel).astype(bool)
        inflation_grid = dilated_obs & (~raw_obs_grid) & (~boundary_grid)
    else:
        dilated_obs = raw_obs_grid.copy()
        inflation_grid = np.zeros_like(raw_obs_grid)

    # ── 4. Costmap Normalization ──────────────────────────────────────────────
    cost_map[inflation_grid] = 0.55  # Warning buffer cost
    cost_map[raw_obs_grid] = 1.00    # Lethal obstacle cost

    # Total impassable grid for kinematic validator: road edge + dilated obstacles
    total_grid = boundary_grid | dilated_obs

    occ = OccupancyGrid(
        grid=total_grid,
        raw_obs_grid=raw_obs_grid,
        inflation_grid=inflation_grid,
        boundary_grid=boundary_grid,
        cost_map=cost_map,
        resolution=resolution,
        origin=origin,
        width_m=width_m,
        height_m=height_m,
    )

    return occ


if __name__ == "__main__":
    from scenario_builder import build_scenario
    scenario = build_scenario(1)
    occ = build_occupancy_grid(scenario)
    print(f"OccupancyGrid generated: {occ.grid.shape[1]}x{occ.grid.shape[0]} cells")
    print(f"Raw obstacle cells : {occ.raw_obs_grid.sum()}")
    print(f"Inflation buffer   : {occ.inflation_grid.sum()}")
    print(f"Boundary cells     : {occ.boundary_grid.sum()}")
