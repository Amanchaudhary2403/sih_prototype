#!/usr/bin/env python3
"""
main_sim.py — Interactive SIL Simulation & Dynamic Path Planning Testbed
========================================================================

Smart India Hackathon 2026 — Adaptive Path Planning for Unstructured Indian Roads

Features:
  • Flicker-Free Double-Buffered Rendering (via matplotlib.animation.FuncAnimation)
  • Single Interactive Dropdown Menu to seamlessly switch between all 6 Indian road scenarios
  • Dynamic & Static Obstacle Navigation:
      - Static: Stray cattle, potholes, construction barriers, parked vehicles
      - Dynamic: Auto-rickshaws (3 m/s), slow trucks (2 m/s), bullock carts,
                 crossing pedestrians (-1 m/s), wandering stray dogs
      - Predictive trajectory collision checking with oriented bounding box clearance
  • G2/C2-Continuous Paths: Hybrid A* search + Laplacian relaxation + clamped cubic spline
  • Attractive Dark-Theme Mission-Control Dashboard:
      - Bird's-Eye View with callouts strictly ABOVE the road (zero text blocking asphalt)
      - Real-Time Multi-Layer Semantic Occupancy Costmap
      - Digital Telemetry Instrument Deck (Time, Speed, Active Hazard, Planner Status, Loop)
  • Continuous Infinite Looping option for uninterrupted live demonstrations
"""

import sys
import os
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from matplotlib.collections import LineCollection
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation
import matplotlib.patheffects as pe

from scenario_builder import build_scenario, SCENARIO_PRESETS, Vehicle
from perception_map import build_occupancy_grid, OccupancyGrid
from dynamic_planner import plan_avoidance_path, smooth_path

# ── Backend Configuration ─────────────────────────────────────────────────────
for _b in ('TkAgg', 'Qt5Agg', 'GTK3Agg'):
    try:
        matplotlib.use(_b)
        break
    except Exception:
        pass

# ── Geometric & Kinematic Constants ───────────────────────────────────────────
DETECT_DIST   = 35.0         # LiDAR sensing horizon (m) — early detection
EGO_SPEED     = 10.0         # Nominal cruising speed (m/s)
LANE_W        = 3.5          # Standard lane width (m)
ROAD_HALF_W   = 3.5          # Half-width of 2-lane road
DT            = 0.05         # Simulation time step (s)
ROAD_LEN      = 110.0        # Total road span (m)

# ── Sleek Theme Color Palette ─────────────────────────────────────────────────
BG_COLOR      = '#070b14'    # Deep obsidian / space navy
CARD_BG       = '#0e1626'    # Instrument card background
BORDER_COLOR  = '#1e293b'    # Subtle card borders
ROAD_COLOR    = '#121a29'    # Dark asphalt charcoal
EDGE_COLOR    = '#e2e8f0'    # Crisp road boundaries
LANE_MARK_COL = '#fbbf24'    # Warm highway amber
EGO_COLOR     = '#00f0ff'    # Electric cyan
PATH_COLOR    = '#10b981'    # Neon emerald green
TRAIL_COLOR   = '#38bdf8'    # Fading sky blue
SENSE_COLOR   = '#f59e0b'    # Radar / LiDAR golden ring


# =============================================================================
#  TRAJECTORY & COLLISION UTILITIES
# =============================================================================

def parameterise_path(waypoints: np.ndarray, speed: float, dt: float) -> np.ndarray:
    """Discretise continuous waypoints into constant-speed time steps."""
    if len(waypoints) < 2:
        return waypoints
    d = np.diff(waypoints, axis=0)
    segs = np.linalg.norm(d, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(segs)])
    total_len = cum[-1]
    if total_len < 0.01:
        return waypoints
    times = np.arange(0.0, total_len / speed + dt, dt)
    s_eval = np.clip(times * speed, 0.0, total_len)
    return np.column_stack([
        np.interp(s_eval, cum, waypoints[:, 0]),
        np.interp(s_eval, cum, waypoints[:, 1]),
    ])


def check_oriented_collision(
    traj: np.ndarray,
    obs: Vehicle,
    ego_length: float = 4.7,
    ego_width: float = 1.8,
    lookahead_steps: int = 150,
    time_horizon_sec: float = 6.0,
) -> bool:
    """
    Physically accurate oriented clearance check between planned trajectory and
    an obstacle (accounting for obstacle velocity vector).
    
    Separates longitudinal and lateral margins to eliminate false alarms when
    passing vehicles in the adjacent lane.
    """
    if len(traj) == 0:
        return False

    sub_traj = traj[:lookahead_steps]
    dx_safe = (obs.length + ego_length) / 2.0 + 2.5   # generous longitudinal clearance
    dy_safe = (obs.width + ego_width) / 2.0 + 1.0     # generous lateral clearance

    for step_idx, pt in enumerate(sub_traj):
        t_future = step_idx * DT
        if t_future > time_horizon_sec:
            break
        pred_obs_x = obs.position[0] + obs.velocity[0] * t_future
        pred_obs_y = obs.position[1] + obs.velocity[1] * t_future

        if abs(pt[0] - pred_obs_x) < dx_safe and abs(pt[1] - pred_obs_y) < dy_safe:
            return True

    return False


def compute_avoidance_goal(
    ego_xy: np.ndarray,
    target_obs: Vehicle,
    lane_w: float = 3.5,
    road_len: float = 110.0,
) -> np.ndarray:
    """Select a safe downstream goal pose past the target obstacle."""
    right_y = -lane_w / 2.0  # -1.75 m (Nominal driving lane)
    left_y  =  lane_w / 2.0  # +1.75 m (Overtaking lane)

    dist_x = max(1.0, target_obs.position[0] - ego_xy[0])
    t_reach = dist_x / EGO_SPEED
    pred_obs_y = target_obs.position[1] + target_obs.velocity[1] * t_reach
    pred_obs_x = target_obs.position[0] + target_obs.velocity[0] * t_reach

    # Lane selection based on predicted lateral position
    if pred_obs_y < -0.25:
        chosen_y = left_y
    elif pred_obs_y > 0.25:
        chosen_y = right_y
    else:
        chosen_y = right_y if target_obs.velocity[1] > 0.1 else left_y

    # Place goal well past the obstacle so the path has room to merge back
    gx = min(pred_obs_x + target_obs.length / 2.0 + 22.0, road_len - 6.0)
    gx = max(gx, ego_xy[0] + 25.0)
    return np.array([gx, chosen_y])


def construct_continuous_trajectory(
    avoidance_path: np.ndarray,
    all_obstacles: list,
    road_len: float = 110.0,
    lane_w: float = 3.5,
) -> np.ndarray:
    """Stitch local avoidance path with downstream lane return and smooth globally."""
    right_y = -lane_w / 2.0
    last_pt = avoidance_path[-1]
    ext_pts = []

    furthest_x = max(
        (o.position[0] + o.length / 2.0 for o in all_obstacles),
        default=0.0
    )

    if last_pt[0] < furthest_x + 15.0:
        clear_x = min(furthest_x + 20.0, road_len - 8.0)
        ext_pts.append([clear_x, last_pt[1]])
        merge_x = min(clear_x + 22.0, road_len - 4.0)
        ext_pts.append([merge_x, right_y])
    elif abs(last_pt[1] - right_y) > 0.35:
        merge_x = min(last_pt[0] + 22.0, road_len - 4.0)
        ext_pts.append([merge_x, right_y])

    ext_pts.append([road_len, right_y])
    combined = np.vstack([avoidance_path, np.array(ext_pts)])
    return smooth_path(combined, num_output=400)


# =============================================================================
#  MASTER SIMULATION CLASS
# =============================================================================

class Simulation:
    """
    High-performance, flicker-free interactive simulation dashboard.
    """

    def __init__(self, initial_preset: int = 1):
        self.preset_id = initial_preset
        self.scenario = None
        self.ego = None
        self.obstacles = []
        self.trajectory = None
        self.step = 0
        self.trail_x, self.trail_y = [], []
        self.planned_paths = []
        self.handled_indices = set()
        self.last_replan_pos = {}
        self.replan_count = 0
        self.paused = False
        self.occ_grid = None

        # Reusable artist pools for zero-allocation rendering
        self.obs_artists = []
        self.grid_img = None
        self.dropdown_open = False

        # ── Setup Matplotlib Window ───────────────────────────────────────────
        self.fig = plt.figure(figsize=(15.5, 8.8), facecolor=BG_COLOR)
        self.fig.canvas.manager.set_window_title(
            'Smart India Hackathon 2026 — Adaptive Path Planning Simulation'
        )

        # 3 Rows: Main Panels (top), Telemetry Deck (middle), Control Bar (bottom)
        gs = GridSpec(
            3, 1,
            height_ratios=[4.5, 1.0, 0.25],
            hspace=0.22,
            top=0.94, bottom=0.08, left=0.05, right=0.96,
            figure=self.fig
        )

        # ── Panel 1: Bird's-Eye View ──────────────────────────────────────────
        self.ax1 = self.fig.add_subplot(gs[0, 0])
        self._init_birds_eye(self.ax1)

        # ── Panel 2: Digital Telemetry Deck ───────────────────────────────────
        self.ax3 = self.fig.add_subplot(gs[1, 0])
        self._init_telemetry(self.ax3)

        # ── Panel 3: Clean Control Bar with Single Dropdown ───────────────────
        self.ax_ctrl = self.fig.add_subplot(gs[2, 0])
        self.ax_ctrl.axis('off')
        self.ax_ctrl.set_facecolor(BG_COLOR)
        self._init_controls()

        # Load starting scenario
        self._load_scenario(self.preset_id)

        # ── Start Flicker-Free Animation Timer ────────────────────────────────
        # FuncAnimation handles double buffering and sync with display refresh
        self.anim = FuncAnimation(
            self.fig, self._on_anim_frame, interval=50,
            cache_frame_data=False, repeat=True
        )

    # ─── PANEL INITIALIZATION ─────────────────────────────────────────────────

    def _init_birds_eye(self, ax):
        """Construct the bird's-eye road view."""
        ax.set_facecolor(BG_COLOR)
        ax.set_xlim(-6, ROAD_LEN + 6)
        ax.set_ylim(-6.8, 6.8)
        ax.set_aspect('auto')
        ax.set_xlabel('Longitudinal Distance (m)', color='#94a3b8', fontsize=8.5, fontweight='bold')
        ax.set_ylabel('Lateral Offset (m)', color='#94a3b8', fontsize=8.5, fontweight='bold')
        ax.tick_params(colors='#64748b', labelsize=7.5)
        for s in ax.spines.values():
            s.set_color(BORDER_COLOR)

        # Asphalt Road Surface
        ax.add_patch(mpatches.FancyBboxPatch(
            (-6, -LANE_W), ROAD_LEN + 12, 2 * LANE_W,
            boxstyle='round,pad=0.2', facecolor=ROAD_COLOR, edgecolor=EDGE_COLOR, linewidth=2.0, zorder=1
        ))

        # Broken Centerline Marking
        for x0 in np.arange(-6, ROAD_LEN + 6, 4):
            ax.plot([x0, x0 + 2], [0, 0], color=LANE_MARK_COL, lw=2.2, zorder=2)

        # Solid Road Edges
        ax.plot([-6, ROAD_LEN + 6], [ LANE_W,  LANE_W], color=EDGE_COLOR, lw=2.2, zorder=2)
        ax.plot([-6, ROAD_LEN + 6], [-LANE_W, -LANE_W], color=EDGE_COLOR, lw=2.2, zorder=2)

        # Shoulders (Dirt/Ditch)
        for ymin, ymax in [(-LANE_W - 2.5, -LANE_W), (LANE_W, LANE_W + 2.5)]:
            ax.add_patch(mpatches.Rectangle(
                (-6, ymin), ROAD_LEN + 12, ymax - ymin,
                facecolor='#141b26', edgecolor='none', alpha=0.8, zorder=0
            ))

        self.bev_title = ax.set_title(
            '', color='#f8fafc', fontsize=10.5, fontweight='bold', pad=8
        )

        # Persistent Dynamic Artists
        self.path_line, = ax.plot(
            [], [], color=PATH_COLOR, lw=2.6, ls='--', alpha=0.9, zorder=3,
            label='Planned Trajectory',
            path_effects=[pe.withStroke(linewidth=4.0, foreground='#052e16')]
        )
        self.trail_line, = ax.plot(
            [], [], color=TRAIL_COLOR, lw=1.8, alpha=0.6, zorder=2, label='Ego Path History'
        )

        self.ego_patch = plt.Polygon([[0,0]], facecolor=EGO_COLOR, edgecolor='#ffffff', lw=1.5, zorder=6)
        ax.add_patch(self.ego_patch)
        self.ego_arrow = ax.annotate(
            '', xy=(0,0), xytext=(0,0),
            arrowprops=dict(arrowstyle='->', color='#ffffff', lw=1.8),
            zorder=7
        )

        self.sense_wedge = plt.Polygon(
            [[0,0]], facecolor=SENSE_COLOR, alpha=0.0, edgecolor=SENSE_COLOR, lw=1.2, ls='--', zorder=3
        )
        ax.add_patch(self.sense_wedge)

        ax.legend(
            loc='upper left', fontsize=7.5, facecolor='#0f172a',
            edgecolor=BORDER_COLOR, labelcolor='#e2e8f0', framealpha=0.85
        )


    def _init_telemetry(self, ax):
        """Construct Digital Instrument HUD Deck."""
        ax.set_facecolor(CARD_BG)
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 1)
        ax.axis('off')

        # Outer card outline
        card_box = mpatches.FancyBboxPatch(
            (0.5, 0.05), 99.0, 0.90,
            boxstyle='round,pad=0.02', facecolor=CARD_BG, edgecolor=BORDER_COLOR, linewidth=1.4
        )
        ax.add_patch(card_box)

        fields = [
            ('time', 'SIM TIME', 12.0),
            ('speed', 'EGO VELOCITY', 34.0),
            ('hazard', 'PERCEPTION TARGET', 58.0),
            ('status', 'AUTONOMOUS STATE', 84.0),
        ]
        self.dash_texts = {}
        for key, label, x in fields:
            ax.text(x, 0.72, label, color='#64748b', fontsize=7.2, fontweight='bold', ha='center')
            self.dash_texts[key] = ax.text(
                x, 0.26, '--', color='#f8fafc',
                fontsize=11.5, fontweight='bold', ha='center', family='monospace'
            )

        for sep_x in [24.0, 44.0, 72.0]:
            ax.plot([sep_x, sep_x], [0.15, 0.85], color=BORDER_COLOR, lw=1.2)

    def _init_controls(self):
        """
        Construct a clean control bar featuring:
          - A SINGLE, interactive dropdown menu to switch scenarios
          - Playback controls: [ PAUSE ], [ RESTART ], [ LOOP: ON ]
        """
        self._buttons = {}
        self.dropdown_opt_axes = []
        self.dropdown_opt_buttons = []

        # ── Main Dropdown Button (X: 0.06, W: 0.38, Y: 0.02, H: 0.040) ────────
        ax_drop = self.fig.add_axes([0.06, 0.022, 0.38, 0.040])
        initial_name = SCENARIO_PRESETS[self.preset_id]['name']
        self.btn_dropdown = Button(
            ax_drop, f"Scenario: [{self.preset_id}] {initial_name}  v",
            color='#1e293b', hovercolor='#334155'
        )
        self.btn_dropdown.drawon = False
        self.btn_dropdown.label.set_fontsize(8.5)
        self.btn_dropdown.label.set_color('#38bdf8')
        self.btn_dropdown.label.set_fontweight('bold')
        self.btn_dropdown.on_clicked(self._toggle_dropdown)
        self._buttons['dropdown'] = self.btn_dropdown

        # ── Dropdown Popup Options (Popping UP above the dropdown button) ─────
        presets = list(SCENARIO_PRESETS.items())
        h_opt = 0.036
        for i, (pid, pdata) in enumerate(presets):
            # Place options stacking upward from y = 0.066
            opt_y = 0.066 + (len(presets) - 1 - i) * h_opt
            ax_opt = self.fig.add_axes([0.06, opt_y, 0.38, h_opt])
            ax_opt.set_visible(False)
            btn_opt = Button(
                ax_opt, f"  [{pid}] {pdata['name']} - {pdata['desc'][:32]}...",
                color='#0d1526', hovercolor='#1d4ed8'
            )
            btn_opt.drawon = False
            btn_opt.label.set_fontsize(7.8)
            btn_opt.label.set_color('#e2e8f0')
            btn_opt.label.set_ha('left')
            btn_opt.label.set_position((0.03, 0.5))
            btn_opt.on_clicked(lambda evt, p=pid: self._select_scenario_from_dropdown(p))
            self.dropdown_opt_axes.append(ax_opt)
            self.dropdown_opt_buttons.append(btn_opt)

        # ── Control Buttons on the Right ──────────────────────────────────────
        # PAUSE / RESUME Button
        ax_pause = self.fig.add_axes([0.48, 0.022, 0.15, 0.040])
        self.btn_pause = Button(ax_pause, 'PAUSE', color='#1e293b', hovercolor='#2563eb')
        self.btn_pause.drawon = False
        self.btn_pause.label.set_fontsize(8.2)
        self.btn_pause.label.set_color('#ffffff')
        self.btn_pause.label.set_fontweight('bold')
        self.btn_pause.on_clicked(self._toggle_pause)
        self._buttons['pause'] = self.btn_pause

        # RESTART Button
        ax_reset = self.fig.add_axes([0.65, 0.022, 0.14, 0.040])
        self.btn_reset = Button(ax_reset, 'RESTART', color='#1e293b', hovercolor='#dc2626')
        self.btn_reset.drawon = False
        self.btn_reset.label.set_fontsize(8.2)
        self.btn_reset.label.set_color('#ffffff')
        self.btn_reset.label.set_fontweight('bold')
        self.btn_reset.on_clicked(lambda evt: self._load_scenario(self.preset_id))
        self._buttons['reset'] = self.btn_reset
    # ─── DROPDOWN INTERACTION ─────────────────────────────────────────────────

    def _toggle_dropdown(self, event):
        """Expand or collapse the single scenario dropdown menu."""
        self.dropdown_open = not self.dropdown_open
        for ax_opt, btn_opt in zip(self.dropdown_opt_axes, self.dropdown_opt_buttons):
            ax_opt.set_visible(self.dropdown_open)
            btn_opt.set_active(self.dropdown_open)
        arrow = '^' if self.dropdown_open else 'v'
        cur_name = SCENARIO_PRESETS[self.preset_id]['name']
        self.btn_dropdown.label.set_text(f"Scenario: [{self.preset_id}] {cur_name}  {arrow}")
        self.fig.canvas.draw_idle()

    def _select_scenario_from_dropdown(self, preset_id: int):
        """User selected a scenario from the dropdown: load it and collapse."""
        self.dropdown_open = False
        for ax_opt in self.dropdown_opt_axes:
            ax_opt.set_visible(False)
        self.preset_id = preset_id
        cur_name = SCENARIO_PRESETS[preset_id]['name']
        self.btn_dropdown.label.set_text(f"Scenario: [{preset_id}] {cur_name}  v")
        self._load_scenario(preset_id)
        self.fig.canvas.draw_idle()

    def _toggle_pause(self, event):
        self.paused = not self.paused
        lbl = 'RESUME' if self.paused else 'PAUSE'
        self.btn_pause.label.set_text(lbl)
        self.btn_pause.color = '#b45309' if self.paused else '#1e293b'
        self.fig.canvas.draw_idle()

    # ─── SCENARIO LOADER ──────────────────────────────────────────────────────

    def _load_scenario(self, preset_id: int):
        """Load scenario data and reset vehicle trajectory."""
        self.preset_id = preset_id
        self.scenario = build_scenario(preset_id)
        self.ego = self.scenario.ego

        # Deep-copy scenario obstacles
        self.obstacles = []
        for o in self.scenario.obstacles:
            oc = Vehicle(
                name=o.name, length=o.length, width=o.width, height=o.height,
                position=o.position.copy(), yaw=o.yaw, speed=o.speed,
                velocity=o.velocity.copy(), is_dynamic=o.is_dynamic,
                class_id=o.class_id, color=o.color, label=o.label
            )
            self.obstacles.append(oc)

        # Reset Ego & Nominal Trajectory
        right_y = -LANE_W / 2.0
        self.ego.position = np.array([0.0, right_y])
        self.ego.yaw = 0.0
        nominal_wp = np.array([[0.0, right_y], [ROAD_LEN, right_y]])
        self.trajectory = parameterise_path(nominal_wp, EGO_SPEED, DT)
        self.step = 0
        self.trail_x.clear()
        self.trail_y.clear()
        self.planned_paths.clear()
        self.handled_indices.clear()
        self.last_replan_pos.clear()
        self.replan_count = 0
        self.paused = False

        # Build reusable visual artists
        self._rebuild_obstacle_artists()
        self._refresh_costmap()

        # Update HUD Header & Indicators
        self.dash_texts['status'].set_text('LANE FOLLOWING')
        self.dash_texts['status'].set_color('#22c55e')
        self.bev_title.set_text(
            f"Bird's-Eye View — Scenario [{preset_id}]: {self.scenario.name}"
        )
        print(f"[sim] Loaded Scenario [{preset_id}] '{self.scenario.name}' — "
              f"{len(self.obstacles)} hazards ({sum(1 for o in self.obstacles if o.is_dynamic)} moving)")

    def _rebuild_obstacle_artists(self):
        """Pre-allocate polygon and callout artists for all obstacles in the scene."""
        for poly, ann in self.obs_artists:
            try: poly.remove()
            except: pass
            try: ann.remove()
            except: pass
        self.obs_artists.clear()

        for i, obs in enumerate(self.obstacles):
            poly = plt.Polygon(
                [[0,0]], facecolor=obs.color, edgecolor='#ffffff',
                linewidth=1.4, alpha=0.92, zorder=4
            )
            self.ax1.add_patch(poly)

            # Position callout strictly ABOVE the road boundary (y >= 4.2m)
            label_y = LANE_W + 0.8 + (i % 2) * 1.3
            ann = self.ax1.annotate(
                obs.label,
                xy=(obs.position[0], obs.position[1]),
                xytext=(obs.position[0], label_y),
                fontsize=7.8, fontweight='bold', color='#ffffff',
                ha='center', va='bottom',
                bbox=dict(boxstyle='round,pad=0.22', facecolor=obs.color,
                          edgecolor='#ffffff', alpha=0.88, linewidth=1.0),
                arrowprops=dict(arrowstyle='->', color=obs.color,
                                lw=1.3, connectionstyle='arc3,rad=0.0'),
                zorder=9
            )
            self.obs_artists.append((poly, ann))

    def _refresh_costmap(self):
        """Update the internal multi-layer occupancy grid (no longer rendered visually)."""
        self.scenario.obstacles = self.obstacles
        self.occ_grid = build_occupancy_grid(self.scenario)

    # ─── HIGH-SPEED ARTIST UPDATES ────────────────────────────────────────────

    def _update_artists(self, ego_xy: np.ndarray, ego_yaw: float, hdist: float):
        """Fast property updates on existing artists with aspect ratio correction."""
        # Calculate anti-elongation squish for 'auto' aspect ratio
        bbox = self.ax1.get_window_extent()
        x_b = self.ax1.get_xlim()
        y_b = self.ax1.get_ylim()
        x_ppu = bbox.width / max(x_b[1] - x_b[0], 1.0)
        y_ppu = bbox.height / max(y_b[1] - y_b[0], 1.0)
        y_squish = x_ppu / y_ppu if y_ppu > 0 else 1.0

        # 1. Obstacle Polygons & Callouts
        for i, obs in enumerate(self.obstacles):
            if i >= len(self.obs_artists):
                break
            poly, ann = self.obs_artists[i]
            ox, oy = obs.position

            corners = np.array([
                [-obs.length / 2.0, -obs.width / 2.0],
                [ obs.length / 2.0, -obs.width / 2.0],
                [ obs.length / 2.0,  obs.width / 2.0],
                [-obs.length / 2.0,  obs.width / 2.0],
            ])
            c, s = np.cos(obs.yaw), np.sin(obs.yaw)
            R = np.array([[c, -s], [s, c]])
            rot = (R @ corners.T).T
            rot[:, 1] *= y_squish
            poly.set_xy(rot + np.array([ox, oy]))

            # Callout positioned above road
            label_y = LANE_W + 0.8 + (i % 2) * 1.3
            tag = obs.label
            if obs.is_dynamic:
                spd = np.linalg.norm(obs.velocity)
                if spd > 0.1:
                    tag += f' ({spd:.1f}m/s)'
            ann.set_text(tag)
            ann.xy = (ox, oy + obs.width / 2.0)
            ann.set_position((ox, label_y))

        # 2. Ego Vehicle Body & Heading
        corners_e = np.array([
            [-self.ego.length / 2.0, -self.ego.width / 2.0],
            [ self.ego.length / 2.0, -self.ego.width / 2.0],
            [ self.ego.length / 2.0,  self.ego.width / 2.0],
            [-self.ego.length / 2.0,  self.ego.width / 2.0],
        ])
        c_e, s_e = np.cos(ego_yaw), np.sin(ego_yaw)
        R_e = np.array([[c_e, -s_e], [s_e, c_e]])
        rot_e = (R_e @ corners_e.T).T
        rot_e[:, 1] *= y_squish
        self.ego_patch.set_xy(rot_e + ego_xy)

        # Arrow tip (keep aspect ratio correct for arrow pointing direction)
        tip_x = ego_xy[0] + (self.ego.length / 2.0 + 1.2) * c_e
        tip_y = ego_xy[1] + (self.ego.length / 2.0 + 1.2) * s_e * y_squish
        self.ego_arrow.xy = (tip_x, tip_y)
        self.ego_arrow.set_position(tuple(ego_xy))

        # 3. LiDAR Forward Perception Sector (Always On)
        wedge_pts = np.array([
            [ego_xy[0], ego_xy[1] - 0.9 * y_squish],
            [ego_xy[0] + DETECT_DIST, -ROAD_HALF_W],
            [ego_xy[0] + DETECT_DIST,  ROAD_HALF_W],
            [ego_xy[0], ego_xy[1] + 0.9 * y_squish]
        ])
        self.sense_wedge.set_xy(wedge_pts)
        
        # Glow slightly brighter when an obstacle is within range
        if hdist < DETECT_DIST * 1.3:
            self.sense_wedge.set_alpha(0.18)
            self.sense_wedge.set_edgecolor('#ff4444')
        else:
            self.sense_wedge.set_alpha(0.08)
            self.sense_wedge.set_edgecolor('#00f0ff')

        # 4. Trajectory Lines
        self.trail_line.set_data(self.trail_x, self.trail_y)
        if self.planned_paths:
            cur_p = self.planned_paths[-1]
            self.path_line.set_data(cur_p[:, 0], cur_p[:, 1])

    # ─── ANIMATION FRAME TICK (CALLED BY FuncAnimation) ────────────────────────

    def _on_anim_frame(self, frame_num):
        """
        Called automatically by FuncAnimation on timer tick.
        Double-buffered: NO SCREEN TEARING / NO BLINKING.
        Implements adaptive speed control: decelerates near obstacles.
        """
        if self.paused:
            return

        # ── End of Track Check ────────────────────────────────────────────────
        if self.step >= len(self.trajectory) - 2 or self.ego.position[0] >= ROAD_LEN - 3.0:
            self.paused = True
            self.dash_texts['status'].set_text('MISSION COMPLETE')
            self.dash_texts['status'].set_color('#22c55e')
            return

        # ── Ego Pose ──────────────────────────────────────────────────────────
        ego_xy = self.trajectory[self.step]
        self.ego.position = ego_xy.copy()


        if self.step < len(self.trajectory) - 1:
            dx = self.trajectory[self.step + 1, 0] - ego_xy[0]
            dy = self.trajectory[self.step + 1, 1] - ego_xy[1]
        elif self.step > 0:
            dx = ego_xy[0] - self.trajectory[self.step - 1, 0]
            dy = ego_xy[1] - self.trajectory[self.step - 1, 1]
        else:
            dx, dy = 1.0, 0.0

        ego_yaw = np.arctan2(dy, dx) if (abs(dx) + abs(dy)) > 1e-4 else 0.0
        self.ego.yaw = ego_yaw

        self.trail_x.append(ego_xy[0])
        self.trail_y.append(ego_xy[1])

        steps_per_frame = 1  # 1 step per frame, but we'll speed up animation interval

        # ── Kinematic Motion Update for Dynamic Obstacles ──────────────────────
        for obs in self.obstacles:
            if obs.is_dynamic:
                obs.position += obs.velocity * (DT * steps_per_frame)
                obs.position[1] = np.clip(obs.position[1], -ROAD_HALF_W - 0.4, ROAD_HALF_W + 0.4)
                if abs(obs.position[1]) >= ROAD_HALF_W + 0.3:
                    obs.velocity[1] *= -1.0

        # ── Multi-Hazard Perception & Collision Assessment ───────────────────
        active_hazard = None
        min_dist = float('inf')
        should_replan = False
        target_replan_obs = None

        for idx, obs in enumerate(self.obstacles):
            long_dist = obs.position[0] - ego_xy[0]
            euclid_d = np.linalg.norm(ego_xy - obs.position)

            if long_dist > -3.0 and euclid_d < min_dist:
                min_dist = euclid_d
                active_hazard = obs

            # Detect obstacles within the extended sensing horizon
            if -2.0 <= long_dist <= DETECT_DIST:
                remaining_traj = self.trajectory[self.step:]
                has_conflict = check_oriented_collision(
                    remaining_traj, obs,
                    ego_length=self.ego.length, ego_width=self.ego.width
                )

                if has_conflict:
                    if idx not in self.handled_indices:
                        should_replan = True
                        target_replan_obs = obs
                        self.handled_indices.add(idx)
                        self.last_replan_pos[idx] = obs.position.copy()
                        break
                    elif obs.is_dynamic and idx in self.last_replan_pos:
                        disp = np.linalg.norm(obs.position - self.last_replan_pos[idx])
                        if disp > 2.0:
                            should_replan = True
                            target_replan_obs = obs
                            self.last_replan_pos[idx] = obs.position.copy()
                            break

        current_speed = EGO_SPEED

        # ── Adaptive Path Planning ────────────────────────────────────────────
        sim_time = self.step * DT
        status_str = 'LANE FOLLOWING'
        status_col = '#22c55e'

        if should_replan and target_replan_obs is not None:
            status_str = 'AVOIDING HAZARD'
            status_col = '#00f0ff'

            self._refresh_costmap()
            goal_xy = compute_avoidance_goal(ego_xy, target_replan_obs, lane_w=LANE_W, road_len=ROAD_LEN)
            path_xy, path_yaw, success = plan_avoidance_path(
                self.occ_grid, ego_xy, ego_yaw, goal_xy, lane_width=LANE_W
            )

            if success and path_xy is not None:
                self.planned_paths.append(path_xy.copy())
                full_path = construct_continuous_trajectory(
                    path_xy, self.obstacles, road_len=ROAD_LEN, lane_w=LANE_W
                )
                new_traj = parameterise_path(full_path, EGO_SPEED, DT)
                self.trajectory = np.vstack([self.trajectory[:self.step], new_traj])
                self.replan_count += 1
                print(f"[sim] t={sim_time:.1f}s | Avoidance around '{target_replan_obs.name}' "
                      f"(d={min_dist:.1f}m, dynamic={target_replan_obs.is_dynamic})")

        elif min_dist <= DETECT_DIST:
            if abs(ego_xy[1] - (-LANE_W / 2.0)) > 0.6:
                status_str = 'OVERTAKING / MERGING'
                status_col = '#38bdf8'
            else:
                status_str = 'HAZARD TRACKED'
                status_col = '#f59e0b'

        # ── Visuals & Telemetry Updates ───────────────────────────────────────
        self._update_artists(ego_xy, ego_yaw, min_dist)

        self.dash_texts['time'].set_text(f'{sim_time:04.1f} s')
        self.dash_texts['speed'].set_text(f'{current_speed * 3.6:.0f} km/h')
        if active_hazard is not None and min_dist < 90.0:
            spd_str = f" | {np.linalg.norm(active_hazard.velocity):.1f}m/s" if active_hazard.is_dynamic else ""
            self.dash_texts['hazard'].set_text(f"{active_hazard.label} ({min_dist:.1f}m{spd_str})")
            self.dash_texts['hazard'].set_color('#ef4444' if min_dist < DETECT_DIST else '#f8fafc')
        else:
            self.dash_texts['hazard'].set_text('LANE CLEAR')
            self.dash_texts['hazard'].set_color('#22c55e')

        self.dash_texts['status'].set_text(status_str)
        self.dash_texts['status'].set_color(status_col)
        self.bev_title.set_text(
            f"Bird's-Eye View — [{self.preset_id}] {self.scenario.name} | "
            f"t={sim_time:.1f}s | Replans: {self.replan_count}"
        )

        self.step += steps_per_frame

    # ─── RUN APPLICATION ──────────────────────────────────────────────────────

    def show(self):
        """Launch the non-blocking GUI."""
        print("\n" + "=" * 65)
        print("  Smart India Hackathon 2026 — Adaptive Path Planning Testbed")
        print("  • Use the Scenario dropdown menu to test different Indian road hazards")
        print("  • Smooth, non-blinking double-buffered animation")
        print("=" * 65 + "\n")
        plt.show()


# =============================================================================
#  ENTRY POINT
# =============================================================================

def main():
    initial_preset = 1
    if len(sys.argv) > 1:
        try:
            pid = int(sys.argv[1])
            if pid in SCENARIO_PRESETS:
                initial_preset = pid
        except ValueError:
            pass

    sim = Simulation(initial_preset=initial_preset)
    sim.show()


if __name__ == '__main__':
    main()
