"""
scenario_builder.py — Scenario Builder with Static & Dynamic Obstacles
========================================================================

6 Indian road presets featuring both static hazards (potholes, parked trucks)
and dynamic actors (slow rickshaws, crossing pedestrians, stray animals).

Each Vehicle has a `velocity` vector [vx, vy] m/s for moving actors.

Smart India Hackathon — Adaptive Path Planning POC
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Any


@dataclass
class Vehicle:
    """Any actor: ego car, static obstacle, or moving hazard."""
    name: str
    length: float
    width: float
    height: float
    position: np.ndarray          # [x, y] world (m) — mutable for dynamics
    yaw: float
    speed: float
    velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    is_dynamic: bool = False
    class_id: int = 1
    color: str = '#ef4444'
    label: str = ''


@dataclass
class Road:
    centers: np.ndarray
    lane_width: float
    num_lanes: int
    half_width: float = field(init=False)

    def __post_init__(self):
        self.half_width = (self.num_lanes * self.lane_width) / 2.0


@dataclass
class Scenario:
    road: Road
    ego: Vehicle
    obstacles: List[Vehicle]
    sample_time: float
    stop_time: float
    name: str = 'Default'
    description: str = ''


SCENARIO_PRESETS: Dict[int, Dict[str, Any]] = {
    1: {
        'name': 'Stray Cattle',
        'desc': 'Static cow resting in the right lane at 50 m.',
        'btn_color': '#92400e',
        'obstacles': [
            {'name': 'Stray Cow', 'length': 2.5, 'width': 1.2, 'height': 1.5,
             'position': [50.0, -1.75], 'velocity': [0.0, 0.0],
             'is_dynamic': False, 'color': '#a16207', 'label': 'Cattle'},
        ],
    },
    2: {
        'name': 'Slow Rickshaw',
        'desc': 'Auto-rickshaw cruising at 3 m/s ahead in right lane.',
        'btn_color': '#a16207',
        'obstacles': [
            {'name': 'Auto-Rickshaw', 'length': 2.8, 'width': 1.4, 'height': 1.8,
             'position': [40.0, -1.75], 'velocity': [3.0, 0.0],
             'is_dynamic': True, 'color': '#eab308', 'label': 'Rickshaw'},
        ],
    },
    3: {
        'name': 'Pedestrian Crossing',
        'desc': 'Pedestrian crossing from left shoulder to right at 50 m.',
        'btn_color': '#b91c1c',
        'obstacles': [
            {'name': 'Pedestrian', 'length': 0.6, 'width': 0.6, 'height': 1.7,
             'position': [50.0, 2.5], 'velocity': [0.0, -1.0],
             'is_dynamic': True, 'color': '#ef4444', 'label': 'Pedestrian'},
        ],
    },
    4: {
        'name': 'Potholes & Bullock Cart',
        'desc': 'Two potholes + slow bullock cart (1.5 m/s) ahead.',
        'btn_color': '#475569',
        'obstacles': [
            {'name': 'Pothole 1', 'length': 1.6, 'width': 1.6, 'height': 0.1,
             'position': [35.0, -1.75], 'velocity': [0.0, 0.0],
             'is_dynamic': False, 'color': '#475569', 'label': 'Pothole'},
            {'name': 'Pothole 2', 'length': 1.8, 'width': 1.4, 'height': 0.1,
             'position': [48.0, -2.1], 'velocity': [0.0, 0.0],
             'is_dynamic': False, 'color': '#475569', 'label': 'Pothole'},
            {'name': 'Bullock Cart', 'length': 3.5, 'width': 1.8, 'height': 2.0,
             'position': [60.0, -1.75], 'velocity': [1.5, 0.0],
             'is_dynamic': True, 'color': '#92400e', 'label': 'Cart'},
        ],
    },
    5: {
        'name': 'Multi-Hazard Gauntlet',
        'desc': 'Cow at 40 m, slow truck at 65 m (2 m/s), barrier at 85 m.',
        'btn_color': '#1d4ed8',
        'obstacles': [
            {'name': 'Stray Cow', 'length': 2.5, 'width': 1.2, 'height': 1.5,
             'position': [40.0, -1.75], 'velocity': [0.0, 0.0],
             'is_dynamic': False, 'color': '#a16207', 'label': 'Cow'},
            {'name': 'Slow Truck', 'length': 6.0, 'width': 2.4, 'height': 3.0,
             'position': [65.0, -1.75], 'velocity': [2.0, 0.0],
             'is_dynamic': True, 'color': '#3b82f6', 'label': 'Truck'},
            {'name': 'Road Barrier', 'length': 3.0, 'width': 1.0, 'height': 1.0,
             'position': [85.0, -1.2], 'velocity': [0.0, 0.0],
             'is_dynamic': False, 'color': '#ea580c', 'label': 'Barrier'},
        ],
    },
    6: {
        'name': 'Stray Dog + Parked Van',
        'desc': 'Dog wandering across road at 45 m, parked van at 70 m.',
        'btn_color': '#7c3aed',
        'obstacles': [
            {'name': 'Stray Dog', 'length': 0.8, 'width': 0.5, 'height': 0.5,
             'position': [45.0, 2.0], 'velocity': [0.3, -0.6],
             'is_dynamic': True, 'color': '#d97706', 'label': 'Dog'},
            {'name': 'Parked Van', 'length': 5.0, 'width': 2.0, 'height': 2.2,
             'position': [70.0, -1.75], 'velocity': [0.0, 0.0],
             'is_dynamic': False, 'color': '#6366f1', 'label': 'Van'},
        ],
    },
}


def build_scenario(preset_id: int = 1) -> Scenario:
    if preset_id not in SCENARIO_PRESETS:
        preset_id = 1
    preset = SCENARIO_PRESETS[preset_id]

    road = Road(
        centers=np.array([[0,0],[25,0.5],[50,0],[75,-0.5],[100,0]]),
        lane_width=3.5, num_lanes=2,
    )

    ego = Vehicle(
        name='EgoCar', length=4.7, width=1.8, height=1.4,
        position=np.array([0.0, -3.5/2]),
        yaw=0.0, speed=10.0, class_id=1,
        color='#00d4ff', label='Ego',
    )

    obstacles = []
    for c in preset['obstacles']:
        obstacles.append(Vehicle(
            name=c['name'], length=c['length'], width=c['width'],
            height=c['height'], position=np.array(c['position'], dtype=float),
            yaw=0.0, speed=0.0,
            velocity=np.array(c['velocity'], dtype=float),
            is_dynamic=c['is_dynamic'], class_id=2,
            color=c['color'], label=c['label'],
        ))

    return Scenario(
        road=road, ego=ego, obstacles=obstacles,
        sample_time=0.05, stop_time=30.0,
        name=preset['name'], description=preset['desc'],
    )


def list_presets():
    print("\n  Available Scenarios:")
    for pid, p in SCENARIO_PRESETS.items():
        dyn = sum(1 for o in p['obstacles'] if o.get('is_dynamic'))
        print(f"  [{pid}] {p['name']} — {p['desc']}  "
              f"({len(p['obstacles'])} hazards, {dyn} moving)")
    print()
