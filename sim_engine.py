import sys
import json
import time
import math
import select
import threading
import numpy as np

from scenario_builder import build_scenario, Vehicle
from perception_map import build_occupancy_grid
from dynamic_planner import plan_avoidance_path
# ============================================================================
# Simulation Settings
# ============================================================================
DT = 0.03               # Physics tick (20 Hz)
EGO_SPEED = 10.0        # Cruise speed (m/s)
DETECT_DIST = 35.0      # LiDAR range
LANE_W = 3.5

def parameterise_path(waypoints: np.ndarray, speed: float, dt: float) -> np.ndarray:
    """Generate time-parameterised trajectory points from spatial waypoints."""
    diffs = np.diff(waypoints, axis=0)
    dists = np.linalg.norm(diffs, axis=1)
    cum_dists = np.insert(np.cumsum(dists), 0, 0.0)
    
    total_time = cum_dists[-1] / speed
    num_steps = int(total_time / dt)
    if num_steps < 2: return waypoints
    
    t_eval = np.linspace(0, cum_dists[-1], num_steps)
    x_interp = np.interp(t_eval, cum_dists, waypoints[:, 0])
    y_interp = np.interp(t_eval, cum_dists, waypoints[:, 1])
    return np.column_stack((x_interp, y_interp))


class SimEngine:
    def __init__(self):
        self.preset_id = 1
        self.paused = False
        self.running = True
        
        self.ego = None
        self.obstacles = []
        self.scenario = None
        
        self.ego_x = 0.0
        self.ego_y = -LANE_W / 2.0
        self.ego_yaw = 0.0
        
        self.planned_path = []
        self.trail = []
        
        self.time_elapsed = 0.0
        
        self.replan_count = 0
        self.stateflow = "CRUISE"
        self.state_desc = "Maintaining lane center and nominal speed."
        self.last_replan_time = 0
        self.replan_latency = 0
        self.collisions = 0
        self.planning = False
        
        self.load_scenario(self.preset_id)
        
        # Thread for reading stdin
        self.cmd_thread = threading.Thread(target=self._read_stdin, daemon=True)
        self.cmd_thread.start()

    def load_scenario(self, preset_id):
        self.preset_id = preset_id
        self.scenario = build_scenario(preset_id)
        self.ego = self.scenario.ego
        
        self.ego_x = 0.0
        self.ego_y = -LANE_W / 2.0
        self.ego_yaw = 0.0
        
        self.obstacles = self.scenario.obstacles
        self.planned_path = []
        self.trail = []
        self.time_elapsed = 0.0
        self.replan_count = 0
        self.collisions = 0
        self.planning = False
        
        self.stateflow = "CRUISE"
        self.state_desc = "Maintaining lane center and nominal speed."
        
    def _read_stdin(self):
        for line in sys.stdin:
            line = line.strip()
            if not line: continue
            try:
                cmd = json.loads(line)
                if cmd.get("action") == "load":
                    self.load_scenario(cmd.get("preset", 1))
                    self.paused = False
                elif cmd.get("action") == "pause":
                    self.paused = True
                elif cmd.get("action") == "resume":
                    self.paused = False
                elif cmd.get("action") == "reset":
                    self.load_scenario(self.preset_id)
                    self.paused = False
            except Exception as e:
                pass # Ignore malformed json

    def spawn_obstacles(self):
        """Procedurally spawn obstacles far ahead (e.g. x + 80m) to keep the simulation endless."""
        # Simple procedural generation: if the furthest obstacle is closer than 60m ahead, spawn a new one
        furthest_x = max([obs.position[0] for obs in self.obstacles] + [self.ego_x])
        if furthest_x < self.ego_x + 60.0:
            spawn_x = self.ego_x + np.random.uniform(70.0, 100.0)
            lane = np.random.choice([-1.75, 1.75])
            
            if self.preset_id == 1:
                # Unmarked village - cows
                self.obstacles.append(Vehicle(
                    name="Stray Cow", length=2.5, width=1.2, height=1.5,
                    position=np.array([spawn_x, lane]), yaw=0.0, speed=0, velocity=np.array([0., 0.]),
                    is_dynamic=False, class_id=2, color="#a16207", label="Cow"
                ))
            elif self.preset_id == 2:
                # Urban - crossing ped or rickshaw
                if np.random.rand() > 0.5:
                    self.obstacles.append(Vehicle(
                        name="Rickshaw", length=2.8, width=1.4, height=1.8,
                        position=np.array([spawn_x, lane]), yaw=0.0, speed=3, velocity=np.array([3., 0.]),
                        is_dynamic=True, class_id=2, color="#eab308", label="Rickshaw"
                    ))
                else:
                    self.obstacles.append(Vehicle(
                        name="Pedestrian", length=0.6, width=0.6, height=1.7,
                        position=np.array([spawn_x, 3.5]), yaw=0.0, speed=1.2, velocity=np.array([0., -1.2]),
                        is_dynamic=True, class_id=2, color="#ef4444", label="Pedestrian"
                    ))
            elif self.preset_id == 3:
                # Highway - trucks
                self.obstacles.append(Vehicle(
                    name="Truck", length=6.0, width=2.4, height=3.0,
                    position=np.array([spawn_x, lane]), yaw=0.0, speed=8, velocity=np.array([8., 0.]),
                    is_dynamic=True, class_id=2, color="#3b82f6", label="Truck"
                ))
            elif self.preset_id == 4:
                # Mixed
                types = [
                    (Vehicle("Pothole", 1.6, 1.6, 0.1, np.array([spawn_x, lane]), 0.0, 0, np.array([0.,0.]), False, 2, "#475569", "Pothole")),
                    (Vehicle("Rickshaw", 2.8, 1.4, 1.8, np.array([spawn_x, lane]), 0.0, 3, np.array([3.,0.]), True, 2, "#eab308", "Rickshaw")),
                    (Vehicle("Cow", 2.5, 1.2, 1.5, np.array([spawn_x, lane]), 0.0, 0, np.array([0.,0.]), False, 2, "#a16207", "Cow"))
                ]
                self.obstacles.append(types[np.random.randint(0, len(types))])
            elif self.preset_id == 5:
                # Cattle crossing (sudden)
                self.obstacles.append(Vehicle(
                    name="Stray Cow", length=2.5, width=1.2, height=1.5,
                    position=np.array([spawn_x, lane]), yaw=0.0, speed=0, velocity=np.array([0., 0.]),
                    is_dynamic=False, class_id=2, color="#a16207", label="Cow"
                ))
            elif self.preset_id == 6:
                # Stray Dog + Parked Van
                self.obstacles.append(Vehicle(
                    name="Parked Van", length=4.5, width=2.0, height=2.2,
                    position=np.array([spawn_x, -1.75]), yaw=0.0, speed=0, velocity=np.array([0., 0.]),
                    is_dynamic=False, class_id=2, color="#6366f1", label="Van"
                ))
                self.obstacles.append(Vehicle(
                    name="Stray Dog", length=0.8, width=0.4, height=0.6,
                    position=np.array([spawn_x - 15.0, 1.75]), yaw=0.0, speed=0.7, velocity=np.array([0., -0.7]),
                    is_dynamic=True, class_id=2, color="#d97706", label="Dog"
                ))

    def cleanup_obstacles(self):
        """Remove obstacles that have passed behind the ego to save memory."""
        self.obstacles = [o for o in self.obstacles if o.position[0] > self.ego_x - 30.0]


    def _run_planner_thread(self, occ_grid, ego_x, ego_y, ego_yaw, goal_x, goal_y):
        self.planning = True
        t_plan = time.time()
        path_xy, path_yaw, success = plan_avoidance_path(
            occ_grid, np.array([ego_x, ego_y]), ego_yaw, np.array([goal_x, goal_y])
        )
        self.replan_latency = int((time.time() - t_plan) * 1000)
        
        if success and path_xy is not None:
            self.planned_path = path_xy.tolist()
            self.replan_count += 1
            self.last_replan_time = time.time()
        else:
            self.stateflow = "BRAKING"
            self.state_desc = "Lateral clearance insufficient. Falling back to braking profile."
        self.planning = False

    def tick(self):
        if self.paused:
            return

        self.time_elapsed += DT
        
        # 1. Spawn & Cleanup
        self.spawn_obstacles()
        self.cleanup_obstacles()
        
        # 2. Move Dynamic Obstacles
        for obs in self.obstacles:
            if obs.is_dynamic:
                obs.position += obs.velocity * DT

        # 3. Perception Map Update
        # Update ego position for occupancy grid building
        self.ego.position = np.array([self.ego_x, self.ego_y])
        self.ego.yaw = self.ego_yaw
        
        # Create temporary scenario object to pass to perception
        self.scenario.ego = self.ego
        self.scenario.obstacles = self.obstacles
        
        t0 = time.time()
        occ_grid = build_occupancy_grid(self.scenario)
        
        # 4. Hybrid A* Planning
        # Find closest obstacle in front
        min_dist = 999.0
        min_ttc = 999.0
        for obs in self.obstacles:
            dx = obs.position[0] - self.ego_x
            dy = obs.position[1] - self.ego_y
            if dx > 0 and dx < DETECT_DIST:
                dist = math.hypot(dx, dy)
                min_dist = min(min_dist, dist)
                rel_v = EGO_SPEED - np.linalg.norm(obs.velocity)
                if rel_v > 0:
                    min_ttc = min(min_ttc, dx / rel_v)
                
                # Collision check
                if abs(dx) < (self.ego.length/2 + obs.length/2) and abs(dy) < (self.ego.width/2 + obs.width/2):
                    self.collisions += 1

        needs_replan = (min_dist < DETECT_DIST)

        if needs_replan and (time.time() - self.last_replan_time > 0.5) and not getattr(self, 'planning', False):
            self.stateflow = "EVADE"
            self.state_desc = "Hazard detected. Evaluating swerve feasibility via Hybrid A*."
            
            goal_x = self.ego_x + 40.0
            # Bias goal to clear lane
            goal_y = -LANE_W/2.0 if self.ego_y > 0 else LANE_W/2.0
            
            import threading
            threading.Thread(target=self._run_planner_thread, args=(occ_grid, self.ego_x, self.ego_y, self.ego_yaw, goal_x, goal_y)).start()
        elif not needs_replan:
            self.stateflow = "CRUISE"
            self.state_desc = "Maintaining lane center and nominal speed."
            # Nominal path
            goal_y = -LANE_W/2.0
            self.planned_path = [
                [self.ego_x, self.ego_y],
                [self.ego_x + 10.0, self.ego_y * 0.5 + goal_y * 0.5],
                [self.ego_x + 30.0, goal_y],
                [self.ego_x + 60.0, goal_y]
            ]
            
        # 5. Move Ego Vehicle along path
        actual_speed = EGO_SPEED
        if self.stateflow == "BRAKING":
            actual_speed = max(0.0, actual_speed - 5.0 * DT)
            
        if len(self.planned_path) > 1:
            # Simple pure pursuit or just pop next point
            target = self.planned_path[1]
            dx = target[0] - self.ego_x
            dy = target[1] - self.ego_y
            dist = math.hypot(dx, dy)
            if dist > 0:
                self.ego_yaw = math.atan2(dy, dx)
                move_dist = min(dist, actual_speed * DT)
                self.ego_x += math.cos(self.ego_yaw) * move_dist
                self.ego_y += math.sin(self.ego_yaw) * move_dist
                
                if move_dist == dist:
                    self.planned_path.pop(0)
            else:
                self.ego_x += actual_speed * DT
        else:
            self.ego_x += actual_speed * DT

        # Update Trail
        self.trail.append([self.ego_x, self.ego_y])
        if len(self.trail) > 100:
            self.trail.pop(0)
            
        # 6. Build JSON Frame
        frame = {
            "time": round(self.time_elapsed, 1),
            "ego": {
                "x": self.ego_x,
                "y": self.ego_y,
                "yaw": self.ego_yaw,
                "speed": round(actual_speed * 3.6, 1), # km/h
                "w": self.ego.length,
                "h": self.ego.width
            },
            "obstacles": [
                {
                    "x": o.position[0],
                    "y": o.position[1],
                    "yaw": o.yaw,
                    "w": o.length,
                    "h": o.width,
                    "color": o.color,
                    "label": o.label,
                    "speed": round(np.linalg.norm(o.velocity) * 3.6, 1)
                } for o in self.obstacles
            ],
            "path": self.planned_path,
            "trail": self.trail,
            "telemetry": {
                "speed": round(actual_speed * 3.6, 1),
                "replanLatency": self.replan_latency,
                "replans": self.replan_count,
                "trackedActors": len(self.obstacles),
                "minTTC": f"{min_ttc:.1f}s" if min_ttc < 99.0 else "— clear",
                "distance": int(self.ego_x),
                "collisions": self.collisions
            },
            "state": self.stateflow,
            "stateDetail": self.state_desc,
            "sensorWedge": [
                [self.ego_x, self.ego_y - 0.9],
                [self.ego_x + DETECT_DIST, -LANE_W],
                [self.ego_x + DETECT_DIST,  LANE_W],
                [self.ego_x, self.ego_y + 0.9]
            ]
        }
        
        # Flush to stdout
        sys.stdout.write(json.dumps(frame) + '\n')
        sys.stdout.flush()

if __name__ == "__main__":
    engine = SimEngine()
    while True:
        engine.tick()
        time.sleep(DT)
