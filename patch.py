with open('sim_engine.py', 'r') as f:
    content = f.read()

# Replace DT
content = content.replace("DT = 0.05", "DT = 0.03")

# Add self.planning = False
content = content.replace("self.collisions = 0", "self.collisions = 0\n        self.planning = False")

# Add the thread function
thread_func = '''
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

    def tick(self):'''
content = content.replace("    def tick(self):", thread_func)

# Replace the blocking call
blocking_code = '''        if needs_replan and (time.time() - self.last_replan_time > 0.5):
            self.stateflow = "EVADE"
            self.state_desc = "Hazard detected. Evaluating swerve feasibility via Hybrid A*."
            
            goal_x = self.ego_x + 40.0
            # Bias goal to clear lane
            goal_y = -LANE_W/2.0 if self.ego_y > 0 else LANE_W/2.0
            
            t_plan = time.time()
            path_xy, path_yaw, success = plan_avoidance_path(
                occ_grid, np.array([self.ego_x, self.ego_y]), self.ego_yaw, np.array([goal_x, goal_y])
            )
            self.replan_latency = int((time.time() - t_plan) * 1000)
            
            if success and path_xy is not None:
                self.planned_path = path_xy.tolist()
                self.replan_count += 1
                self.last_replan_time = time.time()
            else:
                self.stateflow = "BRAKING"
                self.state_desc = "Lateral clearance insufficient. Falling back to braking profile."'''

threaded_code = '''        if needs_replan and (time.time() - self.last_replan_time > 0.5) and not getattr(self, 'planning', False):
            self.stateflow = "EVADE"
            self.state_desc = "Hazard detected. Evaluating swerve feasibility via Hybrid A*."
            
            goal_x = self.ego_x + 40.0
            # Bias goal to clear lane
            goal_y = -LANE_W/2.0 if self.ego_y > 0 else LANE_W/2.0
            
            import threading
            threading.Thread(target=self._run_planner_thread, args=(occ_grid, self.ego_x, self.ego_y, self.ego_yaw, goal_x, goal_y)).start()'''

content = content.replace(blocking_code, threaded_code)

with open('sim_engine.py', 'w') as f:
    f.write(content)
