%% MAIN_SIM  Master script – Adaptive Path Planning for Unstructured Indian Roads
%
% This script orchestrates the full Software-in-the-Loop (SIL) simulation:
%   1. Build the driving scenario          (scenario_builder.m)
%   2. Step the simulation forward in time
%   3. When the obstacle is within 20 m, invoke the local planner
%   4. Update the ego trajectory to follow the replanned path
%   5. Visualise everything in a bird's-eye-view figure
%
% Run this script from the MATLAB command window:
%   >> main_sim
%
% Toolbox Requirements:
%   - Automated Driving Toolbox
%   - Navigation Toolbox
%
% -----------------------------------------------------------------------
% Smart India Hackathon – Adaptive Path Planning POC
% -----------------------------------------------------------------------

clc; clear; close all;
fprintf('=== SIH Adaptive Path Planning – SIL Simulation ===\n\n');

%% ====================================================================
%  CONFIGURATION
%  ====================================================================

DETECT_THRESHOLD = 20.0;    % metres – trigger replanning when obstacle is this close
EGO_SPEED        = 10.0;    % m/s   – nominal cruising speed
LANE_WIDTH       = 3.5;     % m     – must match scenario_builder
SIM_STEP         = 0.05;    % s     – scenario sample time (20 Hz)
REPLAN_COOLDOWN  = 1.0;     % s     – minimum time between replans

% Grid parameters for the occupancy map
gridParams.width      = 120;    % m
gridParams.height     = 20;     % m
gridParams.resolution = 2;      % cells per metre (0.5 m cell)
gridParams.inflateR   = 1.0;    % m – safety inflation

%% ====================================================================
%  1. BUILD SCENARIO
%  ====================================================================

[scenario, egoVehicle, obstacle] = scenario_builder();

%% ====================================================================
%  2. SET UP VISUALISATION
%  ====================================================================

fig = figure('Name', 'SIH – Adaptive Path Planning', ...
             'NumberTitle', 'off', ...
             'Position', [100, 100, 1200, 500]);

% ---- Subplot 1: Bird's-Eye View (road + vehicles) ----
ax1 = subplot(1, 2, 1);
title(ax1, 'Bird''s-Eye View');
xlabel(ax1, 'X (m)');
ylabel(ax1, 'Y (m)');
hold(ax1, 'on');
axis(ax1, 'equal');
grid(ax1, 'on');
xlim(ax1, [-10, 110]);
ylim(ax1, [-12, 12]);

% Draw road boundaries (y = ±3.5 m)
yline(ax1,  LANE_WIDTH, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'Label', 'Road Edge');
yline(ax1, -LANE_WIDTH, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'Label', 'Road Edge');
yline(ax1, 0, ':', 'Color', [0.8 0.8 0.2], 'LineWidth', 1, 'Label', 'Centre Line');

% Draw obstacle rectangle
obsPos = obstacle.Position(1:2);
obsRect = rectangle(ax1, ...
    'Position', [obsPos(1)-obstacle.Length/2, obsPos(2)-obstacle.Width/2, ...
                 obstacle.Length, obstacle.Width], ...
    'FaceColor', [0.9 0.2 0.2 0.6], ...
    'EdgeColor', 'r', ...
    'LineWidth', 2);
text(ax1, obsPos(1), obsPos(2)+2, 'Obstacle', ...
     'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Ego vehicle marker (will be updated each step)
hEgo = plot(ax1, 0, -LANE_WIDTH/2, 'bs', ...
            'MarkerSize', 12, 'MarkerFaceColor', [0.2 0.5 1], ...
            'DisplayName', 'Ego Vehicle');

% Planned path line (initially empty)
hPath = plot(ax1, NaN, NaN, 'g-', 'LineWidth', 2.5, ...
             'DisplayName', 'Planned Path');

% Ego trail
hTrail = plot(ax1, NaN, NaN, 'b-', 'LineWidth', 1, ...
              'DisplayName', 'Ego Trail');

legend(ax1, [hEgo, hPath, hTrail], 'Location', 'northwest');

% ---- Subplot 2: Occupancy Grid View ----
ax2 = subplot(1, 2, 2);
title(ax2, 'Occupancy Grid');

%% ====================================================================
%  3. SIMULATION LOOP
%  ====================================================================

hasReplanned    = false;     % flag: has a replan been triggered?
replanTime      = -inf;      % time of last replan
egoTrailX       = [];
egoTrailY       = [];
plannedPathXY   = [];
stepCount       = 0;

fprintf('\n[main_sim] Starting simulation loop...\n');

while advance(scenario)
    stepCount = stepCount + 1;
    currentTime = scenario.SimulationTime;

    % ---- Get Current Ego State ----------------------------------------
    egoPose   = egoVehicle.Position;
    egoYawDeg = egoVehicle.Yaw;
    egoYawRad = deg2rad(egoYawDeg);
    egoXY     = egoPose(1:2);

    % Record trail
    egoTrailX(end+1) = egoXY(1);  %#ok<SAGROW>
    egoTrailY(end+1) = egoXY(2);  %#ok<SAGROW>

    % ---- Compute Distance to Obstacle ---------------------------------
    distToObs = norm(egoXY - obsPos);

    % ---- Trigger Replanning -------------------------------------------
    if distToObs <= DETECT_THRESHOLD && ~hasReplanned && ...
       (currentTime - replanTime) > REPLAN_COOLDOWN

        fprintf('\n[main_sim] t=%.2f s | Obstacle detected at %.1f m – REPLANNING.\n', ...
                currentTime, distToObs);

        % Build occupancy grid from current state
        [occGrid, ~] = perception_map(scenario, egoVehicle, obstacle, gridParams);

        % Show occupancy grid in subplot 2
        cla(ax2);
        show(occGrid, 'Parent', ax2);
        title(ax2, sprintf('Occupancy Grid (t = %.2f s)', currentTime));
        hold(ax2, 'on');

        % Goal: 20 m past the obstacle, back in the right lane
        goalPos = [obsPos(1) + 20, -LANE_WIDTH/2];

        % Plan a collision-free path
        [pathXY, pathYaw, planOK] = dynamic_planner(occGrid, egoXY, egoYawRad, goalPos);

        if planOK
            plannedPathXY = pathXY;

            % ---- Update Ego Trajectory with New Waypoints --------------
            % Append a straight segment after the avoidance manoeuvre
            % to continue along the right lane to the road end.
            extendedPath = [
                pathXY;
                100, -LANE_WIDTH/2     % road end, right lane
            ];
            speeds = EGO_SPEED * ones(size(extendedPath, 1), 1);

            % Repath the ego vehicle
            smoothTrajectory(egoVehicle, ...
                [extendedPath, zeros(size(extendedPath,1),1)], speeds);

            fprintf('[main_sim] Ego trajectory updated with %d waypoints.\n', ...
                    size(extendedPath, 1));
        else
            fprintf('[main_sim] WARNING – planner returned no path!\n');
        end

        hasReplanned = true;
        replanTime   = currentTime;
    end

    % ---- Update Visualisation (every 4th step to reduce flicker) ------
    if mod(stepCount, 4) == 0
        % Ego position
        set(hEgo, 'XData', egoXY(1), 'YData', egoXY(2));

        % Ego trail
        set(hTrail, 'XData', egoTrailX, 'YData', egoTrailY);

        % Planned path
        if ~isempty(plannedPathXY)
            set(hPath, 'XData', plannedPathXY(:,1), 'YData', plannedPathXY(:,2));
        end

        % Update title with telemetry
        title(ax1, sprintf('Bird''s-Eye View  |  t=%.2f s  |  v=%.1f m/s  |  d_{obs}=%.1f m', ...
                           currentTime, EGO_SPEED, distToObs));

        drawnow limitrate;
    end
end

%% ====================================================================
%  4. POST-SIMULATION SUMMARY
%  ====================================================================

fprintf('\n=== Simulation Complete ===\n');
fprintf('  Total time     : %.2f s\n', scenario.SimulationTime);
fprintf('  Steps executed : %d\n', stepCount);
fprintf('  Replan triggered: %s\n', string(hasReplanned));

if hasReplanned && ~isempty(plannedPathXY)
    fprintf('  Path waypoints : %d\n', size(plannedPathXY, 1));
    fprintf('  Path length    : %.1f m\n', ...
            sum(vecnorm(diff(plannedPathXY), 2, 2)));
end

% ---- Final static plot ----
figure('Name', 'SIH – Final Path Result', 'NumberTitle', 'off');
hold on; grid on; axis equal;
xlim([-10 110]); ylim([-12 12]);

% Road
fill([-10 110 110 -10], [LANE_WIDTH LANE_WIDTH -LANE_WIDTH -LANE_WIDTH], ...
     [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
yline(0, ':', 'Color', [0.8 0.8 0.2]);

% Obstacle
rectangle('Position', [obsPos(1)-obstacle.Length/2, obsPos(2)-obstacle.Width/2, ...
           obstacle.Length, obstacle.Width], ...
          'FaceColor', [0.9 0.2 0.2 0.7], 'EdgeColor', 'r', 'LineWidth', 2);

% Ego trail
plot(egoTrailX, egoTrailY, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ego Trajectory');

% Planned path
if ~isempty(plannedPathXY)
    plot(plannedPathXY(:,1), plannedPathXY(:,2), 'g--', 'LineWidth', 2, ...
         'DisplayName', 'Planned Avoidance Path');
end

legend('Location', 'northwest');
title('Adaptive Path Planning – Final Result');
xlabel('X (m)'); ylabel('Y (m)');

fprintf('\n[main_sim] Done. Figures remain open for inspection.\n');
