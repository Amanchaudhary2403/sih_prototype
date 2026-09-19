function [scenario, egoVehicle, obstacle] = scenario_builder()
% SCENARIO_BUILDER  Programmatically create a drivingScenario for
% "Adaptive Path Planning on Unstructured Indian Roads" (SIH POC).
%
% This function builds:
%   1. A 2-lane road with slight curvature (unstructured geometry).
%   2. An ego vehicle starting at the origin, heading east at 10 m/s.
%   3. A static obstacle (stray cattle / broken auto-rickshaw) at ~[50, 0].
%
% Outputs:
%   scenario   - drivingScenario object (Automated Driving Toolbox)
%   egoVehicle - actor handle for the ego car
%   obstacle   - actor handle for the static obstacle
%
% Toolbox Requirements:
%   - Automated Driving Toolbox (R2022b+)
%
% -----------------------------------------------------------------------
% Smart India Hackathon – Adaptive Path Planning POC
% Author: Auto-generated for SIH Demo
% -----------------------------------------------------------------------

    %% ---- 1. Create Scenario Object ------------------------------------
    % SampleTime controls the simulation tick.  0.05 s gives 20 Hz, which
    % is a common sensor refresh rate on autonomous vehicles.
    scenario = drivingScenario('SampleTime', 0.05, 'StopTime', 30);

    %% ---- 2. Define Road Geometry (Unstructured, 2-Lane) ----------------
    % We model an Indian village road as a gently curving 2-lane path.
    % Road centers are defined as a series of waypoints; the toolbox
    % interpolates a smooth clothoid spline through them.
    %
    % Lane width = 3.5 m (typical Indian state highway lane).
    % Two lanes  → total carriageway ≈ 7 m.

    roadCenters = [
        0    0    0;   % Start – origin
       20    0.5  0;   % Slight drift (unstructured alignment)
       40    0    0;   % Road curves back
       60   -0.5  0;   % Mild S-curve continues
       80    0    0;   % Road straightens
      100    0    0    % End
    ];

    laneWidth = 3.5;                     % metres per lane
    numLanes = 2;                         % 2-lane undivided road

    % lanespec(numLanes) creates lane markings automatically.
    laneSpecObj = lanespec(numLanes, 'Width', laneWidth);

    road(scenario, roadCenters, 'Lanes', laneSpecObj, 'Name', 'VillageRoad');

    %% ---- 3. Add Ego Vehicle -------------------------------------------
    % The ego car is a standard sedan (4.7 m × 1.8 m × 1.4 m).
    % Initial position  : [0, 0, 0]  (road start, centre of right lane)
    % Initial speed     : 10 m/s  (~36 km/h, common village speed)
    % Initial yaw       : 0°      (heading east along the road)

    egoVehicle = vehicle(scenario, ...
        'ClassID',  1, ...                % 1 = Car
        'Length',   4.7, ...
        'Width',    1.8, ...
        'Height',   1.4, ...
        'Name',     'EgoCar');

    % Place ego in the RIGHT lane (offset = -laneWidth/2 from road centre).
    egoStartPos = [0, -laneWidth/2, 0];

    % Waypoints define the default (unperturbed) trajectory.
    % The ego simply drives straight along the right lane initially.
    egoWaypoints = [
        egoStartPos(1:2);
       100, -laneWidth/2
    ];
    egoSpeed = [10; 10];   % constant 10 m/s

    % smoothTrajectory generates a jerk-minimised path for the actor.
    smoothTrajectory(egoVehicle, ...
        [egoWaypoints, zeros(size(egoWaypoints,1),1)], egoSpeed);

    %% ---- 4. Add Static Obstacle ----------------------------------------
    % Represents a stray cow or a broken-down auto-rickshaw sitting in the
    % ego's lane.  Dimensions ≈ 2.5 m × 1.5 m × 1.5 m (auto-rickshaw).
    %
    % Position: [50, -laneWidth/2, 0]  (right lane, 50 m ahead of ego).

    obstacle = vehicle(scenario, ...
        'ClassID',  2, ...                % 2 = Truck/obstacle
        'Length',   2.5, ...
        'Width',    1.5, ...
        'Height',   1.5, ...
        'Name',     'Obstacle');

    obstaclePos = [50, -laneWidth/2, 0];

    % Static obstacle: waypoints = single point, speed = 0.
    smoothTrajectory(obstacle, [obstaclePos; obstaclePos(1)+0.01, obstaclePos(2:3)], [0; 0]);

    %% ---- 5. Summary Print ---------------------------------------------
    fprintf('[scenario_builder] Scenario created.\n');
    fprintf('  Road       : %d waypoints, %d lanes × %.1f m\n', ...
            size(roadCenters,1), numLanes, laneWidth);
    fprintf('  Ego start  : [%.1f, %.1f] @ %.0f m/s\n', ...
            egoStartPos(1), egoStartPos(2), egoSpeed(1));
    fprintf('  Obstacle   : [%.1f, %.1f] (static)\n', ...
            obstaclePos(1), obstaclePos(2));
end
