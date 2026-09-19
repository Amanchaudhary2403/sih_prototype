function [occGrid, gridOrigin] = perception_map(scenario, egoVehicle, obstacle, gridParams)
% PERCEPTION_MAP  Build a 2-D occupancy grid from the current scenario state.
%
% This function converts the scene around the ego vehicle into a
% binaryOccupancyMap (Navigation Toolbox) suitable for path planning.
%
% Inputs:
%   scenario    - drivingScenario object (stepped to current time)
%   egoVehicle  - ego vehicle actor handle
%   obstacle    - obstacle actor handle
%   gridParams  - (optional) struct with fields:
%       .width       : grid width  in metres  (default 120)
%       .height      : grid height in metres  (default 20)
%       .resolution  : cells per metre        (default 2)
%       .inflateR    : obstacle inflation radius in metres (default 1.0)
%
% Outputs:
%   occGrid     - binaryOccupancyMap object (1 = occupied, 0 = free)
%   gridOrigin  - [x, y] world-frame origin of the grid (bottom-left)
%
% Design Rationale (see verification_math.md for full derivation):
%   Resolution 2 cells/m → 0.5 m cell size.
%   This balances planning accuracy against compute cost for real-time
%   replanning at 20 Hz.  Inflation radius adds a safety buffer equal
%   to half the ego vehicle width (≈ 0.9 m, rounded to 1.0 m).
%
% Toolbox Requirements:
%   - Navigation Toolbox (R2022b+)
%   - Automated Driving Toolbox
%
% -----------------------------------------------------------------------
% Smart India Hackathon – Adaptive Path Planning POC
% -----------------------------------------------------------------------

    %% ---- 0. Default Grid Parameters -----------------------------------
    if nargin < 4 || isempty(gridParams)
        gridParams = struct();
    end
    gridWidth  = getFieldDefault(gridParams, 'width',      120);   % m
    gridHeight = getFieldDefault(gridParams, 'height',      20);   % m
    resolution = getFieldDefault(gridParams, 'resolution',   2);   % cells/m
    inflateR   = getFieldDefault(gridParams, 'inflateR',   1.0);   % m

    %% ---- 1. Create Empty Occupancy Grid --------------------------------
    % The grid is centred longitudinally on the road (x ∈ [-10, 110]),
    % and spans ±10 m laterally.  The "origin" is the bottom-left corner
    % in world coordinates.

    gridOrigin = [-10, -gridHeight/2];   % world frame [x, y]

    occGrid = binaryOccupancyMap(gridWidth, gridHeight, resolution);
    occGrid.GridLocationInWorld = gridOrigin;

    %% ---- 2. Mark Road Boundaries as Occupied ---------------------------
    % Anything outside the 7 m carriageway (±3.5 m from road centre y=0)
    % is considered non-traversable (fields, ditches, etc.).

    laneWidth     = 3.5;
    roadHalfWidth = laneWidth;            % 2 lanes × 3.5 m / 2 = 3.5 m

    % Build coordinate vectors
    xVec = gridOrigin(1) : (1/resolution) : gridOrigin(1) + gridWidth  - (1/resolution);
    yVec = gridOrigin(2) : (1/resolution) : gridOrigin(2) + gridHeight - (1/resolution);

    % Set cells outside the road as occupied (boundary walls)
    for iy = 1:length(yVec)
        yWorld = yVec(iy) + 0.5/resolution;   % cell centre
        if abs(yWorld) > roadHalfWidth
            for ix = 1:length(xVec)
                xWorld = xVec(ix) + 0.5/resolution;
                setOccupancy(occGrid, [xWorld, yWorld], 1);
            end
        end
    end

    %% ---- 3. Mark Obstacle Footprint ------------------------------------
    % Read obstacle position & dimensions from the actor handle.

    obsPos    = obstacle.Position(1:2);        % [x, y] world
    obsLength = obstacle.Length;                % along x
    obsWidth  = obstacle.Width;                 % along y
    obsYaw    = obstacle.Yaw;                   % degrees

    % For simplicity we approximate the obstacle as an axis-aligned
    % rectangle (valid for small yaw angles typical on straight roads).

    % Obstacle bounding box corners (world frame)
    xMin = obsPos(1) - obsLength/2;
    xMax = obsPos(1) + obsLength/2;
    yMin = obsPos(2) - obsWidth/2;
    yMax = obsPos(2) + obsWidth/2;

    % Fill obstacle cells
    xObs = xMin : (1/resolution) : xMax;
    yObs = yMin : (1/resolution) : yMax;
    for ix = 1:length(xObs)
        for iy = 1:length(yObs)
            setOccupancy(occGrid, [xObs(ix), yObs(iy)], 1);
        end
    end

    %% ---- 4. Inflate Obstacles for Safety Margin ------------------------
    % inflate() grows every occupied cell outward by inflateR metres.
    % This ensures that planned paths keep a safe clearance from the
    % obstacle surface.

    inflate(occGrid, inflateR);

    %% ---- 5. Logging ----------------------------------------------------
    fprintf('[perception_map] Occupancy grid updated.\n');
    fprintf('  Grid size  : %.0f × %.0f m  @ %d cells/m  (%d × %d cells)\n', ...
            gridWidth, gridHeight, resolution, ...
            gridWidth * resolution, gridHeight * resolution);
    fprintf('  Obstacle   : centre [%.1f, %.1f], L=%.1f W=%.1f\n', ...
            obsPos(1), obsPos(2), obsLength, obsWidth);
    fprintf('  Inflation  : %.2f m\n', inflateR);
end

%% ========================================================================
%  HELPER: get field or default
%  ========================================================================
function val = getFieldDefault(s, fieldName, defaultVal)
    if isfield(s, fieldName)
        val = s.(fieldName);
    else
        val = defaultVal;
    end
end
