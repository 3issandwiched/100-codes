#Abhishek Chatnihal
#use below copy command in your matlab environment to start.
#openExample('driving/ParkingValetExample')
mapLayers = loadParkingLotMapLayers;
plotMapLayers(mapLayers)


costmap = combineMapLayers(mapLayers);

figure
plot(costmap, 'Inflation', 'off')
legend off


costmap.MapExtent % [x, width, y, height] in meters

costmap.CellSize  % cell size in meters


vehicleDims      = vehicleDimensions;
maxSteeringAngle = 35; % in degrees


costmap.CollisionChecker.VehicleDimensions = vehicleDims;

currentPose = [4 12 0]; % [x, y, theta]

data = load('routePlan.mat');
routePlan = data.routePlan %#ok<NOPTS>

hold on
helperPlotVehicle(currentPose, vehicleDims, 'DisplayName', 'Current Pose')
legend

for n = 1 : height(routePlan)
    vehiclePose = routePlan{n, 'EndPose'};
    
    legendEntry = sprintf('Goal %i', n);
    helperPlotVehicle(vehiclePose, vehicleDims, 'DisplayName', legendEntry);
end
hold off

behavioralPlanner = HelperBehavioralPlanner(routePlan, maxSteeringAngle);

motionPlanner = pathPlannerRRT(costmap, 'MinIterations', 1000, ...
    'ConnectionDistance', 10, 'MinTurningRadius', 20);

goalPose = routePlan{1, 'EndPose'};
refPath = plan(motionPlanner, currentPose, goalPose);

refPath.PathSegments

[transitionPoses, directions] = interpolate(refPath);

plot(motionPlanner)


approxSeparation = 0.1; % meters
numSmoothPoses   = round(refPath.Length / approxSeparation);

% Return discretized poses along the smooth path.
[refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);

% Plot the smoothed path.
hold on
hSmoothPath = plot(refPoses(:, 1), refPoses(:, 2), 'r', 'LineWidth', 2, ...
    'DisplayName', 'Smoothed Path');
hold off


maxSpeed   = 5; % in meters/second
startSpeed = 0; % in meters/second
endSpeed   = 0; % in meters/second

%%
% Generate a velocity profile.
refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, startSpeed, endSpeed, maxSpeed);


plotVelocityProfile(cumLengths, refVelocities, maxSpeed)



% Close all the figures.
closeFigures;

% Create the vehicle simulator.
vehicleSim = HelperVehicleSimulator(costmap, vehicleDims);

% Set the vehicle pose and velocity.
vehicleSim.setVehiclePose(currentPose);
currentVel = 0;
vehicleSim.setVehicleVelocity(currentVel);

% Configure the simulator to show the trajectory.
vehicleSim.showTrajectory(true);

% Hide vehicle simulation figure.
hideFigure(vehicleSim);


pathAnalyzer = HelperPathAnalyzer(refPoses, refVelocities, directions, ...
    'Wheelbase', vehicleDims.Wheelbase);

sampleTime = 0.05;
lonController = HelperLongitudinalController('SampleTime', sampleTime);

controlRate = HelperFixedRate(1/sampleTime); % in Hertz

reachGoal = false;

while ~reachGoal    
    % Find the reference pose on the path and the corresponding velocity.
    [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
    
    % Update driving direction for the simulator.
    updateDrivingDirection(vehicleSim, direction);
    
    % Compute steering command.
    steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
        'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);
    
    % Compute acceleration and deceleration commands.
    lonController.Direction = direction;
    [accelCmd, decelCmd] = lonController(refVel, currentVel);
    
    % Simulate the vehicle using the controller outputs.
    drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
    
    % Check if the vehicle reaches the goal.
    reachGoal = helperGoalChecker(goalPose, currentPose, currentVel, endSpeed, direction);
    
    % Wait for fixed-rate execution.
    waitfor(controlRate);
    
    % Get current pose and velocity of the vehicle.
    currentPose  = getVehiclePose(vehicleSim);
    currentVel   = getVehicleVelocity(vehicleSim);
end

% Show vehicle simulation figure.
showFigure(vehicleSim);

currentPose = [4 12 0]; % [x, y, theta]
vehicleSim.setVehiclePose(currentPose);

% Reset velocity.
currentVel  = 0; % meters/second
vehicleSim.setVehicleVelocity(currentVel);

while ~reachedDestination(behavioralPlanner)
    
    % Request next maneuver from behavioral layer.
    [nextGoal, plannerConfig, speedConfig] = requestManeuver(behavioralPlanner, ...
        currentPose, currentVel);
    
    % Configure the motion planner.
    configurePlanner(motionPlanner, plannerConfig);
    
    % Plan a reference path using RRT* planner to the next goal pose.
    refPath = plan(motionPlanner, currentPose, nextGoal);
    
    % Check if the path is valid. If the planner fails to compute a path,
    % or the path is not collision-free because of updates to the map, the
    % system needs to re-plan. This scenario uses a static map, so the path
    % will always be collision-free.
    isReplanNeeded = ~checkPathValidity(refPath, costmap);
    if isReplanNeeded
        warning('Unable to find a valid path. Attempting to re-plan.')
        
        % Request behavioral planner to re-plan
        replanNeeded(behavioralPlanner);
        continue;
    end
    
    % Retrieve transition poses and directions from the planned path.
    [transitionPoses, directions] = interpolate(refPath);
     
    % Smooth the path.
    numSmoothPoses   = round(refPath.Length / approxSeparation);
    [refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);
    
    % Generate a velocity profile.
    refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, startSpeed, endSpeed, maxSpeed);
    
    % Configure path analyzer.
    pathAnalyzer.RefPoses     = refPoses;
    pathAnalyzer.Directions   = directions;
    pathAnalyzer.VelocityProfile = refVelocities;
    
    % Reset longitudinal controller.
    reset(lonController);
    
    reachGoal = false;
    
    % Execute control loop.
    while ~reachGoal  
        % Find the reference pose on the path and the corresponding
        % velocity.
        [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
        
        % Update driving direction for the simulator.
        updateDrivingDirection(vehicleSim, direction);
        
        % Compute steering command.
        steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
            'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);
        
        % Compute acceleration and deceleration commands.
        lonController.Direction = direction;
        [accelCmd, decelCmd] = lonController(refVel, currentVel);
        
        % Simulate the vehicle using the controller outputs.
        drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
        
        % Check if the vehicle reaches the goal.
        reachGoal = helperGoalChecker(nextGoal, currentPose, currentVel, speedConfig.EndSpeed, direction);
        
        % Wait for fixed-rate execution.
        waitfor(controlRate);
        
        % Get current pose and velocity of the vehicle.
        currentPose  = getVehiclePose(vehicleSim);
        currentVel   = getVehicleVelocity(vehicleSim);
    end
end

% Show vehicle simulation figure.
showFigure(vehicleSim);

hideFigure(vehicleSim);

%%
% The |vehicleCostmap| uses inflation-based collision checking. First,
% visually inspect the current collision checker in use.
ccConfig = costmap.CollisionChecker;

figure
plot(ccConfig)
title('Current Collision Checker')


ccConfig.NumCircles = 4;

figure
plot(ccConfig)
title('New Collision Checker')

%%
% Update the costmap to use this collision checker. 
costmap.CollisionChecker = ccConfig;

%%
% Notice that the inflation radius has reduced, allowing the planner to
% find an unobstructed path to the parking spot.
figure
plot(costmap)
title('Costmap with updated collision checker')

parkMotionPlanner = pathPlannerRRT(costmap, 'MinIterations', 1000);

parkPose = [36 44 90];
preParkPose = currentPose;

refPath = plan(parkMotionPlanner, preParkPose, parkPose);

figure
plotParkingManeuver(costmap, refPath, preParkPose, parkPose)

[transitionPoses, directions] = interpolate(refPath);

% Smooth the path.
numSmoothPoses   = round(refPath.Length / approxSeparation);
[refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);

refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, currentVel, 0, 2.2352);

pathAnalyzer.RefPoses     = refPoses;
pathAnalyzer.Directions   = directions;
pathAnalyzer.VelocityProfile = refVelocities;

% Reset longitudinal controller.
reset(lonController);

reachGoal = false;

while ~reachGoal 
    % Find the reference pose on the path and the corresponding velocity.
    [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
    
    % Update driving direction for the simulator.
    updateDrivingDirection(vehicleSim, direction);
    
    % Compute steering command.
    steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
        'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);
    
    % Compute acceleration and deceleration commands.
    lonController.Direction = direction;
    [accelCmd, decelCmd] = lonController(refVel, currentVel);
    
    % Simulate the vehicle using the controller outputs.
    drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
    
    % Check if the vehicle reaches the goal.
    reachGoal = helperGoalChecker(parkPose, currentPose, currentVel, 0, direction);
    
    % Wait for fixed-rate execution.
    waitfor(controlRate);
    
    % Get current pose and velocity of the vehicle.
    currentPose  = getVehiclePose(vehicleSim);
    currentVel   = getVehicleVelocity(vehicleSim);
end

% Show vehicle simulation figure.
closeFigures;
showFigure(vehicleSim);

parkPose = [49 47.2 -90];

% Change the connection method to allow for reverse motions.
parkMotionPlanner.ConnectionMethod = 'Reeds-Shepp';

parkMotionPlanner.MinTurningRadius   = 10; % meters
parkMotionPlanner.ConnectionDistance = 15;

% Reset vehicle pose and velocity.
currentVel = 0;
vehicleSim.setVehiclePose(preParkPose);
vehicleSim.setVehicleVelocity(currentVel);

% Compute the parking maneuver.
replan = true;
while replan
    refPath = plan(parkMotionPlanner, preParkPose, parkPose);
  
    numSamples = 10;
    stepSize   = refPath.Length / numSamples;
    lengths    = 0 : stepSize : refPath.Length;
    
    [transitionPoses, directions] = interpolate(refPath, lengths);

    % Replan if the path contains more than one direction switching poses
    % or if the path is too long.
    replan = sum(abs(diff(directions)))~=2 || refPath.Length > 20;
end

% Visualize the parking maneuver.
figure
plotParkingManeuver(costmap, refPath, preParkPose, parkPose)

numSmoothPoses   = round(refPath.Length / approxSeparation);
[refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses, 0.5);
    
refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, currentVel, 0, 1);

pathAnalyzer.RefPoses     = refPoses;
pathAnalyzer.Directions   = directions;
pathAnalyzer.VelocityProfile = refVelocities;

reset(lonController);

reachGoal = false;

while ~reachGoal    
    currentDir = getDrivingDirection(vehicleSim);
    
    [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
    
    if currentDir ~= direction
        currentVel = 0;
        setVehicleVelocity(vehicleSim, currentVel);
        reset(lonController);
    end
  
    currentVel = updateDrivingDirection(vehicleSim, direction, currentDir);
    
    % Compute steering command.
    steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
        'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);
    
    % Compute acceleration and deceleration commands.
    lonController.Direction = direction;
    [accelCmd, decelCmd] = lonController(refVel, currentVel);
    
    % Simulate the vehicle using the controller outputs.
    drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
    
    % Check if the vehicle reaches the goal.
    reachGoal = helperGoalChecker(parkPose, currentPose, currentVel, 0, direction);
    
    % Wait for fixed-rate execution.
    waitfor(controlRate);
    
    % Get current pose and velocity of the vehicle.
    currentPose  = getVehiclePose(vehicleSim);
    currentVel   = getVehicleVelocity(vehicleSim);
end

% Take a snapshot for the example.
closeFigures;
snapnow;

% Delete the simulator.
delete(vehicleSim);

function mapLayers = loadParkingLotMapLayers()


mapLayers.StationaryObstacles = imread('stationary.bmp');
mapLayers.RoadMarkings        = imread('road_markings.bmp');
mapLayers.ParkedCars          = imread('parked_cars.bmp');
end

function plotMapLayers(mapLayers)
figure
cellOfMaps = cellfun(@imcomplement, struct2cell(mapLayers), 'UniformOutput', false);
montage( cellOfMaps, 'Size', [1 numel(cellOfMaps)], 'Border', [5 5], 'ThumbnailSize', [300 NaN] )
title('Map Layers - Stationary Obstacles, Road markings, and Parked Cars')
end

function costmap = combineMapLayers(mapLayers)

combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
    mapLayers.ParkedCars;
combinedMap = im2single(combinedMap);

res = 0.5; % meters
costmap = vehicleCostmap(combinedMap, 'CellSize', res);
end
function configurePlanner(pathPlanner, config)

fieldNames = fields(config);
for n = 1 : numel(fieldNames)
    if ~strcmpi(fieldNames{n}, 'IsParkManeuver')
        pathPlanner.(fieldNames{n}) = config.(fieldNames{n});
    end
end
end

function plotVelocityProfile(cumPathLength, refVelocities, maxSpeed)

% Plot reference velocity along length of the path.
plot(cumPathLength, refVelocities, 'LineWidth', 2);

% Plot a line to display maximum speed.
hold on
line([0;cumPathLength(end)], [maxSpeed;maxSpeed], 'Color', 'r')
hold off

% Set axes limits.
buffer = 2;
xlim([0 cumPathLength(end)]);
ylim([0 maxSpeed + buffer])

% Add labels.
xlabel('Cumulative Path Length (m)');
ylabel('Velocity (m/s)');

% Add legend and title.
legend('Velocity Profile', 'Max Speed')
title('Generated velocity profile')
end

%%%
% *closeFigures*
function closeFigures()
% Close all the figures except the simulator visualization.

% Find all the figure objects.
figHandles = findobj('Type', 'figure');
for i = 1: length(figHandles)
    if ~strcmp(figHandles(i).Name, 'Automated Valet Parking')
        close(figHandles(i));
    end
end
end

%%%
% *plotParkingManeuver*
% Display the generated parking maneuver on a costmap.
function plotParkingManeuver(costmap, refPath, currentPose, parkPose)
%plotParkingManeuver
% Plot the generated parking maneuver on a costmap.

% Plot the costmap, without inflated areas.
plot(costmap, 'Inflation', 'off')

% Plot reference parking maneuver on the costmap.
hold on
plot(refPath, 'DisplayName', 'Parking Maneuver')

title('Parking Maneuver')

% Zoom into parking maneuver by setting axes limits.
lo = min([currentPose(1:2); parkPose(1:2)]);
hi = max([currentPose(1:2); parkPose(1:2)]);

buffer = 6; % meters

xlim([lo(1)-buffer hi(1)+buffer])
ylim([lo(2)-buffer hi(2)+buffer])
end
