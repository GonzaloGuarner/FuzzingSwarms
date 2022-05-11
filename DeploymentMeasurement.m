%% DeploymentMeasurement.m is an extension to DeploymentSimulation, it's main 
% goal being measuring performance across several simulations or 'runs'. 
% Support to sensor noise and agent failure is also implemented.
% Last modified on 08/07/2021


%% Create and configure a multi-robot environment
% Set simulation parameters
totalIterations = 300;
totalRuns = 25;
R_c = 8; % Agent communication radius (distance unit)
R_s = 3; % Agent surveillance/sensing radius (distance unit)
maxSpeed = 0.3;% (distance/iteration)
calculateAreaOverTime = true; % The simulation returns area over time if true
plotAreaOverTime = true; % Generate an AoT plot at the end of the simulation
showVisuals = false; % Enable this for a visual animation of the env.


% Noise Parameters
simulateNoise = false; % Generate noise in inter-robot distance detections
isRandomNoise = true;
noiseIntensity = 0.5;

% Failure Parameters
simulateFailure = false;
failureStartIt = 100;
failureInterval = 10;

% Set environment parameters
numRobots = 30;
env = MultiRobotEnv(numRobots);
env.showTrajectory = false;
env.robotRadius = 0.08*R_s; % Visual aspect of the agents
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals
env.showRobotIds = false; % Idem with robot IDs

% Set our custom map (a bin. occupancy grid) from a csv file
mapMatrix = table2array(readtable('Arenas\ArenaConvexThick.csv'));
env.hasCustomMap = true;
env.customMap = occupancyMap(mapMatrix);

% Obtain the map's traversable (free) grid cells. We will use these on our 
% area coverage calculations
freeCells = getFreeCells(mapMatrix); 

% Obtain Fuzzy Inference System. Our agents' controllers
swarmAgentsFIS = readfis('FuzzyInferenceSystems\Compound\Main\newTFLC.fis');
swarmAgentsObst = readfis('FuzzyInferenceSystems\ObstAvoidance\FuzzyAgentsObst3.fis');

%% Initialization
% Create and configure robot detectors for all robots
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = R_c;
    detector.fieldOfView = 2*pi;
    detectors{rIdx} = detector;
end
map = env.customMap; % Necessary variable for some helper methods.

% Data matrix containing each run's area over time values. Used when
% calculating the mean and standard deviation.
areaOverTimeRuns = zeros(totalIterations,totalRuns);


for runIdx = 1:totalRuns
    % Run parameter re-initialization
    % The simulation's main output. The area covered by the swarm each iteration
    areaTime = zeros(totalIterations,1);
    
    % Initialize robots at random positions
    poses = rand(3,numRobots).*[4;4;pi]+[14;21;0];
    env.Poses = poses;

    % Velocity and target directions for each robot after an iteration
    vel = zeros(3,numRobots); % Velocity in x,y,w(angular) axis
    dirDisp = zeros(2,numRobots); % Target dispersion direction
    dirObst = zeros(2,numRobots); % Target obst. avoidance direction
    dirTotal = zeros(2,numRobots); % Final target direction
    
    % Failure run initialization
    failedRobots = zeros(1,numRobots);
    failedRobotIdx = 0;
    

    %% Main loop 
    % Each iteration, animate agents, get sensor readings and obtain target
    % translations from agent-to-agent dispersion and obstacle avoidance
    
    tic % For timing each run's duration
    for idx = 1:totalIterations %(time units)
    
        % If simulating failure, check if a new robot has failed
        if simulateFailure == true && mod(idx,failureInterval) == 0 && idx >= failureStartIt
            failedRobotIdx = failedRobotIdx + 1;
            failedRobots(1,failedRobotIdx) = 1;        
        end

        % Get sensor readings and infer a target direction for dispersion and
        % obstacle avoidance
        for rIdx = 1:numRobots
           % Skip robot if it failed
           if simulateFailure  && failedRobots(rIdx) ==  1
                continue;
           end

           sensorPosition = poses(:,rIdx);
            
           %Obstacle Detections and obtained target direction
           scans = ObstSensorGrid(mapMatrix,sensorPosition);
           dirObst(:,rIdx) = ObstAvoidanceGoalDirection(scans,swarmAgentsObst);
           
           %Robot Detections and obtained target direction
           detections = step(detectors{rIdx});

           if simulateNoise && size(detections,1) > 0
                detections(:,1) = DetectionNoise(isRandomNoise,noiseIntensity,detections(:,1),R_c);
           end

           dirDisp(:,rIdx) = DispersionGoalDirection(map,mapMatrix,sensorPosition,detections,scans,swarmAgentsFIS);             
        end
    
        % Loop where we add the target directions, clamp to the maximum speed
        % (normally 0.3) convert into velocity.
        for rIdx = 1:numRobots 
           % Skip robot if it failed
           if simulateFailure  && failedRobots(rIdx) ==  1
               vel(:,rIdx) = [0;0;0];
               continue;
           end
           dirTotal(:,rIdx) = dirDisp(:,rIdx) + dirObst(:,rIdx);
           angle = atan2(dirTotal(2,rIdx),dirTotal(1,rIdx));
           speed = min(maxSpeed,norm(dirTotal(:,rIdx)));
           vel(:,rIdx) = bodyToWorld([speed; 0; angle],poses(:,rIdx));
        end
        poses= poses + vel;
        
        % Update the environment
        if showVisuals
            env(1:numRobots, poses); % Use this for a visual animation of the env.
        else
            env.Poses = poses; % Alternatively use this for a faster simulation
        end 
        
        xlim([11 36]);   % Without this, axis resizing can slow things down
        ylim([11 36]); 
        
        if calculateAreaOverTime
         areaTime(idx) = circlesSmart(poses(1,:),poses(2,:),R_s*ones(1,numRobots),mapMatrix,freeCells);
        end  
    
    end

    if calculateAreaOverTime
    areaOverTimeRuns(:,runIdx) = areaTime;
    end
    toc
end

if plotAreaOverTime
    x = linspace(1,totalIterations,totalIterations);
    plot(x,areaTime)%
    xlabel('Number of Iterations');
    ylabel('Area Covered (%)');
end

if calculateAreaOverTime
    meanAreaOverTime = mean(areaOverTimeRuns(end,:))
    stdAoT = std(areaOverTimeRuns(end,:))
end

%% Helper functions: 
% Robot Dispersion Controller Logic
function dir = DispersionGoalDirection(map,mapMatrix,sensorPosition,detections,scans,inferenceSystem)
    nNeighbours = 3;
    
    psum = [0;0];
    if size(detections,1) < nNeighbours
        nNeighbours = size(detections,1);
    end
    
    if ~isempty(scans)
        dobst = scans(1);
    else
        dobst = 1.4;
    end
    if ~isempty(detections)        
            
            for i = 1:nNeighbours
                d = detections(i,1);
                angle = detections(i,2);
                distX = cos(angle);
                distY = sin(angle);
                neighbourDir = [distX;distY];
                if (ValidateDetection(map,mapMatrix,d,sensorPosition,neighbourDir))
                    if size(detections,1) > nNeighbours
                        nNeighbours = nNeighbours + 1;
                    end
                    continue;
                end
                weight = evalfis(inferenceSystem,[d, dobst]);
                psum = psum + weight*neighbourDir;
            end
    end
    dir = psum;
end

% Robot Obstacle Avoidance Controller Logic
function dir = ObstAvoidanceGoalDirection(detections,inferenceSystem)
    obstDetected = 3; % Max amount of obstacles
    psum = [0;0];

    if size(detections,1) < obstDetected
        obstDetected = size(detections,1);
    end

    if ~isempty(detections)
        
            % Take the obstDetected and drive away the robot from
            % the sum of their positions (in the robot's local coordinates)
            ranges = detections(1:obstDetected,1);
            angles = detections(1:obstDetected,2);
            for i = 1:obstDetected
                d = ranges(i);
                distX = cos(angles(i));
                distY = sin(angles(i));
                obstPos = [distX;distY];

                weight = evalfis(inferenceSystem,d);
                psum = psum - weight*obstPos;
            end
    end
    dir = psum;
end

% Returns the obstacle detections, which are the result of a ray cast
% collision with a map obstacle
function scans = ObstSensorGrid(mapMatrix,sensorPosition)
    preScans = nan(8,2);
    objIdx = 0;
    
    scanAngles = [3*pi/4,pi/2,pi/4,pi,0,-3*pi/4,-pi/2,-pi/4];
    for di = -1:1
        for dj = -1:1
            if (dj == 0 && di == 0)
                continue
            end
            objIdx = objIdx + 1;
            Imatrix = size(mapMatrix,1) - floor(sensorPosition(2));
            Jmatrix = floor(sensorPosition(1))+1;
            if (mapMatrix(Imatrix + di,Jmatrix + dj) == 1)
                offsetX = 0;
                offsetY = 0;
                if (di == 1)
                    offsetX = abs(ceil(sensorPosition(1)) - sensorPosition(1));
                else
                    if (di == -1)
                    offsetX = sensorPosition(1) - floor(sensorPosition(1));
                    end
                end
                
                if (dj == 1)
                    offsetY = abs(ceil(sensorPosition(2)) - sensorPosition(2));
                else
                    if (dj == -1)
                    offsetY = sensorPosition(2) - floor(sensorPosition(2));
                    end
                end 
                range = sqrt(offsetX^2 + offsetY^2);
                if range <= 1
                preScans(objIdx,1) = range;
                end
            end
        end
    end
    preScans(:,2) = scanAngles - sensorPosition(3);
    preScans(any(isnan(preScans), 2), :) = [];
    scans = sortrows(preScans);
end

% Boolean that is true if a ray (defined by a starting position, a
% direction and a range) intersects with a map obstacle
function isBlockedRay = ValidateDetection(map,mapMatrix,range,startPosition,rayDir)
        isBlockedRay = false;

        startPoint2D = [startPosition(1),startPosition(2)];
        newDir = bodyToWorld([rayDir; 0],startPosition);
        endPoint = startPoint2D + range*[newDir(1),newDir(2)];
         rangeIsMax = false(2,1);
         
        [endPoints, middlePoints] = ...
             nav.algs.internal.mex.raycastCells(startPoint2D, endPoint, map.GridSize(1), ...
             map.GridSize(2), map.Resolution, map.GridLocationInWorld,rangeIsMax);
         
         for i = 1:size(middlePoints,1)
            if(mapMatrix(middlePoints(1,1),middlePoints(1,2)) == 1)
                isBlockedRay = true;
                return;
            end
         end
end

% Returns the percentage of map points inside the radius of any agent
function res = circlesSmart(xc,yc,r,mapMatrix,freeCells)
    pointsPerCellSide = 10; % The higher the values the higher the precision
    totalFreeCells = length(freeCells); 
    r2 = r .* r; 
    inside = 0;

    for i = 1:totalFreeCells
        % Conversion from matrix coordinates to world coordinates
        xmin = freeCells(2,i) - 1;
        xmax = xmin + 1;
        ymin = size(mapMatrix,1) - freeCells(1,i);
        ymax = ymin + 1;

        for x = linspace(xmin,xmax,pointsPerCellSide)
            for y = linspace(ymin,ymax,pointsPerCellSide)
                if any(r2 > (x - xc).^2 + (y - yc).^2)
                    inside = inside + 1;
                end
            end
        end
    end
    
    % Consider caching the constant
    res = inside / (totalFreeCells*pointsPerCellSide^2)*100;
 
end

% Returns a 2D matrix traversable (free) grid cells of the input map matrix
function freeCells = getFreeCells(mapMatrix)
    freeCells = int16.empty;
    for i= 1:size(mapMatrix,1)
        for j=1:size(mapMatrix,2)
            if mapMatrix(i,j) == 0 
                if ~isempty(freeCells)
                    freeCells = [freeCells(1,:),i;freeCells(2,:),j];
                else
                    freeCells = [i;j];
                end
            end
        end
    end
end

function detectionRange = DetectionNoise(isRandom,deviation,detectionRange,maxRange)
    
    if isRandom
        error = deviation*randn;
    else 
        error = 0.1*deviation;
    end

    detectionRange = detectionRange + error;

    for i = 1:size(detectionRange,1)
        if detectionRange(i) > maxRange
            detectionRange(i) = maxRange;
        else
             if detectionRange(i) < 0
                detectionRange(i) = 0;
             end
        end
    end
end