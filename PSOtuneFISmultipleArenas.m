%% Algorithm Fuzzy Logic
% 
%% Last modified on 08/07/2021



%% Create and configure a multi-robot environment
% Set environment parameters
numRobots = 30;
env = MultiRobotEnv(numRobots);
env.showTrajectory = false;
env.robotRadius = R_s; % Visual aspect of the agents
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals
env.showRobotIds = false; % Idem with robot IDs
env.hasCustomMap = true;

% Set our custom map (a bin. occupancy grid) from a csv file
mapMatrixArray{1} = table2array(readtable('Arenas\ArenaSimpleThick.csv'));
mapMatrixArray{2} = table2array(readtable('Arenas\ArenaConvexThick.csv'));
mapMatrixArray{3} = table2array(readtable('Arenas\ArenaCorridorTweakThick.csv'));
mapMatrixArray{4} = table2array(readtable('Arenas\ArenaComplexThick.csv'));

% Obtain the map's traversable (free) grid cells. We will use these on our 
% area coverage calculations
freeCells1 = getFreeCells(mapMatrixArray{1});
freeCells2 = getFreeCells(mapMatrixArray{2});
freeCells3 = getFreeCells(mapMatrixArray{3});
freeCells4 = getFreeCells(mapMatrixArray{4});


%% Initialization
% Create and configure robot detectors for all robots
R_c = 8; % Agent communication radius (distance unit)
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = R_c;
    detector.fieldOfView = 2*pi;
    detectors{rIdx} = detector;
end

%% Optimization Process 
% Parameters. Particle Swarm Optimization is used
options = tunefisOptions('Method','particleswarm','OptimizationType','tuning');
options.MethodOptions.Display = 'iter';
options.MethodOptions.MaxIterations = 80;
options.MethodOptions.SwarmSize = 50;
warning('off','fuzzy:general:warnEvalfis_NoRuleFired'); % Increases performance
rng('default');


% Obtain Fuzzy Inference Systems. Our agents' controllers
fisin = readfis('FuzzyInferenceSystems\Compound\Main\newLFLC.fis'); % To be optimized
fisObst = readfis('FuzzyInferenceSystems\ObstAvoidance\FuzzyAgentsObst3.fis');

[in,out] = getTunableSettings(fisin);
out = setTunable(out,false);

% Fitness function used in the PSO
costFunction = @(fis)simulationCost(fis,fisObst,detectors,mapMatrixArray, ...
    freeCells1,freeCells2,freeCells3,freeCells4,env);

runtunefis = true; % Run the FIS optimization process?

% Run the optimization process
tStart = tic; % Timing the optimization process
if runtunefis
    fisout = tunefis(fisin,[in;out],costFunction,options); 
else
   tunedfis = load('tunedfisnavigation.mat'); 
   fisout = tunedfis.fisout;
end
tEnd = toc(tStart)

%% Plots
% Plots the rules of the output FIS
fisout.Rules 

% Generate a surface with the output of the FIS in terms of the input
figure
opt = gensurfOptions;
opt.NumGridPoints = 40;
gensurf(fisout,opt)
title('Output surface of fisout')
xlim([0 8.5]);
ylim([0 1.5]);
            
    
%% Main loop 
% Each iteration, get sensor readings and obtain target
% translations from agent-to-agent dispersion and obstacle avoidance
function cost = simulationCost(fisAgents,fisObst,detectors,mapMatrixArray, ...
    freeCells1,freeCells2,freeCells3,freeCells4,env)

    tic
    
    % Set simulation parameters
    R_s = 3; % Agent surveillance/sensing radius (distance unit)
    maxSpeed = 0.3;% (distance/iteration)
    totalIterations = 200;

    costArena = zeros(4,1);
    
    for arenaIdx = 1:4
        sumCost = 0;
    
        numRobots = env.numRobots;
        poses = rand(3,numRobots).*[4;4;pi]+[14;21;0];
        env.Poses = poses;
    
        mapMatrix = mapMatrixArray{arenaIdx};
        env.customMap = occupancyMap(mapMatrix);
        map = env.customMap; % Necessary variable for some helper methods.
        
        if arenaIdx == 1
            freeCells = freeCells1;
        else
            if arenaIdx == 2
               freeCells = freeCells2; 
            else
                if arenaIdx == 3
                    freeCells = freeCells3; 
                else 
                    freeCells = freeCells4; 
                end
            end
        end

        % Velocity and target directions for each robot after an iteration
        vel = zeros(3,numRobots); % Velocity in x,y,w(angular) axis
        dirDisp = zeros(2,numRobots); % Target dispersion direction
        dirObst = zeros(2,numRobots); % Target obst. avoidance direction
        dirTotal = zeros(2,numRobots); % Final target direction

        for idx = 1:totalIterations %(time units)
    
            % Get sensor readings and infer a target direction for dispersion and
            % obstacle avoidance
            for rIdx = 1:numRobots    
               sensorPosition = poses(:,rIdx);
                
               %Obstacle Detections and obtained target direction
               scans = ObstSensorGrid(mapMatrix,sensorPosition);
               dirObst(:,rIdx) = ObstAvoidanceGoalDirection(scans,fisObst);
               
               %Robot Detections and obtained target direction
               detections = step(detectors{rIdx});      
               dirDisp(:,rIdx) = DispersionGoalDirection(map,mapMatrix,sensorPosition,detections,scans,fisAgents);             
            end
        
            % Loop where we add the target directions, clamp to the maximum speed
            % (normally 0.3) convert into velocity.
            for rIdx = 1:numRobots 
               dirTotal(:,rIdx) = dirDisp(:,rIdx) + dirObst(:,rIdx);
               angle = atan2(dirTotal(2,rIdx),dirTotal(1,rIdx));
               speed = min(maxSpeed,norm(dirTotal(:,rIdx)));
               vel(:,rIdx) = bodyToWorld([speed; 0; angle],poses(:,rIdx));
            end

            poses = poses + vel;
            
            % Update the environment
            env.Poses = poses; % Use this for a faster simulation
            %env(1:numRobots, poses);% Alternatively use this for a visual animation
            sumCost = sumCost + idx*circlesSmart(poses(1,:),poses(2,:),R_s*ones(1,numRobots),mapMatrix,freeCells);    

        end
    collisions = CheckObstacles(mapMatrix,numRobots,poses); 
    costArena(arenaIdx) = (collisions+1)*length(freeCells)*totalIterations*(totalIterations+1)/(2*sumCost); % 2/(T*(T+1)) * sum_i(t_i * area_i)
    end
    costArena % Console log to review the cost for each arena
    cost = sum(costArena);
    toc
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

function numObstacles = CheckObstacles(mapMatrix,numRobots,poses)
    numObstacles = 0;
    for i = 1:numRobots
        a = mapMatrix(size(mapMatrix,1) - floor(poses(2,i)),floor(poses(1,i))+1);
        numObstacles = numObstacles + a;
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
