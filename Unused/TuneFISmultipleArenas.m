numRobots = 30;
env = MultiRobotEnv(numRobots);
env.showTrajectory = false;%[true;true;...;true];
R_c = 8;
R_s = 3;
env.robotRadius = R_s;

env.plotSensorLines = false; % So the sensor lines don't dominate the visuals
env.showRobotIds = false;

mapMatrixArray{1} = table2array(readtable('Arenas\ArenaSimpleThick.csv'));
mapMatrixArray{2} = table2array(readtable('Arenas\ArenaConvexThick.csv'));
mapMatrixArray{3} = table2array(readtable('Arenas\ArenaCorridorTweakThick.csv'));
mapMatrixArray{4} = table2array(readtable('Arenas\ArenaComplexThick.csv'));

areaPoints1 = getAreaPoints(mapMatrixArray{1});
areaPoints2 = getAreaPoints(mapMatrixArray{2});
areaPoints3 = getAreaPoints(mapMatrixArray{3});
areaPoints4 = getAreaPoints(mapMatrixArray{4});

%env.givenMap = occupancyMap(mapMatrix);

%% Create robot detectors for all robots
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = R_c;
    detector.fieldOfView = 2*pi;
    detectors{rIdx} = detector;
end
warning('off','fuzzy:general:warnEvalfis_NoRuleFired')


fisin = readfis('FuzzyInferenceSystems\Compound\newLFLC.fis');
fisObst = readfis('FuzzyInferenceSystems\FuzzyAgentsObst3.fis');

[in,out] = getTunableSettings(fisin);

out = setTunable(out,false);

options = tunefisOptions('Method','particleswarm','OptimizationType','tuning');
options.MethodOptions.Display = 'iter';
options.MethodOptions.MaxIterations = 80;
options.MethodOptions.SwarmSize = 50;


costFunction = @(fis)simulationCost(fis,fisObst,detectors,mapMatrixArray, ...
    areaPoints1,areaPoints2,areaPoints3,areaPoints4,env);

rng('default')

runtunefis = true;

tStart = tic;
if runtunefis
    fisout = tunefis(fisin,[in;out],costFunction,options); 
else
   tunedfis = load('tunedfisnavigation.mat'); 
   fisout = tunedfis.fisout;
end
tEnd = toc(tStart)

fisout.Rules

figure
opt = gensurfOptions;
opt.NumGridPoints = 40;
gensurf(fisout,opt)
title('Output surface of fisout')
xlim([0 8.5]);
ylim([0 1.5]);



function cost = simulationCost(fisAgents,fisObst,detectors,mapMatrixArray, ...
    areaPoints1,areaPoints2,areaPoints3,areaPoints4,env)
tic
costtt = zeros(4,1);
for i = 1:4
sumCost = 0;
%% Initialize robots at random positions
numRobots = env.numRobots;
poses = rand(3,numRobots).*[4;4;pi]+[14;21;0];
env.Poses = poses;
mapMatrix = mapMatrixArray{i};
env.givenMap = occupancyMap(mapMatrix);
map = env.givenMap;

if i == 1
    areaPoints = areaPoints1;
else
    if i == 2
       areaPoints = areaPoints2; 
    else
        if i == 3
        areaPoints = areaPoints3; 
        else
            if i == 4
            areaPoints = areaPoints4; 
            end
        end
    end
end


vel = zeros(3,numRobots);
dirDisp = zeros(2,numRobots);
dirObst = zeros(2,numRobots);
dirTotal = zeros(2,numRobots);

for idx = 1:200%400 %(time units)

    for rIdx = 1:numRobots
       sensorPosition = poses(:,rIdx);

       %Obstacle Detections and obtained target translation
       scans = ObstSensorGrid(mapMatrix,sensorPosition);
       dirObst(:,rIdx) = ObstAvoidanceGoalDirection(scans,fisObst);
       
       %Robot Detections and obtained target direction
       detections = step(detectors{rIdx}); 
       dirDisp(:,rIdx) = DispersionGoalDirection(map,mapMatrix,sensorPosition,detections,scans,fisAgents);
       
    end
    
    for rIdx = 1:numRobots
       
       dirTotal(:,rIdx) = dirDisp(:,rIdx) + dirObst(:,rIdx);
       angle = atan2(dirTotal(2,rIdx),dirTotal(1,rIdx));
       speed = min(0.3,norm(dirTotal(:,rIdx)));
       vel(:,rIdx) = bodyToWorld([speed; 0; angle],poses(:,rIdx));

    end
    
    poses = poses + vel;
    env.Poses = poses;
    sumCost = sumCost + idx*circlesSmart(poses(1,:),poses(2,:),3*ones(1,numRobots),mapMatrix,areaPoints,10);    
end

collisions = CheckObstacles(mapMatrix,numRobots,poses); 
costtt(i) = (collisions+1)*length(areaPoints)/(sumCost)*(200*201)/2; % 2/(t*(t+1)) * sum_i(t_i * area)
end
costtt
cost = sum(costtt);
toc
end
 
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

function dir = ObstAvoidanceGoalDirection(detections,inferenceSystem)
    obstDetected = 3;
    psum = [0;0];
    
    if size(detections,1) < obstDetected
        obstDetected = size(detections,1);
    end
   
    % Else, turn towards the average of detected robots on the same team
    if ~isempty(detections)
        
            % Take the nNeighbours nearest neighbours and drive away the robot from
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
        if (Imatrix <= 1 || Jmatrix <= 1)
             continue
        end
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

function isBlockedRay = ValidateDetection(map,mapMatrix,range,sensorPosition,neighbourDir)
        isBlockedRay = false;

        startPoint = [sensorPosition(1),sensorPosition(2)];
        newDir = bodyToWorld([neighbourDir; 0],sensorPosition);
        endPoint = startPoint + range*[newDir(1),newDir(2)];
         rangeIsMax = false(2,1);
         
        [endPoints, middlePoints] = ...
             nav.algs.internal.mex.raycastCells(startPoint, endPoint, map.GridSize(1), ...
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

function res = circlesSmart(xc,yc,r,mapMatrix,areaPoints,checksPerPoint)

    totalPoints = length(areaPoints);
 
    r2 = r .* r;
 
    inside = 0;
    for i = 1:totalPoints
        %Conversion from matrix coordinates to world coordinates
        xmin = areaPoints(2,i)-1;
        xmax = xmin+1;
        ymin = size(mapMatrix,1) - areaPoints(1,i);
        ymax = ymin +1;

        for x = linspace(xmin,xmax,checksPerPoint)
            for y = linspace(ymin,ymax,checksPerPoint)
                if any(r2 > (x - xc).^2 + (y - yc).^2)
                    inside = inside + 1;
                end
            end
        end
    end

    res = totalPoints * inside / (totalPoints*checksPerPoint^2);
 
end


function areaPoints = getAreaPoints(mapMatrix)
areaPoints = int16.empty;
for i= 1:size(mapMatrix,1)
    for j=1:size(mapMatrix,2)
        if mapMatrix(i,j) == 0
            if ~isempty(areaPoints)
                areaPoints = [areaPoints(1,:),i;areaPoints(2,:),j];
            else
                areaPoints = [i;j];
            end
        end
    end
end
end