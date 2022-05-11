%% Algorithm Fuzzy Logic
% 
%% Create a multi-robot environment

numRobots = 30;
env = MultiRobotEnv(numRobots);
env.showTrajectory = false;%[true;true;...;true];
R_c = 8;
R_s = 3;
env.robotRadius = 0.1*R_s;

env.plotSensorLines = false; % So the sensor lines don't dominate the visuals
env.showRobotIds = false;

mapMatrix = table2array(readtable('Arenas\ArenaComplexThick.csv'));
env.hasCustomMap = true;
env.customMap = occupancyMap(mapMatrix);
areaPoints = getAreaPoints(mapMatrix);

makePlot = false;
%% Obtain Fuzzy Inference System
swarmAgentsFIS1 = readfis('FuzzyInferenceSystems\Compound\newTFLC2.fis');
swarmAgentsFIS2 = readfis('FuzzyInferenceSystems\Compound\Main\MFLC.fis');

swarmAgentsObst = readfis('FuzzyInferenceSystems\FuzzyAgentsObst3.fis');


%% Initialize robots at random positions

map = env.customMap;
%% Create robot detectors for all robots
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = R_c;
    detector.fieldOfView = 2*pi;
    detectors{rIdx} = detector;
end

%% Animate and show the detections [range, angle, index]
vel = zeros(3,numRobots);

totalIterations = 300;

dirDisp = zeros(2,numRobots);
dirObst = zeros(2,numRobots);
dirTotal = zeros(2,numRobots);

staleThreshold = 10;

areaTime1 = zeros(totalIterations,1);
areaTime2 = zeros(totalIterations,1);

totalRuns = 50;
totalData1 = zeros(totalRuns,totalIterations);
totalData2 = zeros(totalRuns,totalIterations);

tic
for run = 1:totalRuns*2
poses = rand(3,numRobots).*[4;4;pi]+[14;21;0];
env.Poses = poses;

failedRobots = zeros(1,numRobots);
failedRobotIdx = 0;
increment = 0;
staleIterations = 0;

    if (run <= totalRuns)
        swarmAgentsFIS = swarmAgentsFIS1; 
    else
        if (run <= totalRuns*2)
        swarmAgentsFIS = swarmAgentsFIS2;
        end
    end
    
    areaTime = zeros(totalIterations,1); 
    
for idx = 1:totalIterations %(time units)
    
    if mod(idx,10) == 0 && idx >= 100
            failedRobotIdx = failedRobotIdx + 1;
            failedRobots(1,failedRobotIdx) = 1;        
    end
    
    %Main loop where we obtain sensor readings and target translations from
    %agent-to-agent dispersion and obstacle avoidance.
    for rIdx = 1:numRobots
%         flag = 0;
%         for frIdx = 1:numRobots
%            if rIdx == failedRobots(frIdx) 
%                 flag = 1;
%                 break;
%            end           
%         end
        
        if failedRobots(rIdx) ==  1
           continue;
        end
        
        sensorPosition = poses(:,rIdx);
        
        %Obstacle Detections and obtained target direction
       scans = ObstSensorGrid(mapMatrix,sensorPosition);
       dirObst(:,rIdx) = ObstAvoidanceGoalDirection(scans,swarmAgentsObst);
       
       %Robot Detections and obtained target direction
       detections = step(detectors{rIdx}); 
       dirDisp(:,rIdx) = DispersionGoalDirection(map,mapMatrix,sensorPosition,detections,scans,swarmAgentsFIS);
            
    end

    %Loop where we add the target directions, clamp so the maximum speed is 
    % 0.3 and convert into velocity.
    for rIdx = 1:numRobots
       if failedRobots(rIdx) ==  1
           vel(:,rIdx) = [0;0;0];
           continue;
       end
       
       dirTotal(:,rIdx) = dirDisp(:,rIdx) + dirObst(:,rIdx);
       angle = atan2(dirTotal(2,rIdx),dirTotal(1,rIdx));
       speed = min(0.3,norm(dirTotal(:,rIdx)));
       vel(:,rIdx) = bodyToWorld([speed; 0; angle],poses(:,rIdx));

    end
    poses= poses + vel;
    
    % Update the environment
    env(1:numRobots, poses);
    %env.Poses = poses;
    
    xlim([11 36]);   % Without this, axis resizing can slow things down
    ylim([11 36]); 
    
    if makePlot
        if staleIterations == staleThreshold
            areaTime(idx) = areaTime(idx - 1);
        else
            if increment == 0
                staleIterations = staleIterations + 1;
            else
                staleIterations = 0;
            end
            areaTime(idx) = circlesSmart(poses(1,:),poses(2,:),R_s*ones(1,numRobots),mapMatrix,areaPoints,10)/length(areaPoints)*100;
            if (idx - 1) > 0
                increment = areaTime(idx) - areaTime(idx - 1);
            end
        end
    else
        if idx == totalIterations
            areaTime(idx) = circlesSmart(poses(1,:),poses(2,:),R_s*ones(1,numRobots),mapMatrix,areaPoints,10)/length(areaPoints)*100;
        end
    end
    
end

    if (run <= totalRuns)
        areaTime1 = areaTime + areaTime1; 
        totalData1(run, :) = areaTime;
    else
        if (run <= totalRuns*2)
            areaTime2 = areaTime + areaTime2;
            totalData2(run - totalRuns, :) = areaTime;
        end
    end

end

area1 = areaTime1/(totalRuns);
area2 = areaTime2/(totalRuns);
std1 = std(totalData1)';
std2 = std(totalData2)';

if makePlot
x = linspace(1,totalIterations,totalIterations);
figure
plot(x,area1,x,area2)%,x,areaTimeR)
xlabel('Number of Iterations');
ylabel('Area Covered (%)');
end


toc
function failedR = failedRobot()
        failedR = false;
        for frIdx = 1:numRobots
           if rIdx == 1
                flag = true;
                break;
           end           
        end
end
%% Helper function: Robot Controller Logic
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