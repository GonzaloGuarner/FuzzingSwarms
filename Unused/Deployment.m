%% Algorithm Fuzzy Logic
% 
%% Create a multi-robot environment
%tic

numRobots = 30;
env = MultiRobotEnv(numRobots);
env.showTrajectory = false;%[true;true;...;true];
env.robotRadius = 0.25;
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals

mymap = table2array(readtable('Arenas\ArenaComplexThick.csv'));
env.givenMap = occupancyMap(mymap);
areaPoints = getAreaPoints(mymap);

makeAreaTimePlot = true;

%% Obtain Fuzzy Inference System
swarmAgentsFIS = readfis('FuzzyInferenceSystems\TuesdayAgents.fis');
swarmAgentsObst = readfis('FuzzyInferenceSystems\FuzzyAgentsObst3.fis');


%% Initialize robots at random positions
%poses = rand(3,numRobots).*[7;7;pi]+8*[ones(2,numRobots);zeros(1,numRobots)];
poses = rand(3,numRobots).*[4;4;pi]+[14;21;0];
env.Poses = poses;

scanAngles = linspace(-pi,pi,8);
maxRange = 8;
map = env.givenMap;
%% Create robot detectors for all robots
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = maxRange;
    detector.fieldOfView = 2*pi;
    detectors{rIdx} = detector;
end

%% Animate and show the detections [range, angle, index]
vel = zeros(3,numRobots);
dirDisp = zeros(2,numRobots);
dirObst = zeros(2,numRobots);
dirTotal = zeros(2,numRobots);

areaTime = zeros(200,1);
tic
for idx = 1:200%400 %(time units)
    %tic
    for rIdx = 1:numRobots
        
       %Robot Detections and obtained target direction
       detections = step(detectors{rIdx}); 
       dirDisp(:,rIdx) = DispersionGoalDirection(detections,swarmAgentsFIS);

       %Obstacle Detections and obtained target direction
       sensorPosition = poses(:,rIdx);
       scans = ObstSensor(map,sensorPosition,scanAngles,1);
       dirObst(:,rIdx) = ObstAvoidanceGoalDirection(scans,swarmAgentsObst);
       
    end
    %toc
    %tic
    for rIdx = 1:numRobots
       
       dirTotal(:,rIdx) = dirDisp(:,rIdx) + dirObst(:,rIdx);
       angle = atan2(dirTotal(2,rIdx),dirTotal(1,rIdx));
       speed = min(0.3,norm(dirTotal(:,rIdx)));
       vel(:,rIdx) = bodyToWorld([speed; 0; angle],poses(:,rIdx));

    end
    poses= poses + vel;
    %checkOccupancy(map,[poses(1,:), poses(2,:)], 'world');
    %toc
    
    % Update the environment
   %env(1:numRobots, poses);
    env.Poses = poses;
    
    xlim([11 36]);   % Without this, axis resizing can slow things down
    ylim([12 36]); 
    
    if makeAreaTimePlot
        
     areaTime(idx) = circlesSmart(poses(1,:),poses(2,:),5*ones(1,numRobots),areaPoints,10)/length(areaPoints)*100;
     
    end

end
toc

%% Helper function: Robot Controller Logic
function dir = DispersionGoalDirection(detections,inferenceSystem)
    nNeighbours = 3;
    
    psum = [0;0];
    if size(detections,1) < nNeighbours
        nNeighbours = size(detections,1);
    end
    
    if ~isempty(detections)
        
            ranges = detections(1:nNeighbours,1);
            angles = detections(1:nNeighbours,2);
            for i = 1:nNeighbours
                d = ranges(i);
                distX = cos(angles(i));
                distY = sin(angles(i));
                neighbourPos = [distX;distY];
                weight = evalfis(inferenceSystem,d);
                psum = psum + weight*neighbourPos;
            end
    end
    dir = psum;
end

function dir = ObstAvoidanceGoalDirection(detections,inferenceSystem)
    obstDetected = 3;
    psum = [0;0];
    if sum(isnan(detections)) > 0

    end
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

function scans = ObstSensor(map,sensorPosition,scanAngles,maxRange)
%tic
    preScans = nan(numel(scanAngles),2);
    
    interPts = [numel(scanAngles);2];
    
    for scanIdx = 1:numel(scanAngles)
        interPts(scanIdx,1:2) = rayIntersection(map, sensorPosition, ...
        scanAngles(scanIdx), maxRange);   
    end
  
    offsets = interPts-[sensorPosition(1),sensorPosition(2)]; 
    preScans(:,1) = sqrt(sum(offsets.^2,2));
    preScans(:,2) = scanAngles;
    
    preScans(any(isnan(preScans), 2), :) = [];
    scans = sortrows(preScans);

%toc
end

%% Function to create a closed arena map of size NxN
function A = oneZeroMatrix(N)
    A = ones(N);
    A(2:end-1,2:end-1) = 0;
end

function res = circlesSmart(xc,yc,r,areaPoints,checksPerPoint)

    totalPoints = length(areaPoints);
 
    r2 = r .* r;
 
    inside = 0;
    for i = 1:totalPoints
        xmin = areaPoints(1,i);
        xmax = xmin+1;
        ymin = areaPoints(2,i);
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
