%% Algorithm Fuzzy Logic
% 
%% Create a multi-robot environment
numRobots = 30;
env = MultiRobotEnv(numRobots);
env.showTrajectory = false;
env.robotRadius = 0.3;
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals

mapMatrix = table2array(readtable('Arenas\ArenaCorridorTweakThick.csv'));
env.givenMap = occupancyMap(mapMatrix);
areaPoints = getAreaPoints(mapMatrix);

totRuns = 50; % For averaging area data
%% Obtain Fuzzy Inference System
maxRange = 8;
map = env.givenMap;
%% Animate and show the detections [range, angle, index]

totalIterations = 900;
areaTime = zeros(totalIterations,1);
areaTimeR = zeros(totalIterations,1);
totalDataR = zeros(totalIterations,totRuns);
tic
for runIdx = 1:totRuns    
%% Initialize robots at random positions
poses = rand(3,numRobots).*[4;4;pi]+[14;21;0];
env.Poses = poses;
vel = zeros(3,numRobots);
angle = zeros(1,numRobots);

for idx = 1:totalIterations%400 %(time units)
    %Main loop where we obtain sensor readings and target rotions (if any)
    %avoiding collisions with other agents and obstacles.
    for rIdx = 1:numRobots
       
       sensorPosition = poses(:,rIdx);
        
       %Obstacle Detections and obtained target direction
       scans = ObstSensorGrid(mapMatrix,sensorPosition);
       if ~isempty(scans)
           angle(rIdx) = CollisionAvoidanceRotation(scans);
       else
           detections = RobotSensorGrid(sensorPosition,poses,rIdx);

           if(~isempty(detections))
               angle(rIdx) = CollisionAvoidanceRotation(detections);
           else
               angle(rIdx) = 0;
           end
       end
    end

    %Loop where we add the target directions, clamp so the maximum speed is 
    % 0.3 and transform into velocity.
    for rIdx = 1:numRobots
       
       speed = 0.3;
       vel(:,rIdx) = bodyToWorld([speed; 0; angle(rIdx)],poses(:,rIdx));

    end
    poses = poses + vel;
    
    % Update the environment
    %env(1:numRobots, poses);
    env.Poses = poses;
    
    xlim([11 36]);   % Without this, axis resizing can slow things down
    ylim([11 36]); 
    
    areaTime(idx) = circlesSmart(poses(1,:),poses(2,:),3*ones(1,numRobots),mapMatrix,areaPoints,20)/length(areaPoints)*100;

end
    areaTimeR = areaTime + areaTimeR;
    totalDataR(:,runIdx) = areaTime;
end
areaTimeR = areaTimeR/totRuns;

x = linspace(1,totalIterations,totalIterations);
plot(x,areaTimeR)
xlabel('Number of Iterations')
ylabel('Area Covered (%)')

toc

function angle = CollisionAvoidanceRotation(detections)
    angle = 0;
    % Steer away from the nearest collision if it's at 0.5 or closer
    rangeCol = detections(1,1);
    angleCol = detections(1,2);
            
       if (rangeCol <= 1)
          angle = angleCol + pi/3*rand+pi/3;
       end
end

function scans = ObstSensorGrid(mapMatrix,sensorPosition)
   preScans = nan(1,2);
   scanAngles = [0];
   
   for i = 1:size(scanAngles)
        %angle = scanAngles(i);
        dp = bodyToWorld([1; 0; 0],sensorPosition);
        Imatrix = size(mapMatrix,1) - floor(sensorPosition(2)+dp(2));
        Jmatrix = floor(sensorPosition(1)+dp(1))+1;
        if (mapMatrix(Imatrix,Jmatrix) == 1)        
            preScans(i,1) = GetRange(dp,sensorPosition);
        end 
   end
   preScans(:,2) = scanAngles + sensorPosition(3);
   preScans(any(isnan(preScans), 2), :) = [];
   scans = sortrows(preScans);
end

function range = GetRange(dp,sensorPosition)
            di = dp(1);
            dj = dp(2);
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
            preRange = sqrt(offsetX^2 + offsetY^2);
            if preRange <= 1
            range = preRange;
            else
                range = [];
            end
end

function detections = RobotSensorGrid(sensorPosition,poses,robotIdx)

offsets = poses(1:2,:) - sensorPosition(1:2);
offsets(:,robotIdx) = inf; % Take out the "self-offset"
        if ~isempty(offsets)
                % Extract ranges and angles
                ranges = sqrt(sum(offsets.^2,1));
                angles = wrapToPi(atan2(offsets(2,:),offsets(1,:))-sensorPosition(3));
                
                % Filter by maximum range and field of vision
                validIdx = ranges <= 0.8;
                ranges = ranges(validIdx);
                angles = angles(validIdx);
                            if ~isempty(ranges)
                            % Sort from nearest
                            [ranges,sortedIdx] = sort(ranges);
                            angles = angles(sortedIdx);
                            end
                detections = [ranges', angles'];
        else
                detections = []; 
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
