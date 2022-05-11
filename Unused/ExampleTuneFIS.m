fisin = readfis('PSOFuzzyAgents.fis');


[in,out] = getTunableSettings(fisin);

options = tunefisOptions('Method','particleswarm','OptimizationType','learning');

options.MethodOptions.MaxIterations = 25;
options.MethodOptions.SwarmSize = 50;


costFunction = @(fis)simulationCost(fis);

rng('default')

runtunefis = true;

if runtunefis
    fisout = tunefis(fisin,[in;out],costFunction,options); 
else
   tunedfis = load('tunedfisnavigation.mat'); 
   fisout = tunedfis.fisout;
end

fisout.Rules

figure
gensurf(fisout)
title('Output surface of fisout')



function cost = simulationCost(fis)
%% Algorithm Fuzzy Logic
% 
%% Create a multi-robot environment
numRobots = 20;
env = MultiRobotEnv(numRobots);
env.showTrajectory = false;
env.robotRadius = 0.25;
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals
env.givenMap = occupancyMap(oneZeroMatrix(20));

%% Obtain Fuzzy Inference System
%swarmAgentsFIS = readfis('FuzzyAgents.fis');
swarmAgentsFIS = fis;

%% Initialize robots at random positions
poses = rand(3,numRobots).*[4;4;pi]+8*[ones(2,numRobots);zeros(1,numRobots)];
env.Poses = poses;
%% Create robot detectors for all robots
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = 8;
    detector.fieldOfView = 2*pi;
    detectors{rIdx} = detector;
end

%% Animate and show the detections [range, angle, index]
vel = zeros(3,numRobots);

    for idx = 1:200 %(time units)
    
        for rIdx = 1:numRobots
            detections = step(detectors{rIdx}); 
            vel(:,rIdx) = swarmTeamController(poses,rIdx,detections,swarmAgentsFIS);
        end
    
        for rIdx = 1:numRobots
            poses(:,rIdx) = poses(:,rIdx) + vel(:,rIdx);
        end
    end
    cost = 100/circles(poses(1,:),poses(2,:),3*ones(1,numRobots));
end

function vel = swarmTeamController(poses,rIdx,detections,inferenceSystem)
    nNeighbours = 2;
    a_max = 0.1;
    r_safe = 3;
    
    psum = [0;0];
    angle = 0;
    if size(detections,1) < nNeighbours
        nNeighbours = size(detections,1);
    end

    pose = poses(:,rIdx);
    
    Dv = [0;0];

    if ~isempty(detections)
        
            ranges = detections(1:nNeighbours,1);
            angles = detections(1:nNeighbours,2);
            for i = 1:nNeighbours
                d = ranges(i) - r_safe;
                distX = cos(angles(i));
                distY = sin(angles(i));
                neighbourPos = [distX;distY];
                weight = evalfis(inferenceSystem,d);
                psum = psum + sign(d)*weight*neighbourPos;
            end
            pnorm = norm(psum);
            Dv = [min(a_max,pnorm);0];
            
            if pnorm ~= 0 
                psum = psum/pnorm;
                if psum(1)==0 
                angle = 0;
                else
                angle = atan2(psum(2),psum(1));
                end  
            end
    end
    vel = bodyToWorld([Dv(1);Dv(2);angle],pose);
end

function A = oneZeroMatrix(N)
    A = ones(N);
    A(2:end-1,2:end-1) = 0;
end

function res = circles(xc,yc,r)
tic

ngrid = 5000;
 
r2 = r .* r;
 
xmin = min(xc-r);
xmax = max(xc+r);
ymin = min(yc-r);
ymax = max(yc+r);
 
inside = 0;
 
for x = linspace(xmin,xmax,ngrid)
    for y = linspace(ymin,ymax,ngrid)
        if any(r2 > (x - xc).^2 + (y - yc).^2)
            inside = inside + 1;
        end
    end
end
 
box_area = (xmax-xmin) * (ymax-ymin);
 
res = box_area * inside / ngrid^2;
toc
 
end