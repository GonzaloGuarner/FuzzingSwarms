%% Multi-Robot Lidar Example
% Copyright 2018-2019 The MathWorks, Inc.
%% Create a multi-robot environment
numRobots = 20;
env = MultiRobotEnv(numRobots);
env.showTrajectory = false;
env.robotRadius = 0.25;
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals
env.givenMap = occupancyMap(oneZeroMatrix(25));
%% Initializa robots at random positions

poses = rand(3,numRobots).*[6;6;pi]+7*[ones(2,numRobots);zeros(1,numRobots)];
env.Poses = poses;

%% Create robot detectors for all robots
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = 3;
    detector.fieldOfView = 2*pi;
    detectors{rIdx} = detector;
end

%% Animate and show the detections [range, angle, index]
vel = zeros(3,numRobots);
ranges = cell(1,numRobots);
for idx = 1:300 %(time units)
    
    for rIdx = 1:numRobots
       detections = step(detectors{rIdx}); 
       vel(:,rIdx) = swarmTeamController(poses,rIdx,detections);
    end
    for rIdx = 1:numRobots
       poses(1,rIdx) = poses(1,rIdx) + vel(1,rIdx);
       poses(2,rIdx) = poses(2,rIdx) + vel(2,rIdx);
       poses(3,rIdx) = poses(3,rIdx) + vel(3,rIdx);
    end
    %poses = poses + vel;
    % Update the environment
    env(1:numRobots, poses);
    xlim([0 20]);   % Without this, axis resizing can slow things down
    ylim([0 20]); 
    
end

%% Helper function: Robot Controller Logic
function vel = swarmTeamController(poses,rIdx,detections)
    c = 2;
    v_max = 0.1;
    r_safe = 3;
    psum = [0;0];
    angle = 0;
    if size(detections,1) < c
        c = size(detections,1);
    end
    % Unpack the robot's pose
    pose = poses(:,rIdx);
    
    % If there are no detections, turn in place
    v = [0;0];
    
    % Else, turn towards the average of detected robots on the same team
    if ~isempty(detections)
        
            % Take the c nearest neighbours and drive away the robot from
            % the sum of their positions (in the robot's coordinates)
            ranges = detections(1:c,1);
            angles = detections(1:c,2);
            for i = 1:c
                if ranges(i) > r_safe
                    v = [0;0];
                    angle = 0;
                    break;
                end
                distX = ranges(i)*cos(angles(i));
                distY = ranges(i)*sin(angles(i));
                neighbourPos = [distX;distY];
                psum = psum + neighbourPos;
            end    

            v = -[v_max/(r_safe*c)*sum(ranges(1:c,1));0];
            angle = atan2(psum(2),psum(1));

    end
    % Convert to global velocity
    vel = bodyToWorld([v(1);v(2);angle],pose);
end
%% Function to create a closed arena map of size NxN
function A = oneZeroMatrix(N)
    A = ones(N);
    A(2:end-1,2:end-1) = 0;
end