%% Description
% Main code to run the Dogbot simulation
clc;clear;close all;

%% 0. Parameters
% User tunable
wheelBase = 1.0; %m
stanleyGain = 0.5; % gain var (k) for Stanley Controller (Steering)
maxSteeringVel = 0.5; % maximum steering velocity
maxVelocity = 0.5; %m/s
wsBounds = [0,50;0,50]; % X,Y boundary definition of the world grid
numObstacle = 5; % Num obstacles to generate
maxObstacleSize = 10;
startPose = [2,2,0]; % m,m,rad
goalPose = [40,35,2*pi/3]; % m,m,rad
Tau = 0.5; % Motor torque constant
goalTol = 2; % Tolerance radius of reaching goal position
angTol = 0.5; % anglular tolerance within goal
stepSize = 0.3; % fixed delta to extend tree by

% Pre-set
x0 = startPose(1); % m
y0 = startPose(2); % m
t0 = startPose(3); % rad
v0 = 0; % m/s

% Generate obstacles
obstacleList = zeros(numObstacle,4);
for i = 1:numObstacle
    obstacleList(i,1) = i;
    obsWidth = maxObstacleSize*rand();
    obstacleList(i,2) = min(wsBounds(1,2)-obsWidth,max(wsBounds(1,1)+obsWidth,wsBounds(1,2)*rand()-wsBounds(1,1)));
    obstacleList(i,3) = min(wsBounds(2,2)-obsWidth,max(wsBounds(2,1)+obsWidth,wsBounds(2,2)*rand()-wsBounds(2,1)));
    obstacleList(i,4) = obsWidth;
end % for
params.obstacles = obstacleList;
params.wsBounds = wsBounds;
params.goalTol = goalTol;
params.angTol = angTol;
params.stepSize = stepSize;

%% 1. Preprocessing - obtain paths
rrtPath = rrtPlanner(startPose,goalPose,true,params);

%% 2. Simulate the model
load_system("RobotSim_v0p1.slx");
simOut = sim("RobotSim_v0p1.slx");
figure;
hold on;
axis equal;
plot(rrtPath(:,1),rrtPath(:,2),'r--','LineWidth',4);
plot(simOut.pathActual(:,1),simOut.pathActual(:,2),'b-','LineWidth',2);
plot(startPose(1),startPose(2),'bo','MarkerSize',10,'DisplayName','Start');
plot(goalPose(1),goalPose(2),'go','MarkerSize',10,'DisplayName','Goal');
xlabel('X Position (m)');
xlabel('X Position (m)');
title('Robot Path (actual vs planned)');
