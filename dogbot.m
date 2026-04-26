%% Description
% Main code to run the Dogbot simulation

%% 0. Parameters
wheelBase = 1.0; %m
stanleyGain = 0.5; % gain var (k) for Stanley Controller (Steering)
maxSteeringVel = 0.5; % maximum steering velocity
maxVelocity = 1.0; %m/s
targetPose = [35,21,3*pi/8];

%% 1. Preprocessing - obtain paths
