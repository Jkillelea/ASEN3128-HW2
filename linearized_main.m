% ASEN 3128, Jacob Killelea, Assignment 2
% Quadcopter sim
clear
clc
close all
addpath(genpath('util'));

tmax = 1000; % seconds

%%% Conditions for steady hover
% inital position, velocity
x0     = [0 0 0]; % inertial displacement
vel0   = [0 0 0]; % body frame velocity
pose0  = [0 0 0]; % phi, theta, psi
omega0 = [0 0 0]; % body angular velocity
hover_conditions = [x0, vel0, pose0, omega0];

%%% Initial deviations from hover conditions
% inital position, velocity
deltaPos0   = [0 0 0]; % inertial displacement
deltaVel0   = [0 0 0]; % body frame velocity
deltaPose0  = [0 0 0]; % phi, theta, psi
deltaOmega0 = [0 0 0]; % body angular velocity
inital_conds = [deltaPos0, deltaVel0, deltaPose0, deltaOmega0];

% there are currently no options implemented on the linearized model
opts = struct('aero',          false, ... % no aerodynamic forces
              'control_motor', false);    % no motor control

% call ode45 on the linearized model
[t, y] = ode45(@(t, y) linear_quadcopter(t, y, opts), [0, tmax], inital_conds);

% deviations from hover over time
deltaPos   = y(:, 1:3);
deltaVel   = y(:, 4:6);
deltaPose  = y(:, 7:9);
deltaOmega = y(:, 10:12);

% recover absolue values over time
x     = x0     + deltaPos;
vel   = vel0   + deltaVel;
pose  = pose0  + deltaPose;
omega = omega0 + deltaOmega;
