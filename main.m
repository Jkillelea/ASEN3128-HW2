% ASEN 3128, Jacob Killelea, Assignment 3
% main script for the nonlinear quadcopter sim
clear
clc
close all
addpath(genpath('util')); % utility functions

tmax = 50; % seconds

% Problem 2a: +5 degrees bank
x0     = [0 0 0];          % position [N, E, D]
vel0   = [0 0 0];          % body frame velocity
pose0  = [deg2rad(5) 0 0]; % phi, theta, psi
omega0 = [0 0 0];          % body angular velocity
inital_conds = [x0, vel0, pose0, omega0];

opts = struct('aero',          true, ... % aerodynamic forces
              'control_motor', false);   % just counteract weight
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
plotQuadCopterDynamics(t, y);
print(['img/', 'prob2a'], '-dpng');
close

% Problem 2b: +5 degrees pitch
x0     = [0 0 0];          % position
vel0   = [0 0 0];          % body frame velocity
pose0  = [0 deg2rad(5) 0]; % phi, theta, psi
omega0 = [0 0 0];          % body angular velocity
inital_conds = [x0, vel0, pose0, omega0];

opts = struct('aero',          true, ... % aerodynamic forces
              'control_motor', false);   % just counteract weight
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
plotQuadCopterDynamics(t, y);
print(['img/', 'prob2b'], '-dpng');
close

% Problem 2c: +5 degrees azimuth
x0     = [0 0 0];          % position
vel0   = [0 0 0];          % body frame velocity
pose0  = [0 0 deg2rad(5)]; % phi, theta, psi
omega0 = [0 0 0];          % body angular velocity
inital_conds = [x0, vel0, pose0, omega0];

opts = struct('aero',          true, ... % aerodynamic forces
              'control_motor', false);   % just counteract weight
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
plotQuadCopterDynamics(t, y);
print(['img/', 'prob2c'], '-dpng');
close

% Problem 2d: +0.1 rad/sec roll rate
x0     = [0 0 0];   % position
vel0   = [0 0 0];   % body frame velocity
pose0  = [0 0 0];   % phi, theta, psi
omega0 = [0.1 0 0]; % body angular velocity
inital_conds = [x0, vel0, pose0, omega0];

opts = struct('aero',          true, ... % aerodynamic forces
              'control_motor', false);   % just counteract weight
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
plotQuadCopterDynamics(t, y);
print(['img/', 'prob2d'], '-dpng');
close

% Problem 2e: +0.1 rad/sec pitch rate
x0     = [0 0 0];   % position
vel0   = [0 0 0];   % body frame velocity
pose0  = [0 0 0];   % phi, theta, psi
omega0 = [0 0.1 0]; % body angular velocity
inital_conds = [x0, vel0, pose0, omega0];

opts = struct('aero',          true, ... % aerodynamic forces
              'control_motor', false);   % just counteract weight
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
plotQuadCopterDynamics(t, y);
print(['img/', 'prob2e'], '-dpng');
close

% Problem 2f: +0.1 rad/sec yaw rate
x0     = [0 0 0];   % position
vel0   = [0 0 0];   % body frame velocity
pose0  = [0 0 0];   % phi, theta, psi
omega0 = [0 0 0.1]; % body angular velocity
inital_conds = [x0, vel0, pose0, omega0];

opts = struct('aero',          true, ... % aerodynamic forces
              'control_motor', false);   % just counteract weight
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
plotQuadCopterDynamics(t, y);
print(['img/', 'prob2f'], '-dpng');
close
