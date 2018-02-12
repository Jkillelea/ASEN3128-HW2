% ASEN 3128, Jacob Killelea, Assignment 4
% main script for the nonlinear quadcopter sim
clear
clc
close all
addpath(genpath('util')); % utility functions

tmax = 5; % seconds

% Problem 3a: +5 degrees bank
x0     = [0 0 0];          % position [N, E, D]
vel0   = [0 0 0];          % body frame velocity
pose0  = [deg2rad(5) 0 0]; % phi, theta, psi
omega0 = [0 0 0];          % body angular velocity
inital_conds = [x0, vel0, pose0, omega0];

opts = struct('aero',          true, ... % aerodynamic forces
              'control_motor', true);   % full motor control
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
plotQuadCopterDynamics(t, y);
print(['img/', 'prob3a'], '-dpng');
close

% Problem 3b: +5 degrees pitch
x0     = [0 0 0];          % position
vel0   = [0 0 0];          % body frame velocity
pose0  = [0 deg2rad(5) 0]; % phi, theta, psi
omega0 = [0 0 0];          % body angular velocity
inital_conds = [x0, vel0, pose0, omega0];

opts = struct('aero',          true, ... % aerodynamic forces
              'control_motor', true);   % full motor control
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
plotQuadCopterDynamics(t, y);
print(['img/', 'prob3b'], '-dpng');
close

% Problem 3c: +0.1 rad/sec roll rate
x0     = [0 0 0];   % position
vel0   = [0 0 0];   % body frame velocity
pose0  = [0 0 0];   % phi, theta, psi
omega0 = [0.1 0 0]; % body angular velocity
inital_conds = [x0, vel0, pose0, omega0];

opts = struct('aero',          true, ... % aerodynamic forces
              'control_motor', true);   % full motor control
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
plotQuadCopterDynamics(t, y);
print(['img/', 'prob3c'], '-dpng');
close

% Problem 3d: +0.1 rad/sec pitch rate
x0     = [0 0 0];   % position
vel0   = [0 0 0];   % body frame velocity
pose0  = [0 0 0];   % phi, theta, psi
omega0 = [0 0.1 0]; % body angular velocity
inital_conds = [x0, vel0, pose0, omega0];

opts = struct('aero',          true, ... % aerodynamic forces
              'control_motor', true);   % full motor control
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
plotQuadCopterDynamics(t, y);
print(['img/', 'prob3d'], '-dpng');
close
