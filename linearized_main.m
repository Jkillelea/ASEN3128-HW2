% ASEN 3128, Jacob Killelea, Assignment 2
% Quadcopter sim
clear
clc
close all
addpath(genpath('util'));

tmax = 50; % seconds

%%% Conditions for steady hover
% inital position, velocity
x0     = [0 0 0]; % inertial displacement
vel0   = [0 0 0]; % body frame velocity
pose0  = [0 0 0]; % phi, theta, psi
omega0 = [0 0 0]; % body angular velocity
hover_conditions = [x0, vel0, pose0, omega0];

% problem 3a: +5 degrees bank
% inital position deviation, velocity deviation
deltaPos0   = [0 0 0];          % inertial displacement
deltaVel0   = [0 0 0];          % body frame velocity
deltaPose0  = [deg2rad(5) 0 0]; % phi, theta, psi
deltaOmega0 = [0 0 0];          % body angular velocity
inital_conds = [deltaPos0, deltaVel0, deltaPose0, deltaOmega0];
% there are currently no options implemented on the linearized model
opts = struct('control_motor', false); % no motor control
% call ode45 on the linearized model
[t, y] = ode45(@(t, y) linear_quadcopter(t, y, opts, hover_conditions), [0, tmax], inital_conds);
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

plotQuadCopterDynamics(t, [x, vel, pose, omega])
print(['img/', 'prob3a'], '-dpng')
close

% problem 3b: +5 degrees pitch
% inital position deviation, velocity deviation
deltaPos0   = [0 0 0];          % inertial displacement
deltaVel0   = [0 0 0];          % body frame velocity
deltaPose0  = [0 deg2rad(5) 0]; % phi, theta, psi
deltaOmega0 = [0 0 0];          % body angular velocity
inital_conds = [deltaPos0, deltaVel0, deltaPose0, deltaOmega0];
% there are currently no options implemented on the linearized model
opts = struct('control_motor', false); % no motor control
% call ode45 on the linearized model
[t, y] = ode45(@(t, y) linear_quadcopter(t, y, opts, hover_conditions), [0, tmax], inital_conds);
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

plotQuadCopterDynamics(t, [x, vel, pose, omega])
print(['img/', 'prob3b'], '-dpng')
close

% problem 3c: +5 degrees azimuth
% inital position deviation, velocity deviation
deltaPos0   = [0 0 0];          % inertial displacement
deltaVel0   = [0 0 0];          % body frame velocity
deltaPose0  = [0 0 deg2rad(5)]; % phi, theta, psi
deltaOmega0 = [0 0 0];          % body angular velocity
inital_conds = [deltaPos0, deltaVel0, deltaPose0, deltaOmega0];
% there are currently no options implemented on the linearized model
opts = struct('control_motor', false); % no motor control
% call ode45 on the linearized model
[t, y] = ode45(@(t, y) linear_quadcopter(t, y, opts, hover_conditions), [0, tmax], inital_conds);
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

plotQuadCopterDynamics(t, [x, vel, pose, omega])
print(['img/', 'prob3c'], '-dpng')
close

% problem 3d: +0.1 rad/sec roll rate
% inital position deviation, velocity deviation
deltaPos0   = [0 0 0];   % inertial displacement
deltaVel0   = [0 0 0];   % body frame velocity
deltaPose0  = [0 0 0];   % phi, theta, psi
deltaOmega0 = [0.1 0 0]; % body angular velocity
inital_conds = [deltaPos0, deltaVel0, deltaPose0, deltaOmega0];
% there are currently no options implemented on the linearized model
opts = struct('control_motor', false); % no motor control
% call ode45 on the linearized model
[t, y] = ode45(@(t, y) linear_quadcopter(t, y, opts, hover_conditions), [0, tmax], inital_conds);
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

plotQuadCopterDynamics(t, [x, vel, pose, omega])
print(['img/', 'prob3d'], '-dpng')
close

% problem 3e: +0.1 rad/sec pitch rate
% inital position deviation, velocity deviation
deltaPos0   = [0 0 0];   % inertial displacement
deltaVel0   = [0 0 0];   % body frame velocity
deltaPose0  = [0 0 0];   % phi, theta, psi
deltaOmega0 = [0 0.1 0]; % body angular velocity
inital_conds = [deltaPos0, deltaVel0, deltaPose0, deltaOmega0];
% there are currently no options implemented on the linearized model
opts = struct('control_motor', false); % no motor control
% call ode45 on the linearized model
[t, y] = ode45(@(t, y) linear_quadcopter(t, y, opts, hover_conditions), [0, tmax], inital_conds);
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

plotQuadCopterDynamics(t, [x, vel, pose, omega])
print(['img/', 'prob3e'], '-dpng')
close

% problem 3f: +0.1 rad/sec yaw rate
% inital position deviation, velocity deviation
deltaPos0   = [0 0 0];   % inertial displacement
deltaVel0   = [0 0 0];   % body frame velocity
deltaPose0  = [0 0 0];   % phi, theta, psi
deltaOmega0 = [0 0 0.1]; % body angular velocity
inital_conds = [deltaPos0, deltaVel0, deltaPose0, deltaOmega0];
% there are currently no options implemented on the linearized model
opts = struct('control_motor', false); % no motor control
% call ode45 on the linearized model
[t, y] = ode45(@(t, y) linear_quadcopter(t, y, opts, hover_conditions), [0, tmax], inital_conds);
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

plotQuadCopterDynamics(t, [x, vel, pose, omega])
print(['img/', 'prob3f'], '-dpng')
close

%%%% Problem 4 - feedback motor control
% problem 4d: +0.1 rad/sec roll rate
% inital position deviation, velocity deviation
deltaPos0   = [0 0 0];   % inertial displacement
deltaVel0   = [0 0 0];   % body frame velocity
deltaPose0  = [0 0 0];   % phi, theta, psi
deltaOmega0 = [0.1 0 0]; % body angular velocity
inital_conds = [deltaPos0, deltaVel0, deltaPose0, deltaOmega0];
% there are currently no options implemented on the linearized model
opts = struct('control_motor', true); % motor control
% call ode45 on the linearized model
[t, y] = ode45(@(t, y) linear_quadcopter(t, y, opts, hover_conditions), [0, tmax], inital_conds);
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

plotQuadCopterDynamics(t, [x, vel, pose, omega])
print(['img/', 'prob4d'], '-dpng')
close

% problem 4e: +0.1 rad/sec pitch rate
% inital position deviation, velocity deviation
deltaPos0   = [0 0 0];   % inertial displacement
deltaVel0   = [0 0 0];   % body frame velocity
deltaPose0  = [0 0 0];   % phi, theta, psi
deltaOmega0 = [0 0.1 0]; % body angular velocity
inital_conds = [deltaPos0, deltaVel0, deltaPose0, deltaOmega0];
% there are currently no options implemented on the linearized model
opts = struct('control_motor', true); % motor control
% call ode45 on the linearized model
[t, y] = ode45(@(t, y) linear_quadcopter(t, y, opts, hover_conditions), [0, tmax], inital_conds);
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

plotQuadCopterDynamics(t, [x, vel, pose, omega])
print(['img/', 'prob4e'], '-dpng')
close

% problem 4f: +0.1 rad/sec yaw rate
% inital position deviation, velocity deviation
deltaPos0   = [0 0 0];   % inertial displacement
deltaVel0   = [0 0 0];   % body frame velocity
deltaPose0  = [0 0 0];   % phi, theta, psi
deltaOmega0 = [0 0 0.1]; % body angular velocity
inital_conds = [deltaPos0, deltaVel0, deltaPose0, deltaOmega0];
% there are currently no options implemented on the linearized model
opts = struct('control_motor', true); % motor control
% call ode45 on the linearized model
[t, y] = ode45(@(t, y) linear_quadcopter(t, y, opts, hover_conditions), [0, tmax], inital_conds);
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

plotQuadCopterDynamics(t, [x, vel, pose, omega])
print(['img/', 'prob4f'], '-dpng')
close
