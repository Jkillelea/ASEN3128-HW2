% ASEN 3128, Jacob Killelea, Assignment 2
% Quadcopter sim
clear
clc
close all
addpath(genpath('util'));

tmax = 1000; % seconds

% inital position, velocity
x0   = [0 0 0];
vel0 = [0 0 0]; % body frame velocity

% pose (x, y, z rotation), and angular velocity
pose0  = [0 0 0]; % phi, theta, psi
omega0 = [0 0 0]; % body angular velocity

% options and starting conditions
inital_conds = [x0, vel0, pose0, omega0];

opts = struct('aero',          false, ... % no aerodynamic forces
              'control_motor', false); % no motor control
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
r     = y(:, 1:3);
v     = y(:, 4:6);
pose  = y(:, 7:9);
omega = y(:, 10:12);

figure; hold on; grid on;
xlabel('Time (sec)');
ylabel('Displacement (m)');
title('Probelm 9 - Hover, No Aerodynamic Drag/Moments')
plot(t, v(:, 1), 'DisplayName', 'u', 'LineWidth', 2) % body x velocity
plot(t, v(:, 2), 'DisplayName', 'v', 'LineWidth', 2) % y
plot(t, v(:, 3), 'DisplayName', 'w', 'LineWidth', 2) % z
legend('show')

% Probelm 10
% options and starting conditions - hover
inital_conds = [x0, vel0, pose0, omega0];
opts = struct('aero',                  true, ... % include aerodynamic forces
              'control_motor', false);
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
r     = y(:, 1:3);
v     = y(:, 4:6);
pose  = y(:, 7:9);
omega = y(:, 10:12);

figure; hold on; grid on;
xlabel('Time (sec)');
ylabel('Displacement (m)');
title('Probelm 10 - Hover, With Aerodynamic Drag/Moments')
plot(t, v(:, 1), 'DisplayName', 'u', 'LineWidth', 2) % body x velocity
plot(t, v(:, 2), 'DisplayName', 'v', 'LineWidth', 2) % y
plot(t, v(:, 3), 'DisplayName', 'w', 'LineWidth', 2) % z
legend('show')


% Options and starting conditions - 5 m/s east
% inital position, velocity
x0   = [0 0 0];
vel0 = [0 4.996497332 -0.0070045176]; % body frame velocity

% pose (x, y, z rotation), and angular velocity
pose0  = [0.0374330168 0 0]; % phi, theta, psi
omega0 = [0 0 0]; % body angular velocity

% options and starting conditions
inital_conds = [x0, vel0, pose0, omega0];
opts = struct('aero',                  true, ... % include aerodynamic forces
              'control_motor', true);
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
r     = y(:, 1:3);
v     = y(:, 4:6);
pose  = y(:, 7:9);
omega = y(:, 10:12);

figure; hold on; grid on;
xlabel('Time (sec)');
ylabel('Displacement (m)');
title('Probelm 10 - Translation, With Aerodynamic Drag/Moments')
plot(t, v(:, 1), 'DisplayName', 'u', 'LineWidth', 2) % body x velocity
plot(t, v(:, 2), 'DisplayName', 'v', 'LineWidth', 2) % y
plot(t, v(:, 3), 'DisplayName', 'w', 'LineWidth', 2) % z
legend('show')

% Options and starting conditions - 5 m/s north (psi = 90)
% inital position, velocity
x0   = [0 0 0];
vel0 = [4.996497332, 0, -0.0070045176]; % body frame velocity

% pose (x, y, z rotation), and angular velocity
pose0  = [0, -0.0374330168, 90]; % phi, theta, psi
omega0 = [0 0 0]; % body angular velocity

% options and starting conditions
inital_conds = [x0, vel0, pose0, omega0];
opts = struct('aero',                  true, ... % include aerodynamic forces
              'control_motor', true);
% call ode45
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, tmax], inital_conds);
r     = y(:, 1:3);
v     = y(:, 4:6);
pose  = y(:, 7:9);
omega = y(:, 10:12);

figure; hold on; grid on;
xlabel('Time (sec)');
ylabel('Displacement (m)');
title('Probelm 10 - Translation, \psi = 90, With Aerodynamic Drag/Moments')
plot(t, v(:, 1), 'DisplayName', 'u', 'LineWidth', 2) % body x velocity
plot(t, v(:, 2), 'DisplayName', 'v', 'LineWidth', 2) % y
plot(t, v(:, 3), 'DisplayName', 'w', 'LineWidth', 2) % z
legend('show')

% hover test with inital upset
% start with nose pitched 1 degree down, all other conditions set
% to a stable hover
% inital position, velocity
x0   = [0 0 0];
vel0 = [0 0 0]; % body frame velocity

% pose (x, y, z rotation), and angular velocity
pose0  = [deg2rad(5) -deg2rad(10) 0]; % phi, theta, psi
omega0 = [0 0 0]; % body angular velocity

% options and starting conditions
inital_conds = [x0, vel0, pose0, omega0];
opts = struct('aero',                  false, ... % no aerodynamic forces
              'control_motor', false);
% call ode45 (only to 10 seconds, better timescale to show the divergence
[t, y] = ode45(@(t, y) quadcopter(t, y, opts), [0, 10], inital_conds);
r     = y(:, 1:3);
v     = y(:, 4:6);
pose  = y(:, 7:9);
omega = y(:, 10:12);

figure; hold on; grid on;
xlabel('Time (sec)');
ylabel('Displacement (m)');
title('Probelm 11 - Hover Attempt, \theta_0 = -10^\circ, \phi_0 = 5^\circ')
plot(t, v(:, 1), 'DisplayName', 'u', 'LineWidth', 2) % body x velocity
plot(t, v(:, 2), 'DisplayName', 'v', 'LineWidth', 2) % y
plot(t, v(:, 3), 'DisplayName', 'w', 'LineWidth', 2) % z
legend('show')
