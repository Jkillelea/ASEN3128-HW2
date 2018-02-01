% ASEN 3128, Jacob Killelea, Assignment 2
% Quadcopter sim
clear
clc
close all
addpath(genpath('util'));

tmax = 10; % seconds

% inital position, velocity
x0  = [0 0 0];
vel0 = [0 0 0]; % body frame velocity

% pose (x, y, z rotation), and angular velocity
pose0  = [0 0 0];
omega0 = [0 0 0]; % body inertial velocity

inital_conds = [x0, vel0, pose0, omega0];

[t, y] = ode45('quadcopter', [0, tmax], inital_conds);
