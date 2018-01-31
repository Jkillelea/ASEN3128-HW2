% ASEN 3128, Jacob Killelea, Assignment 2
% Quadcopter sim
clear
clc
close all
addpath(genpath('util'));

tmax = 10; % seconds

% inital position, velocity
x0  = [0 0 0];
dx0 = [1 0 0];

% pose (x, y, z rotation), and angular velocity
pose0  = [0 0 0];
dpose0 = [0 0 0];

inital_conds = [x0, dx0, pose0, dpose0];

[t, y] = ode45('quadcopter', [0, tmax], inital_conds);
