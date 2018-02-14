% ASEN 3128, Jacob Killelea, HW 2
% Quadcopter data visualization
clear; close all; clc;
addpath(genpath('util'));

% rt_calib, rt_cmd, rt_estim, rt_motor, rt_optical, rt_posref, rt_sensor
% rt_tout, rt_yout
load('RSdata_Drone09_1330.mat');

% The columns of values are organized with columns:
% X Y Z yaw pitch roll dx dy dz p q r

t = rt_sensor.time;
X     = rt_estim.signals.values(:, 1);
Y     = rt_estim.signals.values(:, 2);
Z     = rt_estim.signals.values(:, 3);
yaw   = rt_estim.signals.values(:, 4); % radians
pitch = rt_estim.signals.values(:, 5);
roll  = rt_estim.signals.values(:, 6);
dx    = rt_estim.signals.values(:, 7);
dy    = rt_estim.signals.values(:, 8);
dz    = rt_estim.signals.values(:, 9);
p     = rt_estim.signals.values(:, 10);
q     = rt_estim.signals.values(:, 11);
r     = rt_estim.signals.values(:, 12);

% convert [X, Y, Z] to [N, E, D]
sz = length(X);
N = zeros(sz, 1);
E = zeros(sz, 1);
D = zeros(sz, 1);
for i = 1:sz
  Qeb  = R1(-roll(i))*R2(-pitch(i))*R3(-yaw(i)); % body vector to inertial vector
  tmp  = Qeb * [X(i); Y(i); Z(i)];
  N(i) = tmp(1);
  E(i) = tmp(2);
  D(i) = tmp(3);
end

plotQuadCopterDynamics(t, [N, E, D, dx, dy, dz, roll, pitch, yaw, p, q, r])
