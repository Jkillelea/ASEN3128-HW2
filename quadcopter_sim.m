% ASEN 3128, Jacob Killelea, HW 2
% Quadcopter data visualization
clear; close all; clc;
addpath(genpath('util'));

load('RSdata.mat'); % [rt_tout, rt_yout]

signals = rt_yout.signals;

% Field labels
% [empty], <X>, <Y>, <Z>, <yaw>, <pitch>, <roll>, <dx>, <dy>, <dz>, <p>, <q>,
% <r>, [empty], [empty], <ddx>, <ddy>, <ddz>, <p>, <q>, <r>, <altitude_sonar>,
% <prs>, [empty], [empty], [empty], [empty], [empty], [empty]

x   = getDataByLabel(signals, '<X>');
y   = getDataByLabel(signals, '<Y>');
z   = getDataByLabel(signals, '<Z>');

dx  = getDataByLabel(signals, '<dx>');
dy  = getDataByLabel(signals, '<dy>');
dz  = getDataByLabel(signals, '<dz>');

ddx = getDataByLabel(signals, '<ddx>');
ddy = getDataByLabel(signals, '<ddy>');
ddz = getDataByLabel(signals, '<ddz>');

z = -z;
dz = -dz;
ddz = -ddz;

% print out flightpath
figure; hold on;
plot3(x, y, z)
plot3(x(1), y(1), z(1), 'ro')
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-1, 1])
ylim([-1, 1])

% velocity vectors
% for i = 1:10:length(dx)
%   line( [x(i), x(i) + 0.1*dx(i)], ...
%         [y(i), y(i) + 0.1*dy(i)], ...
%         [z(i), z(i) + 0.1*dz(i)])
% end
