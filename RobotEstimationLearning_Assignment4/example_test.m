% Robotics: Estimation and Learning 
% WEEK 4
% 
% This script is to help run your algorithm and visualize the result from it.

%% Load data
clear all;
close all;

load practice.mat
% This will load four variables: ranges, scanAngles, t, pose
% [1] t is K-by-1 array containing time in second. (K=3701)
%     You may not need to use time info for implementation.
% [2] ranges is 1081-by-K lidar sensor readings. 
%     e.g. ranges(:,k) is the lidar measurement at time index k.
% [3] scanAngles is 1081-by-1 array containing at what angles the 1081-by-1 lidar
%     values ranges(:,k) were measured. This holds for any time index k. The
%     angles are with respect to the body coordinate frame.
% [4] M is a 2D array containing the occupancy grid map of the location
%     e.g. map(x,y) is a log odds ratio of occupancy probability at (x,y)

%% Set parameters
param = {};
% 1. Decide map resolution, i.e., the number of grids for 1 meter.
param.resol = 25;

% 3. Indicate where you will put the origin in pixels
param.origin = [685,572]';

param.init_pose = init_pose;

%% Plot LIDAR data
lidar_local = [ranges(:,1).*cos(scanAngles) -ranges(:,1).*sin(scanAngles)];


figure,
plot(0,0,'rs'); hold on;
plot(lidar_local(:,1),lidar_local(:,2),'.-'); 
axis equal;
set(gca,'YDir','reverse');
xlabel('x');
ylabel('y');
grid on;
title('Lidar measurement in the body frame');

%% Run algorithm
% Call your mapping function here.
% Running time could take long depending on the efficiency of your code.
% For a quicker test, you may take some hundreds frames as input arguments as
% shown.
pose = particleLocalization(ranges(:,1:500), scanAngles, M, param);
%load practice-answer.mat;

%% Plot final solution
% The final grid map:
figure;
imagesc(M); hold on;

%% Plot LIDAR data
lidar_global(:,1) =  (ranges(:,1).*cos(scanAngles + pose(3,1)) + pose(1,1))*param.resol + param.origin(1);
lidar_global(:,2) = (-ranges(:,1).*sin(scanAngles + pose(3,1)) + pose(2,1))*param.resol + param.origin(2);

plot(lidar_global(:,1), lidar_global(:,2), 'g.'); 


colormap('gray');
axis equal;
hold on;
plot(pose(1,:)*param.resol+param.origin(1), ...
    pose(2,:)*param.resol+param.origin(2), 'r.-');