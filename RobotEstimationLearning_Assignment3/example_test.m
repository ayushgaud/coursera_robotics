% Robotics: Estimation and Learning 
% WEEK 3
% 
% This script is to help run your algorithm and visualize the result from it.
% 
% Please see example_lidar first to understand the lidar measurements, 
% and see example_bresenham to understand how to use it.
clear all;
close all;

load practice.mat 
% This will load four variables: ranges, scanAngles, t, pose
% [1] t is K-by-1 array containing time in second. (K=3701)
%     You may not use time info for implementation.
% [2] ranges is 1081-by-K lidar sensor readings. 
%     e.g. ranges(:,k) is the lidar measurement (in meter) at time index k.
% [3] scanAngles is 1081-by-1 array containing at what angles (in radian) the 1081-by-1 lidar
%     values ranges(:,k) were measured. This holds for any time index k. The
%     angles are with respect to the body coordinate frame.
% [4] pose is 3-by-K array containing the pose of the mobile robot over time. 
%     e.g. pose(:,k) is the [x(meter),y(meter),theta(in radian)] at time index k.

% 1. Decide map resolution, i.e., the number of grids for 1 meter.
param.resol = 25;

% 2. Decide the initial map size in pixels
param.size = [900, 900];

% 3. Indicate where you will put the origin in pixels
param.origin = [700,600]'; 

% 4. Log-odd parameters 
param.lo_occ = 1;
param.lo_free = 0.5; 
param.lo_max = 100;
param.lo_min = -100;


% Call your mapping function here.
% Running time could take long depending on the efficiency of your code.
% For a quicker test, you may take some hundreds frames as input arguments as
% shown.
myMap = occGridMapping(ranges(:,1:1000), scanAngles, pose(:,1:1000), param);

% The final grid map: 
figure,
imagesc(myMap); 
colormap('gray'); axis equal;
