clear all;
close all;

load practice.mat 
% This will load four variables: ranges, scanAngles, t, pose
% [1] t is K-by-1 array containing time in second. (K=3701)
%     You may not need this time info for implementation.
% [2] ranges is 1081-by-K lidar sensor readings. 
%     e.g. ranges(:,k) is the lidar measurement at time index k.
% [3] scanAngles is 1081-by-1 array containing at what angles (in radian) the 1081-by-1 lidar
%     values ranges(:,k) were measured. This holds for any time index k. The
%     angles are with respect to the body coordinate frame.
% [4] pose is 3-by-K array containing the pose of the mobile robot over time. 
%     e.g. pose(:,k) is the [x,y,theta(in radian)] at time index k.

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

% Note: There are some noise close to the robot, but they should not affect
% the mapping result. 
