% Robotics: Estimation and Learning 
% WEEK 3
% 
% This script is to show how to use bresenham.
map = zeros(30,30);
orig = [10,5]; % start point
occ = [20,20]; % end point 
% get cells in between
[freex, freey] = bresenham(orig(1),orig(2),occ(1),occ(2));  
% convert to 1d index
free = sub2ind(size(map),freey,freex);
% set end point value 
map(occ(2),occ(1)) = 3;
% set free cell values
map(free) = 1;

figure(1),
imagesc(map); hold on;
plot(orig(1),orig(2),'rx','LineWidth',3); % indicate start point
axis equal;
