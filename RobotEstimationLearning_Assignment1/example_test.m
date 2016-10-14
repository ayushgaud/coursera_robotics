% Robotics: Estimation and Learning
% WEEK 1
% 
% This script is to help run your algorithm and visualize the result from it.
close all

imagepath = './train';
for k=1:19
    % Load image
    I = imread(sprintf('%s/%03d.png',imagepath,k));
    
    % Implement your own detectBall function
    [segI, loc] = detectBall(I);
    
    figure, imshow(segI); hold on; 
    plot(loc(1), loc(2), '+b','MarkerSize',7); 
    
    disp('Press any key to continue. (Ctrl+c to exit)')
    pause
end


