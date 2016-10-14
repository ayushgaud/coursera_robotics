% Robotics: Estimation and Learning 
% WEEK 1
% 
% This is an example code for bwconncomp
% http://www.mathworks.com/help/images/ref/regionprops.html
a = imread('circlesBrightDark.png');
bw = a < 100;
figure, imshow(bw); 
title('Image with Circles')

% create new empty binary image
bw_biggest = false(size(bw));

% http://www.mathworks.com/help/images/ref/bwconncomp.html
CC = bwconncomp(bw);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
figure,
imshow(bw_biggest); hold on;

% show the centroid
% http://www.mathworks.com/help/images/ref/regionprops.html
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
plot(loc(1), loc(2),'r+');