% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%

mu =[149.2171,143.9402,61.2023];
sig = [13.8973,11.5445,18.4580];
thre = 5.5e-05;
covar=[193.1341, 119.5973, -201.5219;
            119.5973, 133.2750, -174.9704;
            -201.5219, -174.9704, 340.6981];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
x_u=double(I)-cat(3,repmat(mu(1),120,160),repmat(mu(2),120,160),repmat(mu(3),120,160));
prob=zeros(120,160);
for i=1:120
    for j=1:160
        prob(i,j)=reshape(x_u(i,j,:),1,3)*inv(covar)*reshape(x_u(i,j,:),3,1);
    end
end
prob=(1/(124.0251*sqrt(det(covar))))*prob;%Correction 1/(124.0251*sqrt(det(covar))))*exp(-0.5*prob)
index=find(prob>thre);
I=true(size(prob));
I(index)=false;%Correction Flase->True

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%
bw_biggest = false(size(I));
CC = bwconncomp(I);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
segI = bw_biggest;
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
