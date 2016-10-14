 % Robotics: Estimation and Learning 
% WEEK 2
% 
% This script is to help run your algorithm and visualize the result from it.

%% Load data
clear all;
%close all;

load training5.mat
% This will load three variables: t, ball, rgb
% [1] t is K-by-1 array containing time in second. (K=3701)
%     You may not use time info for implementation.
% [2] ball is a 2-by-K array of ball position readings. 
%     e.g. ball(1,k) is the x position at time index k, and ball(2,k) is
%     the y position at time index k
% [3] rgb is a cell array of images containing the image of the scene at
%     the time of image recording. Only used to help you visualize the
%     scene; not used in the function

%% Plot the path of the ball
figure(1);
clf;
plot(ball(1, :), ball(2, :), 'bo-');
hold on;
% End at red
plot(ball(1, end), ball(2, end), 's', ...
    'MarkerSize', 10, 'MarkerEdgeColor', [.5 0 0], 'MarkerFaceColor', 'r');
% start at green
plot(ball(1, 1), ball(2, 1), 's', ...
    'MarkerSize', 10, 'MarkerEdgeColor', [0 .5 0], 'MarkerFaceColor', 'g');
hold off;
axis equal;
title('Ball Position tracks');
xlabel('X (meters)');
ylabel('Y (meters)');

%% Run algorithm
% Call your mapping function here.
% Running time could take long depending on the efficiency of your code.
% This overlays the predicted positions against the observed positions
state = [0,0,0,0];
last_t = -1;
N = numel(t);
myPredictions = zeros(2, N);
param = {};
for i=1:N
    [ px, py, state, param ] = kalmanFilter( t(i), ball(1,i), ball(2,i), state, param, last_t);
    if numel(state)~=4
        error('Your state should be four dimensions.');
    end
    last_t = t(i);
    myPredictions(1, i) = px;
    myPredictions(2, i) = py;
end
clear px py;

%% Overlay the predictions
figure(1);
hold on;
plot(myPredictions(1, :), myPredictions(2, :), 'k+-');
hold off;

%% Show the error
nSkip = 10;
myError = myPredictions(:, 1:end-nSkip) - ball(:, 1+nSkip:end);
myError_dist = sqrt(myError(1,:).^2 + myError(2,:).^2);
myError_mean = mean(myError_dist);
figure(2);
clf;
plot(myError_dist);
title('Prediction Error Over Time');
xlabel('Frame');
xlim([1, numel(myError_dist)]);
ylabel('Error (meters)');
legend(sprintf('Naive Prediction: %.2f mean', myError_mean));

%% Load the solution
load solution5.mat

% Error
error = predictions(:, 1:end-nSkip) - ball(:, 1+nSkip:end);
error_dist = sqrt(error(1,:).^2 + error(2,:).^2);
error_mean = mean(error_dist);
figure(2);
hold on;
plot(error_dist);
hold off;
%title(sprintf('Kalman Prediction Error: %.2f mean', error_mean));
legend(sprintf('Naive Prediction: %.2f mean', myError_mean),...
    sprintf('Kalman Prediction: %.2f mean', error_mean));

figure(1);
hold on;
plot(predictions(1, :), predictions(2, :), 'mo-');
hold off;
legend('Observed','End','Start','Naive Prediction','Kalman Prediction');

% figure(20);
% clf;hold on;
% plot(ball(1, 1+nSkip:end));plot(predictions(1, 1:end-nSkip));
