% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose  =  particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N  =  size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose  =  zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% the number of grids for 1 meter.
myResolution  =  param.resol;
% the origin of the map in pixels
myOrigin  =  param.origin; 

% The initial pose is given
myPose(:,1)  =  param.init_pose;
% You should put the given initial pose into myPose for j = 1, ignoring the j = 1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M  =  1000;                           % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P  =  repmat(myPose(:,1), [1, M]);
weights = ones(M,1)/M;
load practice-answer.mat;
close all
figure;
imagesc(map); hold on;
colormap('gray');
axis equal;
Sigma=(diag([0.1, 0.1, pi]));
for j  =  2:N % You will start estimating myPose from j = 2 using ranges(:,2).
    
    % 1) Propagate the particles 
    P = P + (randn(M,3)*Sigma)';
    Sigma=(diag([0.15, 0.15, 0.05]));
    % 2) Measurement Update 
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
    score = zeros(M,1);
    for m = 1:M
        occ =  (repmat(ranges(:,j),1,2).*[cos(P(3,m) + scanAngles), -sin(P(3,m) + scanAngles)]) + repmat([P(1,m),P(2,m)],size(ranges,1),1);
        i_occ = ceil(myResolution*occ) + repmat(myOrigin,1,size(ranges,1))';
        pos = ceil(myResolution*P(1:2,m)) + myOrigin;
    %   2-2) For each particle, calculate the correlation scores of the particles
        if(sum((i_occ(:,1) > size(map,2)) + (i_occ(:,2) > size(map,1))) || sum((pos(1) > size(map,2)) + (pos(2) > size(map,1))) || pos(1) <= 0 || pos(2)<= 0 || map(pos(2),pos(1)) > 0.48 || sum(i_occ(:,1) <= 0) || sum(i_occ(:,2) <= 0))
            score(m) = -20000;
        else
            occupied = sub2ind(size(map),i_occ(:,2),i_occ(:,1));
            score(m) = 10*sum(map(occupied) > 0.5) - 5*sum(map(occupied) < 0.48);
        end
        
%        plot(pos(1), pos(2), 'b.-');
    end
    %   2-3) Update the particle weights         
    score = score - min(score);
    weights = weights.*score;
    weights = weights/sum(weights);
    %   2-4) Choose the best particle to update the pose
    [~, indx] = max(weights);
    myPose(:,j) = P(:,indx);
    % 3) Resample if the effective number of particles is smaller than a threshold
    resampling_factor = sum(weights);
    if (resampling_factor < 85)
        idx = zeros(M,1);
        [a,b] = sort(weights);
        c = cumsum(a);
        for k = 1:M
            idx(k) = b(find(rand<=c,1,'first'));
        end
        P = P(:,idx);
       weights = ones(M,1)/M;
    end
    % 4) Visualize the pose on the map as needed
%     plot(myPose(1,j)*param.resol+param.origin(1), ...
%     myPose(2,j)*param.resol+param.origin(2), 'r.-');
%     plot(pose(1,j)*param.resol+param.origin(1), ...
%     pose(2,j)*param.resol+param.origin(2), 'y.-');
%     drawnow;
%     colormap('gray');
%     axis equal;
end
    plot(myPose(1,:)*param.resol+param.origin(1), ...
    myPose(2,:)*param.resol+param.origin(2), 'r.-');
    plot(pose(1,1:j)*param.resol+param.origin(1), ...
    pose(2,1:j)*param.resol+param.origin(2), 'y.-');
    drawnow;
    colormap('gray');
    axis equal;

end

