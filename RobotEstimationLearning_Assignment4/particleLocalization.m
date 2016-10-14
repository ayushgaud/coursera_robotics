% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of myPoses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% the number of grids for 1 meter.
myResolution = param.resol;
% the origin of the map in pixels
myOrigin = param.origin; 

% The initial myPose is given
myPose(:,1) = param.init_pose;
% You should put the given initial myPose into myPose for j=1, ignoring the j=1 ranges. 
% The myPose(:,1) should be the myPose when ranges(:,j) were measured.
Sigma=chol(diag([0.005, 0.005, pi/40]));

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 50;                            % Please decide a reasonable number of M, 
                           % based on your experiment using the practice data.
weight=ones(M,1)/M;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particls
P = repmat(myPose(:,1), [1, M]);

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles 
    P=P+(randn(M,3)*Sigma)';
    score=zeros(M,1);  
    % 2) Measurement Update 
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
    for m=1:M
        occ= (repmat(ranges(:,j),1,2).*[cos(P(3,m)+scanAngles), -sin(P(3,m)+scanAngles)])+repmat([P(1,m),P(2,m)],size(ranges,1),1);
        i_occ=ceil(myResolution*occ)+repmat(myOrigin,1,size(ranges,1))';
        pos=ceil(myResolution*P(1:2,m))+myOrigin;
        g_idx=unique(i_occ,'rows');
        pos(1) = min(pos(1),size(map,2));
        pos(2) = min(pos(2),size(map,1));
        pos(1) = max(pos(1),1);
        pos(2) = max(pos(2),1);
        if(map(pos(2),pos(1))>=-0.2 || sum(max(g_idx)>(size(map)*[0,1;1,0])))
            weight(m)=10e-6;
        else
        %max(g_idx)
        lidar_predict=ray_tracing(P(1,m),P(2,m),P(3,m),map,scanAngles,myOrigin,myResolution);
        d=abs(i_occ - lidar_predict);
        
        score(m)=exp(-0.5*(sum(d(:,1).^2 + d(:,2).^2))/10000);
%         return
        g_idx(:,1)=min(g_idx(:,1),size(map,2));
        g_idx(:,2)=min(g_idx(:,2),size(map,1));
        g_idx(:,1)=max(g_idx(:,1),1);
        g_idx(:,2)=max(g_idx(:,2),1);
        occupied = sub2ind(size(map),g_idx(:,2),g_idx(:,1));
        score(m)=0.8*score(m)+0.2*(10*sum(map(occupied)>=0.5)-5*sum(map(occupied)<0.5)-1*sum(map(occupied)<0));
        if(score(m)<0)
            score(m)=10e-6;
        end
%         for i=1:size(g_idx,1)    
% %     get cells in between
%         [freex, freey] = bresenham(pos(1),pos(2),g_idx(i,1),g_idx(i,2));  
%         % convert to 1d index
%         try
%         free = sub2ind(size(map),freey,freex);
%         catch
%             [freex, freey]
%             return
%         end
%         score(m)=score(m)+sum(map(free)<0);
%         score(m)=score(m)-5*sum(map(free)>0);
% %         score(m)=score(m)+10*sum(map(occupied)>1);
% %         score(m)=score(m)-5*sum(map(occupied)<1);
%          end
        end
     end
    %   2-2) For each particle, calculate the correlation scores of the particles
    
    %   2-3) Update the particle weights         
    weight=weight.*score;
    weight=weight/sum(weight);
    %   2-4) Choose the best particle to update the myPose
    [~,indx]=max(weight);
    weight(indx)
    myPose(:,j) =P(:,indx);
    % 3) Resample if the effective number of particles is smaller than a threshold
    resampling_factor=1/sum(weight.^2);
    if (resampling_factor<30)
        idx=zeros(M,1);
        
        [a,b]=sort(weight);
        c=cumsum(a);
        for k=1:M
            idx(k)=b(find(rand<=c,1,'first'));
        end
        
        P=P(:,idx);
        weight=weight(idx);
        weight=weight/sum(weight);
    end
    % 4) Visualize the myPose on the map as needed
if(j>N-350)
    imagesc(map); hold on;

%% Plot LIDAR data
lidar_global(:,1) =  (ranges(:,j).*cos(scanAngles + myPose(3,j)) + myPose(1,j))*param.resol + param.origin(1);
lidar_global(:,2) = (-ranges(:,j).*sin(scanAngles + myPose(3,j)) + myPose(2,j))*param.resol + param.origin(2);

plot(lidar_global(:,1), lidar_global(:,2), 'g.'); 
local=ray_tracing(myPose(1,j),myPose(2,j),myPose(3,j),map,scanAngles,myOrigin,myResolution);
plot(local(:,1), local(:,2), 'y.'); 
colormap('gray');
axis equal;
hold on;
plot(myPose(1,:)*param.resol+param.origin(1), ...
    myPose(2,:)*param.resol+param.origin(2), 'r.-');   
pause
end
end

end

