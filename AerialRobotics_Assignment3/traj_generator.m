function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 poly coeff
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    coeff=getCoff_v2(waypoints);

else
    %syms T;
    %poly=[1 T T^2 T^3 T^4 T^5 T^6 T^7];
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
    else

            scale = (t-traj_time(t_index-1))/d0(t_index-1);
            %desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
            coe = coeff((t_index-2)*8+1:(t_index-1)*8,:);      
%             t
%             pos_x = vpa(poly*coe(:,1));
%             pos_y = vpa(poly*coe(:,2));
%             pos_z = vpa(poly*coe(:,3));
%             desired_state.pos(1,1)=double(subs(pos_x,T,scale));
%             desired_state.pos(2,1)=double(subs(pos_y,T,scale));
%             desired_state.pos(3,1)=double(subs(pos_z,T,scale));
%             
%             vel_x = diff(pos_x,T,1);
%             vel_y = diff(pos_y,T,1);
%             vel_z = diff(pos_z,T,1);
% 
%             desired_state.vel(1,1)=double(subs(vel_x,T,scale));
%             desired_state.vel(2,1)=double(subs(vel_y,T,scale));
%             desired_state.vel(3,1)=double(subs(vel_z,T,scale));
% 
%             acc_x = diff(pos_x,T,2);
%             acc_y = diff(pos_y,T,2);
%             acc_z = diff(pos_z,T,2);
% 
%             desired_state.acc(1,1)=double(subs(acc_x,T,scale));
%             desired_state.acc(2,1)=double(subs(acc_y,T,scale));
%             desired_state.acc(3,1)=double(subs(acc_z,T,scale));
        desired_state.pos(:,1)=polyT(8,0,scale)*coe;
        desired_state.vel(:,1)=polyT(8,1,scale)*coe;
        desired_state.acc(:,1)=polyT(8,2,scale)*coe;
         fprintf('x=%d y=%d z=%d \n',desired_state.pos(1,1),desired_state.pos(2,1),desired_state.pos(3,1));
    end
    %desired_state.vel = zeros(3,1);
    %desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%


%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

