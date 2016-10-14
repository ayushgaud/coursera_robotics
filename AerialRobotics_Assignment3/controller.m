function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
Kv=[8;9;10];
Kp=[180;180;550];
% Thrust
R_1=(des_state.acc(1)+Kv(1)*(des_state.vel(1)-state.vel(1))+Kp(1)*(des_state.pos(1)-state.pos(1)));
R_2=(des_state.acc(2)+Kv(2)*(des_state.vel(2)-state.vel(2))+Kp(2)*(des_state.pos(2)-state.pos(2)));
R_3=(des_state.acc(3)+Kv(3)*(des_state.vel(3)-state.vel(3))+Kp(3)*(des_state.pos(3)-state.pos(3)));
F = params.mass*(params.gravity+R_3);
 if F > params.maxF
     F=params.maxF;
 end
% Moment

des_rot=[(1/params.gravity)*(R_1*(des_state.yaw) - R_2); (1/params.gravity)*(R_1+ R_2*(des_state.yaw)); des_state.yaw];
%M=zeros(3,1);
M =[50;50;50].*(des_rot-state.rot) + [2;2;2].*([0;0;des_state.yawdot]-state.omega);

% =================== Your code ends here ===================

end
