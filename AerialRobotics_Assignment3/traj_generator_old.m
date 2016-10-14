function [ desired_state ] = traj_generator_old(T,state,varargin)
%{ TRAJ_GENERATOR: Generate the trajectory passing through all
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
% waypoints: The 3xP matrix listing all the points you must visited in order
% along the generated trajectory. 
% Assumption: Waypoints must be equi-distant. And approximate time with max
% velocity that we need to go from one point to next is known
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

persistent waypoints0 traj_time d0



%% Fill in your code here
persistent coeff tmax 

 desired_state.pos = zeros(3,1);
 desired_state.vel = zeros(3,1);
 desired_state.acc = zeros(3,1);
 desired_state.yaw = 0;
 
 if(nargin>2)
 waypoints=varargin{1};
 waypoints0=waypoints;
 end

 
if (T==0) % Initialisation of trajectory
    
    display('Initializing Trajectory');
%     fprintf('Time passed %d',T);
    
    coeff = getCoff_v2(waypoints0);
    vmax=2;
    n=size(waypoints0,2)-1;
    tmax = zeros(n+1,1);
    display(coeff);
    for i=1:(n)
        dist=sqrt((waypoints0(:,i)-waypoints0(:,i+1))'*(waypoints0(:,i)-waypoints0(:,i+1)));
        tmax(i+1)=tmax(i)+(dist/vmax);
    end
    
elseif (T==inf)
    
    desired_state.pos = waypoints0(:,size(waypoints0,2));
else
    
    %following the trajectory
%     display('Getting Desired point');
%     
%     fprintf('Time passed %d',T);
    syms t;
    eqn = [1 t t^2 t^3 t^4 t^5 t^6 t^7];
    n=(size(waypoints0,2)-1);
    pathnum=0;
    for i=1:(n)
        if(T>tmax(i))
            pathnum=i;
        end
    end
    
    normT=(T-tmax(pathnum))/(tmax(pathnum+1)-tmax(pathnum)); %normalised time
    %deriving equations
    coe = coeff((pathnum-1)*8+1:pathnum*8,:);    
    
    pos_x = vpa(eqn*coe(:,1));
    pos_y = vpa(eqn*coe(:,2));
    pos_z = vpa(eqn*coe(:,3));
    
    desired_state.pos(1,1)=subs(pos_x,t,normT);
    desired_state.pos(2,1)=subs(pos_y,t,normT);
    desired_state.pos(3,1)=subs(pos_z,t,normT);
    
%     fprintf('x=%d y=%d z=%d',desired_state.pos(1,1),desired_state.pos(2,1),desired_state.pos(3,1));
    
    %DEBUGGING BLOCK
    
%     if(T<(10^(-2)))        
%     desired_state.pos(1,1)=1;
%     desired_state.pos(2,1)=1;
%     desired_state.pos(3,1)=1;
%     end
    
    
    vel_x = diff(pos_x,t,1);
    vel_y = diff(pos_y,t,1);
    vel_z = diff(pos_z,t,1);
    
    desired_state.vel(1,1)=subs(vel_x,t,normT);
    desired_state.vel(2,1)=subs(vel_y,t,normT);
    desired_state.vel(3,1)=subs(vel_z,t,normT);
    
    acc_x = diff(pos_x,t,2);
    acc_y = diff(pos_y,t,2);
    acc_z = diff(pos_z,t,2);
    
    desired_state.acc(1,1)=subs(acc_x,t,normT);
    desired_state.acc(2,1)=subs(acc_y,t,normT);
    desired_state.acc(3,1)=subs(acc_z,t,normT);

    fprintf('x=%d y=%d z=%d \n',desired_state.acc(1,1),desired_state.acc(2,1),desired_state.acc(3,1));
    
    

end
       
        
desired_state;
end
% function [T] = polyT(n,k,t)
%      
%         T = zeros(n,1);
%         D = zeros(n,1);
%         
%         % Initial
%         for i=1:n
%             D(i)=i-1;
%             T(i)=1;
%         end
%         
%         %Derivative
%         for  j= 1:k
%             for i= 1:n
%                 T(i)=T(i)*D(i);
%                 if D(i)>0
%                     D(i)=D(i)-1;
%                 end
%             end
%         end
%         
%         %put t value
%         for i=1:n
%             T(i)=T(i)*t^D(i);
%         end
%         T=T';
% end
% function[coff,A,b]=getCoff(waypoints)
%    n=size(waypoints,1)-1;
%    A=zeros(8*n,8*n);
%    b=zeros(1,8*n);
%    for i=1:n
%        b(:,i)=waypoints(:,i);
%        b(:,i+n)=waypoints(:,i+1);
%    end
%    
%    %constraint1 Pi(0)=Wi for all i=1...n
%    row=1;
%    for i=1:n
%        A(row,((i-1)*8)+1:i*8)=polyT(8,0,0);
%        row=row+1;
%    end
%     %constraint2 Pi(1)=Wi+1 for all i=1...n
%    for i=1:n
%        A(row,((i-1)*8)+1:i*8)=polyT(8,0,1);
%        row=row+1;
%    end
%    %constraint3 P1(k)(0)=0 for all 1<=k<=3
%    for i=1:n
%        for k=1:3
%            A(row,((i-1)*8)+1:i*8)=polyT(8,k,0);
%            row=row+1;
%        end
%    end
%     %constraint4 Pn(k)(1)=0 for all 1<=k<=3
%    for n=1:4
%        for k=1:3
%            A(row,((n-1)*8)+1:n*8)=polyT(8,k,1);
%            row=row+1;
%        end
%    end
%     %constraint5 Pi-1(k)(1)=Pi(k)(0) for all 1<=k<=3
%    for i=2:n
%        for k=1:6
%            A(row,((n-1)*16)+1:i*16)=[polyT(8,k,1)-polyT(8,k,0)];
%        end
%    end
%    coff=pinv(A)*b';
% end
% 
%    