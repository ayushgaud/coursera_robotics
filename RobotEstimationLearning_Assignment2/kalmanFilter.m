function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
        P = eye(4);
        R = eye(2);
        dt=1;
        A=[1,0,dt,0;0,1,0,dt;0,0,1,0;0,0,0,1];
        C=[1,0,0,0;
                0,1,0,0];
        sig_m=diag([1e1,1e1,1e1,1e1]);
        sig_o=diag([0.01,0.01]);
    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    P=A*param.P*A'+sig_m;
    R=C*P*C'+sig_o;
    K=P*C'*inv(R+C*P*C');
    
%     vx = (x - state(1)) / (t - previous_t);
%     vy = (y - state(2)) / (t - previous_t);
%     % Predict 330ms into the future
%     predictx = x + vx * 0.330;
%     predicty = y + vy * 0.330;
    temp=A*state'+K*([x;y]-C*A*state');
   
    vx=temp(3);
    vy=temp(4);
    predictx=temp(1)+vx*10*dt
    predicty=temp(2)+vy*10*dt
    [predictx, predicty, x,y];
    param.P=P-K*C*P;
    % State is a four dimensional element
    %state=temp';
    state = [x, y, vx, vy];
end