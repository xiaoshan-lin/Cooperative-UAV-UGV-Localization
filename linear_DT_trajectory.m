function [x_DT,y_DT] = linear_DT_trajectory(disturb, x_nom, time_steps)
% input:  disturb - disturbance of state x at t = 0
%         x_nom - nominal state traejectory
%         time_steps - number of time steps                      integer

% output: x_DT - linearized dicretized state trajectory
%         y_DT - linearized dicretized measurement trajectory

% parameter
    v_g = 2; v_a = 12; 
    dt = 0.1;
    x_DT = zeros(6,time_steps+1);
    y_DT = zeros(5, time_steps+1);
    dx = disturb;
    for idx = 1:time_steps+1
        if idx == 1
            A_tilt = zeros(6);
        else
            theta_g = x_nom(3,idx);
            theta_a = x_nom(6,idx);
            A_tilt = [0,0,-v_g*sin(theta_g),0,0,0;   % compute A_tilt as shown in Part I-1
                      0,0,v_g*cos(theta_g),0,0,0;
                      0,0,0,0,0,0;
                      0,0,0,0,0,-v_a*sin(theta_a);
                      0,0,0,0,0,v_a*cos(theta_a);
                      0,0,0,0,0,0]; 
        end
        F_tilt = eye(6) + dt*A_tilt;
        dx = F_tilt*dx;
        x_DT(:,idx) = dx;
        
        R = (x_nom(4,idx)-x_nom(1,idx))^2 + (x_nom(5,idx)-x_nom(2,idx))^2; % see solution of Part I-1
        C_tilt = [[x_nom(5,idx)-x_nom(2,idx), x_nom(1,idx)-x_nom(4,idx), -R, x_nom(2,idx)-x_nom(5,idx),x_nom(4,idx)-x_nom(1,idx), 0]/R;
                  [x_nom(1,idx)-x_nom(4,idx), x_nom(2,idx)-x_nom(5,idx), 0, x_nom(4,idx)-x_nom(1,idx), x_nom(5,idx)-x_nom(2,idx), 0]/sqrt(R);
                  [x_nom(5,idx)-x_nom(2,idx), x_nom(1,idx)-x_nom(4,idx), 0, x_nom(2,idx)-x_nom(5,idx),x_nom(4,idx)-x_nom(1,idx), -R]/R;
                  [0,0,0,1,0,0];
                  [0,0,0,0,1,0]]; % compute C_tilt as shown in Part I-1
        dy = C_tilt*dx;
        y_DT(:,idx) = dy;
    end
    x_DT = wrapx(x_DT);
    y_DT = wrapy(y_DT);
end