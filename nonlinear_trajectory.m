function [x_full, y_full] = nonlinear_trajectory(disturb, time_steps)    
% generate nonlinear trajectory
% input:  disturb - disturbance of x at t = 0     size(6,1)
%         time_steps - number of time steps       integer

% output: x_full - nominal state                  size(6,N)
%         y_full - nominal measurements           ize(5,N) 
    % UGV init condition
    xi_g_init = 10 + disturb(1);
    eta_g_init = 0 + disturb(2);
    theta_g_init = pi/2 + disturb(3);
    
    % UAV init condition
    xi_a_init = -60 + disturb(4);
    eta_a_init = 0 + disturb(5);
    theta_a_init = -pi/2 + disturb(6);
    
    % parameter
    dt = 0.1;
    
    % ode45
    time_span = 0:dt:dt*time_steps;
    %options = odeset('RelTol',1e-2,'AbsTol',1e-3);
    init_x = [xi_g_init;eta_g_init;theta_g_init;xi_a_init;eta_a_init;theta_a_init];
    [t,x_full] = ode45(@derivative,time_span,init_x);
    x_full = x_full';
    % wrap theta to the range of (-pi, pi)
    x_full = wrapx(x_full);
    % get the measurements
    y_full = zeros(5,time_steps+1);
    for i = 1:time_steps+1
        x_i = x_full(:,i);
        y_full(:,i) = [atan((x_i(5)-x_i(2))/(x_i(4)-x_i(1))) - x_i(3) + (pi/2)*sign(x_i(5) - x_i(2))*(1 - sign(x_i(4) - x_i(1))), ...
                  sqrt((x_i(4)-x_i(1))^2 + (x_i(5)-x_i(2))^2 ), ...
                  atan((x_i(5)-x_i(2))/(x_i(4)-x_i(1))) - x_i(6) + (pi/2)*sign(x_i(2) - x_i(5))*(1 - sign(x_i(1) - x_i(4))), ...
                  x_i(4), x_i(5)]';
    end
    y_full = wrapy(y_full);
    function dxdt = derivative(t,x)
    v_g = 2; phi_g = -pi/18;v_a = 12; omega_a = pi/25; % control inputs
    L = 0.5; % parameter
    dxdt = [v_g*cos(x(3)); v_g*sin(x(3)); v_g/L*tan(phi_g); 
            v_a*cos(x(6)); v_a*sin(x(6)); omega_a];
    end
end
