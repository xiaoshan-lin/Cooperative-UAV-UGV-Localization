function [x_nom,y_nom] = nominal_traj(time_steps)    
% generate nominal trajectory
% input:  time_steps - N                     integer

% output: x_nom - nominal state              size(6,N)
%         y_nom - nominal measurements       size(5,N)
    % UGV init condition
    xi_g_init = 10;
    eta_g_init = 0;
    theta_g_init = pi/2;
    
    % UAV init condition
    xi_a_init = -60;
    eta_a_init = 0;
    theta_a_init = -pi/2;
    
    % parameter
    dt = 0.1;
    
    % ode45
    time_span = 0:dt:dt*time_steps;
    options = odeset('RelTol',1e-2,'AbsTol',1e-4);
    init_x = [xi_g_init;eta_g_init;theta_g_init;xi_a_init;eta_a_init;theta_a_init];
    [t,x_nom] = ode45(@derivative,time_span,init_x,options);
    % wrap theta to the range of (-pi, pi)
    x_nom = x_nom';
    x_nom = wrapx(x_nom);
    % get the measurements
    y_nom = zeros(5,time_steps+1);
    for i = 1:time_steps+1
        x_i = x_nom(:,i);
        y_nom(:,i) = [atan((x_i(5)-x_i(2))/(x_i(4)-x_i(1))) - x_i(3) + (pi/2)*sign(x_i(5) - x_i(2))*(1 - sign(x_i(4) - x_i(1))), ...
                  sqrt((x_i(4)-x_i(1))^2 + (x_i(5)-x_i(2))^2 ), ...
                  atan((x_i(5)-x_i(2))/(x_i(4)-x_i(1))) - x_i(6) + (pi/2)*sign(x_i(2) - x_i(5))*(1 - sign(x_i(1) - x_i(4))), ...
                  x_i(4), x_i(5)]';
    end
    y_nom = wrapy(y_nom);
    function dxdt = derivative(t,x)
    v_g = 2; phi_g = -pi/18;v_a = 12; omega_a = pi/25; % control inputs
    L = 0.5; % parameter
    dxdt = [v_g*cos(x(3)); v_g*sin(x(3)); v_g/L*tan(phi_g); 
            v_a*cos(x(6)); v_a*sin(x(6)); omega_a];
    end
end
