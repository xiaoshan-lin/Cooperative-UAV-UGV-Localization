% generate truth model trajectory
function [x, y] = nonlinear_trajectory_noise(dx_0,P_0,time_steps) 
    %Generate inital disturbance:
    disturb = mvnrnd(dx_0,P_0)';
    %Enable/disable Noise (set to 0 or 1):
    procNoiseOn = 1;
    measNoiseOn = 1;
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
    
    %Generate process noise:
    Qtrue = [0.001 0 0 0 0 0; 0 0.001 0 0 0 0; 0 0 0.01 0 0 0; 0 0 0 0.001 0 0; 0 0 0 0 0.001 0; 0 0 0 0 0 0.01];
    mu_w = [0 0 0 0 0 0]';
    w = procNoiseOn*(mvnrnd(mu_w,Qtrue,time_steps+1));
    
    % Generate measurement noise:
    Rtrue = [0.0225 0 0 0 0; 0 64 0 0 0 ; 0 0 0.04 0 0 ; 0 0 0 36 0; 0 0 0 0 36];
    mu_v = [0 0 0 0 0]';
    v = measNoiseOn*mvnrnd(mu_v,Rtrue,time_steps+1);
    
    % ode45
    xtt0 = [xi_g_init eta_g_init theta_g_init xi_a_init eta_a_init theta_a_init];
    xtt0 = wrapx(xtt0')';
    x = zeros(time_steps+1,6);
    x(1,:) = xtt0;
    time_span = [0 dt];
    %options = odeset('RelTol',1e-2,'AbsTol',1e-3);
    for tt=1:time_steps
    wtt = w(tt,:);
    [~,xtt] = ode45(@(t,x) derivative(t,x,wtt),time_span,xtt0);
    xtt = wrapx(xtt')';
    xtt = (xtt(length(xtt),:));
    x(tt+1,:) = xtt;
    xtt0 = xtt;
    end
    x = wrapx(x')';
    
    % get the measurements
    y = zeros(time_steps+1, 5);
     for i = 1:time_steps+1
        x_i = x(i,:);
        y(i,:) = [atan2( (x_i(5)-x_i(2)),(x_i(4)-x_i(1))) - x_i(3) + v(i,1), ...
                  sqrt( (x_i(4)-x_i(1))^2 + (x_i(5)-x_i(2))^2 ) + v(i,2), ...
                  atan2( (x_i(2)-x_i(5)),(x_i(1)-x_i(4))) - x_i(6) + v(i,3), ...
                  x_i(4) + v(i,4), x_i(5) + v(i,5)];
     end
    %Wrap Angles to [-pi pi]:
    x = wrapx(x');
    y = wrapy(y');
    
    function dxdt = derivative(~,x,w)
    v_g = 2; phi_g = -pi/18;v_a = 12; omega_a = pi/25; % control inputs
    L = 0.5; % parameter
    dxdt = [v_g*cos(x(3)) + w(1); v_g*sin(x(3)) + w(2); v_g/L*tan(phi_g) + w(3); 
            v_a*cos(x(6)) + w(4); v_a*sin(x(6)) + w(5); omega_a + w(6)];
    dxdt = wrapx(dxdt);
    end

end
