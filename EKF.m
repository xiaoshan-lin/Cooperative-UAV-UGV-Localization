function [est_x, est_y, sigma_array, P_plus_array, S_array] = EKF(x_plus_init, P_plus_init, Qtilde, Rtilde, gt_y, tvec)
% Extended Kalman Filter
% input:  x_plus_init - init x              size(6,1)
%         P_plus_init - init P              size(6,6)
%         Qtilde - process noise            size(6,6)
%         Rtilde - measurement noise        size(5,5)
%         gt_y - ground-truth y             size(5,N)
%         tvec - time sequences             size(1,N)
% output: est_x - state esimation           size(6,N)
%         sigma_array - sigma bound         size(6,N)

    % estimated x
    est_x = zeros(6,length(tvec));
    est_y = zeros(5,length(tvec));
    sigma_array = zeros(6,length(tvec));
    P_plus_array = zeros(6,6,length(tvec));
    S_array = zeros(5,5,length(tvec));
    S_array(:,:,1) = eye(5);
    
    % x_plus
    x_plus = x_plus_init;
    x_plus = wrapx(x_plus);
    est_x(:,1) = x_plus;

    % P_plus
    P_plus = P_plus_init;
    P_plus_array(:,:,1) = P_plus;
    
    sigma_array(:,1) = sqrt(diag(P_plus));
    sigma_array(:,1) = wrapx(sigma_array(:,1));
    utt = [2; -pi/18; 12; pi/25];
    L = 0.5; % parameter
    dt = 0.1;
    for i=2:length(tvec)
        [Ttt, Xtt] = ode45(@(t,x) derivative(t,x,utt,L),[0 dt],x_plus);
        x_minus = Xtt(end,:)';
        x_minus = wrapx(x_minus);
        
        A_tilt = [0,0,-utt(1)*sin(x_plus(3)),0,0,0;   % compute A_tilt as shown in Part I-1
                      0,0,utt(1)*cos(x_plus(3)),0,0,0;
                      0,0,0,0,0,0;
                      0,0,0,0,0,-utt(3)*sin(x_plus(6));
                      0,0,0,0,0,utt(3)*cos(x_plus(6));
                      0,0,0,0,0,0]; 
        F_tilt = eye(6) + dt*A_tilt;
        % Omega
        Gamma_tilt = eye(6);
        Omega_tilt = dt*Gamma_tilt;
        % P
        P_minus = F_tilt*P_plus*F_tilt' + Omega_tilt*Qtilde*Omega_tilt';
        % H
        R = (x_minus(4)-x_minus(1))^2 + (x_minus(5)-x_minus(2))^2; % see solution of Part I-1
        H_tilt = [[x_minus(5)-x_minus(2), x_minus(1)-x_minus(4), -R, x_minus(2)-x_minus(5),x_minus(4)-x_minus(1), 0]/R;
                  [x_minus(1)-x_minus(4), x_minus(2)-x_minus(5), 0, x_minus(4)-x_minus(1), x_minus(5)-x_minus(2), 0]/sqrt(R);
                  [x_minus(5)-x_minus(2), x_minus(1)-x_minus(4), 0, x_minus(2)-x_minus(5),x_minus(4)-x_minus(1), -R]/R;
                  [0,0,0,1,0,0];
                  [0,0,0,0,1,0]]; % compute H_tilt as shown in Part I-1

        % S_k
        S_k = H_tilt*P_minus*H_tilt' + Rtilde;
        S_array(:,:,i) = S_k;
        K = P_minus*H_tilt'/S_k;
        y_hat = zero_noise_measurement(x_minus);
        y_hat = wrapy(y_hat);
        est_y(:,i) = y_hat;
        y_diff = gt_y(:,i)-y_hat;
        y_diff = wrapy(y_diff);
        x_plus = x_minus + K*y_diff;
        x_plus = wrapx(x_plus);
        P_plus = (eye(6)-K*H_tilt)*P_minus;
        P_plus_array(:,:,i) = P_plus;
        est_x(:,i) = x_plus;
        sigma_array(:,i) = sqrt(diag(P_plus));
        sigma_array(:,i) = wrapx(sigma_array(:,i));
    
    end
    est_x = wrapx(est_x);

    function dxdt = derivative(t, x, utt, L)
        dxdt = [utt(1)*cos(x(3)); utt(1)*sin(x(3)); utt(1)/L*tan(utt(2)); 
                utt(3)*cos(x(6)); utt(3)*sin(x(6)); utt(4)];
    end

    function yy = zero_noise_measurement(xx)
        yy = [atan((xx(5)-xx(2))/(xx(4)-xx(1))) - xx(3) + (pi/2)*sign(xx(5) - xx(2))*(1 - sign(xx(4) - xx(1))), ... 
                  sqrt((xx(4)-xx(1))^2 + (xx(5)-xx(2))^2), ...
                  atan((xx(5)-xx(2))/(xx(4)-xx(1))) - xx(6) + (pi/2)*sign(xx(2) - xx(5))*(1 - sign(xx(1) - xx(4))), ...
                  xx(4), xx(5)]';
    end
end

