function nonlinear_filtering()
load("cooplocalization_finalproj_KFdata.mat");
N = 10;
times_steps = 1000;
dt = 0.1;
x_plus_init = [8, 2, pi/4, -48, 10, 0]';
dx0_mean = [0, 0, 0, 0, 0, 0]';
P_plus_init = diag([1,1,0.1,1,1,0.1]);
alpha_NEES = 0.05;
alpha_NIS = 0.05;
R_EKF = Rtrue;
Q_EKF = diag([0.003,0.003,0.01,0.003,0.003,0.01]);
NEES_array = zeros(N,length(tvec));
NIS_array = zeros(N,length(tvec));
for i=1:N
    fprintf('%d ', i);   
    [gt_x, gt_y] = nonlinear_trajectory_noise(dx0_mean,P_plus_init,times_steps);
    [est_x, est_y, sigma_array, P_plus_array, S_array] = EKF(x_plus_init, P_plus_init, Q_EKF, R_EKF, gt_y, tvec);
    for t=1:length(tvec)
        x_diff = gt_x(:,t) - est_x(:,t);
        x_diff = wrapx(x_diff);
        epsilon_x_k = x_diff'/P_plus_array(:,:,t)*x_diff;
        NEES_array(i,t) = epsilon_x_k;
        y_diff = gt_y(:,t) - est_y(:,t);
        y_diff = wrapy(y_diff);
        epsilon_y_k = y_diff'/S_array(:,:,t)*y_diff;
        NIS_array(i,t) = epsilon_y_k;
    end
end
epsilon_x_k_bar = sum(NEES_array)/N;
r1_x = chi2inv(alpha_NEES/2,N*6)/N;
r2_x = chi2inv(1-alpha_NEES/2,N*6)/N;

figure
subplot(1,2,1)
hold on
t = 0:0.1:times_steps*dt;
plot(t,r1_x*ones(1,length(tvec)), 'r--') % lower bound
plot(t,r2_x*ones(1,length(tvec)), 'r--') % upper bound
scatter(t, epsilon_x_k_bar, 'k')
title('NEES')

epsilon_y_k_bar = sum(NIS_array)/N;
r1_y = chi2inv(alpha_NIS/2,N*5)/N;
r2_y = chi2inv(1-alpha_NIS/2,N*5)/N;
subplot(1,2,2)
hold on
t_2 = 0.1:0.1:times_steps*dt;
plot(t_2,r1_y*ones(1,length(tvec)-1), 'r--') % lower bound
plot(t_2,r2_y*ones(1,length(tvec)-1), 'r--') % upper bound
scatter(t_2, epsilon_y_k_bar(2:end), 'k')
title('NIS')

plot_result_part2(t,est_x, gt_x, gt_y, sigma_array)
end