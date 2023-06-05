function linearized_state_and_measurement()
disturb = [-0.25, -0.3, pi/10, 6, -7, pi/10]';
times_steps = 1000;
dt = 0.1;
[x_nom, y_nom] = nominal_traj(times_steps);
[x_full, y_full] = nonlinear_trajectory(disturb, times_steps);
[x_DT, y_DT] = linear_DT_trajectory(disturb, x_nom, times_steps);
x_approx = x_nom + x_DT;
x_approx = wrapx(x_approx);
y_approx = y_nom + y_DT;
y_approx = wrapy(y_approx);
t = 0:0.1:times_steps*dt;
plot_result_part1(t,x_full, y_full, x_approx, y_approx);    
end