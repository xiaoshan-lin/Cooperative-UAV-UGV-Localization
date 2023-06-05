%% lienarized states and measurements given disturbance (without noise)
   % plot lienarized states vs full nonlinear states
   % plot lienarized measurements vs actual measurements
linearized_state_and_measurement();

%% run EKF to estimate states from noisy measurements
   % plot estimated states vs full-dynamics nonlinear states
   % plot estimation error and 2-sigma bound
   % plot noisy measurements
   % plot NIS and NEES test results
nonlinear_filtering()