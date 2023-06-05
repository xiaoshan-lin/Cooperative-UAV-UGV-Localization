# Cooperative-UAV-UGV-Localization with Extended Kalman Filter

As a part of the final project for UMN AEM 5451 Optimal Esitimation, these codes are developed to cooperatively estimate the pose of a UAV and a UGV, given noisy ranges and azimuth angles of the UGV relative to the UAV, noisy azimuth angles of  the UAV relative to the UGV, and noisy UAV GPS measurements.

**Authors:** Xiaoshan Lin

## Table of content
- [Usage](#usage)
- [Acknowledgments](#acknowledgments)

## Usage
Open the code in MATLAB. Run the `main` function to execute the codes. There are several modules in the `main` function: 
- `linearized_state_and_measurement()` simulates the linearized discrete-time dynamics and measurement models near the nominal trajectory, assuming a reasonable initial state perturbation and assuming no process noise, measurement noise, or control input perturbations. It plots
    + lienarized states vs full nonlinear states
    + lienarized measurements vs actual measurements
- `nonlinear_filtering()`use EKF to estimate UAV and UGV states from noisy measurements. It plots
    + estimated states vs full-dynamics nonlinear states 
    + estimation error and 2-sigma bound
    + noisy measurements
    + NIS and NEES test results

## Acknowledgments

- Special thanks to my teammate Nathan Bich.
