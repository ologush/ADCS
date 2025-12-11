#include "control_algo.h"
#include <math.h>
#include <stdio.h>

PID_parameters_s attitude_control = {
    .Kp=10,
    .Ki=.01,
    .Kd=0,
    .time_step=.01,
    .error=0,
    .prev_error=0,
    .target=0,
    .measured=0
};

PID_parameters_s spin_control = {
    .Kp=10,
    .Ki=.01,
    .Kd=0,
    .time_step=.01,
    .error=0,
    .prev_error=0,
    .target=0,
    .measured=0
};

algo_target_e algo_target_type = ALGO_OFF;

control_algo_error_e set_attitude_target(float target) {
    attitude_control.target = target;

    return CONTROL_ALGO_ERR_OK;
}

control_algo_error_e set_spin_rate_target(float spin_rate) {
    spin_control.target = spin_rate;

    return CONTROL_ALGO_ERR_OK;
}

control_algo_error_e set_target_type(algo_target_e target_type) {
    algo_target_type = target_type;

    return CONTROL_ALGO_ERR_OK;
}

control_algo_error_e iteration(PID_parameters_s *parameters, float *command) {

    parameters->prev_error = parameters->error;
    parameters->error = parameters->target - parameters->measured;

    float derivative = parameters->error - parameters->prev_error;

    //Leaving out integral for now
    //Command is velocity in rad/s
    *command = parameters->Kp * parameters->error + parameters->Kd * derivative;

    return CONTROL_ALGO_ERR_OK;
}

control_algo_error_e wrap_pi(float input_angle, float *target_angle) {
    while (input_angle > M_PI) input_angle -= 2.0f * M_PI;
    while (input_angle < M_PI) input_angle += 2.0f * M_PI;
    *target_angle = input_angle;

    return CONTROL_ALGO_ERR_OK;
}