#include "control_algo.h"
#include <math.h>
#include <stdio.h>

const PID_parameters_s attitude_control = {
    .Kp=10.0f,
    .Ki=.01f,
    .Kd=0.0f
};

const PID_parameters_s spin_control = {
    .Kp=10.0f,
    .Ki=.01f,
    .Kd=0.0f
};

const PID_parameters_s no_control = {
    .Kp=0.0f,
    .Ki=0.0f,
    .Kd=0.0f
};

static PID_frame_s control_frame = {
    .error = 0,
    .prev_error = 0,
    .pid_params = no_control,
    .target = {
        .target_type = ALGO_OFF,
        .target_value = 0
    }
};

control_algo_error_e update_target(algo_target_s new_target) {
    switch(new_target.target_type) {
        case ALGO_TARGET_ATTITUDE:
            control_frame.pid_params = attitude_control;
            break;
        case ALGO_TARGET_SPIN_RATE:
            control_frame.pid_params = spin_control;
            break;
        case ALGO_OFF:
            control_frame.pid_params = no_control;
            break;
        default:
            return CONTROL_ALGO_ERR_ERR;
            break;
    }

    control_frame.target = new_target;
    control_frame.error = 0;
    control_frame.prev_error = 0;

    return CONTROL_ALGO_ERR_OK;
}

control_algo_error_e PID_iteration(float measured_value, float *command) {

    control_frame.prev_error = control_frame.error;
    control_frame.error = control_frame.target.target_value - measured_value;

    float derivative = control_frame.error - control_frame.prev_error;

    // Just PD control for now
    *command = (control_frame.pid_params.Kp * control_frame.error) +
               (control_frame.pid_params.Kd * derivative / PID_TIME_STEP);

    return CONTROL_ALGO_ERR_OK;
}

algo_target_type_e get_target_type() {
    return control_frame.target.target_type;
}

control_algo_error_e wrap_pi(float input_angle, float *target_angle) {
    while (input_angle > M_PI) input_angle -= 2.0f * M_PI;
    while (input_angle < M_PI) input_angle += 2.0f * M_PI;
    *target_angle = input_angle;

    return CONTROL_ALGO_ERR_OK;
}