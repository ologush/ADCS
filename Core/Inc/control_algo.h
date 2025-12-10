#ifndef CONTROL_ALGO
#define CONTROL_ALGO

#include "ism330bx.h"

typedef enum {
    CONTROL_ALGO_ERR_OK,
    CONTROL_ALGO_ERR_ERR
} control_algo_error_e;

typedef enum {
    ALGO_OFF,
    ALGO_TARGET_SPIN_RATE,
    ALGO_TARGET_ATTITUDE
} algo_target_e;

//Just PD for now
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float time_step;
    float error;
    float prev_error;
    float target;
    float measured;
} PID_paramaters_s;

#define RW_MOI 1.0f //placeholder
#define SAT_MOI 1.0f //placeholder




PID_parameters_s attitude_PID;
PID_parameters_s spin_rate_PID;

extern PID_parameters_s attitude_control;
extern PID_parameters_s spin_control;
extern algo_target_e algo_target_type;
extern float target_attitude;
extern float target_spin_rate;

control_algo_error_e set_attitude_target(float target);
control_algo_error_e set_spin_rate_target(float spin_rate);
control_algo_error_e set_target_type(algo_target_e target_type);

control_algo_error_e iteration(PID_parameters_s *parameters, float *command);

control_algo_error_e wrap_pi(float input_angle, float *target_angle);

#endif