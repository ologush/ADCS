#ifndef CONTROL_ALGO
#define CONTROL_ALGO

#include "ism330bx.h"

#define PID_TIME_STEP 0.01f

typedef enum {
    CONTROL_ALGO_ERR_OK,
    CONTROL_ALGO_ERR_ERR
} control_algo_error_e;

typedef enum {
    ALGO_OFF,
    ALGO_TARGET_SPIN_RATE,
    ALGO_TARGET_ATTITUDE
} algo_target_type_e;

typedef struct {
    algo_target_type_e target_type;
    float target_value;
} algo_target_s;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_parameters_s;

typedef struct {
    float error;
    float prev_error;
    PID_parameters_s pid_params;
    algo_target_s target;
} PID_frame_s;

#define RW_MOI 1.0f //placeholder
#define SAT_MOI 1.0f //placeholder

control_algo_error_e update_target(algo_target_s new_target);
control_algo_error_e PID_iteration(float measured_value, float *command);
control_algo_error_e wrap_pi(float input_angle, float *target_angle);

algo_target_type_e get_target_type(void);

#endif