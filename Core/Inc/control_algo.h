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

#define RW_MOI 1.0f //placeholder
#define SAT_MOI 1.0f //placeholder

static Quaternion target_attitude;
static float target_spin_rate;
static algo_target_e algo_target_type;


control_algo_error_e set_attitude_target(Quaternion *attitude);
control_algo_error_e set_spin_rate_target(float spin_rate);
control_algo_error_e set_target_type(algo_target_e target_type);

control_algo_error_e spin_rate_iteration(float current_spin_rate);
control_algo_error_e attitude_iteration(Quaternion *current_attitude);

#endif