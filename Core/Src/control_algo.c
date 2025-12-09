#include "control_algo.h"
#include <stdio.h>


control_algo_error_e set_attitude_target(Quaternion *attitude) {
    target_attitude = *attitude;

    return CONTROL_ALGO_ERR_OK;
}

control_algo_error_e set_spin_rate_target(float spin_rate) {
    target_spin_rate = spin_rate;

    return CONTROL_ALGO_ERR_OK;
}

control_algo_error_e set_target_type(algo_target_e target_type) {
    algo_target_type = target_type;

    return CONTROL_ALGO_ERR_OK;
}

control_algo_error_e spin_rate_iteration(float current_spin_rate) {

}

control_algo_error_e attitude_iteration(Quaterion *current_attitude) {

}