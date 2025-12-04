// Includes

#include "motor_control.h"
#include <math.h>


/* Private function definitions */

static motor_ctrl_err_e i2c_write_data_word(i2c_data_word_s *data_word)
{
    
}

/* Global function definitions */

motor_ctrl_err_e motor_ctrl_init(I2C_HandleTypeDef *hi2c)
{
    hi2c_motor_ctrl = hi2c;
    return MOTOR_CTRL_ERR_OK;
}