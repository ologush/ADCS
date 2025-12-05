// Includes

#include "motor_control.h"
#include <math.h>


/* Private function definitions */

static motor_ctrl_err_e motor_write_data_word(i2c_data_word_s *data_word)
{
    uint8_t data_length = 4;
    
    switch(data_word->control_word.d_len) {
        case 0:
            data_length += 2;
            break;
        case 1:
            data_length += 4;
            break;
        case 2:
            data_length += 8;
            break;
        default:
            return MOTOR_CTRL_ERR_ERROR;
            break;
    }

    switch(data_word->control_word.crc_en) {
        case 0:
            break;
        case 1:
            data_length += 1;
            break;
        default:
            return MOTOR_CTRL_ERR_ERROR;
            break;
    }

    uint8_t tx_buffer[data_length];

    tx_buffer[0] = (data_word->target_id << 1) | (data_word->read_write_bit & 0x01);
    tx_buffer[1] = (data_word->control_word.op_rw & 0x01) << 7  |
                   (data_word->control_word.crc_en & 0x01) << 6 |
                   (data_word->control_word.d_len & 0x03) << 4 |
                   (data_word->control_word.mem_sec & 0x0F);
    tx_buffer[2] = (data_word->control_word.mem_page & 0x0F) << 4 | 
                    ((data_word->control_word.mem_addr >> 8) & 0x0F);
    tx_buffer[3] = data_word->control_word.mem_addr & 0xFF;
    if(data_length % 2 == 0) {
        for(uint8_t i = 0; i < (data_length - 4); i++) {
            tx_buffer[4 + i] = data_word->data[i];
        }
    } else {
        for(uint8_t i = 0; i < (data_length - 5); i++) {
            tx_buffer[4 + i] = data_word->data[i];
        }
        tx_buffer[data_length - 1] = data_word->crc;
    }

    HAL_I2C_Master_Transmit(hi2c_motor_ctrl, tx_buffer[0], tx_buffer + 1, data_length - 1, HAL_MAX_DELAY);
    return MOTOR_CTRL_ERR_OK;
}

static motor_ctrl_err_e motor_read_data_word(i2c_data_word_s *data_word_tx, uint8_t *receive_buffer) {

    uint8_t data_length = 0;
    
    switch(data_word_tx->control_word.d_len) {
        case 0:
            data_length += 2;
            break;
        case 1:
            data_length += 4;
            break;
        case 2:
            data_length += 8;
            break;
        default:
            return MOTOR_CTRL_ERR_ERROR;
            break;
    }

    switch(data_word_tx->control_word.crc_en) {
        case 0:
            break;
        case 1:
            data_length += 1;
            break;
        default:
            return MOTOR_CTRL_ERR_ERROR;
            break;
    }

    uint8_t tx_buffer[4];
    uint8_t rx_buffer[data_length - 1];

    tx_buffer[0] = (data_word_tx->target_id << 1) | (data_word_tx->read_write_bit & 0x01);
    tx_buffer[1] = (data_word_tx->control_word.op_rw & 0x01) << 7  |
                   (data_word_tx->control_word.crc_en & 0x01) << 6 |
                   (data_word_tx->control_word.d_len & 0x03) << 4 |
                   (data_word_tx->control_word.mem_sec & 0x0F);
    tx_buffer[2] = (data_word_tx->control_word.mem_page & 0x0F) << 4 | 
                    ((data_word_tx->control_word.mem_addr >> 8) & 0x0F);
    tx_buffer[3] = data_word_tx->control_word.mem_addr & 0xFF;

    HAL_I2C_Master_Transmit(hi2c_motor_ctrl, tx_buffer[0], tx_buffer + 1, 3, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c_motor_ctrl, tx_buffer[0] | 0x01, rx_buffer, data_length - 1, HAL_MAX_DELAY);

    for(uint8_t i = 0; i < (data_length); i++) {
            receive_buffer[i] = rx_buffer[i];
    }
}

/* Global function definitions */

motor_ctrl_err_e motor_ctrl_init(I2C_HandleTypeDef *hi2c)
{
    hi2c_motor_ctrl = hi2c;
    return MOTOR_CTRL_ERR_OK;
}