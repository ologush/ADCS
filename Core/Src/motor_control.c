// Includes

#include "motor_control.h"
#include <math.h>
#include "motor_reg_defs.h"

/* Private Variables */
eeprom_register_s eeprom_default_config[] = {
    {MCF8315_EEPROM_ISD_CONFIG_REG,        0x7C700484},
    {MCF8315_EEPROM_REV_DRIVE_CONFIG_REG,  0x00000000},
    {MCF8315_EEPROM_MOTOR_STARTUP_1_REG,   0x40000096},
    {MCF8315_EEPROM_MOTOR_STARTUP_2_REG,   0x00000000},
    {MCF8315_EEPROM_CLOSED_LOOP1_REG,      0x00000000},
    {MCF8315_EEPROM_CLOSED_LOOP2_REG,      0x00000000},
    {MCF8315_EEPROM_CLOSED_LOOP3_REG,      0x00000000},
    {MCF8315_EEPROM_MOTOR_PARAMS_REG,      0x00000000},
    {MCF8315_EEPROM_REF_PROFILES1_REG,     0x00000000},
    {MCF8315_EEPROM_REF_PROFILES2_REG,     0x00000000},
    {MCF8315_EEPROM_REF_PROFILES3_REG,     0x00000000},
    {MCF8315_EEPROM_REF_PROFILES4_REG,     0x00000000},
    {MCF8315_EEPROM_REF_PROFILES5_REG,     0x00000000},
    {MCF8315_EEPROM_REF_PROFILES6_REG,     0x00000000}
};
/* Private function definitions */

static MOTOR_ERRORS_e motor_write_data_word(motor_data_word_s *data_word)
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

    tx_buffer[0] = (uint8_t)((data_word->target_id << 1) | (data_word->read_write_bit & 0x01));
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
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Transmit(hi2c_motor_ctrl, tx_buffer[0], tx_buffer + 1, data_length - 1, HAL_MAX_DELAY);
    if(ret != HAL_OK) {
        return MOTOR_CTRL_ERR_ERROR;
    }
    return MOTOR_CTRL_ERR_OK;
}

static MOTOR_ERRORS_e motor_read_data_word(motor_data_word_s *data_word_tx, uint8_t *receive_buffer) {

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

static MOTOR_ERRORS_e calculate_crc(motor_data_word_s *data_word) {

    /* Todo: Implement Algorithm */
    return MOTOR_CTRL_ERR_OK;
}



static MOTOR_ERRORS_e read_eeprom_config(uint32_t *config_data) {
    motor_data_word_s retreival_data_word;

    //Write 0x40000000 to register 0x0000EA to read the EEPROM data into the shadow/RAM registers 0x000080 to 0x0000AE

    retreival_data_word.target_id = MCF8315D_I2C_ADDRESS;
    retreival_data_word.read_write_bit = OP_RW_WRITE;
    retreival_data_word.control_word.op_rw = OP_RW_WRITE;
    retreival_data_word.control_word.crc_en = CRC_EN_DISABLE;
    retreival_data_word.control_word.d_len = D_LEN_32_BIT;
    retreival_data_word.data[0] = 0x40;
    retreival_data_word.data[1] = 0x00;
    retreival_data_word.data[2] = 0x00;
    retreival_data_word.data[3] = 0x00;
    retreival_data_word.control_word.mem_sec = 0;
    retreival_data_word.control_word.mem_page = 0;
    retreival_data_word.control_word.mem_addr = 0xEA;

    motor_write_data_word(&retreival_data_word);

    HAL_Delay(100); //Wait for EEPROM read to complete

    for(uint8_t i = MCF8315_EEPROM_ISD_CONFIG_REG; i<= MCF8315_EEPROM_GD_CONFIG2_REG; i+=2) {
        retreival_data_word.target_id = MCF8315D_I2C_ADDRESS;
        retreival_data_word.read_write_bit = OP_RW_WRITE;
        retreival_data_word.control_word.op_rw = OP_RW_READ;
        retreival_data_word.control_word.crc_en = CRC_EN_DISABLE;
        retreival_data_word.control_word.d_len = D_LEN_32_BIT;
        retreival_data_word.control_word.mem_sec = 0;
        retreival_data_word.control_word.mem_page = 0;
        retreival_data_word.control_word.mem_addr = i;

        uint8_t receive_buffer[4];
        motor_read_data_word(&retreival_data_word, receive_buffer);

        uint8_t index = (i - MCF8315_EEPROM_ISD_CONFIG_REG) / 2;
        uint32_t value = (uint32_t)((receive_buffer[0]) << 24 | (receive_buffer[1] << 16) | (receive_buffer[2] << 8) | (receive_buffer[3]));

        config_data[(i - MCF8315_EEPROM_ISD_CONFIG_REG) / 2] = (uint32_t)((receive_buffer[0]) << 24 | (receive_buffer[1] << 16) | (receive_buffer[2] << 8) | (receive_buffer[3]));
    }
}

/* Global function definitions */


MOTOR_ERRORS_e motor_ctrl_init(I2C_HandleTypeDef *hi2c)
{
    hi2c_motor_ctrl = hi2c;
    uint32_t config_data[14];
    read_eeprom_config(config_data);
    return MOTOR_CTRL_ERR_OK;
}

MOTOR_ERRORS_e motor_startup_sequence(void) {
    //First set speed ref > 0
    uint32_t motor_speed = 100.0f;
    motor_set_speed(motor_speed);

}

MOTOR_ERRORS_e motor_set_speed(float speed_rpm) {

    if(speed_rpm > MAX_RPM) {
        return MOTOR_CTRL_ERR_ERROR;
    }

    uint32_t speed_mask = 0x8000FFFF; // Mask to clear speed bits

    //Convert speed from RPM to register value
    uint16_t speed = (uint16_t)roundf((speed_rpm / MAX_RPM) * 65535.0f);

    motor_data_word_s retreival_data_word;
    uint32_t current_register_value;
    
    retreival_data_word.target_id = MCF8315D_I2C_ADDRESS;
    retreival_data_word.read_write_bit = OP_RW_WRITE;
    retreival_data_word.control_word.op_rw = OP_RW_READ;
    retreival_data_word.control_word.crc_en = CRC_EN_DISABLE;
    retreival_data_word.control_word.d_len = D_LEN_32_BIT;
    retreival_data_word.control_word.mem_sec = 0;
    retreival_data_word.control_word.mem_page = 0;
    retreival_data_word.control_word.mem_addr = MCF8315_ALGO_DEBUG1_REG;

    motor_read_data_word(&retreival_data_word, (uint8_t*)&current_register_value);

    current_register_value &= speed_mask;
    current_register_value |= ((uint32_t)speed << 16);

    motor_data_word_s tx_data_word;
    tx_data_word.target_id = MCF8315D_I2C_ADDRESS; // Example target ID
    tx_data_word.read_write_bit = OP_RW_WRITE;
    tx_data_word.control_word.op_rw = OP_RW_WRITE;
    tx_data_word.control_word.crc_en = CRC_EN_DISABLE;
    tx_data_word.control_word.d_len = D_LEN_32_BIT;
    tx_data_word.control_word.mem_sec = 0;
    tx_data_word.control_word.mem_page = 0;
    tx_data_word.control_word.mem_addr = MCF8315_ALGO_DEBUG1_REG;

    motor_write_data_word(&tx_data_word);

    return MOTOR_CTRL_ERR_OK;
}

MOTOR_ERRORS_e motor_get_speed(float *speed_rpm) {
    motor_data_word_s speed_access;

    speed_access.target_id = MCF8315D_I2C_ADDRESS;
    speed_access.read_write_bit = OP_RW_WRITE;
    speed_access.control_word.op_rw = OP_RW_READ;
    speed_access.control_word.crc_en = CRC_EN_DISABLE;
    speed_access.control_word.d_len = D_LEN_32_BIT;
    speed_access.control_word.mem_sec = 0;
    speed_access.control_word.mem_page = 0;
    speed_access.control_word.mem_addr = MCF8315_SPEED_FDBK_REG;

    int32_t register_value;

    motor_read_data_word(&speed_access, (uint8_t*)&register_value);

    *speed_rpm = (register_value/pow(2, 27)) * MAX_SPEED * 60;
}

MOTOR_ERRORS_e extract_motor_params(motor_parameters_s *extracted_params) {

    motor_data_word_s extract_word;
    extract_word.target_id = MCF8315D_I2C_ADDRESS;
    extract_word.read_write_bit = OP_RW_WRITE;
    extract_word.control_word.op_rw = OP_RW_READ;
    extract_word.control_word.crc_en = CRC_EN_DISABLE;
    extract_word.control_word.d_len = D_LEN_32_BIT;
    extract_word.control_word.mem_sec = 0;
    extract_word.control_word.mem_page = 0;
    extract_word.control_word.mem_addr = MCF8315_MTR_PARAMS_REG;

    uint8_t motor_params_data[4];

    motor_read_data_word(&extract_word, motor_params_data);

    extracted_params->motor_inductance_hex = motor_params_data[1];
    extracted_params->motor_bemf_constant_hex = motor_params_data[2];
    extracted_params->motor_resistance_hex = motor_params_data[3];

    uint32_t current_loop_data;
    uint32_t speed_loop_data;

    extract_word.control_word.mem_addr = MCF8315_CURRENT_PI_REG;

    motor_read_data_word(&extract_word, (uint8_t*)&current_loop_data);

    extract_word.control_word.mem_addr = MCF8315_SPEED_PI_REG;

    motor_read_data_word(&extract_word, (uint8_t*)&speed_loop_data);

    extracted_params->current_loop_ki = 1000*((0xFF0000 & current_loop_data) >> 16)/(pow(10,((0x3000000 & current_loop_data) >> 18)));
    extracted_params->current_loop_kp = (current_loop_data & 0xFF)/(pow(10,((current_loop_data & 0x300) >> 8)));

    extracted_params->speed_loop_ki = 0.1*((0xFF0000 & speed_loop_data) >> 16)/(pow(10,((0x3000000 & current_loop_data) >> 18)));
    extracted_params->speed_loop_kp = 0.01*(0xFF & speed_loop_data)/(pow(10,((0x300 & speed_loop_data) >> 8)));

    return MOTOR_CTRL_ERR_OK;
}

MOTOR_ERRORS_e run_mpet(void) {

    motor_data_word_s mpet_enable;
    mpet_enable.target_id = MCF8315D_I2C_ADDRESS;
    mpet_enable.read_write_bit = OP_RW_WRITE;
    mpet_enable.control_word.op_rw = OP_RW_WRITE;
    mpet_enable.control_word.crc_en = CRC_EN_DISABLE;
    mpet_enable.control_word.d_len = D_LEN_32_BIT;
    mpet_enable.control_word.mem_sec = 0;
    mpet_enable.control_word.mem_page = 0;
    mpet_enable.control_word.mem_addr = MCF8315_ALGO_DEBUG2_REG;
    mpet_enable.data[0] = 0x1F;
    mpet_enable.data[1] = 0;
    mpet_enable.data[2] = 0;
    mpet_enable.data[3] = 0;

    motor_write_data_word(&mpet_enable);

    return MOTOR_CTRL_ERR_OK;
}

MOTOR_ERRORS_e get_fault(uint32_t *gate_driver_fault, uint32_t *controller_fault) {
    motor_data_word_s read_fault;
    read_fault.target_id = MCF8315D_I2C_ADDRESS;
    read_fault.read_write_bit = OP_RW_WRITE;
    read_fault.control_word.op_rw = OP_RW_READ;
    read_fault.control_word.crc_en = CRC_EN_DISABLE;
    read_fault.control_word.d_len = D_LEN_32_BIT;
    read_fault.control_word.mem_sec = 0;
    read_fault.control_word.mem_page = 0;
    read_fault.control_word.mem_addr = MCF8315_GATE_DRIVER_FAULT_STATUS_REG;

    uint32_t fault_data;

    motor_read_data_word(&read_fault, (uint8_t*)&fault_data);
    *gate_driver_fault = fault_data;

    read_fault.control_word.mem_addr = MCF8315_CONTROLLER_FAULT_STATUS_REG;
    motor_read_data_word(&read_fault, (uint8_t*)&fault_data);
    *controller_fault = fault_data;

}

MOTOR_ERRORS_e clear_fault() {
    motor_data_word_s clear_fault;
    clear_fault.target_id = MCF8315D_I2C_ADDRESS;
    clear_fault.read_write_bit = OP_RW_WRITE;
    clear_fault.control_word.op_rw = OP_RW_READ;
    clear_fault.control_word.crc_en = CRC_EN_DISABLE;
    clear_fault.control_word.d_len = D_LEN_32_BIT;
    clear_fault.control_word.mem_sec = 0;
    clear_fault.control_word.mem_page = 0;
    clear_fault.control_word.mem_addr = MCF8315_ALGO_CTRL1_REG;

    uint32_t reg_value;

    motor_read_data_word(&clear_fault, (uint8_t*)&reg_value);

    reg_value = reg_value & 0xDFFFFFFF;

    clear_fault.control_word.op_rw = OP_RW_WRITE;

    clear_fault.data[0] = reg_value & 0xFF;
    clear_fault.data[1] = (reg_value & 0xFF00) >> 8;
    clear_fault.data[2] = (reg_value & 0xFF0000) >> 16;
    clear_fault.data[3] = (reg_value & 0xFF000000) >> 24;

    motor_write_data_word(&clear_fault);
}