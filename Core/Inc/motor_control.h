#ifndef MOTOR_CONTROL
#define MOTOR_CONTROL

// Includes

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
#include <stdint.h>

/* Macros */

#define MCF8315D_I2C_ADDRESS    0x01

#define D_LEN_16_BIT      0x00
#define D_LEN_32_BIT      0x01
#define D_LEN_64_BIT      0x02

#define OP_RW_WRITE      0x00
#define OP_RW_READ       0x01

#define CRC_EN_DISABLE  0x00
#define CRC_EN_ENABLE   0x01

#define OP_RW_MASK        0x800000
#define CRC_EN_MASK       0x400000
#define D_LEN_MASK        0x300000
#define MEM_SEC_MASK      0x0F0000
#define MEM_PAGE_MASK     0x00F000
#define MEM_ADDR_MASK     0x000FFF

#define MAX_RPM          7200.0f
#define MAX_SPEED           100000 //Placeholder




/* Typedefs */
typedef struct {
    //Actual measured values
    float motor_resistance;
    float motor_inductance;
    float motor_bemf_constant;

    float current_loop_ki;
    float current_loop_kp;

    float speed_loop_ki;
    float speed_loop_kp;

    //LUT Locations of nearest values
    uint8_t motor_resistance_hex;
    uint8_t motor_inductance_hex;
    uint8_t motor_bemf_constant_hex;
} motor_parameters_s;

typedef struct {
    uint8_t op_rw           :   1;
    uint8_t crc_en          :   1;
    uint8_t d_len           :   2;
    uint8_t mem_sec         :   4;
    uint8_t mem_page        :   4;
    uint16_t mem_addr       :   12;
} motor_control_word_s;

typedef struct {
    uint8_t target_id       :   7;
    uint8_t read_write_bit  :   1;
    motor_control_word_s control_word;
    uint8_t data[8];
    uint8_t crc             :   8;
} motor_data_word_s;

typedef struct {
    uint8_t reg_address;
    uint32_t reg_value;
} eeprom_register_s;

typedef enum {
    MOTOR_CTRL_ERR_OK,
    MOTOR_CTRL_ERR_ERROR
} motor_ctrl_err_e;



/* Private Variables */
static I2C_HandleTypeDef *hi2c_motor_ctrl;

/* Global Variables */
extern float set_speed;

/* Private function prototypes*/
static motor_ctrl_err_e motor_write_data_word(motor_data_word_s *data_word);
static motor_ctrl_err_e motor_read_data_word(motor_data_word_s *data_word, uint8_t *receive_buffer);
static motor_ctrl_err_e calculate_crc(motor_data_word_s *data_word);
static motor_ctrl_err_e initial_eeprom_config(void);
static motor_ctrl_err_e read_eeprom_config(uint32_t *config_data);


/* Public function prototypes */
motor_ctrl_err_e motor_control_init(I2C_HandleTypeDef *hi2c);
motor_ctrl_err_e motor_parameter_extraction(motor_parameters_s *motor_params);
motor_ctrl_err_e write_config_to_eeprom();
motor_ctrl_err_e read_config_from_eeprom();
motor_ctrl_err_e motor_startup_sequence(void);
motor_ctrl_err_e motor_set_speed(float speed_rpm);
motor_ctrl_err_e motor_get_speed(float *speed_rpm);
motor_ctrl_err_e get_fault();
motor_ctrl_err_e clear_fault(void);
motor_ctrl_err_e extract_motor_params(motor_parameters_s *extracted_params);
motor_ctrl_err_e run_mpet(void);

#endif