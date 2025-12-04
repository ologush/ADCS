// Includes

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"

/* Macros */

#define MCF8315D_I2C_ADDRESS    0x01

#define D_LEN_16_BIT      0x00
#define D_LEN_32_BIT      0x01
#define D_LEN_64_BIT      0x02

#define OP_RW_MASK        0x800000
#define CRC_EN_MASK       0x400000
#define D_LEN_MASK        0x300000
#define MEM_SEC_MASK      0x0F0000
#define MEM_PAGE_MASK     0x00F000
#define MEM_ADDR_MASK     0x000FFF


/* Typedefs */
typedef struct {
    //Actual measured values
    float_t motor_resistance;
    float_t motor_inductance;
    float_t back_emf_constant;

    //LUT Locations of nearest values
    uint8_t motor_resistance_hex;
    uint8_t motor_inductance_hex;
    uint8_t back_emf_constant_hex;
} motor_parameters_s;

typedef struct {
    uint8_t op_rw           :   1;
    uint8_t crc_en          :   1;
    uint8_t d_len           :   2;
    uint8_t mem_sec         :   4;
    uint8_t mem_page        :   4;
    uint16_t mem_addr       :   12;
} i2c_control_word_format_s;

typedef struct {
    uint8_t target_id       :   7;
    uint8_t read_write_bit  :   1;
    i2c_control_word_format_s control_word;
    uint8_t data[8];
    uint8_t crc             :   8;
} i2c_data_word_s;



typedef struct {

} motor_config_s;

typedef enum {
    MOTOR_CTRL_ERR_OK,
    MOTOR_CTRL_ERR_ERROR
} motor_ctrl_err_e;



/* Private Variables */
I2C_HandleTypeDef *hi2c_motor_ctrl;

/* Global Variables */

/* Private function prototypes*/
static motor_ctrl_err_e i2c_write_data_word(i2c_data_word_s *data_word);

/* Public function prototypes */
motor_ctrl_err_e motor_control_init(I2C_HandleTypeDef *hi2c);
motor_ctrl_err_e motor_parameter_extraction(motor_parameters_s *motor_params);
motor_ctrl_err_e write_config_to_eeprom(motor_config_s *motor_config);
motor_ctrl_err_e read_config_from_eeprom(motor_config_s *motor_config);