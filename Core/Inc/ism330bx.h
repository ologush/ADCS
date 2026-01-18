#ifndef ISM330BX
#define ISM330BX
// Includes

#include "ism330bx_reg.h"
#include "stm32f3xx_hal.h"
#include <math.h>


/* Macros */
#define BOOT_TIME           10
#define FIFO_WATERMARK      4

#define INT1_FIFO_TH        0x07u

#define XL_RATE             ISM330BX_XL_ODR_AT_30Hz
#define GY_RATE             ISM330BX_GY_ODR_AT_30Hz
#define SFLP_RATE           ISM330BX_SFLP_30Hz

#define FIFO_XL_BATCH_RATE  ISM330BX_XL_BATCHED_AT_30Hz
#define FIFO_GY_BATCH_RATE  ISM330BX_GY_BATCHED_AT_30Hz

/* Enums */

//Error codes
typedef enum {
    ISM330BX_ERR_OK,
    ISM330BX_ERR_ERROR
} ISM330BX_ERRORS_e;

//Weight of the bits in the accelerometer offset registers
typedef enum {
    ISM330BX_XL_OFS_0,  // 2^-10 g/LSB
    ISM330BX_XL_OFS_1   // 2^-6 g/LSB
} ISM330BX_XL_OFFSET_e;

//Enable/Disable for the SFLP modes
typedef enum {
    SFLP_MODE_ENABLE,
    SFLP_MODE_DISABLE
} SFLP_MODE_SET_e;

/* Structs */

// Raw gyroscope bias data from registers
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} raw_gyroscope_bias_s;

// Quaternion struct
typedef struct {
    float x;
    float y;
    float z;
    float w;
} Quaternion;


typedef struct {
    float x;
    float y;
    float z;
} gyroscope_bias_s;

typedef struct {
    float pitch;
    float roll;
    float yaw;
} gyroscope_data_s;

typedef struct {
    float x;
    float y;
    float z;
} accelerometer_data_s;

typedef struct {
    Quaternion game_rotation;
    gyroscope_data_s gyroscope;
    accelerometer_data_s accelerometer;
    float yaw;
    float yaw_rate;
} sflp_data_frame_s;

typedef struct {
    SFLP_MODE_SET_e game_rotation;
    SFLP_MODE_SET_e gravity;
    SFLP_MODE_SET_e gbias;

    ism330bx_xl_full_scale_t xl_scale;
    ism330bx_gy_full_scale_t gy_scale;
    
    ism330bx_xl_data_rate_t xl_data_rate;
    ism330bx_gy_data_rate_t gy_data_rate;
    ism330bx_sflp_data_rate_t sflp_data_rate;

    ism330bx_fifo_xl_batch_t xl_batch_rate;
    ism330bx_fifo_gy_batch_t gy_batch_rate;
    ISM330BX_XL_OFFSET_e offset_xl;
    gyroscope_bias_s gy_offset;

} SFLP_CONFIG_s;

/*Hardware Platform Specific Functions*/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

static void platform_delay(uint32_t ms);

/* Private Functions */
static ISM330BX_ERRORS_e get_game_rotation(Quaternion *quaternion_target, uint16_t data[3]);
static ISM330BX_ERRORS_e get_gyroscope_bias(gyroscope_bias_s *target, raw_gyroscope_bias_s data);

static uint32_t npy_halfbits_to_floatbits(uint16_t h);
static float_t npy_half_to_float(uint16_t h);

static ISM330BX_ERRORS_e reg_accelerometer_raw_to_float(accelerometer_data_s *target_vector, int16_t data[3]);
static ISM330BX_ERRORS_e fifo_accelerometer_raw_to_float(accelerometer_data_s *target_vector, uint16_t data[3]);
static ISM330BX_ERRORS_e gyroscope_raw_to_float(gyroscope_data_s *target_vector, uint16_t data[3]);
static ISM330BX_ERRORS_e apply_gyroscope_bias(gyroscope_data_s *target);
static ISM330BX_ERRORS_e get_yaw_angle(Quaternion *quat, float *yaw);
static ISM330BX_ERRORS_e deg_s_to_rad_s(float deg_per_second, float *rad_per_second);

/* Public Functions */
ISM330BX_ERRORS_e SFLP_INIT(SPI_HandleTypeDef *handle);
ISM330BX_ERRORS_e sflp_init_interrupt(void);
ISM330BX_ERRORS_e get_fifo_frame(sflp_data_frame_s *target_data_frame);
ISM330BX_ERRORS_e calibrate_gyroscope(SFLP_CONFIG_s *config);
ISM330BX_ERRORS_e calibrate_accelerometer(void);



#endif