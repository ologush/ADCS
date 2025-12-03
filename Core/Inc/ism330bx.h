// Includes

#include "ism330bx_reg.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
#include <math.h>


/* Macros */
#define BOOT_TIME           10
#define FIFO_WATERMARK      32

#define INT1_FIFO_TH        0x07u

#define XL_RATE             ISM330BX_XL_ODR_AT_30Hz
#define GY_RATE             ISM330BX_GY_ODR_AT_30Hz
#define SFLP_RATE           ISM330BX_SFLP_30Hz

/* Enums */
typedef enum {
    ISM330BX_ERR_OK,
    ISM330BX_ERR_ERROR
} ISM330BX_ERRORS_e;

typedef enum {
    SFLP_MODE_ENABLE,
    SFLP_MODE_DISABLE
} SFLP_MODE_SET_e;

/* Structs */
typedef struct {
    SFLP_MODE_SET_e game_rotation;
    SFLP_MODE_SET_e gravity;
    SFLP_MODE_SET_e gbias;

    ism330bx_xl_full_scale_t xl_scale;
    ism330bx_gy_full_scale_t gy_scale;
    
    ism330bx_xl_data_rate_t xl_data_rate;
    ism330bx_gy_data_rate_t gy_data_rate;
    ism330bx_sflp_data_rate_t sflp_data_rate;

} SFLP_CONFIG_s;

typedef struct {
    float_t x;
    float_t y;
    float_t z;
    float_t w;
} Quaternion;

typedef struct {
    float_t x;
    float_t y;
    float_t z;
} gravity_vector_s;

typedef struct {
    float_t x;
    float_t y;
    float_t z;
} gyroscope_bias_s;

typedef struct {
    Quaternion game_rotation;
    gravity_vector_s gravity;
    gyroscope_bias_s gbias;
} sflp_data_frame_s;

/* Private Variables */
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static ism330bx_fifo_sflp_raw_t fifo_sflp;

/* Public Variables */
extern stmdev_ctx_t dev_ctx;
extern ism330bx_fifo_status_t fifo_status;
extern ism330bx_reset_t rst;
extern ism330bx_sflp_gbias_t gbias;
/*Hardware Platform Specific Functions*/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

static void platform_delay(uint32_t ms);

static ISM330BX_ERRORS_e get_game_rotation(Quaternion *quaternion_target, uint16_t data[3]);
static ISM330BX_ERRORS_e get_gravity(gravity_vector_s *target_vector, uint16_t data[3]);  
static ISM330BX_ERRORS_e get_gyroscope_bias(gyroscope_bias_s *target, uint16_t data[3]);

static uint32_t npy_halfbits_to_floatbits(uint16_t h);
static float_t npy_half_to_float(uint16_t h);



static SFLP_CONFIG_s sflp_config;



/* Public Functions */
ISM330BX_ERRORS_e SFLP_INIT(void);
ISM330BX_ERRORS_e sflp_init_interrupt(void);
ISM330BX_ERRORS_e get_fifo_frame(sflp_data_frame_s *target_data_frame);



