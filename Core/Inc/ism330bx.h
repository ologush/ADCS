// Includes

#include "ism330bx_reg.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"


/* Macros */
#define BOOT_TIME           10
#define FIFO_WATERMARK      32

#define XL_RATE             ISM330BX_XL_ODR_AT_30Hz
#define GY_RATE             ISM330BX_GY_ODR_AT_30Hz
#define SFLP_RATE           ISM330BX_SFLP_30Hz

/* Private Variables */
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static ism330bx_fifo_sflp_raw_t fifo_sflp;

/*Hardware Platform Specific Functions*/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

static void platform_delay(uint32_t ms);

/* Public Functions */
void SFLP_INIT(void);

stmdev_ctx_t dev_ctx;
ism330bx_fifo_status_t fifo_status;
ism330bx_reset_t rst;
ism330bx_sflp_gbias_t gbias;

