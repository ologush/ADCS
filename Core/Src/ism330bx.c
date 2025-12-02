#include "ism330bx.h"
#include "ism330bx_reg.h"
#include "stm32f3xx_hal.h"

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  /* USER CODE BEGIN WRITE */
  HAL_I2C_Mem_Write(handle, ISM330BX_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp,
                    len, 1000);
  /* USER CODE END WRITE */
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  /* USER CODE BEGIN READ */
    HAL_I2C_Mem_Read(handle, ISM330BX_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp,
                     len, 1000);
  return 0;
  /* USER CODE END READ */
}

static void platform_delay(uint32_t ms)
{
  /* USER CODE BEGIN DELAY */
  //HAL_Delay(ms);
  /* USER CODE END DELAY */
}

void SFLP_INIT(void) {
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    //dev_ctx.handle
    platform_delay(BOOT_TIME);

    /* Check Device ID */
    ism330bx_device_id_get(&dev_ctx, &whoamI);

    if (whoamI != ISM330BX_ID) {
        /* Throw Error */
    }

    /* Reset the device */
    ism330bx_reset_set(&dev_ctx, ISM330BX_RESTORE_CTRL_REGS);
    do {
        ism330bx_reset_get(&dev_ctx, &rst);
    } while (rst != ISM330BX_READY);

    /* Configure device to enable SFLP */
    ism330bx_sflp_configure(&dev_ctx);

    /* Enable Block Data Update to ensure data is from the same sample */
    ism330bx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set the scale of the accelerometer and gyroscope. Can be optimized later */
    ism330bx_xl_full_scale_set(&dev_ctx, ISM330BX_4g);
    ism330bx_gy_full_scale_set(&dev_ctx, ISM330BX_250dps);

    /*
        Set FIFO watermark. 
        This is the number of bytes placed in the FIFO buffer to trigger an action
    */
    ism330bx_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);

    /* Set initial directions of the system */
    fifo_sflp.game_rotation = 1;
    fifo_sflp.gravity = 1;
    fifo_sflp.gbias = 1;
    ism330bx_fifo_sflp_batch_set(&dev_ctx, fifo_sflp);

    /* Set FIFO to stream mode */
    ism330bx_fifo_mode_set(&dev_ctx, ISM330BX_STREAM_MODE);

    /* Set output rates.
        These can be optimized later
        The SFLP rate bust be less than or equal to the accelerometer and gyroscope rates
    */
    ism330bx_xl_data_rate_set(&dev_ctx, XL_RATE);
    ism330bx_gy_data_rate_set(&dev_ctx, GY_RATE);
    ism330bx_sflp_data_rate_set(&dev_ctx, SFLP_RATE);

    /* Enable tracking of game rotation with the SFLP */
    ism330bx_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);


}