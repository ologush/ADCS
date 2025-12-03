#include "ism330bx.h"
#include "ism330bx_reg.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
#include <math.h>

stmdev_ctx_t dev_ctx;
ism330bx_fifo_status_t fifo_status;
ism330bx_reset_t rst;
ism330bx_sflp_gbias_t gbias;

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

static void platform_delay(uint32_t ms) {
  HAL_Delay(ms);
}

SFLP_CONFIG_s SFLP_config = {
    .xl_scale = ISM330BX_2g,
    .gy_scale = ISM330BX_125dps,
    .xl_data_rate = XL_RATE,
    .gy_data_rate = GY_RATE,
    .sflp_data_rate = SFLP_RATE,
    .game_rotation = SFLP_MODE_ENABLE,
    .gravity = SFLP_MODE_DISABLE,
    .gbias = SFLP_MODE_DISABLE
};

ISM330BX_ERRORS_e SFLP_INIT(void) {
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    //dev_ctx.handle = &hi2c2;
    platform_delay(BOOT_TIME);

    /* Check Device ID */
    ism330bx_device_id_get(&dev_ctx, &whoamI);

    if (whoamI != ISM330BX_ID) {
        /* Throw Error */
        return ISM330BX_ERR_ERROR;
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
    ism330bx_xl_full_scale_set(&dev_ctx, SFLP_config.xl_scale);
    ism330bx_gy_full_scale_set(&dev_ctx, SFLP_config.gy_scale);

    /*
        Set FIFO watermark. 
        This is the number of bytes placed in the FIFO buffer to trigger an action
    */
    ism330bx_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);

    /* Enable game rotation in FIFO, ensure gravity and gyroscope bias are disabled */
    if(SFLP_config.game_rotation == SFLP_MODE_ENABLE) {
        fifo_sflp.game_rotation = 1;
    } else {
        fifo_sflp.game_rotation = 0;
    }

    if(SFLP_config.gravity == SFLP_MODE_ENABLE) {
        fifo_sflp.gravity = 1;
    } else {
        fifo_sflp.gravity = 0;
    }
    if(SFLP_config.gbias == SFLP_MODE_ENABLE) {
        fifo_sflp.gbias = 1;
    } else {
        fifo_sflp.gbias = 0;
    }
    ism330bx_fifo_sflp_batch_set(&dev_ctx, fifo_sflp);

    /* Set FIFO to stream mode */
    ism330bx_fifo_mode_set(&dev_ctx, ISM330BX_STREAM_MODE);

    /* Set output rates.
        These can be optimized later
        The SFLP rate bust be less than or equal to the accelerometer and gyroscope rates
    */
    ism330bx_xl_data_rate_set(&dev_ctx, SFLP_config.xl_data_rate);
    ism330bx_gy_data_rate_set(&dev_ctx, SFLP_config.gy_data_rate);
    ism330bx_sflp_data_rate_set(&dev_ctx, SFLP_config.sflp_data_rate);

    /* Enable tracking of game rotation with the SFLP - might not be necessary, circle back */
    ism330bx_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);

    return ISM330BX_ERR_OK;

}

ISM330BX_ERRORS_e sflp_init_interrupt(void) {

    // Set interrupt pin to push-pull and active high
    int32_t err;
    err = ism330bx_int_pin_mode_set(&dev_ctx, ISM330BX_PUSH_PULL);
    if(err != 0) {
        return ISM330BX_ERR_ERROR;
    }

    err = ism330bx_pin_polarity_set(&dev_ctx, ISM330BX_ACTIVE_HIGH);
    if(err != 0) {
        return ISM330BX_ERR_ERROR;
    }

    // Route FIFO watermark interrupt to INT1 pin
    ism330bx_pin_int_route_t int1_route;
    int1_route.fifo_th = PROPERTY_ENABLE;
    err = ism330bx_pin_int1_route_set(&dev_ctx, int1_route);
    if(err != 0) {
        return ISM330BX_ERR_ERROR;
    }
    return ISM330BX_ERR_OK;
}


static ISM330BX_ERRORS_e get_fifo_frame() {
    return ISM330BX_ERR_OK;
}

static ISM330BX_ERRORS_e get_fifo_buffer() {
    return ISM330BX_ERR_OK;
}

static ISM330BX_ERRORS_e get_game_rotation(Quaternion *quaternion_target, uint16_t data[3]) {
    float_t sumsq = 0;

    quaternion_target->x = npy_half_to_float(data[0]);
    quaternion_target->y = npy_half_to_float(data[1]);
    quaternion_target->z = npy_half_to_float(data[2]);
    
    sumsq += quaternion_target->x * quaternion_target->x;
    sumsq += quaternion_target->y * quaternion_target->y;
    sumsq += quaternion_target->z * quaternion_target->z;
    if (sumsq > 1.0f) {
        float_t n = sqrtf(sumsq);
        quaternion_target->x /= n;
        quaternion_target->y /= n;
        quaternion_target->z /= n;
        sumsq = 1.0f;
    }

    quaternion_target->w = sqrtf(1.0f - sumsq); 
    return ISM330BX_ERR_OK;
}

static ISM330BX_ERRORS_e get_gravity(float_t gravity_mg[3], uint16_t data[3]) {
    switch(SFLP_config.xl_scale) {
        case ISM330BX_2g:
            gravity_mg[0] = ism330bx_from_fs2_to_mg(data[0]);
            gravity_mg[1] = ism330bx_from_fs2_to_mg(data[1]);
            gravity_mg[2] = ism330bx_from_fs2_to_mg(data[2]);
            break;
        case ISM330BX_4g:
            gravity_mg[0] = ism330bx_from_fs4_to_mg(data[0]);
            gravity_mg[1] = ism330bx_from_fs4_to_mg(data[1]);
            gravity_mg[2] = ism330bx_from_fs4_to_mg(data[2]);
            break;
        case ISM330BX_8g:
            gravity_mg[0] = ism330bx_from_fs8_to_mg(data[0]);
            gravity_mg[1] = ism330bx_from_fs8_to_mg(data[1]);
            gravity_mg[2] = ism330bx_from_fs8_to_mg(data[2]);
            break;
        default:
            return ISM330BX_ERR_ERROR;
            break;
    }

    return ISM330BX_ERR_OK;
}

static ISM330BX_ERRORS_e get_gyroscope_bias(float_t gbias_mdps[3], uint16_t data[3]) {
        switch(SFLP_config.gy_scale) {
        case ISM330BX_125dps:
            gbias_mdps[0] = ism330bx_from_fs125_to_mdps(data[0]);
            gbias_mdps[1] = ism330bx_from_fs125_to_mdps(data[1]);
            gbias_mdps[2] = ism330bx_from_fs125_to_mdps(data[2]);
            break;
        case ISM330BX_250dps:
            gbias_mdps[0] = ism330bx_from_fs250_to_mdps(data[0]);
            gbias_mdps[1] = ism330bx_from_fs250_to_mdps(data[1]);
            gbias_mdps[2] = ism330bx_from_fs250_to_mdps(data[2]);
            break;
        case ISM330BX_500dps:
            gbias_mdps[0] = ism330bx_from_fs500_to_mdps(data[0]);
            gbias_mdps[1] = ism330bx_from_fs500_to_mdps(data[1]);
            gbias_mdps[2] = ism330bx_from_fs500_to_mdps(data[2]);
            break;
        case ISM330BX_1000dps:
            gbias_mdps[0] = ism330bx_from_fs1000_to_mdps(data[0]);
            gbias_mdps[1] = ism330bx_from_fs1000_to_mdps(data[1]);
            gbias_mdps[2] = ism330bx_from_fs1000_to_mdps(data[2]);
            break;
        case ISM330BX_2000dps:
            gbias_mdps[0] = ism330bx_from_fs2000_to_mdps(data[0]);
            gbias_mdps[1] = ism330bx_from_fs2000_to_mdps(data[1]);
            gbias_mdps[2] = ism330bx_from_fs2000_to_mdps(data[2]);
            break;
        case ISM330BX_4000dps:
            gbias_mdps[0] = ism330bx_from_fs4000_to_mdps(data[0]);
            gbias_mdps[1] = ism330bx_from_fs4000_to_mdps(data[1]);
            gbias_mdps[2] = ism330bx_from_fs4000_to_mdps(data[2]);
            break;

        default:
            return ISM330BX_ERR_ERROR;
            break;
    }
    return ISM330BX_ERR_OK;
}

static float_t npy_half_to_float(uint16_t h) {
    union {
        uint32_t u;
        float f;
    } convert;

    convert.u = npy_halfbits_to_floatbits(h);
    return convert.f;
}

static uint32_t npy_halfbits_to_floatbits(uint16_t h) {
    uint16_t h_exp, h_sig;
    uint32_t f_sgn, f_exp, f_sig;

    /* Extract the exponent and sign from the half float */
    h_exp = (h & 0x7C00u);
    f_sgn = ((uint32_t) h & 0x8000u) << 16;

    switch(h_exp) {
        case 0x0000u: /* Subnormal or zero Case */
            h_sig = (h & 0x03FFu);
            if (h_sig == 0) {
                /* Signed zero */
                return f_sgn;
            }
            /* Normalize the subnormal number */
            do {
                h_sig <<= 1;
                h_exp++;
            } while ((h_sig & 0x0400u) == 0);
            f_exp = ((uint32_t)(h_exp + (127 -15))) << 23;
            f_sig = ((uint32_t)(h_sig & 0x03FFu)) << 13;
            return f_sgn + f_exp + f_sig;
    }
}