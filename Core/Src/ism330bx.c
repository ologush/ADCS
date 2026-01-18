#include "ism330bx.h"
#include "ism330bx_reg.h"
#include "motor_control.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include "stm32f3xx_hal_i2c.h"
#include <math.h>
#include <stdint.h>
#include "usbd_cdc_if.h"

// Private Variables
static stmdev_ctx_t dev_ctx;
static ism330bx_fifo_status_t fifo_status;
static ism330bx_sflp_gbias_t gbias;
static ism330bx_fifo_sflp_raw_t fifo_sflp;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  /* USER CODE BEGIN WRITE */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    return 0;
  /* USER CODE END WRITE */
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  /* USER CODE BEGIN READ */
    uint8_t read_reg = (reg | 0x80); //Set MSB read flag
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &read_reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  return 0;
  /* USER CODE END READ */
}

static void platform_delay(uint32_t ms) {
  HAL_Delay(ms);
}

static SFLP_CONFIG_s SFLP_config = {
    .xl_scale = ISM330BX_2g,
    .gy_scale = ISM330BX_125dps,
    .xl_data_rate = XL_RATE,
    .gy_data_rate = GY_RATE,
    .xl_batch_rate = FIFO_XL_BATCH_RATE,
    .gy_batch_rate = FIFO_GY_BATCH_RATE,
    .sflp_data_rate = SFLP_RATE,
    .game_rotation = SFLP_MODE_ENABLE,
    .gravity = SFLP_MODE_DISABLE,
    .gbias = SFLP_MODE_DISABLE,
    .offset_xl = ISM330BX_XL_OFS_0
};

ISM330BX_ERRORS_e SFLP_INIT(SPI_HandleTypeDef *handle) {
    
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = handle;

    platform_delay(BOOT_TIME);

    /* Check Device ID */
    uint8_t whoamI = 0;
    ism330bx_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != ISM330BX_ID) {
        /* Throw Error */
        return ISM330BX_ERR_ERROR;
    }

    /* Reset the device, wait until the reset is complete */
    ism330bx_reset_t rst;
    ism330bx_reset_set(&dev_ctx, ISM330BX_RESTORE_CTRL_REGS);
    do {
        ism330bx_reset_get(&dev_ctx, &rst);
    } while (rst != ISM330BX_READY);

    /* Configure device to enable SFLP this function follows the steps in the datasheet, however the PAGE_ADDRESS it is using is not listed (strange)*/
    ism330bx_sflp_configure(&dev_ctx);

    /* Enable Block Data Update to ensure data is from the same sample */
    ism330bx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set the scale of the accelerometer and gyroscope. Keep low for best accuracy */
    ism330bx_xl_full_scale_set(&dev_ctx, SFLP_config.xl_scale);
    ism330bx_gy_full_scale_set(&dev_ctx, SFLP_config.gy_scale);

    calibrate_accelerometer();

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

    /* Set FIFO to stream (continuous) mode */
    ism330bx_fifo_mode_set(&dev_ctx, ISM330BX_STREAM_MODE);

    /* Limits the FIFO size to the watermark set */
    ism330bx_fifo_stop_on_wtm_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set output rates.
        These can be optimized later
        The SFLP rate bust be less than or equal to the accelerometer and gyroscope rates
    */
    ism330bx_xl_data_rate_set(&dev_ctx, SFLP_config.xl_data_rate);
    ism330bx_gy_data_rate_set(&dev_ctx, SFLP_config.gy_data_rate);
    ism330bx_sflp_data_rate_set(&dev_ctx, SFLP_config.sflp_data_rate);
    ism330bx_fifo_xl_batch_set(&dev_ctx, SFLP_config.xl_batch_rate);
    ism330bx_fifo_gy_batch_set(&dev_ctx, SFLP_config.gy_batch_rate);

    ism330bx_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);

    return ISM330BX_ERR_OK;

}

ISM330BX_ERRORS_e sflp_init_interrupt(void) {

    // Set interrupt pin to push-pull and active high
    int32_t err = 0;

    err = ism330bx_int_pin_mode_set(&dev_ctx, ISM330BX_PUSH_PULL);
    if(err != 0) {
        return ISM330BX_ERR_ERROR;
    }

    err = ism330bx_pin_polarity_set(&dev_ctx, ISM330BX_ACTIVE_HIGH);
    if(err != 0) {
        return ISM330BX_ERR_ERROR;
    }

    // Route FIFO watermark interrupt to INT1 pin
    ism330bx_pin_int_route_t int1_route = {
        .boot = 0,
        .drdy_xl = 0,
        .drdy_gy = 0,
        .drdy_temp = 0,
        .drdy_ah_qvar = 0,
        .fifo_th = 1,
        .fifo_ovr = 0,
        .fifo_full = 0,
        .fifo_bdr = 0,
        .den_flag = 0,
        .timestamp = 0,
        .six_d = 0,
        .double_tap = 0,
        .free_fall = 0,
        .wake_up = 0,
        .single_tap = 0,
        .sleep_change = 0,
        .sleep_status = 0,
        .step_detector = 0,
        .tilt = 0,
        .sig_mot = 0,
        .fsm_lc = 0,
        .fsm1 = 0,
        .fsm2 = 0,
        .fsm3 = 0,
        .fsm4 = 0,
        .fsm5 = 0,
        .fsm6 = 0,
        .fsm7 = 0,
        .fsm8 = 0,
        .mlc1 = 0,
        .mlc2 = 0,
        .mlc3 = 0,
        .mlc4 = 0,
    };

    err = ism330bx_pin_int1_route_set(&dev_ctx, int1_route);
    if(err != 0) {
        return ISM330BX_ERR_ERROR;
    }

    return ISM330BX_ERR_OK;
}


ISM330BX_ERRORS_e get_fifo_frame(sflp_data_frame_s *target_data_frame) {
    ism330bx_fifo_status_t fifo_status;
    int32_t err;
    err = ism330bx_fifo_status_get(&dev_ctx, &fifo_status);
    if(err != 0) {
        return ISM330BX_ERR_ERROR;
    }

    if(fifo_status.fifo_th == 1) {
        uint16_t num = fifo_status.fifo_level;
        for(uint16_t i = 0; i < num; i++) {
            ism330bx_fifo_out_raw_t fifo_data;

            err = ism330bx_fifo_out_raw_get(&dev_ctx, &fifo_data);
            if(err != 0) {
                return ISM330BX_ERR_ERROR;
            }

            switch (fifo_data.tag) {
                case ISM330BX_SFLP_GAME_ROTATION_VECTOR_TAG:
                    get_game_rotation(&target_data_frame->game_rotation, (uint16_t*)&fifo_data.data[0]);
                    get_yaw_angle(&target_data_frame->game_rotation, &target_data_frame->yaw);
                    break;
                case ISM330BX_GY_NC_TAG: //Not sure what the differnt gyroscope tags are, assuming this first one is correct for now
                    gyroscope_raw_to_float(&target_data_frame->gyroscope, (uint16_t*)&fifo_data.data[0]);
                    apply_gyroscope_bias(&target_data_frame->gyroscope);
                    deg_s_to_rad_s(target_data_frame->gyroscope.yaw, &target_data_frame->yaw_rate);
                    break;
                case ISM330BX_XL_NC_TAG: //Not sure what the differnt accelerometer tags are, assuming this first one is correct for now
                    fifo_accelerometer_raw_to_float(&target_data_frame->accelerometer, (uint16_t*)&fifo_data.data[0]);
                    break;
                default:
                    // Unknown tag, skip
                    break;
            }
        }
    }
    return ISM330BX_ERR_OK;
}

static ISM330BX_ERRORS_e apply_gyroscope_bias(gyroscope_data_s *target) {

    target->pitch -= SFLP_config.gy_offset.x;
    target->roll -= SFLP_config.gy_offset.y;
    target->yaw -= SFLP_config.gy_offset.z;

    return ISM330BX_ERR_OK;
}

ISM330BX_ERRORS_e get_fifo_buffer() {
    return ISM330BX_ERR_OK;
}

// Converts raw accelerometer data from the registers (not FIFO buffer) to float in mg units
static ISM330BX_ERRORS_e reg_accelerometer_raw_to_float(accelerometer_data_s *target_vector, int16_t data[3]) {
    switch(SFLP_config.xl_scale) {
        case ISM330BX_2g:
            target_vector->x = ism330bx_from_fs2_to_mg(data[0]);
            target_vector->y = ism330bx_from_fs2_to_mg(data[1]);
            target_vector->z = ism330bx_from_fs2_to_mg(data[2]);
            break;
        case ISM330BX_4g:
            target_vector->x = ism330bx_from_fs4_to_mg(data[0]);
            target_vector->y = ism330bx_from_fs4_to_mg(data[1]);
            target_vector->z = ism330bx_from_fs4_to_mg(data[2]);
            break;
        case ISM330BX_8g:
            target_vector->x = ism330bx_from_fs8_to_mg(data[0]);
            target_vector->y = ism330bx_from_fs8_to_mg(data[1]);
            target_vector->z = ism330bx_from_fs8_to_mg(data[2]);
            break;
        default:
            return ISM330BX_ERR_ERROR;
            break;
    }

    return ISM330BX_ERR_OK;
}

static ISM330BX_ERRORS_e fifo_accelerometer_raw_to_float(accelerometer_data_s *target_vector, uint16_t data[3]) {
    switch(SFLP_config.xl_scale) {
        case ISM330BX_2g:
            target_vector->z = ism330bx_from_fs2_to_mg(data[0]);
            target_vector->y = ism330bx_from_fs2_to_mg(data[1]);
            target_vector->x = ism330bx_from_fs2_to_mg(data[2]);
            break;
        case ISM330BX_4g:
            target_vector->z = ism330bx_from_fs4_to_mg(data[0]);
            target_vector->y = ism330bx_from_fs4_to_mg(data[1]);
            target_vector->x = ism330bx_from_fs4_to_mg(data[2]);
            break;
        case ISM330BX_8g:
            target_vector->z = ism330bx_from_fs8_to_mg(data[0]);
            target_vector->y = ism330bx_from_fs8_to_mg(data[1]);
            target_vector->x = ism330bx_from_fs8_to_mg(data[2]);
            break;
        default:
            return ISM330BX_ERR_ERROR;
            break;
    }

    return ISM330BX_ERR_OK;
}

static ISM330BX_ERRORS_e gyroscope_raw_to_float(gyroscope_data_s *target_vector, uint16_t data[3]) {
        switch(SFLP_config.gy_scale) {
        case ISM330BX_125dps:
            target_vector->pitch = ism330bx_from_fs125_to_mdps(data[0]);
            target_vector->roll = ism330bx_from_fs125_to_mdps(data[1]);
            target_vector->yaw = ism330bx_from_fs125_to_mdps(data[2]);
            break;
        case ISM330BX_250dps:
            target_vector->pitch = ism330bx_from_fs250_to_mdps(data[0]);
            target_vector->roll = ism330bx_from_fs250_to_mdps(data[1]);
            target_vector->yaw = ism330bx_from_fs250_to_mdps(data[2]);
            break;
        case ISM330BX_500dps:
            target_vector->pitch = ism330bx_from_fs500_to_mdps(data[0]);
            target_vector->roll = ism330bx_from_fs500_to_mdps(data[1]);
            target_vector->yaw = ism330bx_from_fs500_to_mdps(data[2]);
            break;
        case ISM330BX_1000dps:
            target_vector->pitch = ism330bx_from_fs1000_to_mdps(data[0]);
            target_vector->roll = ism330bx_from_fs1000_to_mdps(data[1]);
            target_vector->yaw = ism330bx_from_fs1000_to_mdps(data[2]);
            break;
        case ISM330BX_2000dps:
            target_vector->pitch = ism330bx_from_fs2000_to_mdps(data[0]);
            target_vector->roll = ism330bx_from_fs2000_to_mdps(data[1]);
            target_vector->yaw = ism330bx_from_fs2000_to_mdps(data[2]);
            break;
        case ISM330BX_4000dps:
            target_vector->pitch = ism330bx_from_fs4000_to_mdps(data[0]);
            target_vector->roll = ism330bx_from_fs4000_to_mdps(data[1]);
            target_vector->yaw = ism330bx_from_fs4000_to_mdps(data[2]);
            break;

        default:
            return ISM330BX_ERR_ERROR;
            break;
    }
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

static ISM330BX_ERRORS_e get_gyroscope_bias(gyroscope_bias_s *target, raw_gyroscope_bias_s data) {
        switch(SFLP_config.gy_scale) {
        case ISM330BX_125dps:
            target->x = ism330bx_from_fs125_to_mdps(data.x);
            target->y = ism330bx_from_fs125_to_mdps(data.y);
            target->z = ism330bx_from_fs125_to_mdps(data.z);
            break;
        case ISM330BX_250dps:
            target->x = ism330bx_from_fs250_to_mdps(data.x);
            target->y = ism330bx_from_fs250_to_mdps(data.y);
            target->z = ism330bx_from_fs250_to_mdps(data.z);
            break;
        case ISM330BX_500dps:
            target->x = ism330bx_from_fs500_to_mdps(data.x);
            target->y = ism330bx_from_fs500_to_mdps(data.y);
            target->z = ism330bx_from_fs500_to_mdps(data.z);
            break;
        case ISM330BX_1000dps:
            target->x = ism330bx_from_fs1000_to_mdps(data.x);
            target->y = ism330bx_from_fs1000_to_mdps(data.y);
            target->z = ism330bx_from_fs1000_to_mdps(data.z);
            break;
        case ISM330BX_2000dps:
            target->x = ism330bx_from_fs2000_to_mdps(data.x);
            target->y = ism330bx_from_fs2000_to_mdps(data.y);
            target->z = ism330bx_from_fs2000_to_mdps(data.z);
            break;
        case ISM330BX_4000dps:
            target->x = ism330bx_from_fs4000_to_mdps(data.x);
            target->y = ism330bx_from_fs4000_to_mdps(data.y);
            target->z = ism330bx_from_fs4000_to_mdps(data.z);
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

            h_sig <<= 1;
            while ((h_sig & 0x0400u) == 0) {
                h_sig <<= 1;
                h_exp++;
            }
            f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
            f_sig = ((uint32_t)(h_sig&0x03ffu)) << 13;
            return f_sgn + f_exp + f_sig;
        case 0x7C00u: /* inf or NaN */
            /* All-ones exponent and a copy of the significand */
            return f_sgn + 0x7f800000u + (((uint32_t)(h&0x03ffu)) << 13);
        default: /* normalized */
            /* Just need to adjust the exponent and shift */
            return f_sgn + (((uint32_t)(h&0x7fffu) + 0x1c000u) << 13);
    }
}

// Function to convert the quaternion to yaw angle in radians
static ISM330BX_ERRORS_e get_yaw_angle(Quaternion *quat, float *yaw) {
    float s1 = 2.0f * (quat->w * quat->z + quat->x * quat->y);
    float s2 = 1.0f - 2.0f * (quat->y * quat->y + quat->z * quat->z);
    *yaw = atan2f(s1, s2);
    return ISM330BX_ERR_OK;
}

// Convert degrees per second to radians per second
static ISM330BX_ERRORS_e deg_s_to_rad_s(float deg_per_second, float *rad_per_second) {
    *rad_per_second = deg_per_second * (M_PI / 180.0f);
    return ISM330BX_ERR_OK;
}

ISM330BX_ERRORS_e calibrate_gyroscope(SFLP_CONFIG_s *config) {

    //GBIAS regs are available when PAGE_SEL[3:0] = 0x0000
    //FInd on page 124

    ism330bx_write_reg(&dev_ctx, ISM330BX_PAGE_SEL, 0x00, 1);
    
    uint8_t gyroscope_bias[6] = {0};

    ism330bx_read_reg(&dev_ctx, ISM330BX_SFLP_GAME_GBIASX_L, gyroscope_bias, 6);

    raw_gyroscope_bias_s raw_bias;

    raw_bias.x = (int16_t)((gyroscope_bias[1] << 8) | gyroscope_bias[0]);
    raw_bias.y = (int16_t)((gyroscope_bias[3] << 8) | gyroscope_bias[2]);
    raw_bias.z = (int16_t)((gyroscope_bias[5] << 8) | gyroscope_bias[4]);

    get_gyroscope_bias(&config->gy_offset, raw_bias);

    return ISM330BX_ERR_OK;
}

//This function calibrates the accelerometer by calculating offsets based on stationary readings
ISM330BX_ERRORS_e calibrate_accelerometer(void) {

    int8_t x_offset;
    int8_t y_offset;
    int8_t z_offset;

    uint8_t n_samples = 5;

    ism330bx_ctrl9_t ctrl9;

    accelerometer_data_s offset = {0};

    for(uint8_t i = 0; i < n_samples; i++) {
        int16_t acc_raw[3];
        accelerometer_data_s current_sample = {0};
        ism330bx_acceleration_raw_get(&dev_ctx, acc_raw);
        reg_accelerometer_raw_to_float(&current_sample, acc_raw);
        offset.x += current_sample.x / n_samples;
        offset.y += current_sample.y / n_samples;
        offset.z += current_sample.z / n_samples;
        platform_delay(20);
    }

    ism330bx_read_reg(&dev_ctx, ISM330BX_CTRL9, (uint8_t*)&ctrl9, 1);

    switch(SFLP_config.offset_xl) {
        case ISM330BX_XL_OFS_0:
            x_offset = (int8_t)(-offset.x / 0.9765625f); // 2^-10 g/LSB
            y_offset = (int8_t)(-offset.y / 0.9765625f);
            z_offset = (int8_t)((1000.0f - offset.z) / 0.9765625f); //Assuming static 1g on Z axis
            
            break;
        case ISM330BX_XL_OFS_1:
            x_offset = (int8_t)(-offset.x / 15.625f); // 2^-6 g/LSB
            y_offset = (int8_t)(-offset.y / 15.625f);
            z_offset = (int8_t)((1000.0f - offset.z) / 15.625f); //Assuming static 1g on Z axis
            ctrl9.usr_off_w = 0x01;
            break;
        default:
            return ISM330BX_ERR_ERROR;
            break;
    }

    ctrl9.usr_off_on_out = 0x01;
    
    ism330bx_write_reg(&dev_ctx, ISM330BX_CTRL9, (uint8_t*)&ctrl9, 1);
    
    ism330bx_write_reg(&dev_ctx, ISM330BX_X_OFS_USR, (uint8_t*)&x_offset, 1);
    ism330bx_write_reg(&dev_ctx, ISM330BX_Y_OFS_USR, (uint8_t*)&y_offset, 1);
    ism330bx_write_reg(&dev_ctx, ISM330BX_Z_OFS_USR, (uint8_t*)&z_offset, 1);

    return ISM330BX_ERR_OK;
}