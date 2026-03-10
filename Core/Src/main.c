/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MCF8315D.h"
#include "usbd_cdc_if.h"
#include "ism330bx.h"
#include "control_algo.h"
#include "commands.h"
#include "helpers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define USART_RX_BUFFER_SIZE 64
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

USART_HandleTypeDef husart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
sflp_data_frame_s current_sflp_data;
sflp_data_frame_s new_sflp_data;

extern volatile uint8_t data_received_flag;
extern volatile uint32_t received_data_length;
extern volatile uint8_t received_data_buffer[USB_PACKET_SIZE];

uint8_t usart_cmd;
uint8_t usart_rx_payload_buf[USART_RX_BUFFER_SIZE];

// Need to replace this with proper attitude control structure

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_Init(void);
/* USER CODE BEGIN PFP */
static void print_imu_data(sflp_data_frame_s *data);
static uint8_t calculate_checksum(uint8_t *data, uint8_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Initialize current attitude quaternion
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  SFLP_INIT(&hspi1);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  //MCF8315_init(&hi2c2);
  sflp_init_interrupt();
 
  GPIO_PinState fault_status = HAL_GPIO_ReadPin(GPIOB, Motor_Fault_Pin);

  if (fault_status == GPIO_PIN_RESET) {
    fault_status = GPIO_PIN_SET;
  }

  // Arm USART for commands
  HAL_USART_Receive_IT(&husart3, &usart_cmd, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (data_received_flag) {
        // Process received data

        data_received_flag = 0;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00201D2B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  husart3.Instance = USART3;
  husart3.Init.BaudRate = 115200;
  husart3.Init.WordLength = USART_WORDLENGTH_8B;
  husart3.Init.StopBits = USART_STOPBITS_1;
  husart3.Init.Parity = USART_PARITY_NONE;
  husart3.Init.Mode = USART_MODE_TX_RX;
  husart3.Init.CLKPolarity = USART_POLARITY_LOW;
  husart3.Init.CLKPhase = USART_PHASE_1EDGE;
  husart3.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_Interrupt_Pin */
  GPIO_InitStruct.Pin = IMU_Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_Interrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 LED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_Fault_Pin */
  GPIO_InitStruct.Pin = Motor_Fault_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Motor_Fault_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == IMU_Interrupt_Pin) {
      // Call IMU data handler
      get_fifo_frame(&new_sflp_data);
      // May need to disable interrupt, but will kep this as the highest priority for now
      memcpy(&current_sflp_data, &new_sflp_data, sizeof(sflp_data_frame_s));

      print_imu_data(&current_sflp_data);

      switch(get_target_type()) {
        float set_speed;
        case ALGO_TARGET_ATTITUDE:
          PID_iteration(current_sflp_data.yaw, &set_speed);
          MCF8315_ramp_speed((int32_t)set_speed);
          break;
        case ALGO_TARGET_SPIN_RATE:
          PID_iteration(current_sflp_data.yaw_rate, &set_speed);
          MCF8315_ramp_speed((int32_t)set_speed);
          break;
        case ALGO_OFF:
          // Do nothing
          break;
        default:
          break;
      }
  } else if (GPIO_Pin == Motor_Fault_Pin) {
      // Handle motor fault
      MCF8315_handle_fault();
  }
}

void HAL_USART_RxCpltCallback(USART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
      // Handle received data
      static CMD_e received_cmd = CMD_NONE;
      static uint8_t awaiting_payload = 0;
      
      if (!awaiting_payload) {
          received_cmd = (CMD_e)usart_cmd;

          switch (received_cmd) {
              case CMD_SET_ADCS_MODE:

                HAL_USART_Receive_IT(&husart3, usart_rx_payload_buf, SET_ADCS_MODE_PAYLOAD_SIZE);
                awaiting_payload = 1;

                break;
              case CMD_SET_ADCS_TARGET:

                  HAL_USART_Receive_IT(&husart3, usart_rx_payload_buf, SET_ADCS_TARGET_PAYLOAD_SIZE);
                  awaiting_payload = 1;

                  break;
              case CMD_GET_SAT_TELEMETRY_DATA:
                  
                  // For now, can look to add more things such as temperature in the future
                  uint8_t sensor_data_payload[TELEMETRY_DATA_PAYLOAD_SIZE];

                  float temperature;
                  get_temperature(&temperature);

                  floatToBytes(current_sflp_data.yaw, &sensor_data_payload[0]);
                  floatToBytes(current_sflp_data.yaw_rate, &sensor_data_payload[4]);
                  floatToBytes(temperature, &sensor_data_payload[8]);

                  HAL_USART_Transmit(&husart3, sensor_data_payload, TELEMETRY_DATA_PAYLOAD_SIZE, 1000);

                  HAL_USART_Receive_IT(&husart3, &usart_cmd, 1);
                  awaiting_payload = 0;
                  break;
              default:
                  
                  break;
          }
      } else {

          switch (received_cmd) {

              uint8_t response = CMD_RESP_ACK;

              case CMD_SET_ADCS_MODE:
                  
                  
                  algo_target_type_e new_target_type = (algo_target_type_e)usart_rx_payload_buf[0];
                  update_target_type(new_target_type);
                  HAL_USART_Transmit(&husart3, &response, 1, 1000);
                  break;
              case CMD_SET_ADCS_TARGET:
                  
                  float new_target_value;

                  bytesToFloat(usart_rx_payload_buf, &new_target_value);
                  update_target_value(new_target_value);
                  HAL_USART_Transmit(&husart3, &response, 1, 1000);
                  break;
              default:
                  // Should not be here, but just in case
                  break;
          }
          
          awaiting_payload = 0;
          HAL_USART_Receive_IT(&husart3, &usart_cmd, 1);
      }
      HAL_USART_Receive_IT(&husart3, &usart_cmd, 1);
  }
}

static void print_imu_data(sflp_data_frame_s *data) {

  char print_buffer[355];
  char section_break[] = "-------------------\n\r";

  uint16_t print_buffer_index = snprintf(print_buffer, sizeof(print_buffer), "Game rotation vector:\n\rX: %.4f\n\rY: %.4f\n\rZ: %.4f\n\rScalar: %.4f\n\r%s\n\r",
                                data->game_rotation.x,
                                data->game_rotation.y,
                                data->game_rotation.z,
                                data->game_rotation.w,
                                section_break);
  
  print_buffer_index += snprintf(print_buffer + print_buffer_index, sizeof(print_buffer) - print_buffer_index, "Gyroscope data:\n\rX: %.4f\n\rY: %.4f\n\rZ: %.4f\n\r%s\n\r",
                                data->gyroscope.pitch,
                                data->gyroscope.roll,
                                data->gyroscope.yaw,
                                section_break);

  print_buffer_index += snprintf(print_buffer + print_buffer_index, sizeof(print_buffer) - print_buffer_index, "Accelerometer data:\n\rX: %.4f\n\rY: %.4f\n\rZ: %.4f\n\r%s\n\r",
                                data->accelerometer.x,
                                data->accelerometer.y,
                                data->accelerometer.z,
                                section_break);

  print_buffer_index += snprintf(print_buffer + print_buffer_index, sizeof(print_buffer) - print_buffer_index, "Yaw is: %.4f radians \n\r%s\n\r", data->yaw, section_break);

  CDC_Transmit_FS(print_buffer, sizeof(print_buffer));
}

static void send_IMU_data(void) {
  USB_Packet_u IMU_packet = {
    .packet.start_byte = 0xAA,
    .packet.type = 0x01, // IMU data type
    .packet.length = IMU_PACKET_SIZE,
    .packet.payload = {0},
    .packet.checksum = 0,
    .packet.end_byte = 0x55
  };

  memcpy(IMU_packet.packet.payload, &current_sflp_data, IMU_PACKET_SIZE);

  IMU_packet.packet.checksum = calculate_checksum(IMU_packet.packet.payload, IMU_PACKET_SIZE);

  CDC_Transmit_FS(IMU_packet.buffer, sizeof(IMU_packet));
}

static uint8_t calculate_checksum(uint8_t *data, uint8_t length) {
  uint8_t checksum = 0;
  for(uint8_t i = 0; i < length; i++) {
    checksum += data[i];
  }
  return checksum;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
