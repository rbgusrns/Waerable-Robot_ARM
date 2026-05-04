/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "dynamixel.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  int32_t sensor_anchor_0;
  int32_t sensor_anchor_1;
  int32_t sensor_anchor_2;
  uint16_t dxl_anchor_0;
  uint16_t dxl_anchor_1;
  uint16_t dxl_anchor_2;
} BluetoothServoMap;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

#define UART1_RX_BUF_SIZE 64U
#define UART3_RX_BUF_SIZE 64U
volatile uint8_t g_uart1_rx_byte;
volatile uint8_t g_uart1_rx_buf[UART1_RX_BUF_SIZE];
volatile uint8_t g_uart1_rx_index = 0U;
volatile uint8_t g_uart1_rx_ready = 0U;
volatile uint8_t g_uart3_rx_byte;
volatile uint8_t g_uart3_rx_buf[UART3_RX_BUF_SIZE];
volatile uint8_t g_uart3_rx_index = 0U;
volatile uint8_t g_uart3_rx_ready = 0U;

static const BluetoothServoMap g_bluetooth_servo_maps[DXL_SERVO_COUNT] = {
    {-90, 0, 90, 2644U, 1628U, 623U},
    {-90, 0, 90, 268U, 562U, 876U},
    {1023, 512, 0, 739U, 450U, 162U},
    {1023, 512, 0, 824U, 507U, 190U},
    {-90, 0, 90, 775U, 450U, 202U},
    {1023, 512, 0, 851U, 551U, 253U},
};

volatile uint16_t g_expected_positions[DXL_SERVO_COUNT] = {1628U, 876U, 528U,
                                                           824U,  490U, 530U};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const *argument);

/* USER CODE BEGIN PFP */

static void Console_WriteString(const char *text);
static const char *Dxl_StatusToString(DxlStatus status);

static uint8_t Bluetooth_ParseSignedValue(const volatile uint8_t *buffer, uint8_t *index, int32_t *value)
{
  uint8_t i;
  int32_t sign = 1;
  int32_t parsed_value = 0;
  uint8_t has_digit = 0U;

  if ((buffer == NULL) || (index == NULL) || (value == NULL)) {
    return 0U;
  }

  i = *index;

  if (buffer[i] == '-') {
    sign = -1;
    ++i;
  }

  while ((buffer[i] >= '0') && (buffer[i] <= '9')) {
    parsed_value = (parsed_value * 10) + (int32_t)(buffer[i] - '0');
    has_digit = 1U;
    ++i;
  }

  if (!has_digit) {
    return 0U;
  }

  if ((buffer[i] != ',') && (buffer[i] != '\0')) {
    return 0U;
  }

  *value = parsed_value * sign;
  *index = i;
  return 1U;
}

static int32_t Bluetooth_MapLinearClamped(int32_t input,
                                          int32_t input_start,
                                          int32_t input_end,
                                          int32_t output_start,
                                          int32_t output_end)
{
  int32_t clamped_input = input;
  int32_t input_low;
  int32_t input_high;
  int64_t numerator;
  int32_t denominator;

  if (input_start == input_end) {
    return output_start;
  }

  input_low = (input_start < input_end) ? input_start : input_end;
  input_high = (input_start < input_end) ? input_end : input_start;

  if (clamped_input < input_low) {
    clamped_input = input_low;
  } else if (clamped_input > input_high) {
    clamped_input = input_high;
  }

  numerator = (int64_t)(clamped_input - input_start) * (int64_t)(output_end - output_start);
  denominator = input_end - input_start;

  return output_start + (int32_t)(numerator / denominator);
}

static uint16_t Bluetooth_MapSensorToGoal(uint8_t servo_index, int32_t sensor_value)
{
  const BluetoothServoMap *map;
  int32_t mapped_goal;

  if (servo_index >= DXL_SERVO_COUNT) {
    return 0U;
  }

  map = &g_bluetooth_servo_maps[servo_index];

  if (map->sensor_anchor_0 <= map->sensor_anchor_2) {
    if (sensor_value <= map->sensor_anchor_0) {
      return map->dxl_anchor_0;
    }
    if (sensor_value >= map->sensor_anchor_2) {
      return map->dxl_anchor_2;
    }
    if (sensor_value <= map->sensor_anchor_1) {
      mapped_goal = Bluetooth_MapLinearClamped(sensor_value,
                                               map->sensor_anchor_0,
                                               map->sensor_anchor_1,
                                               (int32_t)map->dxl_anchor_0,
                                               (int32_t)map->dxl_anchor_1);
    } else {
      mapped_goal = Bluetooth_MapLinearClamped(sensor_value,
                                               map->sensor_anchor_1,
                                               map->sensor_anchor_2,
                                               (int32_t)map->dxl_anchor_1,
                                               (int32_t)map->dxl_anchor_2);
    }
  } else {
    if (sensor_value >= map->sensor_anchor_0) {
      return map->dxl_anchor_0;
    }
    if (sensor_value <= map->sensor_anchor_2) {
      return map->dxl_anchor_2;
    }
    if (sensor_value >= map->sensor_anchor_1) {
      mapped_goal = Bluetooth_MapLinearClamped(sensor_value,
                                               map->sensor_anchor_0,
                                               map->sensor_anchor_1,
                                               (int32_t)map->dxl_anchor_0,
                                               (int32_t)map->dxl_anchor_1);
    } else {
      mapped_goal = Bluetooth_MapLinearClamped(sensor_value,
                                               map->sensor_anchor_1,
                                               map->sensor_anchor_2,
                                               (int32_t)map->dxl_anchor_1,
                                               (int32_t)map->dxl_anchor_2);
    }
  }

  if (mapped_goal < 0) {
    return 0U;
  }

  return (uint16_t)mapped_goal;
}

static uint8_t Motion_ParseSensorFrame(const volatile uint8_t *buffer,
                                       uint16_t positions[DXL_SERVO_COUNT])
{
  int32_t sensor_values[DXL_SERVO_COUNT] = {0};
  uint8_t count = 0U;
  uint8_t i = 2U;

  if ((buffer == NULL) || (positions == NULL)) {
    return 0U;
  }

  if ((buffer[0] != 'P') || (buffer[1] != ',')) {
    return 0U;
  }

  while (buffer[i] != '\0') {
    if (count >= DXL_SERVO_COUNT) {
      return 0U;
    }

    if (!Bluetooth_ParseSignedValue(buffer, &i, &sensor_values[count])) {
      return 0U;
    }

    ++count;

    if (buffer[i] == ',') {
      ++i;

      if (buffer[i] == '\0') {
        return 0U;
      }
    }
  }

  if (count != DXL_SERVO_COUNT) {
    return 0U;
  }

  for (i = 0U; i < DXL_SERVO_COUNT; ++i) {
    positions[i] = Bluetooth_MapSensorToGoal(i, sensor_values[i]);
  }

  return 1U;
}

static void Motion_ExecuteSensorFrame(const volatile uint8_t *buffer,
                                      const char *source_tag)
{
  uint16_t positions[DXL_SERVO_COUNT] = {0};
  DxlStatus status;

  if (!Motion_ParseSensorFrame(buffer, positions)) {
    Console_WriteString("\r\n[");
    Console_WriteString(source_tag);
    Console_WriteString("] Invalid parameter format\r\n> ");
    return;
  }

  status = Dxl_MoveArmSafe(positions);
  if (status == DXL_STATUS_OK) {
    Console_WriteString("\r\n[");
    Console_WriteString(source_tag);
    Console_WriteString("] Move OK\r\n> ");
  } else {
    Console_WriteString("\r\n[");
    Console_WriteString(source_tag);
    Console_WriteString("] Move Failed: ");
    Console_WriteString(Dxl_StatusToString(status));
    Console_WriteString("\r\n> ");
  }
}

static void Bluetooth_ProcessCommand(void)
{
  if (!g_uart3_rx_ready) {
    return;
  }

  g_uart3_rx_ready = 0U;

  if (g_uart3_rx_buf[0] == 'P' && g_uart3_rx_buf[1] == ',') {
    Motion_ExecuteSensorFrame(g_uart3_rx_buf, "BT");
  }
}

static void Console_WriteString(const char *text) {
  size_t length = 0U;

  if (text == NULL) {
    return;
  }

  while (text[length] != '\0') {
    ++length;
  }

  if (length > 0U) {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)length, 100U);
  }
}

static void Console_PrintBanner(void) {
  Console_WriteString("\r\nRobotARM ready\r\n");
  Console_WriteString("h: move home\r\n");
  Console_WriteString("s: move stretched\r\n");
  Console_WriteString("?: show help\r\n> ");
}

static void Console_ProcessCommand(void) {
  if (!g_uart1_rx_ready) {
    return;
  }

  g_uart1_rx_ready = 0U;

  if ((g_uart1_rx_buf[0] == 'P') && (g_uart1_rx_buf[1] == ',')) {
    Motion_ExecuteSensorFrame(g_uart1_rx_buf, "UART1");
    return;
  }

  switch (g_uart1_rx_buf[0]) {
  case 'h':
  case 'H':
    g_arm_move_request = ARM_MOVE_REQUEST_HOME;
    Console_WriteString("\r\nHOME requested\r\n> ");
    break;

  case 's':
  case 'S':
    g_arm_move_request = ARM_MOVE_REQUEST_STRETCHED;
    Console_WriteString("\r\nSTRETCHED requested\r\n> ");
    break;

  case '?':
  case 'm':
  case 'M':
    Console_WriteString("\r\nh: move home\r\n");
    Console_WriteString("s: move stretched\r\n");
    Console_WriteString("?: show help\r\n> ");
    break;

  case '\r':
  case '\n':
  case '\0':
    break;

  default:
    Console_WriteString("\r\nUnknown command\r\n> ");
    break;
  }
}

static void Console_WriteUInt(uint32_t value) {
  char buffer[11];
  uint8_t index = 0U;

  if (value == 0U) {
    Console_WriteString("0");
    return;
  }

  while ((value > 0U) && (index < (uint8_t)sizeof(buffer))) {
    buffer[index] = (char)('0' + (value % 10U));
    value /= 10U;
    ++index;
  }

  while (index > 0U) {
    --index;
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[index], 1U, 100U);
  }
}

static const char *Dxl_StatusToString(DxlStatus status) {
  switch (status) {
  case DXL_STATUS_OK:
    return "OK";
  case DXL_STATUS_ERROR:
    return "ERROR";
  case DXL_STATUS_TIMEOUT:
    return "TIMEOUT";
  case DXL_STATUS_BAD_PARAM:
    return "BAD_PARAM";
  case DXL_STATUS_BAD_PACKET:
    return "BAD_PACKET";
  case DXL_STATUS_BAD_CHECKSUM:
    return "BAD_CHECKSUM";
  case DXL_STATUS_HAL_ERROR:
    return "HAL_ERROR";
  case DXL_STATUS_RANGE:
    return "RANGE";
  default:
    return "UNKNOWN";
  }
}

static void Console_ReportArmMoveStatus(uint32_t request, DxlStatus status) {
  const char *pose_name;

  if (request == ARM_MOVE_REQUEST_HOME) {
    pose_name = "HOME";
  } else if (request == ARM_MOVE_REQUEST_STRETCHED) {
    pose_name = "STRETCHED";
  } else {
    pose_name = "UNKNOWN";
  }

  Console_WriteString("\r\n");
  Console_WriteString(pose_name);
  Console_WriteString(" move ");
  Console_WriteString((status == DXL_STATUS_OK) ? "done" : "failed");
  Console_WriteString(" (");
  Console_WriteString(Dxl_StatusToString(status));
  Console_WriteString(")\r\n> ");
}

static void Console_ReportServoDebugState(void) {
  uint8_t index;

  for (index = 0U; index < DXL_SERVO_COUNT; ++index) {
    uint8_t id = g_dxl_servo_configs[index].id;
    uint8_t torque_enabled = 0U;
    uint8_t torque_error = 0xFFU;
    uint16_t goal_position = 0U;
    uint8_t goal_error = 0xFFU;
    DxlPresentPosition present_position;
    DxlStatus torque_status;
    DxlStatus goal_status;
    DxlStatus present_status;

    torque_status = Dxl_ReadTorqueEnable(id, &torque_enabled, &torque_error);
    goal_status = Dxl_ReadGoalPosition(id, &goal_position, &goal_error);
    present_status = Dxl_GetPresentPosition(id, &present_position);

    Console_WriteString("ID");
    Console_WriteUInt(id);
    Console_WriteString(" T=");
    if (torque_status == DXL_STATUS_OK) {
      Console_WriteUInt(torque_enabled);
    } else {
      Console_WriteString("ERR:");
      Console_WriteString(Dxl_StatusToString(torque_status));
    }

    Console_WriteString(" G=");
    if (goal_status == DXL_STATUS_OK) {
      Console_WriteUInt(goal_position);
    } else {
      Console_WriteString("ERR:");
      Console_WriteString(Dxl_StatusToString(goal_status));
    }

    Console_WriteString(" P=");
    if ((present_status == DXL_STATUS_OK) && (present_position.valid != 0U)) {
      Console_WriteUInt(present_position.position);
    } else {
      Console_WriteString("ERR");
    }

    Console_WriteString("\r\n");
  }

  Console_WriteString("> ");
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  Dxl_Init(&huart2);
  Console_PrintBanner();
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_uart1_rx_byte, 1U);
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&g_uart3_rx_byte, 1U);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Scheduler owns application execution after osKernelStart(). */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    if ((g_uart1_rx_index == 0U) &&
        ((g_uart1_rx_byte == 'h') || (g_uart1_rx_byte == 'H') ||
         (g_uart1_rx_byte == 's') || (g_uart1_rx_byte == 'S') ||
         (g_uart1_rx_byte == '?') || (g_uart1_rx_byte == 'm') ||
         (g_uart1_rx_byte == 'M'))) {
      g_uart1_rx_buf[0] = g_uart1_rx_byte;
      g_uart1_rx_buf[1] = '\0';
      g_uart1_rx_ready = 1U;
      g_uart1_rx_index = 0U;
    } else if ((g_uart1_rx_byte == '\n') || (g_uart1_rx_byte == '\r')) {
      if (g_uart1_rx_index > 0U) {
        g_uart1_rx_buf[g_uart1_rx_index] = '\0';
        g_uart1_rx_ready = 1U;
      }
      g_uart1_rx_index = 0U;
    } else {
      g_uart1_rx_buf[g_uart1_rx_index++] = g_uart1_rx_byte;
      if (g_uart1_rx_index >= (UART1_RX_BUF_SIZE - 1U)) {
        g_uart1_rx_buf[UART1_RX_BUF_SIZE - 1U] = '\0';
        g_uart1_rx_ready = 1U;
        g_uart1_rx_index = 0U;
      }
    }
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_uart1_rx_byte, 1U);
  } else if (huart->Instance == USART3) {
    if (g_uart3_rx_byte == '\n' || g_uart3_rx_byte == '\r') {
      g_uart3_rx_buf[g_uart3_rx_index] = '\0';
      g_uart3_rx_ready = 1U;
      g_uart3_rx_index = 0U;
    } else {
      g_uart3_rx_buf[g_uart3_rx_index++] = g_uart3_rx_byte;
      if (g_uart3_rx_index >= (UART3_RX_BUF_SIZE - 1U)) {
        g_uart3_rx_buf[UART3_RX_BUF_SIZE - 1U] = '\0';
        g_uart3_rx_ready = 1U;
        g_uart3_rx_index = 0U;
      }
    }
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&g_uart3_rx_byte, 1U);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
  /* USER CODE BEGIN 5 */
  uint32_t pending_request;

  /* Infinite loop */
  for (;;) {
    Console_ProcessCommand();
    Bluetooth_ProcessCommand();
    Dxl_Process(HAL_GetTick());
    pending_request = g_arm_move_request;
    Dxl_ProcessArmMoveRequest();
    if ((pending_request == ARM_MOVE_REQUEST_HOME) ||
        (pending_request == ARM_MOVE_REQUEST_STRETCHED)) {
      Console_ReportArmMoveStatus(pending_request,
                                  (DxlStatus)g_arm_move_last_status);
      if (g_arm_move_last_status == DXL_STATUS_OK) {
        Console_ReportServoDebugState();
      }
    }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
