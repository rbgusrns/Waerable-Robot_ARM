#include "dynamixel.h"

#define DXL_PACKET_OVERHEAD 6U
#define DXL_MAX_PACKET_SIZE 16U

#define DXL_INSTR_READ 0x02U
#define DXL_INSTR_WRITE 0x03U

#define DXL_ADDR_TORQUE_ENABLE 24U
#define DXL_ADDR_GOAL_POSITION 30U
#define DXL_ADDR_MOVING_SPEED 32U
#define DXL_ADDR_PRESENT_POSITION 36U

#define DXL_PRESENT_POSITION_SIZE 2U
#define DXL_POLL_INTERVAL_MS 50U
#define DXL_TX_TIMEOUT_MS 50U
#define DXL_RX_TIMEOUT_MS 50U
#define DXL_SLOW_SPEED_VALUE 80U
#define DXL_WRITE_SETTLE_MS 10U
#define DXL_WRITE_VERIFY_RETRIES 3U

static UART_HandleTypeDef *s_dxl_uart;
static uint32_t s_last_poll_ms;
static uint8_t s_next_poll_index;

volatile DxlPresentPosition g_dxl_present_positions[DXL_SERVO_COUNT];
volatile uint32_t g_dxl_poll_cycle;
volatile uint32_t g_dxl_last_status;
volatile uint32_t g_dxl_last_hal_error;
volatile uint32_t g_dxl_timeout_count;
volatile uint32_t g_dxl_bad_packet_count;
volatile uint32_t g_dxl_rx_ok_count;
volatile uint32_t g_arm_move_request;
volatile uint32_t g_arm_move_last_status;

const uint16_t g_arm_home_pose[DXL_SERVO_COUNT] = {1700U, 876U, 539U, 815U, 513U, 851U};
const uint16_t g_arm_stretched_pose[DXL_SERVO_COUNT] = {1700U, 350U, 169U, 501U, 775U, 253U};

const DxlServoConfig g_dxl_servo_configs[DXL_SERVO_COUNT] = {
    {0U, 623U, 2644U, DXL_SLOW_SPEED_VALUE},
    {1U, 268U, 876U, DXL_SLOW_SPEED_VALUE},
    {2U, 162U, 739U, DXL_SLOW_SPEED_VALUE},
    {3U, 190U, 824U, DXL_SLOW_SPEED_VALUE},
    {4U, 202U, 775U, DXL_SLOW_SPEED_VALUE},
    {5U, 253U, 851U, DXL_SLOW_SPEED_VALUE},
};

static uint8_t Dxl_Checksum(const uint8_t *packet, uint8_t packet_length)
{
  uint16_t sum = 0U;
  uint8_t index;

  for (index = 2U; index < (packet_length - 1U); ++index) {
    sum = (uint16_t)(sum + packet[index]);
  }

  return (uint8_t)(~sum);
}

static const DxlServoConfig *Dxl_FindConfig(uint8_t id)
{
  uint8_t index;

  for (index = 0U; index < DXL_SERVO_COUNT; ++index) {
    if (g_dxl_servo_configs[index].id == id) {
      return &g_dxl_servo_configs[index];
    }
  }

  return NULL;
}

static DxlPresentPosition *Dxl_FindPositionSlot(uint8_t id)
{
  uint8_t index;

  for (index = 0U; index < DXL_SERVO_COUNT; ++index) {
    if (g_dxl_servo_configs[index].id == id) {
      return (DxlPresentPosition *)&g_dxl_present_positions[index];
    }
  }

  return NULL;
}

static uint16_t Dxl_SaturateGoalPosition(uint8_t id, uint16_t goal_position)
{
  const DxlServoConfig *config = Dxl_FindConfig(id);

  if (config == NULL) {
    return goal_position;
  }

  if (goal_position < config->min_position) {
    return config->min_position;
  }
  if (goal_position > config->max_position) {
    return config->max_position;
  }

  return goal_position;
}

static DxlStatus Dxl_SetDirectionTx(void)
{
  if (s_dxl_uart == NULL) {
    return DXL_STATUS_BAD_PARAM;
  }

  __HAL_UART_CLEAR_PEFLAG(s_dxl_uart);
  __HAL_UART_CLEAR_FEFLAG(s_dxl_uart);
  __HAL_UART_CLEAR_NEFLAG(s_dxl_uart);
  __HAL_UART_CLEAR_FLAG(s_dxl_uart, UART_CLEAR_OREF);
  __HAL_UART_FLUSH_DRREGISTER(s_dxl_uart);
  s_dxl_uart->ErrorCode = HAL_UART_ERROR_NONE;

  if (HAL_HalfDuplex_EnableTransmitter(s_dxl_uart) != HAL_OK) {
    return DXL_STATUS_HAL_ERROR;
  }

  return DXL_STATUS_OK;
}

static DxlStatus Dxl_SetDirectionRx(void)
{
  if (s_dxl_uart == NULL) {
    return DXL_STATUS_BAD_PARAM;
  }

  __HAL_UART_CLEAR_PEFLAG(s_dxl_uart);
  __HAL_UART_CLEAR_FEFLAG(s_dxl_uart);
  __HAL_UART_CLEAR_NEFLAG(s_dxl_uart);
  __HAL_UART_CLEAR_FLAG(s_dxl_uart, UART_CLEAR_OREF);
  __HAL_UART_FLUSH_DRREGISTER(s_dxl_uart);
  s_dxl_uart->ErrorCode = HAL_UART_ERROR_NONE;

  if (HAL_HalfDuplex_EnableReceiver(s_dxl_uart) != HAL_OK) {
    return DXL_STATUS_HAL_ERROR;
  }

  return DXL_STATUS_OK;
}

static DxlStatus Dxl_ReceiveByte(uint8_t *value, uint32_t timeout_ms)
{
  if ((s_dxl_uart == NULL) || (value == NULL)) {
    return DXL_STATUS_BAD_PARAM;
  }

  switch (HAL_UART_Receive(s_dxl_uart, value, 1U, timeout_ms)) {
    case HAL_OK:
      g_dxl_last_hal_error = s_dxl_uart->ErrorCode;
      ++g_dxl_rx_ok_count;
      return DXL_STATUS_OK;
    case HAL_TIMEOUT:
      g_dxl_last_hal_error = s_dxl_uart->ErrorCode;
      ++g_dxl_timeout_count;
      return DXL_STATUS_TIMEOUT;
    default:
      g_dxl_last_hal_error = s_dxl_uart->ErrorCode;
      return DXL_STATUS_HAL_ERROR;
  }
}

static DxlStatus Dxl_ReceiveStatusPacket(uint8_t expected_id,
                                         uint8_t *parameters,
                                         uint8_t parameter_capacity,
                                         uint8_t *parameter_length,
                                         uint8_t *status_error)
{
  uint8_t state = 0U;
  uint8_t id = 0U;
  uint8_t length = 0U;
  uint8_t error = 0U;
  uint8_t checksum = 0U;
  uint8_t index = 0U;
  uint8_t raw[DXL_MAX_PACKET_SIZE] = {0U};
  uint8_t value = 0U;
  DxlStatus status;

  while (state < 2U) {
    status = Dxl_ReceiveByte(&value, DXL_RX_TIMEOUT_MS);
    if (status != DXL_STATUS_OK) {
      return status;
    }

    if (value == 0xFFU) {
      ++state;
    } else {
      state = 0U;
    }
  }

  raw[0] = 0xFFU;
  raw[1] = 0xFFU;

  status = Dxl_ReceiveByte(&id, DXL_RX_TIMEOUT_MS);
  if (status != DXL_STATUS_OK) {
    return status;
  }

  status = Dxl_ReceiveByte(&length, DXL_RX_TIMEOUT_MS);
  if (status != DXL_STATUS_OK) {
    return status;
  }

  if (length < 2U) {
    ++g_dxl_bad_packet_count;
    return DXL_STATUS_BAD_PACKET;
  }

  if ((uint8_t)(length + 4U) > DXL_MAX_PACKET_SIZE) {
    ++g_dxl_bad_packet_count;
    return DXL_STATUS_BAD_PACKET;
  }

  status = Dxl_ReceiveByte(&error, DXL_RX_TIMEOUT_MS);
  if (status != DXL_STATUS_OK) {
    return status;
  }

  raw[2] = id;
  raw[3] = length;
  raw[4] = error;

  if (id != expected_id) {
    ++g_dxl_bad_packet_count;
    return DXL_STATUS_BAD_PACKET;
  }

  if ((length - 2U) > parameter_capacity) {
    ++g_dxl_bad_packet_count;
    return DXL_STATUS_BAD_PACKET;
  }

  for (index = 0U; index < (length - 2U); ++index) {
    status = Dxl_ReceiveByte(&parameters[index], DXL_RX_TIMEOUT_MS);
    if (status != DXL_STATUS_OK) {
      return status;
    }
    raw[5U + index] = parameters[index];
  }

  status = Dxl_ReceiveByte(&checksum, DXL_RX_TIMEOUT_MS);
  if (status != DXL_STATUS_OK) {
    return status;
  }

  raw[5U + index] = checksum;

  if (Dxl_Checksum(raw, (uint8_t)(length + 4U)) != checksum) {
    ++g_dxl_bad_packet_count;
    return DXL_STATUS_BAD_CHECKSUM;
  }

  if (parameter_length != NULL) {
    *parameter_length = (uint8_t)(length - 2U);
  }

  if (status_error != NULL) {
    *status_error = error;
  }

  if (error != 0U) {
    return DXL_STATUS_ERROR;
  }

  return DXL_STATUS_OK;
}

static DxlStatus Dxl_TransmitInstruction(uint8_t id,
                                         uint8_t instruction,
                                         const uint8_t *parameters,
                                         uint8_t parameter_length,
                                         uint8_t expect_status,
                                         uint8_t *response_parameters,
                                         uint8_t response_capacity,
                                         uint8_t *response_length,
                                         uint8_t *status_error)
{
  uint8_t packet[DXL_MAX_PACKET_SIZE] = {0U};
  uint8_t index;
  DxlStatus status;

  if (parameter_length > (DXL_MAX_PACKET_SIZE - DXL_PACKET_OVERHEAD)) {
    return DXL_STATUS_BAD_PARAM;
  }

  packet[0] = 0xFFU;
  packet[1] = 0xFFU;
  packet[2] = id;
  packet[3] = (uint8_t)(parameter_length + 2U);
  packet[4] = instruction;

  for (index = 0U; index < parameter_length; ++index) {
    packet[5U + index] = parameters[index];
  }

  packet[5U + parameter_length] = Dxl_Checksum(packet, (uint8_t)(parameter_length + DXL_PACKET_OVERHEAD));

  status = Dxl_SetDirectionTx();
  if (status != DXL_STATUS_OK) {
    return status;
  }

  if (HAL_UART_Transmit(s_dxl_uart,
                        packet,
                        (uint16_t)(parameter_length + DXL_PACKET_OVERHEAD),
                        DXL_TX_TIMEOUT_MS) != HAL_OK) {
    return DXL_STATUS_HAL_ERROR;
  }

  if (__HAL_UART_GET_FLAG(s_dxl_uart, UART_FLAG_TC) == RESET) {
    uint32_t start = HAL_GetTick();

    while (__HAL_UART_GET_FLAG(s_dxl_uart, UART_FLAG_TC) == RESET) {
      if ((HAL_GetTick() - start) > DXL_TX_TIMEOUT_MS) {
        return DXL_STATUS_TIMEOUT;
      }
    }
  }

  status = Dxl_SetDirectionRx();
  if (status != DXL_STATUS_OK) {
    return status;
  }

  if (expect_status == 0U) {
    return DXL_STATUS_OK;
  }

  return Dxl_ReceiveStatusPacket(id,
                                 response_parameters,
                                 response_capacity,
                                  response_length,
                                  status_error);
}

static DxlStatus Dxl_ReadData(uint8_t id,
                              uint8_t address,
                              uint8_t size,
                              uint8_t *response,
                              uint8_t *status_error)
{
  uint8_t parameters[2] = {address, size};
  uint8_t response_length = 0U;
  DxlStatus status;

  if ((response == NULL) || (Dxl_FindConfig(id) == NULL) || (size == 0U)) {
    return DXL_STATUS_BAD_PARAM;
  }

  status = Dxl_TransmitInstruction(id,
                                   DXL_INSTR_READ,
                                   parameters,
                                   2U,
                                   1U,
                                   response,
                                   size,
                                   &response_length,
                                   status_error);
  if (status != DXL_STATUS_OK) {
    return status;
  }

  if (response_length != size) {
    return DXL_STATUS_BAD_PACKET;
  }

  return DXL_STATUS_OK;
}

static DxlStatus Dxl_WriteWord(uint8_t id, uint8_t address, uint16_t value)
{
  uint8_t parameters[3];

  parameters[0] = address;
  parameters[1] = (uint8_t)(value & 0xFFU);
  parameters[2] = (uint8_t)((value >> 8) & 0xFFU);

  return Dxl_TransmitInstruction(id,
                                 DXL_INSTR_WRITE,
                                 parameters,
                                 3U,
                                 0U,
                                 NULL,
                                 0U,
                                 NULL,
                                 NULL);
}

static DxlStatus Dxl_WriteByte(uint8_t id, uint8_t address, uint8_t value)
{
  uint8_t parameters[2];

  parameters[0] = address;
  parameters[1] = value;

  return Dxl_TransmitInstruction(id,
                                 DXL_INSTR_WRITE,
                                 parameters,
                                 2U,
                                 0U,
                                 NULL,
                                 0U,
                                 NULL,
                                 NULL);
}

void Dxl_Init(UART_HandleTypeDef *huart)
{
  uint8_t index;

  s_dxl_uart = huart;
  s_last_poll_ms = 0U;
  s_next_poll_index = 0U;
  g_dxl_poll_cycle = 0U;
  g_dxl_last_status = DXL_STATUS_OK;
  g_dxl_last_hal_error = HAL_UART_ERROR_NONE;
  g_dxl_timeout_count = 0U;
  g_dxl_bad_packet_count = 0U;
  g_dxl_rx_ok_count = 0U;
  g_arm_move_request = ARM_MOVE_REQUEST_NONE;
  g_arm_move_last_status = DXL_STATUS_OK;

  for (index = 0U; index < DXL_SERVO_COUNT; ++index) {
    g_dxl_present_positions[index].position = 0U;
    g_dxl_present_positions[index].error = 0xFFU;
    g_dxl_present_positions[index].valid = 0U;
  }

  if (s_dxl_uart != NULL) {
    (void)Dxl_SetDirectionRx();
  }
}

void Dxl_Process(uint32_t now_ms)
{
  const DxlServoConfig *config;
  DxlPresentPosition *slot;
  uint16_t position = 0U;
  uint8_t status_error = 0xFFU;
  DxlStatus status;

  if (s_dxl_uart == NULL) {
    return;
  }

  if ((now_ms - s_last_poll_ms) < DXL_POLL_INTERVAL_MS) {
    return;
  }

  s_last_poll_ms = now_ms;
  config = &g_dxl_servo_configs[s_next_poll_index];
  slot = Dxl_FindPositionSlot(config->id);
  status = Dxl_ReadPresentPosition(config->id, &position, &status_error);
  g_dxl_last_status = (uint32_t)status;

  if (slot != NULL) {
    slot->error = status_error;
    if (status == DXL_STATUS_OK) {
      slot->position = position;
      slot->valid = 1U;
    } else {
      slot->valid = 0U;
    }
  }

  ++s_next_poll_index;
  if (s_next_poll_index >= DXL_SERVO_COUNT) {
    s_next_poll_index = 0U;
    ++g_dxl_poll_cycle;
  }
}

DxlStatus Dxl_ReadPresentPosition(uint8_t id, uint16_t *position, uint8_t *status_error)
{
  uint8_t response[DXL_PRESENT_POSITION_SIZE] = {0U};
  DxlStatus status;

  if (position == NULL) {
    return DXL_STATUS_BAD_PARAM;
  }

  status = Dxl_ReadData(id,
                        DXL_ADDR_PRESENT_POSITION,
                        DXL_PRESENT_POSITION_SIZE,
                        response,
                        status_error);
  if (status != DXL_STATUS_OK) {
    return status;
  }

  *position = (uint16_t)((uint16_t)response[0] | ((uint16_t)response[1] << 8));

  return DXL_STATUS_OK;
}

DxlStatus Dxl_ReadTorqueEnable(uint8_t id, uint8_t *enabled, uint8_t *status_error)
{
  uint8_t response[1] = {0U};
  DxlStatus status;

  if (enabled == NULL) {
    return DXL_STATUS_BAD_PARAM;
  }

  status = Dxl_ReadData(id, DXL_ADDR_TORQUE_ENABLE, 1U, response, status_error);
  if (status != DXL_STATUS_OK) {
    return status;
  }

  *enabled = response[0];
  return DXL_STATUS_OK;
}

DxlStatus Dxl_ReadGoalPosition(uint8_t id, uint16_t *position, uint8_t *status_error)
{
  uint8_t response[2] = {0U};
  DxlStatus status;

  if (position == NULL) {
    return DXL_STATUS_BAD_PARAM;
  }

  status = Dxl_ReadData(id, DXL_ADDR_GOAL_POSITION, 2U, response, status_error);
  if (status != DXL_STATUS_OK) {
    return status;
  }

  *position = (uint16_t)((uint16_t)response[0] | ((uint16_t)response[1] << 8));
  return DXL_STATUS_OK;
}

DxlStatus Dxl_GetPresentPosition(uint8_t id, DxlPresentPosition *present_position)
{
  DxlPresentPosition *slot;

  if (present_position == NULL) {
    return DXL_STATUS_BAD_PARAM;
  }

  slot = Dxl_FindPositionSlot(id);
  if (slot == NULL) {
    return DXL_STATUS_BAD_PARAM;
  }

  present_position->position = slot->position;
  present_position->error = slot->error;
  present_position->valid = slot->valid;

  return DXL_STATUS_OK;
}

DxlStatus Dxl_GetPresentPositions(DxlPresentPosition *present_positions, uint8_t count)
{
  uint8_t index;

  if ((present_positions == NULL) || (count < DXL_SERVO_COUNT)) {
    return DXL_STATUS_BAD_PARAM;
  }

  for (index = 0U; index < DXL_SERVO_COUNT; ++index) {
    present_positions[index].position = g_dxl_present_positions[index].position;
    present_positions[index].error = g_dxl_present_positions[index].error;
    present_positions[index].valid = g_dxl_present_positions[index].valid;
  }

  return DXL_STATUS_OK;
}

DxlStatus Dxl_SetGoalPositionSafe(uint8_t id, uint16_t goal_position)
{
  const DxlServoConfig *config = Dxl_FindConfig(id);
  uint8_t attempt;
  DxlStatus status;
  uint16_t safe_goal;

  if (config == NULL) {
    return DXL_STATUS_BAD_PARAM;
  }

  safe_goal = Dxl_SaturateGoalPosition(id, goal_position);

  for (attempt = 0U; attempt < DXL_WRITE_VERIFY_RETRIES; ++attempt) {
    uint16_t verified_goal = 0U;
    uint8_t status_error = 0xFFU;

    status = Dxl_WriteByte(id, DXL_ADDR_TORQUE_ENABLE, 1U);
    if (status != DXL_STATUS_OK) {
      return status;
    }

    HAL_Delay(DXL_WRITE_SETTLE_MS);

    status = Dxl_WriteWord(id, DXL_ADDR_MOVING_SPEED, config->speed);
    if (status != DXL_STATUS_OK) {
      return status;
    }

    HAL_Delay(DXL_WRITE_SETTLE_MS);

    status = Dxl_WriteWord(id, DXL_ADDR_GOAL_POSITION, safe_goal);
    if (status != DXL_STATUS_OK) {
      return status;
    }

    HAL_Delay(DXL_WRITE_SETTLE_MS);

    status = Dxl_ReadGoalPosition(id, &verified_goal, &status_error);
    if ((status == DXL_STATUS_OK) && (verified_goal == safe_goal)) {
      return DXL_STATUS_OK;
    }

    HAL_Delay(DXL_WRITE_SETTLE_MS);
  }

  return DXL_STATUS_TIMEOUT;
}

DxlStatus Dxl_MoveArmSafe(const uint16_t goal_positions[DXL_SERVO_COUNT])
{
  uint8_t index;
  DxlStatus status;
  uint16_t safe_goals[DXL_SERVO_COUNT];

  if (goal_positions == NULL) {
    return DXL_STATUS_BAD_PARAM;
  }

  for (index = 0U; index < DXL_SERVO_COUNT; ++index) {
    safe_goals[index] = Dxl_SaturateGoalPosition(g_dxl_servo_configs[index].id, goal_positions[index]);
  }

  for (index = 0U; index < DXL_SERVO_COUNT; ++index) {
    status = Dxl_SetGoalPositionSafe(g_dxl_servo_configs[index].id, safe_goals[index]);
    if (status != DXL_STATUS_OK) {
      return status;
    }

    HAL_Delay(DXL_WRITE_SETTLE_MS);
  }

  return DXL_STATUS_OK;
}

void Dxl_ProcessArmMoveRequest(void)
{
  const uint16_t *target_pose = NULL;
  DxlStatus status;

  if (g_arm_move_request == ARM_MOVE_REQUEST_HOME) {
    target_pose = g_arm_home_pose;
  } else if (g_arm_move_request == ARM_MOVE_REQUEST_STRETCHED) {
    target_pose = g_arm_stretched_pose;
  } else {
    return;
  }

  status = Dxl_MoveArmSafe(target_pose);
  g_arm_move_last_status = (uint32_t)status;
  g_arm_move_request = ARM_MOVE_REQUEST_NONE;
}
