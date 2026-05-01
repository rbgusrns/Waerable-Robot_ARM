#ifndef __DYNAMIXEL_H
#define __DYNAMIXEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define DXL_SERVO_COUNT 6U

#define ARM_MOVE_REQUEST_NONE 0U
#define ARM_MOVE_REQUEST_HOME 1U
#define ARM_MOVE_REQUEST_STRETCHED 2U

typedef enum {
  DXL_STATUS_OK = 0,
  DXL_STATUS_ERROR,
  DXL_STATUS_TIMEOUT,
  DXL_STATUS_BAD_PARAM,
  DXL_STATUS_BAD_PACKET,
  DXL_STATUS_BAD_CHECKSUM,
  DXL_STATUS_HAL_ERROR,
  DXL_STATUS_RANGE
} DxlStatus;

typedef struct {
  uint8_t id;
  uint16_t min_position;
  uint16_t max_position;
  uint16_t speed;
} DxlServoConfig;

typedef struct {
  uint16_t position;
  uint8_t error;
  uint8_t valid;
} DxlPresentPosition;

extern volatile DxlPresentPosition g_dxl_present_positions[DXL_SERVO_COUNT];
extern volatile uint32_t g_dxl_poll_cycle;
extern volatile uint32_t g_dxl_last_status;
extern volatile uint32_t g_dxl_last_hal_error;
extern volatile uint32_t g_dxl_timeout_count;
extern volatile uint32_t g_dxl_bad_packet_count;
extern volatile uint32_t g_dxl_rx_ok_count;
extern volatile uint32_t g_arm_move_request;
extern volatile uint32_t g_arm_move_last_status;
extern const DxlServoConfig g_dxl_servo_configs[DXL_SERVO_COUNT];
extern const uint16_t g_arm_home_pose[DXL_SERVO_COUNT];
extern const uint16_t g_arm_stretched_pose[DXL_SERVO_COUNT];

void Dxl_Init(UART_HandleTypeDef *huart);
void Dxl_Process(uint32_t now_ms);
DxlStatus Dxl_ReadPresentPosition(uint8_t id, uint16_t *position, uint8_t *status_error);
DxlStatus Dxl_ReadTorqueEnable(uint8_t id, uint8_t *enabled, uint8_t *status_error);
DxlStatus Dxl_ReadGoalPosition(uint8_t id, uint16_t *position, uint8_t *status_error);
DxlStatus Dxl_GetPresentPosition(uint8_t id, DxlPresentPosition *present_position);
DxlStatus Dxl_GetPresentPositions(DxlPresentPosition *present_positions, uint8_t count);
DxlStatus Dxl_SetGoalPositionSafe(uint8_t id, uint16_t goal_position);
DxlStatus Dxl_MoveArmSafe(const uint16_t goal_positions[DXL_SERVO_COUNT]);
void Dxl_ProcessArmMoveRequest(void);

#ifdef __cplusplus
}
#endif

#endif
