import os

filepath = r'c:\STproject\RobotARM\Core\Src\main.c'

with open(filepath, 'r', encoding='utf-8') as f:
    content = f.read()

# 1. PTD (Typedefs)
ptd_marker = "/* USER CODE BEGIN PTD */\n"
if ptd_marker not in content:
    ptd_marker = "/* USER CODE BEGIN PTD */\r\n"
    
ptd_code = """
typedef struct {
  int32_t sensor_anchor_0;
  int32_t sensor_anchor_1;
  int32_t sensor_anchor_2;
  uint16_t dxl_anchor_0;
  uint16_t dxl_anchor_1;
  uint16_t dxl_anchor_2;
} BluetoothServoMap;
"""
content = content.replace(ptd_marker, ptd_marker + ptd_code.strip() + "\n", 1)

# 2. PV (Variables)
pv_marker = "volatile uint8_t g_uart3_rx_ready = 0U;\n"
if pv_marker not in content:
    pv_marker = "volatile uint8_t g_uart3_rx_ready = 0U;\r\n"

pv_code = """
static const BluetoothServoMap g_bluetooth_servo_maps[DXL_SERVO_COUNT] = {
    {-90, 0, 90, 2644U, 1628U, 623U},
    {-90, 0, 90, 268U, 562U, 876U},
    {1023, 512, 0, 739U, 450U, 162U},
    {1023, 512, 0, 824U, 507U, 190U},
    {-90, 0, 90, 775U, 450U, 202U},
    {1023, 512, 0, 851U, 551U, 253U},
};
"""
content = content.replace(pv_marker, pv_marker + pv_code, 1)

# 3. PFP (Prototypes)
pfp_marker = "static void Bluetooth_ProcessCommand(void);\n"
if pfp_marker not in content:
    pfp_marker = "static void Bluetooth_ProcessCommand(void);\r\n"

pfp_code = """
static uint8_t Bluetooth_ParseSignedValue(const volatile uint8_t *buffer, uint8_t *index, int32_t *value);
static int32_t Bluetooth_MapLinearClamped(int32_t input,
                                          int32_t input_start,
                                          int32_t input_end,
                                          int32_t output_start,
                                          int32_t output_end);
static uint16_t Bluetooth_MapSensorToGoal(uint8_t servo_index, int32_t sensor_value);
"""
content = content.replace(pfp_marker, pfp_marker + pfp_code.strip() + "\n", 1)

# 4. Implementation in USER CODE BEGIN 0
# First find the existing Bluetooth_ProcessCommand
start_idx = content.find("static void Bluetooth_ProcessCommand(void)")
if start_idx != -1:
    # Find the end of this function
    brace_count = 0
    started = False
    func_end = start_idx
    search_start = content.find("{", start_idx)
    for j in range(search_start, len(content)):
        if content[j] == '{':
            brace_count += 1
            started = True
        elif content[j] == '}':
            brace_count -= 1
            if started and brace_count == 0:
                func_end = j + 1
                break
                
    impl_code = """static uint8_t Bluetooth_ParseSignedValue(const volatile uint8_t *buffer, uint8_t *index, int32_t *value)
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

  if ((buffer[i] != ',') && (buffer[i] != '\\0')) {
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

static void Bluetooth_ProcessCommand(void)
{
  if (!g_uart3_rx_ready) {
    return;
  }

  g_uart3_rx_ready = 0U;

  if (g_uart3_rx_buf[0] == 'P' && g_uart3_rx_buf[1] == ',') {
    uint16_t positions[DXL_SERVO_COUNT] = {0};
    int32_t sensor_values[DXL_SERVO_COUNT] = {0};
    uint8_t count = 0U;
    uint8_t i = 2U;
    uint8_t valid_frame = 1U;

    while (g_uart3_rx_buf[i] != '\\0') {
      if (count >= DXL_SERVO_COUNT) {
        valid_frame = 0U;
        break;
      }

      if (!Bluetooth_ParseSignedValue(g_uart3_rx_buf, &i, &sensor_values[count])) {
        valid_frame = 0U;
        break;
      }

      ++count;

      if (g_uart3_rx_buf[i] == ',') {
        ++i;

        if (g_uart3_rx_buf[i] == '\\0') {
          valid_frame = 0U;
          break;
        }
      }
    }

    if (valid_frame && (count == DXL_SERVO_COUNT)) {
      for (i = 0U; i < DXL_SERVO_COUNT; ++i) {
        positions[i] = Bluetooth_MapSensorToGoal(i, sensor_values[i]);
      }

      DxlStatus status = Dxl_MoveArmSafe(positions);
      if (status == DXL_STATUS_OK) {
        Console_WriteString("\\r\\n[BT] Move OK\\r\\n> ");
      } else {
        Console_WriteString("\\r\\n[BT] Move Failed: ");
        Console_WriteString(Dxl_StatusToString(status));
        Console_WriteString("\\r\\n> ");
      }
    } else if (!valid_frame) {
      Console_WriteString("\\r\\n[BT] Invalid parameter format\\r\\n> ");
    } else {
      Console_WriteString("\\r\\n[BT] Invalid parameter count\\r\\n> ");
    }
  }
}"""
    content = content[:start_idx] + impl_code + content[func_end:]

with open(filepath, 'w', encoding='utf-8') as f:
    f.write(content)

print("Mapping logic restored successfully.")
