# Dynamixel RobotARM Handoff

## 목적

이 문서는 현재 `RobotARM` 펌웨어에서 구현된 다이나믹셀 통신/제어 상태를 다음 작업자가 빠르게 이해하고 이어서 개발할 수 있도록 정리한 handoff 문서다.

문서 내용은 두 종류로 구분한다.

- 코드 기준 사실: 현재 저장소 코드에서 확인 가능한 내용
- 실기 확인 사실: 이번 작업 중 실제 보드/모터에서 확인한 내용


## 현재 하드웨어/통신 전제

- MCU: STM32G474VET6
- Dynamixel bus: TTL, half-duplex UART
- Dynamixel UART: `USART2` on `PA2`
- PC/터미널 UART: `USART1`
- Baud rate: `115200`
- Servo count: `6`
- Servo IDs:
  - ID0: MX-64
  - ID1~ID5: AX-12


## 현재 코드 구조

핵심 파일:

- `Core/Inc/dynamixel.h`
- `Core/Src/dynamixel.c`
- `Core/Src/main.c`

역할 분리:

- `Core/Src/main.c`
  - `USART1`, `USART2`, `USART3` 초기화
  - FreeRTOS default task 생성
  - 스케줄러 시작
  - 현재는 `StartDefaultTask()` 안에서 아래 3개를 반복 호출
    - `Console_ProcessCommand()`
    - `Dxl_Process(HAL_GetTick())`
    - `Dxl_ProcessArmMoveRequest()`
- `Core/Src/dynamixel.c`
  - Dynamixel Protocol 1.0 패킷 송수신
  - half-duplex TX/RX 전환
  - 현재 위치 polling
  - 단일 모터 안전 이동
  - 6축 전체 안전 이동
  - home/stretch pose 요청 처리


## 현재 RTOS 반영 상태

현재 프로젝트는 더 이상 순수 bare-metal 루프만 도는 상태가 아니다.

현재 반영된 내용:

- FreeRTOS middleware가 프로젝트에 들어와 있음
- HAL timebase가 `SysTick`에서 `TIM6`로 전환됨
- 기존 Dynamixel 제어 루프는 `StartDefaultTask()` 안으로 옮겨져 동작 유지 중
- GCC/CMake 기준 RTOS 빌드가 통과하는 상태까지 연결됨

중요 해석:

- 아직 Console task / Dxl task로 분리된 구조는 아니다.
- 현재는 **기존 제어 루프를 RTOS task 하나 안으로 감싼 1차 이행 상태**로 보는 것이 정확하다.


## 현재 지원 기능

### 1. 현재 위치 polling

현재 위치는 `Dxl_Process()`가 주기적으로 읽어서 캐시에 저장한다.

관련 API:

- `Dxl_ReadPresentPosition()`
- `Dxl_GetPresentPosition()`
- `Dxl_GetPresentPositions()`

캐시 변수:

- `g_dxl_present_positions[DXL_SERVO_COUNT]`
- `g_dxl_poll_cycle`

주의:

- 전체 6축을 한 번에 읽는 동기화 read가 아니라, polling 기반 캐시 갱신이다.
- 따라서 6축 상태는 완전 동시 스냅샷이 아니라 약간의 시간차가 있을 수 있다.


### 2. 단일 모터 안전 이동

API:

- `Dxl_SetGoalPositionSafe(uint8_t id, uint16_t goal_position)`

동작 방식:

1. ID 유효성 확인
2. 안전 범위 포화(Saturation) 처리 (min 미만은 min으로, max 초과는 max로 제한)
3. Torque Enable write
4. Moving Speed write
5. Goal Position write


### 3. 6축 전체 안전 이동

API:

- `Dxl_MoveArmSafe(const uint16_t goal_positions[DXL_SERVO_COUNT])`

중요 동작 (Saturation 적용):

- 먼저 6축 전체 목표값에 대해 각각 안전 범위(Min/Max)를 검사합니다.
- 범위를 벗어난 목표값은 해당 축의 한계값(min 또는 max)으로 자동 고정(Saturate)됩니다.
- 거부(all-or-nothing) 없이, 포화된 안전한 값으로 각 축을 순차적으로 제어하여 모터가 낼 수 있는 최대 범위까지만 움직이도록 보장합니다.


### 4. pose 요청 기반 이동

요청 상수:

- `ARM_MOVE_REQUEST_NONE = 0`
- `ARM_MOVE_REQUEST_HOME = 1`
- `ARM_MOVE_REQUEST_STRETCHED = 2`

관련 변수:

- `g_arm_move_request`
- `g_arm_move_last_status`

처리 함수:

- `Dxl_ProcessArmMoveRequest()`

동작 방식:

- `g_arm_move_request`에 값이 써지면 현재는 `StartDefaultTask()` 안의 제어 루프에서 한 번 처리한다.
- 처리 후 `g_arm_move_request`는 다시 `0`으로 돌아간다.
- 결과 status는 `g_arm_move_last_status`에 남는다.


## 현재 안전 범위

현재 코드 기준 안전 범위는 `g_dxl_servo_configs`에 정의되어 있다.

| ID | Model | Min | Max |
|---|---|---:|---:|
| 0 | MX-64 | 623 | 2644 |
| 1 | AX-12 | 268 | 876 |
| 2 | AX-12 | 162 | 739 |
| 3 | AX-12 | 190 | 824 |
| 4 | AX-12 | 202 | 775 |
| 5 | AX-12 | 253 | 851 |

실제 검사 함수:

- `Dxl_ValidateGoalPosition()`

status 해석:

- `DXL_STATUS_OK = 0`
- `DXL_STATUS_RANGE = 7`


## 현재 pose 상수

현재 코드에 저장된 pose는 다음과 같다.

### Home pose

`g_arm_home_pose`

- ID0: `1700`
- ID1: `876`
- ID2: `539`
- ID3: `815`
- ID4: `513`
- ID5: `851`

### Stretched pose

`g_arm_stretched_pose`

- ID0: `1700`
- ID1: `350`
- ID2: `169`
- ID3: `501`
- ID4: `775`
- ID5: `253`

중요:

- 위 값은 현재 코드 기준 pose 상수다.
- 실제 기구적으로 원하는 홈/편 상태와 정확히 일치하는지는 반드시 실기에서 다시 확인해야 한다.


## USART1 콘솔 명령

현재 `USART1`은 PC 터미널용 명령 포트로 사용된다.

설정:

- Baud: `115200`
- Format: `8-N-1`

지원 명령:

- `h` 또는 `H`: home pose 요청
- `s` 또는 `S`: stretched pose 요청
- `?`, `m`, `M`: 도움말 출력

출력 배너:

```text
RobotARM ready
h: move home
s: move stretched
?: show help
>
```

현재 구현 특징:

- 엔터 기반 line parser가 아니라, 문자 1개를 즉시 처리하는 방식이다.
- 따라서 터미널에서 키 하나만 눌러도 바로 동작한다.
- 현재는 이 콘솔 처리도 별도 task가 아니라 `StartDefaultTask()` 내부에서 함께 돌고 있다.
- 최근 복구에서 `USART1`은 더 이상 1바이트 polling만 쓰지 않고, **인터럽트 기반 RX 버퍼**를 통해 콘솔 명령과 프레임 명령을 함께 처리하도록 정리되었다.

### USART1 센서 프레임 입력

현재 `USART1`은 콘솔 외에도 아래 형식의 센서 프레임을 직접 받을 수 있다.

```text
P,val0,val1,val2,val3,val4,val5
```

동작 방식:

- `h`, `s`, `?` 같은 단문 명령은 기존처럼 즉시 처리된다.
- `P,...` 형식은 줄 종료(`CR`/`LF`)까지 버퍼링한 뒤 파싱된다.
- 파싱/매핑 규칙은 `USART3`의 블루투스 입력과 동일하다.
- 최종적으로는 `Dxl_MoveArmSafe()`를 호출하므로, 센서값 매핑 후에도 servo별 안전 범위 saturation이 한 번 더 적용된다.

실기에서 확인한 예:

- `P,0,0,512,512,0,512` → `[UART1] Move OK`
- `P,1,2,3` → `[UART1] Invalid parameter format`


## USART3 블루투스 센서 입력 매핑

현재 `USART3`은 블루투스(HC-05) 입력 포트로 사용된다.

입력 포맷:

```text
P,val0,val1,val2,val3,val4,val5
```

현재 구현 해석:

- 이 입력은 더 이상 raw Dynamixel 목표값을 직접 받는 용도가 아니다.
- 각 값은 **센서값(sensor value)** 으로 해석된다.
- `Core/Src/main.c`의 `Bluetooth_ProcessCommand()`가 6개 센서값을 파싱한 뒤,
  각 ID별 보정 테이블을 이용해 **Dynamixel goal position**으로 변환한다.
- 변환된 6개 목표값은 마지막에 `Dxl_MoveArmSafe()`로 전달된다.

중요 구현 포인트:

- signed 숫자 파싱을 지원하므로 `-90` 같은 값도 받을 수 있다.
- 각 축은 **3점(anchor) 기준 조각별 선형 매핑(piecewise linear mapping)** 을 사용한다.
- 센서 입력이 범위를 벗어나면 해당 축의 끝 anchor 값으로 clamp 된다.
- 최종 goal position은 이후 `Dxl_MoveArmSafe()`에서 한 번 더 안전 범위 saturation을 거친다.

현재 ID별 매핑 기준:

| ID | Sensor anchors | DXL anchors |
|---|---|---|
| 0 | `-90`, `0`, `90` | `2644`, `1628`, `623` |
| 1 | `-90`, `0`, `90` | `268`, `562`, `876` |
| 2 | `1023`, `512`, `0` | `739`, `450`, `162` |
| 3 | `1023`, `512`, `0` | `824`, `507`, `190` |
| 4 | `-90`, `0`, `90` | `775`, `450`, `202` |
| 5 | `1023`, `512`, `0` | `851`, `551`, `253` |

예시:

- `P,-90,-90,1023,1023,-90,1023`
  - 각 축은 첫 번째 anchor에 대응하는 DXL 값으로 변환된다.
- `P,0,0,512,512,0,512`
  - 각 축은 중간 anchor에 대응하는 DXL 값으로 변환된다.
- `P,90,90,0,0,90,0`
  - 각 축은 마지막 anchor에 대응하는 DXL 값으로 변환된다.

주의:

- 축마다 센서 증가 방향이 같지 않다.
- 예를 들어 ID0/1/4는 `-90 -> 90`으로 갈수록 DXL 값이 증가하거나 감소할 수 있고,
  ID2/3/5는 `1023 -> 0`처럼 **센서 범위 자체가 내림차순**이다.
- 따라서 단일 공통 공식보다 **축별 테이블 기반 매핑**으로 이해하는 것이 정확하다.


## 실기에서 확인된 내용

### 확인된 것

- `USART2` half-duplex Dynamixel 통신이 실제로 살아났다.
- 현재 위치 polling 값이 실제 모터 응답으로 들어오는 것을 확인했다.
- `USART1` 콘솔에서 `?` 도움말 응답이 실제로 출력되는 것을 확인했다.
- `COM14`가 ST-LINK Virtual COM Port로 잡히는 것을 확인했다.
- `USART1`에서 `h` 명령이 다시 정상 동작하며, 실기에서 `HOME move done (OK)`와 servo별 `T/G/P` debug 출력이 확인됐다.
- `USART1`에서 `P,0,0,512,512,0,512` 센서 프레임을 보내 `[UART1] Move OK`가 실제로 확인됐다.
- `s` 요청 직전/직후 `g_dxl_present_positions`를 SWD로 읽어 확인한 결과, read path 자체는 살아 있지만 write 직후에는 실제 위치가 그대로인 경우가 있었다.
- `torque / goal / present` readback 계측 결과, write pacing이 없을 때는 일부 축에서 `Goal Position` 레지스터가 목표값으로 바뀌지 않는 현상을 확인했다.
- `PA2`를 push-pull로 맞춘 뒤에도 write 불안정성이 완전히 사라지지는 않았고, write pacing + goal readback/retry를 넣은 뒤에야 한 번의 `s`로 6축 goal register가 모두 목표값으로 반영되는 것을 확인했다.
- 이후 한 번 `Core/Src/stm32g4xx_hal_msp.c`의 `USART2/PA2`가 `GPIO_MODE_AF_OD`로 되돌아가 있었고, 그 상태에서는 `h/s`와 `UART1 P,...`가 모두 파싱은 되지만 공통 하위 경로에서 `DXL_STATUS_TIMEOUT`으로 실패하는 것을 확인했다.
- `PA2`를 다시 `GPIO_MODE_AF_PP`로 복구하고 재플래시한 뒤에는 timeout counter가 0으로 유지되고, polling/cache와 pose/frame 이동이 모두 정상으로 돌아왔다.

### 과거 작업 중 확인했던 위치 예시

세션 중 실제 읽혔던 위치 예시 중 하나:

- ID0: `1632`
- ID1: `877`
- ID2: `539`
- ID3: `815`
- ID4: `513`
- ID5: `528`

주의:

- 이 값은 당시 자세에서의 실측 예시다.
- 현재 코드의 home/stretch 상수와 동일하다고 가정하면 안 된다.


## RAM 직접 트리거 방식

콘솔 없이도 SWD 메모리 쓰기로 pose 요청을 줄 수 있다.

현재 빌드 기준 심볼 주소 예시:

- `g_arm_move_last_status`: `0x2000002C`
- `g_arm_move_request`: `0x20000030`

주의:

- 이 주소는 빌드가 바뀌면 달라질 수 있다.
- 항상 최신 ELF 기준으로 `arm-none-eabi-nm`로 다시 확인하는 것이 안전하다.

예시:

```powershell
# home
STM32_Programmer_CLI.exe -c port=SWD mode=HotPlug -w32 0x2000002C 0x00000001 -nv

# stretched
STM32_Programmer_CLI.exe -c port=SWD mode=HotPlug -w32 0x2000002C 0x00000002 -nv
```


## 통신 구현에서 중요한 포인트

### USART2 전기적 설정

- Dynamixel 직결 환경에서 `PA2`는 push-pull (`GPIO_MODE_AF_PP`)로 동작하도록 맞춰져 있다.
- 이 변경 전에는 응답이 timeout만 나고 RX가 전혀 들어오지 않는 문제가 있었다.
- 현재 코드에서도 `Core/Src/stm32g4xx_hal_msp.c`의 `USART2` GPIO 설정은 `GPIO_MODE_AF_PP` 기준으로 맞춰져 있어야 한다.
- 실제로 `GPIO_MODE_AF_OD`로 회귀한 상태에서는 `g_dxl_timeout_count`가 지속 증가하고, `g_dxl_present_positions[].valid`가 모두 0으로 남는 현상이 다시 재현됐다.
- 따라서 CubeMX 재생성이나 수동 병합 이후에는 이 항목을 가장 먼저 다시 확인해야 한다.

### UART 전환 처리

- TX 전환 전 / RX 전환 전 UART error flag 정리
- DR flush
- TX 완료(`UART_FLAG_TC`) 확인 후 RX 전환

이 흐름은 `Dxl_SetDirectionTx()`, `Dxl_SetDirectionRx()`, `Dxl_TransmitInstruction()`에 구현되어 있다.

### write 안정화 처리

현재 코드 기준 write 경로는 아래와 같이 동작한다.

- write instruction은 `expect_status = 0`으로 전송한다.
- 각 write 사이에 `DXL_WRITE_SETTLE_MS`만큼 delay를 둔다.
- 각 servo에 대해 `Torque Enable -> Moving Speed -> Goal Position` 순으로 쓴다.
- `Goal Position` write 뒤에는 `Dxl_ReadGoalPosition()`으로 실제 register 값을 읽어 확인한다.
- 목표값과 다르면 최대 `DXL_WRITE_VERIFY_RETRIES`만큼 다시 시도한다.

중요 해석:

- 이 로직은 현재 **애플리케이션 레벨에서 성공 조건을 강화한 것**이다.
- 즉, 단순히 "보냈다"를 성공으로 치지 않고, **goal register가 실제로 바뀌었는지 확인한 뒤에만 성공**으로 처리한다.
- 다만 이것만으로 lower-level write 불안정성의 최종 원인이 완전히 제거됐다고 단정할 수는 없다.


## 현재 진단 변수

런타임 디버깅용으로 아래 변수가 있다.

- `g_dxl_last_status`
- `g_dxl_last_hal_error`
- `g_dxl_timeout_count`
- `g_dxl_bad_packet_count`
- `g_dxl_rx_ok_count`
- `g_arm_move_last_status`

이 변수들은 통신 실패 원인이 timeout인지, packet 문제인지, range 거부인지 구분할 때 유용하다.


## 현재 주의사항 / caveat

### 1. pose 상수는 반드시 실기 기준으로 다시 검증 필요

현재 `g_arm_home_pose`, `g_arm_stretched_pose` 값은 코드에 들어 있지만, 실제 원하는 자세와 완전히 맞는지 다시 검증해야 한다.

### 2. `g_expected_positions`는 현재 실사용되지 않음

`Core/Src/main.c`의 `g_expected_positions`는 과거 확인용 흔적으로 보이며, 현재 제어 흐름에는 직접 사용되지 않는다.

### 3. 6축 동시 write가 아니라 순차 write

현재는 Sync Write/Bulk Write를 쓰지 않는다.
각 모터를 순차적으로 write 하므로, 완전 동시 도착은 아니다.

추가로 현재 구현은 순차 write + readback/retry 기반이다.
따라서 한 번의 `s`로 6축 goal register는 모두 반영되도록 개선됐지만,
wire-level에서 완전 동시 update가 필요한 요구에는 아직 맞지 않는다.

### 4. stretched/home는 범위를 벗어나면 즉시 거부됨

pose 상수에 한 축이라도 범위 초과값이 들어가면 `Dxl_MoveArmSafe()` 전체가 막힌다.

### 5. 현재는 RTOS 위에서 동작하지만 구조 분리는 아직 안 됨

현재 Dynamixel 제어는 FreeRTOS 아래에서 실행되지만, 구조적으로는 아직 `defaultTask` 내부에 콘솔/폴링/이동 요청 처리가 함께 들어가 있다.

즉, 아래는 아직 안 된 상태다.

- Console task와 Dynamixel task 분리
- queue 기반 pose 요청 전달
- mutex/snapshot 기반 상태 공유

따라서 현재 문맥에서는 `g_arm_move_request` 같은 전역값이 여전히 단순하게 쓰이고 있지만, 다음 단계에서 task 분리를 시작하면 이 부분은 재설계가 필요하다.

### 6. USART2/Dynamixel 버스는 단일 owner 원칙을 유지하는 편이 안전함

현재 `dynamixel.c`는 half-duplex 전환, blocking UART 송수신, 현재 위치 polling, pose 실행이 한 묶음으로 움직인다.

그래서 RTOS 구조를 더 쪼개더라도 아래 원칙을 유지하는 것이 안전하다.

- 여러 task가 동시에 `dynamixel.c` API를 직접 두드리지 않기
- USART2/Dynamixel wire-level 제어는 한 task가 단독 소유하기
- command ingress와 bus execution을 분리하더라도 bus owner는 하나로 유지하기

### 7. 현재 write 안정화는 근본 원인 제거라기보다 신뢰성 보강 성격이 섞여 있음

현재 상태에서 확실히 말할 수 있는 것:

- read path는 실제로 동작한다.
- goal register write는 pacing + readback/retry를 통해 실사용 가능한 수준으로 안정화됐다.

하지만 아직 불확실한 것:

- 왜 일부 write가 첫 시도에 바로 안 먹는지
- 이것이 status return policy 문제인지, turnaround timing 문제인지, electrical margin 문제인지
- `Torque Enable` / `Moving Speed` write도 항상 첫 시도에 안정적으로 들어가는지

따라서 다음 작업자는 현재 구현을 **"동작하는 안정화판"**으로 보되,
하위 레벨 write 신뢰성 자체가 완전히 root-cause fix 됐다고 가정하면 안 된다.


## 다음 개발 추천 순서

1. 현재 write 안정화 로직을 기준선으로 삼고 장시간 반복 테스트 수행
   - `h` / `s`를 여러 번 반복해도 일부 축만 반영되는 현상이 다시 나오지 않는지 확인
   - 필요 시 `g_dxl_timeout_count`, `g_dxl_bad_packet_count`, readback 결과를 같이 기록
2. 현재 RTOS 구조에서 Dynamixel 실행 owner를 어떻게 유지할지 정리
   - `defaultTask` 유지 vs `RobotTask` 명시화
   - 필요 시 Console/Dxl 분리 여부 판단
3. 현재 code pose 기준으로 실기 재검증
   - `h`, `s`를 각각 눌러 실제 원하는 자세와 맞는지 확인
4. lower-level write 불안정성의 원인 분리
   - write status return policy를 다시 쓸지
   - pacing이 어느 수준까지 필요한지
   - servo별 readback 실패 패턴이 있는지
5. 관절 이름 기반 API 추가
   - 예: `base`, `shoulder`, `elbow` 등
6. pose 저장/갱신 기능 추가
   - 현재 위치를 읽어서 새 pose로 저장하는 흐름
7. 보간 이동 추가
   - 한 번에 점프하지 않고 여러 step으로 천천히 이동
8. 필요 시 Sync Write 검토
   - 더 자연스러운 동시 움직임이 필요하면 고려


## 빠른 사용 요약

### PC 터미널로 제어

- 포트: ST-LINK Virtual COM Port
- 속도: `115200`
- 명령:
  - `h`: home
  - `s`: stretched
  - `?`: help

### 코드에서 직접 호출

```c
DxlPresentPosition positions[DXL_SERVO_COUNT];
uint16_t goals[DXL_SERVO_COUNT] = {1700U, 876U, 539U, 815U, 513U, 851U};

(void)Dxl_GetPresentPositions(positions, DXL_SERVO_COUNT);
(void)Dxl_MoveArmSafe(goals);
```
