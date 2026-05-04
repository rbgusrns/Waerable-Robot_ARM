# RobotARM 프로젝트 진행상황 인수인계

## 목적

이 문서는 다음 프롬프트나 다음 작업자가 `RobotARM` 저장소의 현재 상태를 빠르게 파악하고 바로 이어서 작업할 수 있도록, 현재 구현 범위와 미완성 영역을 짧게 정리한 handoff 문서입니다.

핵심 목표는 아래 네 가지입니다.

- 이 프로젝트가 지금 어느 단계인지 빠르게 이해하기
- 이미 구현된 기능과 아직 남은 작업을 분리해서 보기
- 어디부터 읽어야 하는지 바로 알기
- 다음 세션에서 중복 탐색 없이 바로 작업을 이어가기

## 한 줄 상태 요약

현재 저장소는 **STM32G474 기반 RobotARM 펌웨어 프로젝트**이며, **Dynamixel 6축 기본 제어 + UART1 콘솔/센서 프레임 기반 제어 + FreeRTOS/TIM6 timebase 전환 + Dynamixel write 안정화(readback/retry) + USART2 PA2 push-pull 복구까지 반영된 상태**입니다.

즉, 초기 bring-up 단계는 지났고, 지금은 **기본 제어와 RTOS 기반 빌드 연결은 완료됐으며, write 불안정성에 대한 실사용 가능한 안정화까지 들어갔지만, RTOS 구조 정리와 하위 레벨 write root-cause 규명, 상위 기능 확장이 남아 있는 상태**로 보는 것이 가장 정확합니다.

## 저장소 성격

- MCU: `STM32G474VET6`
- 프로젝트 형태: STM32CubeMX 기반 임베디드 펌웨어
- 주요 제어 대상: Dynamixel 서보 6축
- 개발 워크플로우: `CubeMX + FreeRTOS + CMake + VS Code + ST-LINK`

루트 기준 핵심 디렉터리/파일:

- `Core/`
  - 애플리케이션 코드
- `Drivers/`
  - STM32 HAL, CMSIS 등 벤더 코드
- `docs/`
  - handoff 및 개발환경 문서
- `CMakeLists.txt`
  - 빌드 대상/소스/링크 설정
- `CMakePresets.json`
  - `Debug` 빌드 프리셋
- `RobotARM.ioc`
  - CubeMX 하드웨어 설정 원본
- `build/Debug/`
  - 검증된 빌드 산출물 위치

## 가장 먼저 봐야 할 파일

### 1. `Core/Src/main.c`

애플리케이션 진입점입니다.

현재 이 파일에서 확인되는 역할:

- `USART1`, `USART2`, `USART3` 초기화
- FreeRTOS default task 생성
- 스케줄러 시작
- 기존 콘솔/Dynamixel 제어 루프를 `StartDefaultTask()` 안에서 반복 실행

### 2. `Core/Src/dynamixel.c`

현재 프로젝트의 핵심 제어 로직이 들어 있습니다.

현재 구현된 내용:

- Dynamixel Protocol 1.0 패킷 송수신
- half-duplex UART TX/RX 전환
- 현재 위치 polling
- 단일 모터 안전 이동
- 6축 전체 안전 이동
- home/stretch pose 요청 처리
- goal register readback + retry 기반 write 안정화

### 3. `Core/Inc/dynamixel.h`

외부에서 호출 가능한 API와 전역 상태가 선언되어 있습니다.

중요 API:

- `Dxl_Init()`
- `Dxl_Process()`
- `Dxl_ReadPresentPosition()`
- `Dxl_GetPresentPosition()`
- `Dxl_GetPresentPositions()`
- `Dxl_SetGoalPositionSafe()`
- `Dxl_MoveArmSafe()`
- `Dxl_ProcessArmMoveRequest()`

### 4. `docs/dynamixel-hand-off.ko.md`

Dynamixel 제어 쪽만 따로 상세 정리한 문서입니다.

이 문서에는 아래가 들어 있습니다.

- 하드웨어/통신 전제
- 현재 지원 기능
- 안전 범위
- pose 상수
- 콘솔 명령
- 실기 확인 사실
- 다음 개발 추천 순서

### 5. `docs/stm32-vscode-workflow.md`

빌드/플래시/디버그 재현용 문서입니다.

다음 세션에서 환경 복구나 검증이 필요하면 이 문서를 기준으로 보면 됩니다.

### 6. `Core/Src/app_freertos.c`

FreeRTOS 애플리케이션 파일입니다.

현재는 CubeMX가 생성한 최소 골격 수준이며, 실질적인 제어 루프는 아직 `main.c`의 `StartDefaultTask()` 안에 남아 있습니다.

### 7. `Core/Src/stm32g4xx_hal_timebase_tim.c`

HAL tick source를 `TIM6`로 옮긴 생성 파일입니다.

RTOS가 SysTick을 쓰는 동안 HAL 쪽 `HAL_GetTick()`은 TIM6 기반으로 유지되도록 하는 핵심 파일입니다.

### 8. `Core/Inc/FreeRTOSConfig.h`

FreeRTOS 커널 설정 파일입니다.

현재 프로젝트는 이 파일 기준으로 CMSIS-RTOS V1 wrapper와 Cortex-M4F GCC port를 통해 빌드됩니다.

## 현재까지 구현된 기능

### 1. 콘솔 명령 기반 pose 호출

`USART1`이 PC 터미널용 콘솔 포트로 쓰입니다.

지원 명령:

- `h` / `H`: home pose 요청
- `s` / `S`: stretched pose 요청
- `?`, `m`, `M`: 도움말 출력

배너 출력:

```text
RobotARM ready
h: move home
s: move stretched
?: show help
>
```

특징:

- line parser가 아니라 문자 1개를 즉시 처리합니다.
- 엔터 없이도 키 입력 하나로 바로 동작합니다.

### 2. Dynamixel 현재 위치 polling

`Dxl_Process()`가 주기적으로 각 서보의 현재 위치를 읽어 캐시에 저장합니다.

관련 상태:

- `g_dxl_present_positions[DXL_SERVO_COUNT]`
- `g_dxl_poll_cycle`
- `g_dxl_last_status`
- `g_dxl_last_hal_error`
- `g_dxl_timeout_count`
- `g_dxl_bad_packet_count`
- `g_dxl_rx_ok_count`

주의:

- 완전 동시 스냅샷이 아니라 순차 polling 기반입니다.
- 축마다 약간의 시간차가 있을 수 있습니다.

### 3. 단일 모터 안전 이동

`Dxl_SetGoalPositionSafe()`가 아래 순서로 동작합니다.

1. ID 유효성 확인
2. 안전 범위 확인
3. Torque Enable write
4. Moving Speed write
5. Goal Position write

즉, 무조건 raw write 하는 형태가 아니라 최소한의 안전 확인이 들어가 있습니다.

### 4. 6축 전체 안전 이동

`Dxl_MoveArmSafe()`는 6개 축 목표값을 모두 먼저 검사한 뒤, 전부 통과한 경우에만 순차적으로 전송합니다.

의미:

- 한 축이라도 범위를 벗어나면 전체 이동이 거부됩니다.
- 부분 적용 없이 all-or-nothing 방식입니다.

현재 추가된 안정화 포인트:

- 각 servo는 `Torque Enable -> Moving Speed -> Goal Position` 순으로 처리됩니다.
- `Goal Position` write 뒤에는 register readback을 수행합니다.
- readback 결과가 목표값과 다르면 servo별로 bounded retry를 수행합니다.

중요 해석:

- 이 변경으로 **한 번의 `s` 명령에서 일부 축만 움직이고 나머지는 다음 `s`에서 따라오는 현상**을 크게 줄였습니다.
- 다만 이것은 lower-level write reliability를 완전히 증명한 것이 아니라, **application-level verification을 강화한 안정화 로직**입니다.

### 5. home/stretch pose 요청 처리

현재 지원되는 요청 상수:

- `ARM_MOVE_REQUEST_NONE`
- `ARM_MOVE_REQUEST_HOME`
- `ARM_MOVE_REQUEST_STRETCHED`

현재 저장된 pose 상수:

- `g_arm_home_pose`
- `g_arm_stretched_pose`

메인 루프에서 `Dxl_ProcessArmMoveRequest()`가 요청을 한 번 처리하고, 결과를 `g_arm_move_last_status`에 남긴 뒤 요청 값을 다시 비웁니다.

### 6. FreeRTOS + TIM6 timebase 전환

현재 저장소에는 FreeRTOS와 TIM6 timebase 전환이 반영되어 있습니다.

현재 확인되는 상태:

- `RobotARM.ioc`에 FreeRTOS middleware 설정 존재
- HAL timebase가 `SysTick`에서 `TIM6`로 전환됨
- `Core/Src/stm32g4xx_hal_timebase_tim.c` 생성됨
- `Core/Inc/FreeRTOSConfig.h` 생성됨
- `Middlewares/Third_Party/FreeRTOS/...` 생성됨
- `CMakeLists.txt`가 FreeRTOS kernel/CMSIS_RTOS/GCC port/TIM HAL 소스를 포함하도록 갱신됨

### 7. 기존 제어 루프의 RTOS 이관

CubeMX가 기본 `defaultTask` 구조를 생성한 뒤, 기존 bare-metal 제어 루프는 실행 위치를 옮겨야 했습니다.

현재는 아래 제어 루프가 `StartDefaultTask()` 안으로 이관되어 있습니다.

- `Console_ProcessCommand()`
- `Dxl_Process(HAL_GetTick())`
- `Dxl_ProcessArmMoveRequest()`

의미:

- RTOS가 시작된 뒤에도 기존 기능이 계속 동작합니다.
- 아직 task 분리는 하지 않았고, **기존 제어 루프를 하나의 RTOS task 안에 보존한 상태**입니다.

### 8. 최근 Dynamixel 디버깅으로 확인된 것

- read path는 현재 실제로 동작합니다.
- 과거에는 write path가 timeout 또는 부분 적용 형태로 불안정했습니다.
- `PA2` push-pull 설정은 read path 복구에 중요한 조건이었습니다.
- 현재는 write pacing + goal readback/retry를 통해 한 번의 pose 요청에서 6축 goal register가 모두 반영되도록 안정화한 상태입니다.
- 그러나 왜 일부 write가 첫 시도에 바로 안 먹는지에 대한 lower-level root cause는 아직 완전히 분리되지 않았습니다.

추가로 최근 실기 복구에서 다시 확인된 사실:

- `Core/Src/stm32g4xx_hal_msp.c`의 `USART2` `PA2`가 `GPIO_MODE_AF_OD`로 돌아가 있으면 Dynamixel 버스 전체가 다시 timeout 상태로 무너질 수 있습니다.
- 실제로 이 상태에서는 `h/s`와 `UART1 P,...` 프레임이 모두 파싱까지는 되지만, 공통 하위 경로인 `Dxl_MoveArmSafe()`에서 `DXL_STATUS_TIMEOUT`이 발생했습니다.
- `PA2`를 다시 `GPIO_MODE_AF_PP`로 복구한 뒤에는 현재 위치 polling, `h`/`s` pose 이동, `UART1` 프레임 이동이 모두 실기에서 정상 복구됐습니다.

### 9. 블루투스(UART3) 기반 6축 원격 제어

`USART3`을 통해 외부 블루투스 모듈과 연결되며, `P,값0,값1,값2,값3,값4,값5\n` 형태의 문자열 패킷을 수신해 6축 모터를 동시 제어합니다.

- **데이터 수신 안정성**: 115200bps 통신 시 데이터 유실을 방지하기 위해 UART3 인터럽트 방식과 링 버퍼(Ring Buffer)가 적용되었습니다.
- **문자열 파싱**: `main.c` 내 `Bluetooth_ProcessCommand()` 함수가 데이터를 `,` 기준으로 파싱합니다. 현재는 signed 숫자도 허용하므로 `-90` 같은 값도 받을 수 있습니다.
- **센서값 매핑**: 수신된 6개 값은 raw Dynamixel 위치값이 아니라 **센서값**으로 해석되며, 각 축의 3점 보정(anchor) 테이블에 따라 **Dynamixel goal position**으로 변환됩니다.
- **축별 매핑 방향 차이**: ID0/1/4는 `-90 -> 0 -> 90`, ID2/3/5는 `1023 -> 512 -> 0` 기준을 사용합니다. 따라서 일부 축은 센서 증가와 DXL 증가 방향이 같지 않습니다.
- **범위 처리**: 센서값이 정의된 범위를 벗어나면 해당 축의 끝 anchor 값으로 clamp 된 뒤 사용됩니다.
- **안전 이동 검증**: 센서값에서 변환된 goal position 배열은 `Dxl_MoveArmSafe()`로 전달되어, 동작 전 각 축의 하드웨어 물리적 한계(`min_position` ~ `max_position`) 내에서 한 번 더 saturation 됩니다.

현재 반영된 anchor 기준은 아래와 같습니다.

| ID | Sensor anchors | DXL anchors |
|---|---|---|
| 0 | `-90`, `0`, `90` | `2644`, `1628`, `623` |
| 1 | `-90`, `0`, `90` | `268`, `562`, `876` |
| 2 | `1023`, `512`, `0` | `739`, `450`, `162` |
| 3 | `1023`, `512`, `0` | `824`, `507`, `190` |
| 4 | `-90`, `0`, `90` | `775`, `450`, `202` |
| 5 | `1023`, `512`, `0` | `851`, `551`, `253` |

### 10. UART1 기반 센서 프레임 제어

현재 `USART1`은 기존 콘솔 포트 역할을 유지하면서, 동일한 포트에서 `P,값0,값1,값2,값3,값4,값5` 형식의 센서 프레임도 받을 수 있습니다.

- 단문 명령은 그대로 즉시 처리됩니다.
  - `h` / `H`: home
  - `s` / `S`: stretched
  - `?`, `m`, `M`: help
- 프레임 명령은 줄 종료(`CR`/`LF`) 기준으로 수신되며, `UART3`와 동일한 센서값 매핑 경로를 재사용합니다.
- 즉, `UART1`의 `P,...` 입력도 최종적으로는 anchor 기반 변환 후 `Dxl_MoveArmSafe()`로 들어갑니다.

실기에서 확인된 예시:

- `P,0,0,512,512,0,512` → `[UART1] Move OK`
- 잘못된 형식 예: `P,1,2,3` → `[UART1] Invalid parameter format`

## 현재 하드웨어/통신 구성

- Dynamixel bus: TTL, half-duplex UART
- Dynamixel UART: `USART2` on `PA2`
- PC/터미널 UART: `USART1`
- 블루투스 제어용 UART: `USART3` (RX 인터럽트 활성화됨)
- Baud rate: `115200`
- Servo count: `6`
- Servo IDs:
  - ID0: MX-64
  - ID1~ID5: AX-12

## 현재 확인된 진행 수준

현재 저장소는 단순 생성 직후 상태가 아닙니다.

이미 확인되는 성숙도 신호:

- `CMakeLists.txt`와 `CMakePresets.json`이 정리돼 있음
- `build/Debug/` 기준 빌드 산출물 경로가 문서화돼 있음
- VS Code task / launch 기반 build, flash, debug 흐름이 정리돼 있음
- `docs/dynamixel-hand-off.ko.md`에 실기 확인 내용이 남아 있음
- `README.vscode.md`와 `docs/stm32-vscode-workflow*.md`가 있어 환경 복구 경로가 명확함
- `RobotARM.ioc`에 FreeRTOS + TIM6 timebase 설정이 반영됨
- GCC용 FreeRTOS port와 kernel source까지 CMake/GCC 빌드에 연결됨
- `cmake --preset Debug` / `cmake --build --preset Debug`가 RTOS 반영 후에도 통과함

즉, **빌드 체계와 기본 동작 검증뿐 아니라 RTOS 기반 빌드 연결까지는 이미 한 번 정리된 프로젝트**입니다.

## 현재 RTOS 구조 상태

현재 구조는 **완전한 RTOS 설계 완료 상태는 아니고, 1차 이행 단계**입니다.

현재 상태 요약:

- FreeRTOS kernel은 들어옴
- HAL tick은 TIM6 기반으로 전환됨
- 기존 애플리케이션 루프는 `defaultTask` 안으로 옮겨져 동작 유지됨
- Console / Dynamixel / move request는 아직 분리 task가 아님
- queue / mutex 기반 구조는 아직 도입되지 않음

즉, 현재는 **"bare-metal while loop를 RTOS task 하나 안으로 감싼 상태"**에 가깝습니다.

## 아직 구현 안 된 부분

현재 남아 있는 것은 저수준 통신 자체보다, 그 위의 상위 기능들입니다.

### 1. pose 값 실기 재검증

`g_arm_home_pose`, `g_arm_stretched_pose` 값은 코드에 들어 있지만, 실제 기구적으로 원하는 자세와 정확히 일치하는지는 다시 확인해야 합니다.

즉, 값은 존재하지만 **최종 캘리브레이션 완료 상태라고 보기는 어렵습니다**.

### 2. pose 저장/갱신 기능

현재는 pose가 코드 상수로 하드코딩되어 있습니다.

아직 없는 기능:

- 현재 위치를 읽어서 새 pose로 저장
- 기존 pose를 런타임에서 갱신
- 여러 pose를 구조적으로 관리

### 3. 관절 이름 기반 상위 API

현재 제어는 서보 ID와 raw position 중심입니다.

아직 없는 방향:

- `base`, `shoulder`, `elbow` 같은 의미 있는 관절 이름 API
- 관절 단위 추상화
- 사람이 읽기 쉬운 상위 제어 인터페이스

### 4. 보간 이동

현재는 목표 자세를 바로 write 하는 방식입니다.

아직 없는 기능:

- step 기반 점진 이동
- 속도/시간 기반 보간
- 더 자연스러운 자세 전환

### 5. Sync Write/Bulk Write 기반 동시성 개선

현재는 각 축을 순차 write 합니다.

따라서 완전한 동시 도착은 아닙니다.

더 자연스러운 다축 동작이 필요하면 이후에 아래를 검토해야 합니다.

- Sync Write
- Bulk Write
- 명령 batching 전략

### 6. RTOS 구조 정리

현재는 `defaultTask` 안에 기존 제어 루프가 들어가 있을 뿐, RTOS 구조가 의미 있게 정리되지는 않았습니다.

아직 남아 있는 RTOS 쪽 작업:

- `defaultTask`를 목적이 드러나는 `RobotTask` 수준으로 정리
- Console / Dynamixel 역할 분리 여부 결정
- 필요 시 queue 기반 명령 전달 구조 도입
- 필요 시 mutex/snapshot 기반 상태 공유 구조 도입

중요:

- 지금 단계에서 멀티태스크를 무리하게 늘리는 것보다, **USART2/Dynamixel 버스 단일 owner 원칙을 유지하면서 천천히 분리**하는 것이 안전합니다.

## 현재 의심되거나 정리 필요한 부분

### 1. `g_expected_positions`

`Core/Src/main.c`에 정의돼 있지만 현재 제어 흐름에서는 직접 사용되지 않습니다.

성격상 과거 확인용 흔적에 가깝습니다.

### 2. `USART3`

(업데이트됨) 이전에는 사용 흔적이 없어 의심되는 영역이었으나, 현재는 외부 센서값을 수신하기 위한 **블루투스 전용 포트**로 활성화되어 사용 중입니다.

다음 작업 시 참고할 부분입니다.

## 프로젝트 자체와 무관한 TODO 주의

검색 시 `Drivers/` 아래 CMSIS/DSP 테스트 하네스 쪽 TODO와 `#if 0` 블록이 보일 수 있습니다.

하지만 이들은 대체로 **외부 벤더 코드의 미완성 흔적**이며, 현재 RobotARM 애플리케이션 기능 미구현과 직접 연결되지는 않습니다.

따라서 다음 세션에서 진행상황을 판단할 때는 아래를 분리해서 봐야 합니다.

- 프로젝트 앱 코드의 미완성
- 외부 드라이버/테스트 코드의 TODO

## 다음 작업 추천 순서

우선순위는 아래 순서가 가장 자연스럽습니다.

1. RTOS 기본 구조 정리 (`defaultTask` 역할 명확화)
2. Console / Dynamixel 분리 여부 설계
3. home/stretch pose 실기 재검증
4. 미사용 흔적(`g_expected_positions`)의 역할 확인 및 제거
5. pose 저장/갱신 기능 추가
6. 관절 이름 기반 상위 API 추가
7. 보간 이동 추가
8. 필요 시 Sync Write 검토

## 다음 세션에서 바로 물어보면 좋은 질문

다음 프롬프트에서는 아래처럼 물어보면 바로 이어서 작업하기 좋습니다.

- `현재 defaultTask 기반 RTOS 구조를 RobotTask 중심으로 어떻게 정리할지 설계해줘`
- `ConsoleTask와 DxlTask로 나누는 게 지금 구조에서 안전한지 판단해줘`
- `home/stretch pose를 실기 기준으로 다시 검증해야 하는 부분부터 정리해줘`
- `g_expected_positions 변수가 실제로 필요한지 코드 기준으로 판단해줘`
- `현재 Dynamixel 제어 위에 pose 저장 기능을 추가하려면 어디를 바꾸면 되는지 설계해줘`
- `보간 이동을 현재 구조에 최소 수정으로 넣는 방법을 제안해줘`

## 빠른 시작 메모

다음 세션에서 가장 먼저 할 일:

1. 이 문서를 읽기
2. `docs/dynamixel-hand-off.ko.md` 읽기
3. `Core/Src/main.c`, `Core/Src/dynamixel.c`, `Core/Src/app_freertos.c` 읽기
4. `Core/Src/stm32g4xx_hal_timebase_tim.c`, `Core/Inc/FreeRTOSConfig.h` 확인
5. 필요하면 `docs/stm32-vscode-workflow.md`로 빌드/플래시/디버그 절차 확인

이 순서면 현재 프로젝트 상태를 가장 빠르게 재구성할 수 있습니다.
