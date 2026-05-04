# RobotARM STM32 VS Code 개발환경 요약

## 목적

이 문서는 `C:\STproject\RobotARM` 저장소에서 STM32 개발환경을 다시 빠르게 복구할 수 있도록, 실제로 검증된 VS Code 빌드, 플래시, 디버그 절차만 간단히 정리한 문서입니다.

상세판은 `docs/stm32-vscode-workflow.md`를 참고합니다.

## 1. 준비물

Windows 기준으로 아래가 필요합니다.

- `STM32CubeCLT`
- `VS Code`
- VS Code 확장
  - `CMake Tools`
  - `C/C++`
  - `Cortex-Debug`

현재 기준 설치 루트는 아래입니다.

- `C:\ST\STM32CubeCLT_1.21.0`

## 2. 환경변수

반드시 아래 환경변수를 잡아야 합니다.

- 이름: `STM32CubeCLT_ROOT`
- 값: `C:\ST\STM32CubeCLT_1.21.0`

PowerShell 예시:

```powershell
[Environment]::SetEnvironmentVariable(
  'STM32CubeCLT_ROOT',
  'C:\ST\STM32CubeCLT_1.21.0',
  'User'
)
```

설정 후에는 VS Code를 완전히 종료했다가 다시 실행합니다.

## 3. 이 저장소에서 실제로 쓰는 파일

- `CMakePresets.json`
  - `Debug` 프리셋 사용
  - `Ninja` 사용
- `.vscode/tasks.json`
  - `cmake: configure`
  - `cmake: build`
  - `flash: stlink`
  - `build + flash`
  - `debug: prepare`
- `.vscode/launch.json`
  - `RobotARM Debug (ST-LINK GDB Server)` 디버그 설정
- `CMakeLists.txt`
  - 링커 스크립트 `STM32G474VETX_FLASH.ld`
  - startup 파일 `startup_stm32g474xx.s`

## 4. 빌드

### CLI로 확인

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

### VS Code에서 실행

- `Terminal -> Run Task -> cmake: build`

빌드 결과물:

- `build/Debug/RobotARM.elf`
- `build/Debug/RobotARM.hex`
- `build/Debug/RobotARM.bin`

## 5. 플래시

VS Code에서 아래 태스크를 실행합니다.

- `flash: stlink`

이 태스크는 내부적으로 먼저 빌드를 수행한 뒤, `STM32_Programmer_CLI.exe`로 `RobotARM.hex`를 보드에 기록합니다.

추가로 아래 태스크도 있습니다.

- `build + flash`

이 태스크는 결과적으로 아래 순서로 동작합니다.

1. `cmake: configure`
2. `cmake: build`
3. `flash: stlink`

참고로 현재 사용자 VS Code 설정에는 아래 단축키가 들어가 있습니다.

- `Ctrl+Alt+F` -> `build + flash`

이 단축키는 저장소 설정이 아니라 사용자 로컬 설정입니다.

## 6. 디버그

Run and Debug에서 아래 설정을 실행합니다.

- `RobotARM Debug (ST-LINK GDB Server)`

동작 방식:

1. `debug: prepare` 실행
2. `cmake: build` 실행
3. `stlink-gdbserver:start` 실행
4. `ST-LINK_gdbserver.exe`가 `61234` 포트에서 대기
5. `Cortex-Debug`가 `build/Debug/RobotARM.elf`와 `arm-none-eabi-gdb.exe`를 사용해 접속

`launch.json` 기준 주요 값:

- device: `STM32G474VE`
- executable: `${workspaceFolder}/build/Debug/RobotARM.elf`
- gdbTarget: `localhost:61234`

## 7. 자주 막히는 부분

### 환경변수 문제

VS Code에서 아래처럼 보이면:

- `\CMake\bin\cmake.exe --preset Debug`

대부분 `STM32CubeCLT_ROOT`가 비어 있거나 VS Code가 재시작되지 않은 상태입니다.

### CLI는 되는데 VS Code만 안 되는 경우

아래를 확인합니다.

- VS Code를 재시작했는지
- `STM32CubeCLT_ROOT`가 VS Code 프로세스 안에서 보이는지
- `C:\ST\STM32CubeCLT_1.21.0` 아래에 실제 실행 파일들이 있는지

### 디버그가 안 붙는 경우

아래를 확인합니다.

- `build/Debug/RobotARM.elf` 존재 여부
- `ST-LINK_gdbserver.exe` 존재 여부
- `61234` 포트 충돌 여부
- ST-LINK 연결 및 보드 전원 상태

## 8. 유지보수 메모

CubeMX에서 주변장치를 다시 생성한 뒤에는 아래를 같이 봅니다.

- `CMakeLists.txt`에 새 소스 파일이 추가돼야 하는지
- include 경로가 달라지지 않았는지
- 링커 스크립트가 계속 `STM32G474VETX_FLASH.ld`가 맞는지

MCU 이름 관련해서는 현재 설정이 아래 조합으로 되어 있습니다.

- 디버그 device: `STM32G474VE`
- 링커 스크립트: `STM32G474VETX_FLASH.ld`
- startup: `startup_stm32g474xx.s`

지금은 이 조합으로 동작하지만, 보드나 칩이 바뀌면 이 세 군데를 같이 확인해야 합니다.
