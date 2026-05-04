# RobotARM VSCode setup

This project keeps `CubeMX` as the hardware configuration source and uses `CMake` plus `Cortex-Debug` for the daily VSCode workflow.

For the full reproducible setup and troubleshooting guide, see `docs/stm32-vscode-workflow.md`.

## 1. Install tools

Install these components first:

- `STM32CubeCLT`
- `VSCode`
- VSCode extensions: `CMake Tools`, `C/C++`, `Cortex-Debug`

## 2. Set environment variable

Set `STM32CubeCLT_ROOT` to your STM32CubeCLT install root.

Typical example on this machine:

```powershell
$env:STM32CubeCLT_ROOT = 'C:\ST\STM32CubeCLT_1.21.0'
```

To persist it on Windows:

```powershell
[Environment]::SetEnvironmentVariable(
  'STM32CubeCLT_ROOT',
  'C:\ST\STM32CubeCLT_1.21.0',
  'User'
)
```

## 3. Build in VSCode

- `Terminal -> Run Task -> cmake: build`
- Output files land in `build/Debug/`

Generated firmware files:

- `build/Debug/RobotARM.elf`
- `build/Debug/RobotARM.hex`
- `build/Debug/RobotARM.bin`

## 4. Flash

- `Terminal -> Run Task -> flash: stlink`
- The current workspace is already configured to use `C:\ST\STM32CubeCLT_1.21.0`

## 5. Debug

- Open `Run and Debug`
- Start `RobotARM Debug (ST-LINK GDB Server)`

## Notes

- This project was originally generated with the `EWARM` target, so the VSCode build uses the GCC startup template from `Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g474xx.s` and a new GNU linker script at `STM32G474VETX_FLASH.ld`.
- When you add new peripherals in `CubeMX`, update `CMakeLists.txt` if CubeMX adds new generated source files that must be compiled.
- A verified local build produced `build/Debug/RobotARM.elf`, `build/Debug/RobotARM.hex`, and `build/Debug/RobotARM.bin` with the installed `STM32CubeCLT_1.21.0` toolchain.
