set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(_cubeclt_root "$ENV{STM32CubeCLT_ROOT}")

set(_gcc_hints)
if(_cubeclt_root)
  list(APPEND _gcc_hints
    "${_cubeclt_root}/GNU-tools-for-STM32/bin"
    "${_cubeclt_root}/GNU-tools-for-STM32/bin/bin"
  )
endif()

find_program(CMAKE_C_COMPILER arm-none-eabi-gcc HINTS ${_gcc_hints} REQUIRED)
find_program(CMAKE_ASM_COMPILER arm-none-eabi-gcc HINTS ${_gcc_hints} REQUIRED)
find_program(CMAKE_CXX_COMPILER arm-none-eabi-g++ HINTS ${_gcc_hints})
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy HINTS ${_gcc_hints} REQUIRED)
find_program(CMAKE_SIZE arm-none-eabi-size HINTS ${_gcc_hints} REQUIRED)
find_program(CMAKE_GDB arm-none-eabi-gdb HINTS ${_gcc_hints} REQUIRED)
