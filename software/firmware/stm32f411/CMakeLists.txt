# Mico MCU firmware (STM32F411CEU6, FreeRTOS). Two images:
#   mico_mcu        release
#   mico_mcu_m0test release + TestHook (overrun / hang injection) for the M0 acceptance run
# Build (PowerShell or bash):
#   cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release
#   cmake --build build
cmake_minimum_required(VERSION 3.22)
project(mico_mcu C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_C_FLAGS_RELEASE "-O2 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g")

set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
set(CORE ${CMAKE_CURRENT_SOURCE_DIR}/../../core)
set(HAL ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver)
set(RTOS ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/Source)

# Vendored code: built without our warning flags.
file(GLOB HAL_SRC ${HAL}/Src/*.c)
add_library(vendor STATIC
  ${HAL_SRC}
  ${RTOS}/tasks.c ${RTOS}/list.c ${RTOS}/queue.c
  ${RTOS}/portable/GCC/ARM_CM4F/port.c
  ${RTOS}/portable/MemMang/heap_4.c
  Core/Src/system_stm32f4xx.c)
target_compile_definitions(vendor PUBLIC STM32F411xE USE_HAL_DRIVER)
target_include_directories(vendor PUBLIC
  Core/Inc
  ${HAL}/Inc
  Drivers/CMSIS/Device/ST/STM32F4xx/Include
  Drivers/CMSIS/Include
  ${RTOS}/include
  ${RTOS}/portable/GCC/ARM_CM4F)

file(GLOB CORE_SRC ${CORE}/controller/*.cpp ${CORE}/trajectory/*.cpp ${CORE}/joint_manager/*.cpp
     ${CORE}/link/*.cpp ${CORE}/safety/*.cpp ${CORE}/timing/*.cpp)
set(APP_SRC
  Core/Startup/startup_stm32f411xe.s
  Core/Src/main.c
  Core/Src/stm32f4xx_hal_msp.c
  Core/Src/stm32f4xx_hal_timebase_tim.c
  Core/Src/stm32f4xx_it.c
  Core/Src/syscalls.c
  app/tasks.cpp
  app/diagnostics.cpp
  app/app_start.cpp
  hardware/board_stm32.cpp
  ${CORE_SRC})
set(APP_INC app hardware ${CORE} ${CORE}/controller ${CORE}/trajectory ${CORE}/joint_manager
    ${CORE}/link ${CORE}/safety ${CORE}/timing ${CORE}/generated)

option(MICO_BENCH "P0 bench: no arms PCA9685, no IMU" OFF)
set(BOARD_DEFS)
if(MICO_BENCH)
  set(BOARD_DEFS MICO_HAS_IMU=0 MICO_HAS_ARMS_PCA=0)
endif()

function(mico_image name hooks)
  add_executable(${name} ${APP_SRC})
  target_include_directories(${name} PRIVATE ${APP_INC})
  target_compile_definitions(${name} PRIVATE MICO_TEST_HOOKS=${hooks} ${BOARD_DEFS})
  target_compile_options(${name} PRIVATE $<$<COMPILE_LANGUAGE:C,CXX>:-Wall -Wextra -Werror -Wno-missing-field-initializers>)
  target_link_libraries(${name} PRIVATE vendor)
  target_link_options(${name} PRIVATE -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F411CEUx_FLASH.ld
                      -Wl,-Map=${name}.map -Wl,--undefined=uxTopUsedPriority)
  add_custom_command(TARGET ${name} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${name}> ${name}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${name}> ${name}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${name}>)
endfunction()

mico_image(mico_mcu 0)
mico_image(mico_mcu_m0test 1)
