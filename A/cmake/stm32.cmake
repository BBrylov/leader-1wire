set(FPU_FLAGS "-mfloat-abi=soft")
set(CPU_FLAGS "-mcpu=cortex-m3")

set(HAL_DRIVERS_DIR "STM32F1xx")
set(LINKER_SCRIPT_NAME "STM32F103C8Tx_FLASH")

set(OPENOCD_SCRIPT_NAME "STM32F103xx")
set(OPENOCD_CPU_FREQ "64000000")

add_definitions(-DSTM32F103xB -DUSE_HAL_DRIVER)

message(STATUS "Used MCU: STM32F103C8T6")
