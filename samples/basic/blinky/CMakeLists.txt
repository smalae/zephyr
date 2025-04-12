# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(blinky)

target_sources(app PRIVATE src/main.c)
# Add include directory for a specific target
target_include_directories(app PRIVATE src)
zephyr_code_relocate(FILES src/main.c LOCATION RAM)

set(WISECONNECT_DIR ${ZEPHYR_HAL_SILABS_MODULE_DIR}/wiseconnect)
set(SISDK_DIR ${ZEPHYR_HAL_SILABS_MODULE_DIR}/simplicity_sdk)

file(GLOB timer_driver_files ${ZEPHYR_BASE}/drivers/timer/*.c)

zephyr_code_relocate(FILES ${timer_driver_files} LOCATION RAM)
zephyr_code_relocate(FILES ${SISDK_DIR}/platform/service/sleeptimer/src/sl_sleeptimer.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/mcu/drivers/service/sleeptimer/src/sl_sleeptimer_hal_si91x_sysrtc.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/src/rsi_sysrtc.c LOCATION RAM)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/arch/arm/core/cortex_m/isr_wrapper.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/mcu/core/chip/src/rsi_deepsleep_soc.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/mcu/drivers/service/power_manager/src/sl_si91x_power_manager.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/mcu/drivers/service/power_manager/src/sli_si91x_power_manager.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/wireless/ahb_interface/src/rsi_hal_mcu_m4_ram.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/wireless/ahb_interface/src/rsi_hal_mcu_m4_rom.c LOCATION RAM)
#zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/wireless/src/sl_si91x_driver.c LOCATION RAM)
#zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/wireless/ahb_interface/src/sl_si91x_bus.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/mcu/drivers/cmsis_driver/UDMA.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/src/rsi_sysrtc.c LOCATION RAM)
#zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/wireless/src/sl_rsi_utility.c LOCATION RAM)
#zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/wireless/threading/sli_si91x_multithreaded.c LOCATION RAM)
