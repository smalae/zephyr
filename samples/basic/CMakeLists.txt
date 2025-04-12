# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(flash_vs_interrupts_test)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/state_machine.c)
# Add include directory for a specific target
target_include_directories(app PRIVATE src)
zephyr_code_relocate(FILES src/main.c LOCATION RAM)
zephyr_code_relocate(FILES src/state_machine.c LOCATION RAM)

set(WISECONNECT_DIR ${ZEPHYR_HAL_SILABS_MODULE_DIR}/wiseconnect)

file(GLOB uart_driver_files ${ZEPHYR_BASE}/drivers/serial/*.c)
file(GLOB wdt_driver_files ${ZEPHYR_BASE}/drivers/watchdog/*.c)
file(GLOB gpio_driver_files ${ZEPHYR_BASE}/drivers/gpio/*.c)

zephyr_code_relocate(FILES ${uart_driver_files} LOCATION RAM)
zephyr_code_relocate(FILES ${wdt_driver_files} LOCATION RAM)
zephyr_code_relocate(FILES ${gpio_driver_files} LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/wireless/ahb_interface/src/rsi_hal_mcu_m4_ram.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/wireless/ahb_interface/src/rsi_hal_mcu_m4_rom.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/mcu/drivers/systemlevel/src/rsi_wwdt.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/mcu/drivers/unified_api/src/sl_si91x_driver_gpio.c LOCATION RAM)
zephyr_code_relocate(FILES ${WISECONNECT_DIR}/components/device/silabs/si91x/mcu/drivers/unified_peripheral_drivers/src/sl_si91x_peripheral_gpio.c LOCATION RAM)
