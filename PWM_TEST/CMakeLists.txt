#
# Copyright (c) 2019 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.20.0)

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/spm.conf")
  set(spm_CONF_FILE
    prj.conf
    ${CMAKE_CURRENT_LIST_DIR}/spm.conf
  )
endif()

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mcuboot.conf")
  set(mcuboot_CONF_FILE
    prj.conf
    ${CMAKE_CURRENT_LIST_DIR}/mcuboot.conf
  )
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(E2)
zephyr_compile_definitions(PROJECT_NAME=${PROJECT_NAME})

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)
# NORDIC SDK APP END

# Include source directory
target_include_directories(app PRIVATE src)
target_include_directories(app PRIVATE $ENV{ZEPHYR_BASE}/subsys/fs/nvs)

# Application directories
add_subdirectory(src/datetime)
add_subdirectory(src/font)
add_subdirectory(src/image)
add_subdirectory(src/lcd)
add_subdirectory(src/key)
add_subdirectory(src/settings)
add_subdirectory(src/inner_flash)
add_subdirectory(src/external_flash)
add_subdirectory(src/uart_ble)
add_subdirectory(src/screen)
add_subdirectory(src/pmu)
add_subdirectory(src/codetrans)
add_subdirectory(src/communicate)
add_subdirectory(src/ucs2)
add_subdirectory(src/logger)

add_subdirectory_ifdef(CONFIG_PPG_SUPPORT src/ppg)
add_subdirectory_ifdef(CONFIG_ECG_SUPPORT src/ecg)
add_subdirectory_ifdef(CONFIG_BUZZER_SUPPORT src/buzzer)
add_subdirectory_ifdef(CONFIG_WATCHDOG src/watchdog)
add_subdirectory_ifdef(CONFIG_FACTORY_TEST_SUPPORT src/factory_test)