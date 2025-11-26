#
# Copyright (c) 2019 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(N2)
zephyr_compile_definitions(PROJECT_NAME=${PROJECT_NAME})

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)
# NORDIC SDK APP END

# Include source directory
target_include_directories(app PRIVATE src)
target_include_directories(app PRIVATE $ENV{ZEPHYR_BASE}/subsys/fs/nvs)

# Application directories
add_subdirectory(src/datetime)
add_subdirectory(src/settings)
add_subdirectory(src/inner_flash)
add_subdirectory(src/uart)
add_subdirectory(src/codetrans)
add_subdirectory(src/ucs2)
add_subdirectory(src/communicate)
add_subdirectory(src/logger)

add_subdirectory_ifdef(CONFIG_ALARM_SUPPORT src/alarm)
add_subdirectory_ifdef(CONFIG_PPG_SUPPORT src/ppg)
add_subdirectory_ifdef(CONFIG_ECG_SUPPORT src/ecg)
add_subdirectory_ifdef(CONFIG_BLE_SUPPORT src/ble)
add_subdirectory_ifdef(CONFIG_AUDIO_SUPPORT src/audio)
add_subdirectory_ifdef(CONFIG_TEMP_SUPPORT src/temperature)
add_subdirectory_ifdef(CONFIG_CRC_SUPPORT src/crc_check)
add_subdirectory_ifdef(CONFIG_WIFI_SUPPORT src/wifi)
add_subdirectory_ifdef(CONFIG_WATCHDOG src/watchdog)
add_subdirectory_ifdef(CONFIG_FACTORY_TEST_SUPPORT src/factory_test)