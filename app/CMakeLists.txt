# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(helium_mapper)

target_sources(                     app PRIVATE src/main.c)
target_sources(                     app PRIVATE boards/board.c)
target_sources_ifdef(CONFIG_ADC             app PRIVATE src/battery.c)
target_sources_ifdef(CONFIG_SETTINGS        app PRIVATE src/nvm.c)
target_sources_ifdef(CONFIG_SHELL           app PRIVATE src/shell.c)
target_sources_ifdef(CONFIG_BT              app PRIVATE src/ble.c)
target_sources_ifdef(CONFIG_UBLOX_MAX7Q     app PRIVATE src/gps.c)
