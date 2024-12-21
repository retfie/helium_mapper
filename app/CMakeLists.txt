# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(helium_mapper)

# Verify that this sample is built through sysbuild to ensure MCUboot is
# automatically included and that sample specific MCUboot configurations are
# used when building MCUboot.
test_sysbuild()

target_sources(                     app PRIVATE src/main.c)
target_sources(                     app PRIVATE src/config.c)
target_sources(                     app PRIVATE src/leds.c)
target_sources(                     app PRIVATE boards/board.c)
target_sources_ifdef(CONFIG_SENSOR          app PRIVATE src/accelerometer.c)
target_sources_ifdef(CONFIG_LORA            app PRIVATE src/lorawan_app.c)
target_sources_ifdef(CONFIG_BATTERY         app PRIVATE src/battery.c)
target_sources_ifdef(CONFIG_SETTINGS        app PRIVATE src/nvm.c)
target_sources_ifdef(CONFIG_SHELL           app PRIVATE src/shell.c)
target_sources_ifdef(CONFIG_BT              app PRIVATE src/ble.c)
target_sources_ifdef(CONFIG_UBLOX_MAX7Q     app PRIVATE src/gps.c)
target_sources_ifdef(CONFIG_PAYLOAD_ENCRYPTION     app PRIVATE src/encryption.c)

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/../VERSION ver)

string(REGEX MATCH "VERSION_MAJOR = ([0-9]*)" _ ${ver})
set(APP_VERSION_MAJOR ${CMAKE_MATCH_1})

string(REGEX MATCH "VERSION_MINOR = ([0-9]*)" _ ${ver})
set(APP_VERSION_MINOR ${CMAKE_MATCH_1})

string(REGEX MATCH "VERSION_PATCH = ([0-9]*)" _ ${ver})
set(APP_VERSION_PATCH ${CMAKE_MATCH_1})

if(NOT DEFINED BUILD_VERSION)
  find_package(Git QUIET)
  if(GIT_FOUND)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} describe --abbrev=12 --always
      WORKING_DIRECTORY                ${CMAKE_CURRENT_SOURCE_DIR}/../
      OUTPUT_VARIABLE                  BUILD_VERSION
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_STRIP_TRAILING_WHITESPACE
      ERROR_VARIABLE                   stderr
      RESULT_VARIABLE                  return_code
    )
    if(return_code)
      message(STATUS "git describe failed: ${stderr}")
    elseif(NOT "${stderr}" STREQUAL "")
      message(STATUS "git describe warned: ${stderr}")
    endif()
  endif()
endif()

if(DEFINED BUILD_VERSION)
  set(APP_BUILD_VERSION_STR ", build: g${BUILD_VERSION}")
endif()

if (NOT NO_PRINT_VERSION)
  message(STATUS "App version: ${APP_VERSION_MAJOR}.${APP_VERSION_MINOR}.${APP_VERSION_PATCH} ${APP_BUILD_VERSION_STR}")
endif()

configure_file(app_version.h.in app_version.h)
target_include_directories(app PUBLIC "${PROJECT_BINARY_DIR}")
