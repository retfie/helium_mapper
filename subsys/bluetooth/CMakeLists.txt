# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_BT_CONN services)

zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth)
