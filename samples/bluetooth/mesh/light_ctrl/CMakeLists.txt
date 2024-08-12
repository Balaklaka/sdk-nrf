#
# Copyright (c) 2020 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

include_directories(
	${ZEPHYR_NRF_MODULE_DIR}/samples/bluetooth/mesh/common
	${ZEPHYR_NRF_MODULE_DIR}/samples/bluetooth/mesh/dfu/common/src
)

# NORDIC SDK APP START
target_sources(app PRIVATE
	src/main.c
	src/model_handler.c
	src/lc_pwm_led.c)
target_include_directories(app PRIVATE include)

target_sources_ifdef(CONFIG_NCS_SAMPLE_MCUMGR_BT_OTA_DFU app PRIVATE ${app_sources}
	${ZEPHYR_NRF_MODULE_DIR}/samples/bluetooth/mesh/common/smp_bt.c
)

target_sources_ifdef(CONFIG_BT_MESH_DFU_SRV app PRIVATE ${app_sources}
	${ZEPHYR_NRF_MODULE_DIR}/samples/bluetooth/mesh/dfu/common/src/dfu_target.c
)

# NORDIC SDK APP END
