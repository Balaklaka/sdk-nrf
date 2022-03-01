#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_sources(shdn_manager.c)
#zephyr_sources(shdn_storage.c)
zephyr_linker_sources(SECTIONS shdn_types.ld)