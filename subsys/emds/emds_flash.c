/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <emds/emds_flash.h>

int emds_flash_init(struct emds_fs *fs, const char *dev_name)
{
	return 0;
}

int emds_flash_clear(struct emds_fs *fs)
{
	return 0;
}

ssize_t emds_flash_write(struct emds_fs *fs, uint16_t id, const void *data, size_t len)
{
	return 0;
}

ssize_t emds_flash_read(struct emds_fs *fs, uint16_t id, void *data, size_t len)
{
	return 0;
}

int emds_flash_prepare(struct emds_fs *fs, int entry_cnt, int byte_size)
{
	return 0;
}

ssize_t emds_flash_free_space(struct emds_fs *fs)
{
	return 0;
}
