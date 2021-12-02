/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#ifndef SHDN_STORAGE_H_
#define SHDN_STORAGE_H_

#include <sys/types.h>
#include <kernel.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Shutdown Storage
 * @defgroup shdn_storage Shutdown Storage
 * @ingroup shdn
 * @{
 * @}
 */

/**
 * @brief Shutdown Storage Data Structures
 * @defgroup shdn_storage_data_structures Shutdown Storage Data Structures
 * @ingroup shdn_storage
 * @{
 */

/**
 * @brief Shutdown Storage File system structure
 *
 * @param offset File system offset in flash
 * @param ate_wra Allocation table entry write address. Addresses are stored as uint32_t:
 * high 2 bytes correspond to the sector, low 2 bytes are the offset in the sector
 * @param data_wra Data write address
 * @param sector_size File system is split into sectors, each sector must be multiple of pagesize
 * @param sector_count Number of sectors in the file systems
 * @param ready Flag indicating if the filesystem is initialized
 * @param sdm_lock Mutex
 * @param flash_device Flash Device runtime structure
 * @param flash_parameters Flash memory parameters structure
 */
struct shdn_fs {
	off_t offset;
	uint32_t ate_wra;
	uint32_t data_wra;
	uint16_t sector_size;
	uint16_t sector_count;
	bool ready;
	struct k_mutex shdn_lock;
	const struct device *flash_device;
	const struct flash_parameters *flash_parameters;
};

/**
 * @}
 */

/**
 * @brief Shutdown Storage APIs
 * @defgroup shdn_storage_api Shutdown Storage APIs
 * @ingroup shdn_storage
 * @{
 */

/**
 * @brief shdn_storage_init
 *
 * Initializes a Shutdown storage file system in flash.
 *
 * @param fs Pointer to file system
 * @param dev_name Pointer to flash device name
 * @retval 0 Success
 * @retval -ERRNO errno code if error
 */
int shdn_storage_init(struct shdn_fs *fs, const char *dev_name);

/**
 * @brief shdn_storage_clear
 *
 * Clears the Shutdown storage file system from flash.
 * @param fs Pointer to file system
 * @retval 0 Success
 * @retval -ERRNO errno code if error
 */
int shdn_storage_clear(struct shdn_fs *fs);

/**
 * @brief shdn_storage_write
 *
 * Write an entry to the file system.
 *
 * @param fs Pointer to file system
 * @param id Id of the entry to be written
 * @param data Pointer to the data to be written
 * @param len Number of bytes to be written
 *
 * @return Number of bytes written. On success, it will be equal to the number of bytes requested
 * to be written. When a rewrite of the same data already stored is attempted, nothing is written
 * to flash, thus 0 is returned. On error, returns negative value of errno.h defined error codes.
 */
ssize_t shdn_storage_write(struct shdn_fs *fs, uint16_t id, const void *data, size_t len);

/**
 * @brief shdn_storage_read
 *
 * Read an entry from the file system.
 *
 * @param fs Pointer to file system
 * @param id Id of the entry to be read
 * @param data Pointer to data buffer
 * @param len Number of bytes to be read
 *
 * @return Number of bytes read. On success, it will be equal to the number of bytes requested
 * to be read. When the return value is larger than the number of bytes requested to read this
 * indicates not all bytes were read, and more data is available. On error, returns negative
 * value of errno.h defined error codes.
 */
ssize_t shdn_storage_read(struct shdn_fs *fs, uint16_t id, void *data, size_t len);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* SHDN_STORAGE_H_ */
