/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#ifndef SHDN_MANAGER_H_
#define SHDN_MANAGER_H_

/**
 * @file shdn.h
 *
 * @brief Header file for the shutdown manager.
 */

#include <stddef.h>
#include <sys/types.h>
#include <sys/util.h>
#include <sys/slist.h>

/**
 * @defgroup shdn Shutdown process and storage
 * @{
 * @brief Shutdown process and storage.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Shutdown Manager APIs
 * @defgroup shdn_manager_api Shutdown Manager APIs
 * @ingroup shdn
 * @{
 */

struct shdn_entry {
	uint16_t id;
	uint8_t *data;
	size_t len;
};

struct shdn_dynamic_entry {
	struct shdn_entry entry;
	sys_snode_t node;
};

/**
 * 
 */
#define SHDN_ENTRY_DEFINE(_name, _id, _data, _len)                             \
	const STRUCT_SECTION_ITERABLE(shdn_entry, shdn_ ## _name) = {                \
		.id = _id,                                                     \
		.data = (uint8_t *)_data,                                      \
		.len = _len,                                          	       \
	}

/**
 * @brief shdn_manager_init
 *
 * Initializes a Shutdown Manager.
 *
 * @retval 0 Success
 * @retval -ERRNO errno code if error
 */ 
int shdn_manager_init();

/**
 *  @brief Function to add entry that should be saved/restored.
 * 
 * @param entry Entry to add to list.
 *
*/
void shdn_manager_entry_add(struct shdn_dynamic_entry *entry);

/**
 * @brief Start the shutdown process.
 * 
 * This will set of the process of storing all data that has been regitster to 
 * be stored when this process start. When it is finished with storing the data
 * it will shutdown all process.
 * 
 */
void shdn_manager_kill();

/**
 * @brief Restore all data after boot up.
 * 
 * @retval 0 Succes
 * @retval -ERRNO errno code if error
 */
int shdn_manager_restore(void);

/**
 * @brief After data has been restored, prepare flash area for a next shutdown.
 * 
 * @retval 0 Succes
 * @retval -ERRNO errno code if error
 */
int shdn_manager_prepare_shutdown(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* SHDN_MANAGER_H_ */
