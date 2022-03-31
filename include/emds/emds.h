/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @defgroup emds Emergency Data Storage
 *  @ingroup emds
 *  @{
 *  @brief Emergency Data Storage API
 */

#ifndef EMDS_H__
#define EMDS_H__

#include <stddef.h>
#include <sys/types.h>
#include <sys/util.h>
#include <sys/slist.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct emds_entry
 *
 * Information about entries used to store data in the emergency data storage.
 */
struct emds_entry {
	/** Unique ID for each static and dynamic entry. */
	uint16_t id;
	/** Pointer to data that will be stored. */
	uint8_t *data;
	/** Length of data that will be stored. */
	size_t len;
};

/**
 * @struct emds_dynamic_entry
 *
 * Entries with a node element, used for dynamic entries. These are registered
 * using a call to @ref emds_entry_add.
 */
struct emds_dynamic_entry {
	struct emds_entry entry;
	sys_snode_t node;
};

/**
 * @brief Define a static entry for emergency data storage items.
 *
 * @param name The entry name.
 * @param _id Unique ID for the entry. This value and not an overlap with any
 *            other value.
 * @param _data Data pointer to be stored at emergency data store.
 * @param _len Length of data to be stored at emergency data store.
 *
 * This creates a variable _name prepended by emds_.
 */
#define EMDS_STATIC_ENTRY_DEFINE(_name, _id, _data, _len)                      \
	static const STRUCT_SECTION_ITERABLE(emds_entry, emds_##_name) = {     \
		.id = _id,                                                     \
		.data = (uint8_t *)_data,                                      \
		.len = _len,                                                   \
	}

/**
 * @brief Initialize the emergency data storage.
 *
 * Initializes the emergency data storage. This needs to be called before adding
 * entries to the @ref emds_dynamic_entry and loading data.
 *
 * @retval 0 Success
 * @retval -ERRNO errno code if error
 */
int emds_init(void);

/**
 *  @brief Add entry to be saved/restored when emergency data storage is called.
 *
 * Adds the entry to the dynamic entry list. When the @ref emds_store function
 * is called, takes the given data pointer and stores the data to the emergency
 * data storage.
 *
 * @param entry Entry to add to list and load data into.
 *
 * @retval 0 Success
 * @retval -ERRNO errno code if error
 */
int emds_entry_add(const struct emds_entry *entry);

/**
 * @brief Start the emergency data storage process.
 *
 * Triggers the process of storing all data registered to be stored. All data
 * registered either through @ref emds_entry_add function or the
 * @ref EMDS_STATIC_ENTRY_DEFINE macro is stored. Once the data storage is
 * completed, the data should not be changed, and the board should be halted or
 * rebooted at the end. This is a time-critical process and will be processed as
 * fast as possible.
 *
 * @retval 0 Success
 * @retval -ERRNO errno code if error
 */
int emds_store(void);

/**
 * @brief Load all static data from the emergency data storage.
 *
 * This function needs to be called after the static entries are added, as they
 * are used to select the data to be loaded. The function also needs to be
 * called before the @ref emds_prepare function which will delete all the stored
 * data.
 *
 * @retval 0 Success
 * @retval -ERRNO errno code if error
 */
int emds_load(void);

/**
 * @brief Clear flash area from the emergency data storage.
 *
 * This function clears the flash area for all previously stored data.
 *
 * @retval 0 Success
 * @retval -ERRNO errno code if error
 */
int emds_clear(void);

/**
 * @brief Prepare flash area for the next emergency data storage.
 *
 * This function prepares the flash area for the next emergency data storage. It
 * deletes the current entries if there is enough space for the next emergency
 * data storage, and clears the flash storage if there is not enough space for
 * the next storage. This has to be done after all the dynamic entries are
 * added.
 *
 * @retval 0 Success
 * @retval -ERRNO errno code if error
 */
int emds_prepare(void);

/**
 * @brief Calculate the time needed to store the registered data.
 *
 * Calculates how much time it takes to store all dynamic and static data
 * registered in the entries.
 *
 * @return Time needed to store all data (in microseconds).
 */
uint32_t emds_store_time_get(void);

/**
 * @brief Calculate the size needed to store the registered data.
 *
 * Calculates the size it takes to store all dynamic and static data registered
 * in the entries.
 *
 * @return Byte size that is needed to store all data.
 */
uint32_t emds_store_size_get(void);

/**
 * @brief Check if the store operation can be run.
 *
 * @return True if the store operation can be started, otherwise false.
 */
bool emds_is_ready(void);

/**
 * @brief Check if the store operation has finished.
 *
 * @return True if the store operation is completed, otherwise false.
 */
bool emds_store_complete(void);

#ifdef __cplusplus
}
#endif

#endif /* EMDS_H__ */

/** @} */
