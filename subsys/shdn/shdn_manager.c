/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <shdn/shdn_manager.h>

#include <errno.h>
#include <zephyr.h>
#include <irq.h>
#include <logging/log.h>
#include <nrfx.h>
#include <hal/nrf_radio.h>
#include <storage/flash_map.h>
#include <sys/reboot.h>
#include "shdn_storage.h"

LOG_MODULE_REGISTER(shdn_manager, CONFIG_SHDN_MANAGER_LOG_LEVEL);

#define SHDN_STACK_SIZE 500
#define SHDN_PRIORITY -CONFIG_NUM_COOP_PRIORITIES 

#define ERASE_STACK_SIZE 500
#define ERASE_PRIORITY 0
K_SEM_DEFINE(erase_sem, 0, 1);
#include <drivers/flash.h>

K_SEM_DEFINE(shdn_sem, 0, 1);

#define PIN_DEBUG_ENABLE
#include <pin_debug_transport.h>

struct shdn_storage {
	struct shdn_fs cf_shdn;
	uint16_t last_name_id;
	const char *flash_dev_name;
};

sys_slist_t shdn_dynamics_entries;

static struct shdn_storage default_shdn_storage;

static void shdn_handler(void)
{
	LOG_DBG("shdn_handler started.");

	k_sem_reset(&shdn_sem);
	k_sem_take(&shdn_sem, K_FOREVER);

	LOG_DBG("shdn_handler execute.");
	DBP11_ON;
	k_sched_lock();

	struct shdn_dynamic_entry *ch;
	SYS_SLIST_FOR_EACH_CONTAINER(&shdn_dynamics_entries, ch, node) {
		ssize_t len = shdn_storage_write(&default_shdn_storage.cf_shdn, 
						 ch->entry.id, ch->entry.data, 
						 ch->entry.len);
		if (len != ch->entry.len) {
			LOG_ERR("Write failed (%d:%d)\n", ch->entry.len, len);
		} else {
			LOG_DBG("Write CF: %p, Entry: %d, Data length: %d",
				&default_shdn_storage.cf_shdn, 
				ch->entry.id, ch->entry.len);
			LOG_HEXDUMP_DBG(ch->entry.data, ch->entry.len, "Preserved data");
		}
	}

	STRUCT_SECTION_FOREACH(shdn_entry, ch) {
		ssize_t len = shdn_storage_write(&default_shdn_storage.cf_shdn, 
						 ch->id, ch->data, 
						 ch->len);
		if (len != ch->len) {
			LOG_ERR("Write failed (%d:%d)\n", ch->len, len);
		} else {
			LOG_DBG("Write CF: %p, Entry: %d, Data length: %d",
				&default_shdn_storage.cf_shdn, 
				ch->id, ch->len);
			LOG_HEXDUMP_DBG(ch->data, ch->len, "Preserved data");
		}
	}
	
	DBP11_OFF;
	k_sched_unlock();

	/* Hang until reset is pressed or performed. */
	sys_reboot(1);
}

K_THREAD_DEFINE(shdn_tid, SHDN_STACK_SIZE,
		shdn_handler, NULL, NULL, NULL,
		SHDN_PRIORITY, 0, 0);

static int shdn_manager_fs_init(void)
{
	int rc;
	const struct flash_area *fa;
	uint32_t sector_cnt = 1;
	struct flash_sector hw_flash_sector;
	size_t shdn_sector_size;
	size_t shdn_storage_size = 0;
	uint16_t cnt = 0;

	rc = flash_area_open(FLASH_AREA_ID(shdn_storage), &fa);
	if (rc) {
		return rc;
	}

	rc = flash_area_get_sectors(FLASH_AREA_ID(shdn_storage), &sector_cnt,
						  &hw_flash_sector);
	if (rc == -ENODEV) {
		return rc;
	} else if (rc != 0 && rc != -ENOMEM) {
		k_panic();
	}

	shdn_sector_size = CONFIG_SHDN_MANAGER_SECTOR_SIZE_MULT * 
				hw_flash_sector.fs_size;

	if (shdn_sector_size > UINT16_MAX) {
		return -EDOM;
	}

	while(cnt < CONFIG_SHDN_MANAGER_SECTOR_COUNT) {
		shdn_storage_size += shdn_sector_size;
		if (shdn_storage_size > fa->fa_size) {
			break;
		}
		cnt++;
	}

	default_shdn_storage.cf_shdn.sector_size = shdn_sector_size;
	default_shdn_storage.cf_shdn.sector_count = cnt;
	default_shdn_storage.cf_shdn.offset = fa->fa_off;
	default_shdn_storage.flash_dev_name = fa->fa_dev_name;

	rc = shdn_storage_init(&default_shdn_storage.cf_shdn, fa->fa_dev_name);
	if (rc) {
		return rc;
	}

	return 0;
}

int shdn_manager_init()
{
	int rc;

	rc = shdn_manager_fs_init();
	if (rc) {
		LOG_ERR("Shutdown manager file system failed (err %d)\n", rc);
		return rc;
	}

	sys_slist_init(&shdn_dynamics_entries);

	LOG_DBG("Shutdown manager initialized\n");

	return 0;
}

void shdn_manager_entry_add(struct shdn_dynamic_entry *entry)
{
	sys_slist_append(&shdn_dynamics_entries, &entry->node);
}

void shdn_manager_kill(void)
{
	(void)irq_lock();
	/* Stop the radio ... */
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	/* Trigger shutdown process */
	k_sem_give(&shdn_sem);
}

int shdn_manager_restore(void)
{
	struct shdn_dynamic_entry *ch;
	SYS_SLIST_FOR_EACH_CONTAINER(&shdn_dynamics_entries, ch, node) {
		ssize_t len = shdn_storage_read(&default_shdn_storage.cf_shdn, 
						ch->entry.id, ch->entry.data, 
						ch->entry.len);
		if (len != ch->entry.len) {
			LOG_ERR("Read failed (%d:%d)\n", ch->entry.len, len);
		} else {
			LOG_DBG("Read CF: %p, Entry: %d, Data length: %d",
				&default_shdn_storage.cf_shdn, 
				ch->entry.id, ch->entry.len);
			LOG_HEXDUMP_DBG(ch->entry.data, ch->entry.len, "Preserved data");
		}
	}

	STRUCT_SECTION_FOREACH(shdn_entry, ch) {
		ssize_t len = shdn_storage_read(&default_shdn_storage.cf_shdn, 
						ch->id, ch->data, 
						ch->len);
		if (len != ch->len) {
			LOG_ERR("Read failed (%d:%d)\n", ch->len, len);
		} else {
			LOG_DBG("Read CF: %p, Entry: %d, Data length: %d",
				&default_shdn_storage.cf_shdn, 
				ch->id, ch->len);
			LOG_HEXDUMP_DBG(ch->data, ch->len, "Preserved data");
		}
	}

	return 0;
}

int shdn_manager_prepare_shutdown(void)
{
	int rc;
	rc = shdn_storage_clear(&default_shdn_storage.cf_shdn);
	if (rc) {
		return rc;
	}

	rc = shdn_storage_init(&default_shdn_storage.cf_shdn, default_shdn_storage.flash_dev_name);
	if (rc) {
		return rc;
	}

	return rc;
}

void shdn_manager_erase_release(void)
{
	k_sem_give(&erase_sem);
}

static void erase_handler(void)
{
	const struct flash_area *fa;
	const struct device *fd;
	int rc;

	LOG_DBG("erase_handler started.");
	rc = flash_area_open(FLASH_AREA_ID(storage), &fa);
	if (rc) {
		LOG_ERR("Flash area open failed (%d)\n", rc);
		return;
	}

	fd = device_get_binding(fa->fa_dev_name);
	if (!fd) {
		LOG_ERR("No valid flash device found");
		return;
	}

	k_sem_reset(&erase_sem);
	k_sem_take(&erase_sem, K_FOREVER);
	DBP10_ON;
	rc = flash_erase(fd, 0xfe000, 0x1000);
	if (rc) {
		LOG_ERR("Flash area erase failed (%d)\n", rc);
		return;
	}
	DBP10_OFF;
}

K_THREAD_DEFINE(erase_tid, ERASE_STACK_SIZE,
		erase_handler, NULL, NULL, NULL,
		ERASE_PRIORITY, 0, 0);
