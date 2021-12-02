/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <drivers/flash.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <fs/nvs.h>
#include <sys/crc.h>
#include <logging/log.h>
#include "shdn_storage.h"


LOG_MODULE_REGISTER(shdn_storage, CONFIG_SHDN_STORAGE_LOG_LEVEL);
/*
 * MASKS AND SHIFT FOR ADDRESSES
 * an address in nvs is an uint32_t where:
 *   high 2 bytes represent the sector number
 *   low 2 bytes represent the offset in a sector
 */
#define ADDR_SECT_MASK 0xFFFF0000
#define ADDR_SECT_SHIFT 16
#define ADDR_OFFS_MASK 0x0000FFFF

#define SHDN_STORAGE_BLOCK_SIZE 32

/* Allocation Table Entry */
struct shdn_ate {
	uint16_t id;	 /* data id */
	uint16_t offset; /* data offset within sector */
	uint16_t len;	 /* data len within sector */
	uint8_t part;	 /* part of a multipart data - future extension */
	uint8_t crc8;	 /* crc8 check of the entry */
} __packed;

BUILD_ASSERT(offsetof(struct shdn_ate, crc8) ==
		 sizeof(struct shdn_ate) - sizeof(uint8_t),
		 "crc8 must be the last member");

/* basic routines */
/* shdn_storage_al_size returns size aligned to fs->write_block_size */
static inline size_t shdn_storage_al_size(struct shdn_fs *fs, size_t len)
{
	uint8_t write_block_size = fs->flash_parameters->write_block_size;

	if (write_block_size <= 1U) {
		return len;
	}
	return (len + (write_block_size - 1U)) & ~(write_block_size - 1U);
}
/* end basic routines */

/* flash routines */
/* basic aligned flash write to address */
static int shdn_storage_flash_al_wrt(struct shdn_fs *fs, uint32_t addr, const void *data,
				     size_t len)
{
	const uint8_t *data8 = (const uint8_t *)data;
	int rc = 0;
	off_t offset;
	size_t blen;
	uint8_t buf[SHDN_STORAGE_BLOCK_SIZE];

	if (!len) {
		/* Nothing to write, avoid changing the flash protection */
		return 0;
	}

	offset = fs->offset;
	offset += fs->sector_size * (addr >> ADDR_SECT_SHIFT);
	offset += addr & ADDR_OFFS_MASK;

	blen = len & ~(fs->flash_parameters->write_block_size - 1U);
	if (blen > 0) {
		LOG_DBG("flash_write: flash_device: %p, offset: %u", 
			fs->flash_device, offset);
		LOG_HEXDUMP_DBG(data8, blen, "Write data8");
		rc = flash_write(fs->flash_device, offset, data8, blen);
		if (rc) {
			/* flash write error */
			goto end;
		}
		len -= blen;
		offset += blen;
		data8 += blen;
		LOG_DBG("flash_write end"); 
	}
	if (len) {
		memcpy(buf, data8, len);
		(void)memset(buf + len, fs->flash_parameters->erase_value,
			fs->flash_parameters->write_block_size - len);

		LOG_DBG("flash_write: flash_device: %p, offset: %u", 
			fs->flash_device, offset);
		LOG_HEXDUMP_DBG(buf, fs->flash_parameters->write_block_size, "Write buf");
		rc = flash_write(fs->flash_device, offset, buf,
				 fs->flash_parameters->write_block_size);
	}

end:
	return rc;
}

/* allocation entry write */
static int shdn_storage_flash_ate_wrt(struct shdn_fs *fs, const struct shdn_ate *entry)
{
	int rc;

	rc = shdn_storage_flash_al_wrt(fs, fs->ate_wra, entry,
				       sizeof(struct shdn_ate));
	fs->ate_wra -= shdn_storage_al_size(fs, sizeof(struct shdn_ate));

	return rc;
}

/* data write */
static int shdn_storage_flash_data_wrt(struct shdn_fs *fs, const void *data, size_t len)
{
	int rc;

	rc = shdn_storage_flash_al_wrt(fs, fs->data_wra, data, len);
	fs->data_wra += shdn_storage_al_size(fs, len);

	return rc;
}

/* basic flash read from address */
static int shdn_storage_flash_rd(struct shdn_fs *fs, uint32_t addr, void *data,
				 size_t len)
{
	int rc;
	off_t offset;

	offset = fs->offset;
	offset += fs->sector_size * (addr >> ADDR_SECT_SHIFT);
	offset += addr & ADDR_OFFS_MASK;

	rc = flash_read(fs->flash_device, offset, data, len);
	return rc;

}

/* flash ate read */
static int shdn_storage_flash_ate_rd(struct shdn_fs *fs, uint32_t addr,
				     struct shdn_ate *entry)
{
	return shdn_storage_flash_rd(fs, addr, entry, sizeof(struct shdn_ate));
}
/* end of basic flash routines */

/* advanced flash routines */

/* shdn_storage_flash_block_cmp compares the data in flash at addr to data
 * in blocks of size SHDN_STORAGE_BLOCK_SIZE aligned to fs->write_block_size
 * returns 0 if equal, 1 if not equal, errcode if error
 */
static int shdn_storage_flash_block_cmp(struct shdn_fs *fs, uint32_t addr, 
					const void *data, size_t len)
{
	const uint8_t *data8 = (const uint8_t *)data;
	int rc;
	size_t bytes_to_cmp, block_size;
	uint8_t buf[SHDN_STORAGE_BLOCK_SIZE];

	block_size = SHDN_STORAGE_BLOCK_SIZE &
		     ~(fs->flash_parameters->write_block_size - 1U);

	while (len) {
		bytes_to_cmp = MIN(block_size, len);
		rc = shdn_storage_flash_rd(fs, addr, buf, bytes_to_cmp);
		if (rc) {
			return rc;
		}
		rc = memcmp(data8, buf, bytes_to_cmp);
		if (rc) {
			return 1;
		}
		len -= bytes_to_cmp;
		addr += bytes_to_cmp;
		data8 += bytes_to_cmp;
	}
	return 0;
}

/* shdn_storage_flash_cmp_const compares the data in flash at addr to a constant
 * value. returns 0 if all data in flash is equal to value, 1 if not equal,
 * errcode if error
 */
static int shdn_storage_flash_cmp_const(struct shdn_fs *fs, uint32_t addr, 
					uint8_t value, size_t len)
{
	int rc;
	size_t bytes_to_cmp, block_size;
	uint8_t cmp[SHDN_STORAGE_BLOCK_SIZE];

	block_size = SHDN_STORAGE_BLOCK_SIZE &
		     ~(fs->flash_parameters->write_block_size - 1U);

	(void)memset(cmp, value, block_size);
	while (len) {
		bytes_to_cmp = MIN(block_size, len);
		rc = shdn_storage_flash_block_cmp(fs, addr, cmp, bytes_to_cmp);
		if (rc) {
			return rc;
		}
		len -= bytes_to_cmp;
		addr += bytes_to_cmp;
	}
	return 0;
}

/* erase a sector and verify erase was OK.
 * return 0 if OK, errorcode on error.
 */
static int shdn_storage_flash_erase_sector(struct shdn_fs *fs, uint32_t addr)
{
	int rc;
	off_t offset;

	addr &= ADDR_SECT_MASK;

	offset = fs->offset;
	offset += fs->sector_size * (addr >> ADDR_SECT_SHIFT);

	LOG_DBG("Erasing flash at %lx, len %d", (long int) offset,
		fs->sector_size);
	rc = flash_erase(fs->flash_device, offset, fs->sector_size);
	if (rc) {
		return rc;
	}

	rc = shdn_storage_flash_cmp_const(fs, addr, 
					  fs->flash_parameters->erase_value,
					  fs->sector_size);
	if (rc) {
		return -ENXIO;
	}

	return rc;
}

/* crc update on allocation entry */
static void shdn_storage_ate_crc8_update(struct shdn_ate *entry)
{
	uint8_t crc8;

	crc8 = crc8_ccitt(0xff, entry, offsetof(struct shdn_ate, crc8));
	entry->crc8 = crc8;
}

/* crc check on allocation entry
 * returns 0 if OK, 1 on crc fail
 */
static int shdn_storage_ate_crc8_check(const struct shdn_ate *entry)
{
	uint8_t crc8;

	crc8 = crc8_ccitt(0xff, entry, offsetof(struct shdn_ate, crc8));
	if (crc8 == entry->crc8) {
		return 0;
	}
	return 1;
}

/* shdn_storage_ate_cmp_const compares an ATE to a constant value. returns 0 if
 * the whole ATE is equal to value, 1 if not equal.
 */

static int shdn_storage_ate_cmp_const(const struct shdn_ate *entry, uint8_t value)
{
	const uint8_t *data8 = (const uint8_t *)entry;
	int i;

	for (i = 0; i < sizeof(struct shdn_ate); i++) {
		if (data8[i] != value) {
			return 1;
		}
	}

	return 0;
}

/* shdn_ate_valid validates an ate:
 *     return 1 if crc8 and offset valid,
 *            0 otherwise
 */
static int shdn_storage_ate_valid(struct shdn_fs *fs, const struct shdn_ate *entry)
{
	size_t ate_size;

	ate_size = shdn_storage_al_size(fs, sizeof(struct shdn_ate));

	if ((shdn_storage_ate_crc8_check(entry)) ||
	    (entry->offset >= (fs->sector_size - ate_size))) {
		return 0;
	}

	return 1;
}

/* shdn_storage_close_ate_valid validates an sector close ate: a valid sector close ate:
 * - valid ate
 * - len = 0 and id = 0xFFFF
 * - offset points to location at ate multiple from sector size
 * return 1 if valid, 0 otherwise
 */
static int shdn_storage_close_ate_valid(struct shdn_fs *fs, const struct shdn_ate *entry)
{
	size_t ate_size;

	if ((!shdn_storage_ate_valid(fs, entry)) || (entry->len != 0U) ||
	    (entry->id != 0xFFFF)) {
		return 0;
	}

	ate_size = shdn_storage_al_size(fs, sizeof(struct shdn_ate));
	if ((fs->sector_size - entry->offset) % ate_size) {
		return 0;
	}

	return 1;
}

/* store an entry in flash */
static int shdn_storage_flash_wrt_entry(struct shdn_fs *fs, uint16_t id, const void *data,
				        size_t len)
{
	int rc;
	struct shdn_ate entry;
	size_t ate_size;

	ate_size = shdn_storage_al_size(fs, sizeof(struct shdn_ate));

	entry.id = id;
	entry.offset = (uint16_t)(fs->data_wra & ADDR_OFFS_MASK);
	entry.len = (uint16_t)len;
	entry.part = 0xff;
	shdn_storage_ate_crc8_update(&entry);

	rc = shdn_storage_flash_data_wrt(fs, data, len);
	if (rc) {
		return rc;
	}

	rc = shdn_storage_flash_ate_wrt(fs, &entry);
	if (rc) {
		return rc;
	}

	return 0;
}

/* If the closing ate is invalid, its offset cannot be trusted and
 * the last valod ate of the sector should instead try to be recovered by going
 * through all ate's.
 *
 * addr should point to the faulty closing ate and will be updated to the last
 * valid ate. If no valid ate is found it will be left untouched.
 */
static int shdn_storage_recover_last_ate(struct shdn_fs *fs, uint32_t *addr)
{
	uint32_t data_end_addr, ate_end_addr;
	struct shdn_ate end_ate;
	size_t ate_size;
	int rc;

	LOG_DBG("Recovering last ate from sector %d",
		(*addr >> ADDR_SECT_SHIFT));

	ate_size = shdn_storage_al_size(fs, sizeof(struct shdn_ate));

	*addr -= ate_size;
	ate_end_addr = *addr;
	data_end_addr = *addr & ADDR_SECT_MASK;
	while (ate_end_addr > data_end_addr) {
		rc = shdn_storage_flash_ate_rd(fs, ate_end_addr, &end_ate);
		if (rc) {
			return rc;
		}
		if (shdn_storage_ate_valid(fs, &end_ate)) {
			/* found a valid ate, update data_end_addr and *addr */
			data_end_addr &= ADDR_SECT_MASK;
			data_end_addr += end_ate.offset + end_ate.len;
			*addr = ate_end_addr;
		}
		ate_end_addr -= ate_size;
	}

	return 0;
}

/* walking through allocation entry list, from newest to oldest entries
 * read ate from addr, modify addr to the previous ate
 */
static int shdn_storage_prev_ate(struct shdn_fs *fs, uint32_t *addr, struct shdn_ate *ate)
{
	int rc;
	struct shdn_ate close_ate;
	size_t ate_size;

	ate_size = shdn_storage_al_size(fs, sizeof(struct shdn_ate));

	rc = shdn_storage_flash_ate_rd(fs, *addr, ate);
	if (rc) {
		return rc;
	}

	*addr += ate_size;
	if (((*addr) & ADDR_OFFS_MASK) != (fs->sector_size - ate_size)) {
		return 0;
	}

	/* last ate in sector, do jump to previous sector */
	if (((*addr) >> ADDR_SECT_SHIFT) == 0U) {
		*addr += ((fs->sector_count - 1) << ADDR_SECT_SHIFT);
	} else {
		*addr -= (1 << ADDR_SECT_SHIFT);
	}

	rc = shdn_storage_flash_ate_rd(fs, *addr, &close_ate);
	if (rc) {
		return rc;
	}

	rc = shdn_storage_ate_cmp_const(&close_ate, fs->flash_parameters->erase_value);
	/* at the end of filesystem */
	if (!rc) {
		*addr = fs->ate_wra;
		return 0;
	}

	/* Update the address if the close ate is valid.
	 */
	if (shdn_storage_close_ate_valid(fs, &close_ate)) {
		(*addr) &= ADDR_SECT_MASK;
		(*addr) += close_ate.offset;
		return 0;
	}

	/* The close_ate was invalid, `lets find out the last valid ate
	 * and point the address to this found ate.
	 *
	 * remark: if there was absolutely no valid data in the sector *addr
	 * is kept at sector_end - 2*ate_size, the next read will contain
	 * invalid data and continue with a sector jump
	 */
	return shdn_storage_recover_last_ate(fs, addr);
}

static void shdn_storage_sector_advance(struct shdn_fs *fs, uint32_t *addr)
{
	*addr += (1 << ADDR_SECT_SHIFT);
	if ((*addr >> ADDR_SECT_SHIFT) == fs->sector_count) {
		*addr -= (fs->sector_count << ADDR_SECT_SHIFT);
	}
}

static int shdn_storage_startup(struct shdn_fs *fs)
{
	int rc;
	struct shdn_ate last_ate;
	size_t ate_size, empty_len;
	/* Initialize addr to 0 for the case fs->sector_count == 0. This
	 * should never happen as this is verified in nvs_init() but both
	 * Coverity and GCC believe the contrary.
	 */
	uint32_t addr = 0U;
	uint16_t i, closed_sectors = 0;
	uint8_t erase_value = fs->flash_parameters->erase_value;

	k_mutex_lock(&fs->shdn_lock, K_FOREVER);

	ate_size = shdn_storage_al_size(fs, sizeof(struct shdn_ate));
	/* step through the sectors to find a open sector following
	 * a closed sector, this is where NVS can to write.
	 */
	for (i = 0; i < fs->sector_count; i++) {
		addr = (i << ADDR_SECT_SHIFT) +
		       (uint16_t)(fs->sector_size - ate_size);
		rc = shdn_storage_flash_cmp_const(fs, addr, erase_value,
					 sizeof(struct shdn_ate));
		if (rc) {
			/* closed sector */
			closed_sectors++;
			shdn_storage_sector_advance(fs, &addr);
			rc = shdn_storage_flash_cmp_const(fs, addr, erase_value,
						 sizeof(struct shdn_ate));
			if (!rc) {
				/* open sector */
				break;
			}
		}
	}
	/* all sectors are closed, this is not a nvs fs */
	if (closed_sectors == fs->sector_count) {
		rc = -EDEADLK;
		goto end;
	}

	if (i == fs->sector_count) {
		/* none of the sectors where closed, in most cases we can set
		 * the address to the first sector, except when there are only
		 * two sectors. Then we can only set it to the first sector if
		 * the last sector contains no ate's. So we check this first
		 */
		rc = shdn_storage_flash_cmp_const(fs, addr - ate_size, erase_value,
				sizeof(struct shdn_ate));
		if (!rc) {
			/* empty ate */
			shdn_storage_sector_advance(fs, &addr);
		}
	}

	/* addr contains address of closing ate in the most recent sector,
	 * search for the last valid ate using the recover_last_ate routine
	 */

	rc = shdn_storage_recover_last_ate(fs, &addr);
	if (rc) {
		goto end;
	}


	/* addr contains address of the last valid ate in the most recent sector
	 * search for the first ate containing all cells erased, in the process
	 * also update fs->data_wra.
	 */
	fs->ate_wra = addr;
	fs->data_wra = addr & ADDR_SECT_MASK;

	while (fs->ate_wra >= fs->data_wra) {
		rc = shdn_storage_flash_ate_rd(fs, fs->ate_wra, &last_ate);
		if (rc) {
			goto end;
		}

		rc = shdn_storage_ate_cmp_const(&last_ate, erase_value);

		if (!rc) {
			/* found ff empty location */
			break;
		}

		if (shdn_storage_ate_valid(fs, &last_ate)) {
			/* complete write of ate was performed */
			fs->data_wra = addr & ADDR_SECT_MASK;
			/* Align the data write address to the current
			 * write block size so that it is possible to write to
			 * the sector even if the block size has changed after
			 * a software upgrade (unless the physical ATE size
			 * will change)."
			 */
			fs->data_wra += shdn_storage_al_size(fs, last_ate.offset + last_ate.len);

			/* ate on the last possition within the sector is
			 * reserved for deletion an entry
			 */
			if (fs->ate_wra == fs->data_wra && last_ate.len) {
				/* not a delete ate */
				rc = -ESPIPE;
				goto end;
			}
		}

		fs->ate_wra -= ate_size;
	}

	/* if the sector after the write sector is not empty gc was interrupted
	 * we might need to restart gc if it has not yet finished. Otherwise
	 * just erase the sector.
	 * When gc needs to be restarted, first erase the sector otherwise the
	 * data might not fit into the sector.
	 */
	addr = fs->ate_wra & ADDR_SECT_MASK;
	shdn_storage_sector_advance(fs, &addr);
	rc = shdn_storage_flash_cmp_const(fs, addr, erase_value, fs->sector_size);
	if (rc < 0) {
		goto end;
	}
	// if (rc) {
	// 	/* the sector after fs->ate_wrt is not empty, look for a marker
	// 	 * (gc_done_ate) that indicates that gc was finished.
	// 	 */
	// 	bool gc_done_marker = false;
	// 	struct shdn_ate gc_done_ate;

	// 	addr = fs->ate_wra + ate_size;
	// 	while ((addr & ADDR_OFFS_MASK) < (fs->sector_size - ate_size)) {
	// 		rc = shdn_storage_flash_ate_rd(fs, addr, &gc_done_ate);
	// 		if (rc) {
	// 			goto end;
	// 		}
	// 		if (shdn_storage_ate_valid(fs, &gc_done_ate) &&
	// 		    (gc_done_ate.id == 0xffff) &&
	// 		    (gc_done_ate.len == 0U)) {
	// 			gc_done_marker = true;
	// 			break;
	// 		}
	// 		addr += ate_size;
	// 	}

	// 	if (gc_done_marker) {
	// 		/* erase the next sector */
	// 		LOG_INF("GC Done marker found");
	// 		addr = fs->ate_wra & ADDR_SECT_MASK;
	// 		shdn_storage_sector_advance(fs, &addr);
	// 		rc = shdn_storage_flash_erase_sector(fs, addr);
	// 		goto end;
	// 	}
	// 	LOG_INF("No GC Done marker found: restarting gc");
	// 	// rc = shdn_storage_flash_erase_sector(fs, fs->ate_wra);
	// 	// if (rc) {
	// 	// 	goto end;
	// 	// }
	// 	// fs->ate_wra &= ADDR_SECT_MASK;
	// 	// fs->ate_wra += (fs->sector_size - 2 * ate_size);
	// 	// fs->data_wra = (fs->ate_wra & ADDR_SECT_MASK);
	// 	//rc = nvs_gc(fs);
	// 	// goto end;
	// }

	/* possible data write after last ate write, update data_wra */
	while (fs->ate_wra > fs->data_wra) {
		empty_len = fs->ate_wra - fs->data_wra;

		rc = shdn_storage_flash_cmp_const(fs, fs->data_wra, erase_value,
				empty_len);
		if (rc < 0) {
			goto end;
		}
		if (!rc) {
			break;
		}

		fs->data_wra += fs->flash_parameters->write_block_size;
	}

	/* If the ate_wra is pointing to the first ate write location in a
	 * sector and data_wra is not 0, erase the sector as it contains no
	 * valid data (this also avoids closing a sector without any data).
	 */
	if (((fs->ate_wra + 2 * ate_size) == fs->sector_size) &&
	    (fs->data_wra != (fs->ate_wra & ADDR_SECT_MASK))) {
		rc = shdn_storage_flash_erase_sector(fs, fs->ate_wra);
		if (rc) {
			goto end;
		}
		fs->data_wra = fs->ate_wra & ADDR_SECT_MASK;
	}

end:
	/* If the sector is empty add a gc done ate to avoid having insufficient
	 * space when doing gc.
	 */
	// if ((!rc) && ((fs->ate_wra & ADDR_OFFS_MASK) ==
	// 	      (fs->sector_size - 2 * ate_size))) {

	// 	rc = shdn_storage_add_gc_done_ate(fs);
	// }
	k_mutex_unlock(&fs->shdn_lock);
	return rc;
}

int shdn_storage_init(struct shdn_fs *fs, const char *dev_name)
{
	int rc;
	struct flash_pages_info info;
	size_t write_block_size;

	k_mutex_init(&fs->shdn_lock);

	fs->flash_device = device_get_binding(dev_name);
	if (!fs->flash_device) {
		LOG_ERR("No valid flash device found");
		return -ENXIO;
	}

	fs->flash_parameters = flash_get_parameters(fs->flash_device);
	if (fs->flash_parameters == NULL) {
		LOG_ERR("Could not obtain flash parameters");
		return -EINVAL;
	}

	write_block_size = flash_get_write_block_size(fs->flash_device);

	/* check that the write block size is supported */
	if (write_block_size > SHDN_STORAGE_BLOCK_SIZE || write_block_size == 0) {
		LOG_ERR("Unsupported write block size");
		return -EINVAL;
	}

	/* check that sector size is a multiple of pagesize */
	rc = flash_get_page_info_by_offs(fs->flash_device, fs->offset, &info);
	if (rc) {
		LOG_ERR("Unable to get page info");
		return -EINVAL;
	}
	if (!fs->sector_size || fs->sector_size % info.size) {
		LOG_ERR("Invalid sector size");
		return -EINVAL;
	}

	/* check the number of sectors, it should be at least 1 */
	if (fs->sector_count < 1) {
		LOG_ERR("Configuration error - sector count");
		return -EINVAL;
	}

	rc = shdn_storage_startup(fs);
	if (rc) {
		return rc;
	}

	/* shutdown storage is ready for use */
	fs->ready = true;

	LOG_INF("%d Sectors of %d bytes", fs->sector_count, fs->sector_size);
	LOG_INF("alloc wra: %d, %x",
		(fs->ate_wra >> ADDR_SECT_SHIFT),
		(fs->ate_wra & ADDR_OFFS_MASK));
	LOG_INF("data wra: %d, %x",
		(fs->data_wra >> ADDR_SECT_SHIFT),
		(fs->data_wra & ADDR_OFFS_MASK));
	
	return 0;
}

int shdn_storage_clear(struct shdn_fs *fs)
{
	int rc;
	uint32_t addr;

	if (!fs->ready) {
		LOG_ERR("SHDN storage not initialized");
		return -EACCES;
	}

	for (uint16_t i = 0; i < fs->sector_count; i++) {
		addr = i << ADDR_SECT_SHIFT;
		rc = shdn_storage_flash_erase_sector(fs, addr);
		if (rc) {
			return rc;
		}
	}

	/* shdn storage needs to be reinitialized after clearing */
	fs->ready = false;

	return 0;
}

ssize_t shdn_storage_write(struct shdn_fs *fs, uint16_t id, const void *data, size_t len)
{
	int rc;
	size_t ate_size;
	size_t data_size;
	uint16_t required_space = 0U; /* no space, appropriate for delete ate */

	if (!fs->ready) {
		LOG_ERR("SHDN storage not initialized");
		return -EACCES;
	}

	ate_size = shdn_storage_al_size(fs, sizeof(struct shdn_ate));
	data_size = shdn_storage_al_size(fs, len);
	/* The maximum data size is sector size - 4 ate
	 * where: 1 ate for data, 1 ate for sector close, 1 ate for gc done,
	 * and 1 ate to always allow a delete.
	 */
	if ((len > (fs->sector_size - 4 * ate_size)) ||
	    ((len > 0) && (data == NULL))) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	/* calculate required space if the entry contains data */
	if (data_size) {
		/* Leave space for delete ate */
		required_space = data_size + ate_size;
	}

	k_mutex_lock(&fs->shdn_lock, K_FOREVER);

	if (fs->ate_wra >= (fs->data_wra + required_space)) {

		rc = shdn_storage_flash_wrt_entry(fs, id, data, len);
		if (rc) {
			goto end;
		}
	}
	rc = len;

end:
	k_mutex_unlock(&fs->shdn_lock);
	return rc;
}

ssize_t shdn_storage_read(struct shdn_fs *fs, uint16_t id, void *data, size_t len)
{
	int rc;
	uint32_t wlk_addr, rd_addr;
	struct shdn_ate wlk_ate;
	size_t ate_size;
	uint16_t cnt_his;

	if (!fs->ready) {
		LOG_ERR("SHDN storage not initialized");
		return -EACCES;
	}

	ate_size = shdn_storage_al_size(fs, sizeof(struct shdn_ate));

	if (len > (fs->sector_size - 2 * ate_size)) {
		return -EINVAL;
	}

	cnt_his = 0U;

	wlk_addr = fs->ate_wra;
	rd_addr = wlk_addr;

	while (!cnt_his) {
		rd_addr = wlk_addr;
		rc = shdn_storage_prev_ate(fs, &wlk_addr, &wlk_ate);
		if (rc) {
			goto err;
		}
		if ((wlk_ate.id == id) &&  (shdn_storage_ate_valid(fs, &wlk_ate))) {
 			cnt_his++;
		}
		if (wlk_addr == fs->ate_wra) {
			break;
		}
	}

	LOG_DBG("shdn_storage_read 1: wlk_addr:fs->ate_wra (%d:%d), wlk_ate.id:id (%d:%d), wlk_ate.len (%d)", 
		wlk_addr, fs->ate_wra, wlk_ate.id, id, wlk_ate.len);
	if (((wlk_addr == fs->ate_wra) && (wlk_ate.id != id)) ||
	    (wlk_ate.len == 0U)) {
		return -ENOENT;
	}

	rd_addr &= ADDR_SECT_MASK;
	rd_addr += wlk_ate.offset;
	rc = shdn_storage_flash_rd(fs, rd_addr, data, MIN(len, wlk_ate.len));
	if (rc) {
		goto err;
	}

	return wlk_ate.len;

err:
	return rc;
}
