/*
 * sdhc.h
 *
 * Copyright (c) 2025 Jan Rusnak <jan@rusnak.sk>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

 /*
  * Notes on data buffer alignment:
  *
  * The low-level HSMCI SD driver typically uses PDC/DMA for data transfers.
  * On some targets this requires at least 32-bit aligned buffers for reliable
  * operation.
  *
  * Public APIs that transfer data:
  *   - sdhc_read_sectors()
  *   - sdhc_write_sectors()
  *
  * Recommended rule for callers:
  *   - Provide buffers aligned to 4 bytes.
  *   - Avoid unaligned stack slices or packed structs as I/O buffers.
  *
  * If your platform supports unaligned PDC/DMA safely, this is still a good
  * performance recommendation.
  */

#ifndef SDHC_H
#define SDHC_H

#include "hsmci_cmd.h"

#ifndef SDHC_MAX_RW_SECTORS_CNT
/*
 * Default multi-block transfer limit derived from 16-bit HSMCI PDC counters.
 * Assumes 32-bit PDC transfers:
 *   max_sectors = 0xFFFF / (512 / 4) = 511
 *
 * Override this macro if your low-level driver uses a different transfer
 * width/counter model or if you want a smaller conservative limit.
 */
#define SDHC_MAX_RW_SECTORS_CNT (0xFFFFU / (512 / 4))
#endif

/**
 * @enum sdhc_card_type
 * @brief SD card type detected during initialization.
 *
 * Distinguishes between legacy/unknown cards and high-capacity SDHC/SDXC
 * cards that use sector addressing with a fixed 512-byte sector size.
 */
enum sdhc_card_type {
	SDHC_CARD_TYPE_UNKNOWN,
	SDHC_CARD_TYPE_SDHC,
	SDHC_CARD_TYPE_SDXC
};

/**
 * @name sdhc_card_info.flags bit definitions
 * @brief State and capability flags stored in struct sdhc_card_info::flags.
 *
 * These bits describe current card presence / initialization state and
 * negotiated bus capabilities. They are set/cleared by the SDHC layer and
 * can be inspected by higher layers (e.g. disk_status()).
 */
#define SDHC_CARD_FLAG_PRESENT       (1 << 0)
#define SDHC_CARD_FLAG_INITIALIZED   (1 << 1)
#define SDHC_CARD_FLAG_HIGH_SPEED    (1 << 2)
#define SDHC_CARD_FLAG_WIDE_BUS      (1 << 3)
#define SDHC_CARD_FLAG_ERROR         (1 << 4)
#define SDHC_CARD_FLAG_DISCARD_SUP   (1 << 5)

/**
 * @brief SDHC card identification and geometry information.
 *
 * Structure filled by sdhc_init_card() with information obtained from
 * CID, CSD, OCR and related commands. It contains both raw register
 * images and decoded fields commonly used by higher layers (capacity,
 * product identification, timing information, etc.).
 */
struct sdhc_card_info {
	enum sdhc_card_type type;    /**< Detected card type (SDHC/SDXC/UNKNOWN). */
	unsigned int flags;          /**< SDHC_CARD_FLAG_* bitmask. */
	uint16_t rca;		     /**< Relative Card Address (RCA). */
	uint32_t ocr;		     /**< Operation Conditions Register (OCR). */
	uint32_t sector_size;	     /**< Logical sector size in bytes (usually 512). */
	uint32_t sector_count;       /**< Total number of logical sectors. */
	uint64_t capacity_bytes;     /**< Total card capacity in bytes. */
	uint32_t au_size_sectors;    /**< Allocation unit size in 512B sectors / preferred alignment. */
	uint32_t erase_sz;           /**< ERASE_SZ from SD Status (bits 423:408). */
	uint32_t erase_tmo;          /**< ERASE_TMO from SD Status (bits 407:402). */
	uint32_t erase_offs;	     /**< ERASE_OFFS from SD Status (bits 401:400). */
	uint8_t discard_support;     /**< DISCARD_SUPPORT from SD Status (bit b313). */
	uint32_t csd_max_sdclk_hz;   /**< Max SD clock derived from CSD timing fields. */
	uint32_t current_bus_clk_hz; /**< Currently configured bus clock. */
	uint8_t cid[CSD_REG_BSIZE];  /**< 16B CID – Card Identification register image. */
	uint8_t csd[CSD_REG_BSIZE];  /**< 16B CSD – Card Specific Data register image. */
	uint8_t manufacturer_id;     /**< MID – Manufacturer ID. */
	uint16_t oem_id;             /**< OID – OEM/Application ID. */
	char product_name[6];        /**< PNM – Product name (5 chars + '\0'). */
	uint8_t product_rev_major;   /**< PRV[7:4] – Major product revision. */
	uint8_t product_rev_minor;   /**< PRV[3:0] – Minor product revision. */
	uint32_t product_serial;     /**< PSN – Product serial number. */
	uint16_t manuf_date;         /**< MDT – Manufacturing date (raw encoded). */
};

/**
 * @brief Initialize the SDHC host controller hardware.
 *
 * Configures clocks, power and low-level HSMCI controller state so that
 * SD commands can be issued to a card. This function does not enumerate or
 * reset any card; it only prepares the host controller.
 *
 * Must be called once during system startup before sdhc_init_card().
 *
 * @retval 0             Host successfully initialized.
 * @retval negative errno  Initialization failed (clock, pinmux, etc.).
 */
int sdhc_init_host(void);

/**
 * @brief Reset and initialize the card, and read its parameters.
 *
 * Performs the SD card enumeration sequence on the currently attached card:
 * GO_IDLE, voltage check, ACMD41 negotiation, obtain CID/CSD, assign RCA,
 * select card and configure bus width / high-speed mode if supported.
 *
 * On success, provides a pointer to an internally owned sdhc_card_info
 * instance with stable lifetime.
 *
 * @param[out] card_info  Pointer to a variable that receives the address of
 *                        the internal sdhc_card_info structure.
 *
 * @retval 0              Card was successfully initialized.
 * @retval negative errno Card initialization failed or no card present.
 */
int sdhc_init_card(struct sdhc_card_info **card_info);

/**
 * @brief Reset internal card state and drop initialization flags.
 *
 * Clears internal bookkeeping related to the currently selected card and
 * marks the card as not initialized. Typically used after card removal,
 * card re-insertion or before re-running sdhc_init_card() to fully
 * re-enumerate the medium.
 */
void sdhc_reset_card(void);

/**
 * @brief Read one or more 512-byte logical sectors from the card.
 *
 * High-level wrapper around the low-level HSMCI block read API.
 * Works in logical sector units (LBA, 512-byte sectors) as required
 * by SDHC/SDXC cards. Intended primarily for use by the FatFS
 * disk I/O glue layer.
 *
 * The function is blocking.
 *
 * @param lba        Start logical sector address (0-based, 512-byte sectors).
 * @param sector_cnt Number of sectors to read (>= 1).
 * @param buf        Pointer to destination buffer. Buffer size must be at
 *                   least sector_cnt * 512 bytes.
 *
 * Returns: 0 on success; negative errno value on error.
 */
int sdhc_read_sectors(size_t lba, unsigned int sector_cnt, void *buf);

/**
 * @brief Write one or more 512-byte logical sectors to the card.
 *
 * High-level wrapper around the low-level HSMCI block write API.
 * Works in logical sector units (LBA, 512-byte sectors) as required
 * by SDHC/SDXC cards. Intended primarily for use by the FatFS
 * disk I/O glue layer.
 *
 * The function is blocking.
 *
 * @param lba        Start logical sector address (0-based, 512-byte sectors).
 * @param sector_cnt Number of sectors to write (>= 1).
 * @param buf        Pointer to source buffer. Buffer size must be at
 *                   least sector_cnt * 512 bytes.
 *
 * Returns: 0 on success; negative errno value on error.
 */
int sdhc_write_sectors(size_t lba, unsigned int sector_cnt, const void *buf);

/**
 * @brief Flush pending write operations and synchronize card state.
 *
 * Ensures that any previously issued write operations are completed
 * and the card is in a stable, non-busy state. For pure SDHC cards
 * without intermediate caching this may be a no-op, but the function
 * is provided to satisfy FatFS CTRL_SYNC semantics.
 *
 * Intended use: disk_ioctl(CTRL_SYNC).
 *
 * Returns: 0 on success; negative errno value on error.
 */
int sdhc_sync(void);

/**
 * @brief Get logical sector size in bytes.
 *
 * For SDHC/SDXC cards this is typically always 512 bytes.
 *
 * Intended use: disk_ioctl(GET_SECTOR_SIZE) when FF_MAX_SS != FF_MIN_SS.
 *
 * @param sector_size  Output pointer; filled with logical sector size in bytes.
 *
 * Returns: 0 on success; negative errno value on error.
 */
int sdhc_get_sector_size(uint32_t *sector_size);

/**
 * @brief Get total number of logical sectors on the card.
 *
 * The value is derived from CSD information parsed during card
 * initialization and corresponds to the addressable 512-byte sectors.
 *
 * Intended use: disk_ioctl(GET_SECTOR_COUNT).
 *
 * @param sector_count Output pointer; filled with number of logical sectors.
 *
 * Returns: 0 on success; negative errno value on error.
 */
int sdhc_get_sector_count(uint32_t *sector_count);

/**
 * @brief Get Allocation Unit (AU) size expressed in logical sectors.
 *
 * Obtains the card's AU size (preferred erase/alignment granularity) and
 * returns it as a count of 512-byte logical sectors. The value is derived
 * primarily from SD Status (AU_SIZE field) when available.
 *
 * This is not guaranteed to be the physical "atomic erase group" size.
 * It is intended as a practical alignment hint for higher layers.
 *
 * If the AU size cannot be determined, the function returns 1 as a safe
 * fallback.
 *
 * Intended use: disk_ioctl(GET_BLOCK_SIZE) for FatFS f_mkfs() and other tools
 * that want to align to the card's preferred erase/allocation boundaries.
 *
 * @param au_sectors Output pointer; filled with AU size in logical sectors.
 *
 * @return 0 on success; negative errno value on error.
 */
int sdhc_get_au_size_sectors(uint32_t *au_sectors);

/**
 * @brief Get current SDHC card flags.
 *
 * Returns a snapshot of the internal SDHC_CARD_FLAG_* bitmask describing
 * card presence, initialization state and high-speed / wide-bus status.
 *
 * Intended use: helper for disk_status() implementation.
 *
 * @param flags  Output pointer; filled with SDHC_CARD_FLAG_* bitmask.
 *
 * Returns: 0 on success; negative errno value on error.
 */
int sdhc_get_card_flags(unsigned int *flags);

/**
 * @brief Calculate SD erase timeout for a given number of Allocation Units.
 *
 * Uses raw SD Status fields (erase_sz, erase_tmo, erase_offs) stored in
 * sdhc_card_info. The computation follows the Linux SD erase-timeout model:
 *
 *   timeout_ms = per_au_ms * au_cnt + offset_ms
 *
 * where:
 *   per_au_ms  = (ERASE_TMO * 1000) / ERASE_SZ
 *   offset_ms  = ERASE_OFFS * 1000
 *
 * If the SD Status erase fields are not available, a conservative fallback
 * of 250 ms per AU is used. The result is clamped to a minimum of 1000 ms.
 *
 * Intended use: estimating cmd timeout for CMD38 erase of N AUs.
 *
 * @param au_cnt      Number of allocation units affected (>= 1).
 * @param timeout_ms  Output pointer; filled with timeout in milliseconds.
 *
 * @return 0 on success; negative errno value on error.
 */
int sdhc_calc_erase_timeout_ms(unsigned int au_cnt, unsigned int *timeout_ms);

/**
 * @brief Mark a continuous range of logical sectors as no longer used.
 *
 * Optional optimization used to inform the card/flash translation layer that a
 * given range of sectors is not needed anymore. The function is intended to be
 * mapped from FatFS disk_ioctl(CTRL_TRIM) when FF_USE_TRIM != 0.
 *
 * This implementation issues CMD32/CMD33/CMD38 (ERASE_WR_BLK_START/END/(ERASE|DISCARD)).
 *
 * @param lba        Start logical sector address (0-based, 512-byte sectors).
 * @param sector_cnt  Number of consecutive sectors to be trimmed (>= 1).
 *
 * @return 0 on success; negative errno value on error.
 */
int sdhc_trim(size_t lba, unsigned int sector_cnt);

#endif
