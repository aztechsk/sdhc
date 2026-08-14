/*
 * sdhc.c
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

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <gentyp.h>
#include "sysconf.h"
#include "msgconf.h"
#include "criterr.h"
#include "hwerr.h"
#include "hsmci_sd.h"
#include "sdhc.h"

#define SDHC_ACMD41_TMO_MS 3000
#define SDHC_ACMD41_RETRY_MS 20
#define HSMCI_400K_CLOCK 400000
#define SDHC_SECTOR_SIZE 512
#define SDHC_OCR_VOLTAGE_27_36 (OCR_VDD_27_28 | OCR_VDD_28_29 | \
				OCR_VDD_29_30 | OCR_VDD_30_31 | \
				OCR_VDD_31_32 | OCR_VDD_32_33 | \
				OCR_VDD_33_34 | OCR_VDD_34_35 | \
				OCR_VDD_35_36)
#define SDHC_CMD8_ARG 0x1AA

enum sdhc_state {
	SDHC_STATE_NO_CARD,
	SDHC_STATE_IDLE,
	SDHC_STATE_READY,
	SDHC_STATE_IDENT,
	SDHC_STATE_STANDBY,
	SDHC_STATE_TRANSFER,
	SDHC_STATE_ERROR
};

struct sdhc_ctx {
	enum sdhc_state state;
	struct sdhc_card_info card;
	uint8_t scr[SD_SCR_REG_BSIZE] __attribute__((aligned(4)));
	uint8_t sd_status[SD_STATUS_BSIZE] __attribute__((aligned(4)));
	uint8_t switch_status[SD_SW_STATUS_BSIZE] __attribute__((aligned(4)));
};

static struct sdhc_ctx ctx;

static int identify_and_read_csd(void);
static int set_bus_width(void);
static int read_scr(void);
static int go_transfer_state(void);
static int switch_high_speed(void);
static int read_sd_status(void);
static int get_status(unsigned int *stat);
static int parse_csd(void);
static void fill_card_info(void);
static void parse_sd_status(void);
static unsigned int csd_tran_speed_to_hz(unsigned int tran_speed);

/**
 * sdhc_init_host
 */
int sdhc_init_host(void)
{
	int ret = 0;

	init_hsmci();
	if ((ret = hsmci_send_clock())) {
		return (ret);
	}
	ctx.state = SDHC_STATE_NO_CARD;
	return (ret);
}

/**
 * sdhc_init_card
 */
int sdhc_init_card(struct sdhc_card_info **card_info)
{
	int ret = 0;
	hsmci_resp_t resp;
	TickType_t t0;
	uint32_t arg;

	memset(&ctx.card, 0, sizeof(ctx.card));
	ctx.card.type = SDHC_CARD_TYPE_UNKNOWN;
	ctx.card.flags = SDHC_CARD_FLAG_ERROR;
	if (card_info) {
		*card_info = &ctx.card;
	}
	// CMD0: GO_IDLE_STATE [no resp].
	if ((ret = hsmci_send_cmd(SDMMC_MCI_CMD0_GO_IDLE_STATE, 0, NULL))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD0 error\n");
		return (ret);
	}
	ctx.state = SDHC_STATE_IDLE;
	// CMD8: SEND_IF_COND [R7 48 bit].
	if ((ret = hsmci_send_cmd(SD_CMD8_SEND_IF_COND, SDHC_CMD8_ARG, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD8 error\n");
		return (ret);
	}
	// Check echo pattern.
	if ((resp.r1 & 0xFFF) != SDHC_CMD8_ARG) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD8 error response (echo)\n");
		return (-EHW);
	}
	ctx.card.flags |= SDHC_CARD_FLAG_PRESENT;
	t0 = xTaskGetTickCount();
	arg = SDHC_OCR_VOLTAGE_27_36 | SD_ACMD41_HCS;
	for (;;) {
		// CMD55: APP_CMD (RCA=0 in IDLE state) [R1 48 bit].
		if ((ret = hsmci_send_cmd(SDMMC_CMD55_APP_CMD, 0, &resp))) {
			ctx.state = SDHC_STATE_ERROR;
			msg(INF, "sdhc.c: CMD55 error\n");
			return (ret);
		}
		if (resp.r1 & (CARD_STATUS_ILLEGAL_COMMAND | CARD_STATUS_COM_CRC_ERROR)) {
			ctx.state = SDHC_STATE_ERROR;
			msg(INF, "sdhc.c: CMD55 error response\n");
			return (-EHW);
		}
		if (!(resp.r1 & CARD_STATUS_APP_CMD)) {
			ctx.state = SDHC_STATE_ERROR;
			msg(INF, "sdhc.c: CMD55 no app_cmd response\n");
			return (-EHW);
		}
		// ACMD41: SD_SEND_OP_COND [R3 OCR response].
		if ((ret = hsmci_send_cmd(SD_MCI_ACMD41_SD_SEND_OP_COND, arg, &resp))) {
			ctx.state = SDHC_STATE_ERROR;
			msg(INF, "sdhc.c: ACMD41 error\n");
			return (ret);
		}
		ctx.card.ocr = resp.r1;
		// CARD ready: POWER_UP_BUSY == 1.
		if (resp.r1 & OCR_POWER_UP_BUSY) {
			// CCS == 1 -> SDHC/SDXC.
			if (resp.r1 & OCR_CCS) {
				ctx.card.type = SDHC_CARD_TYPE_SDHC;
			} else {
				ctx.state = SDHC_STATE_ERROR;
				msg(INF, "sdhc.c: unsupported card type\n");
				return (-EHW);
			}
			ctx.state = SDHC_STATE_READY;
			if ((ret = identify_and_read_csd())) {
				return (ret);
			}
			if (ctx.card.capacity_bytes > (uint64_t) 32 * 1024 * 1024 * 1024) {
				ctx.card.type = SDHC_CARD_TYPE_SDXC;
			}
			fill_card_info();
			if ((ret = go_transfer_state())) {
				return (ret);
			}
			if ((ret = read_scr())) {
				return (ret);
			}
			if ((ret = set_bus_width())) {
				return (ret);
			}
			if ((ret = switch_high_speed())) {
				return (ret);
			}
			if ((ret = read_sd_status())) {
				return (ret);
			}
			parse_sd_status();
			ctx.card.flags |= SDHC_CARD_FLAG_INITIALIZED;
			ctx.card.flags &= ~SDHC_CARD_FLAG_ERROR;
			return (ret);
		}
		if ((xTaskGetTickCount() - t0) > ms_to_os_ticks(SDHC_ACMD41_TMO_MS)) {
			ctx.state = SDHC_STATE_ERROR;
			msg(INF, "sdhc.c: CMD55+ACMD41 POWER_UP_BUSY==1 timeout\n");
			return (-EHW);
		}
		vTaskDelay(ms_to_os_ticks(SDHC_ACMD41_RETRY_MS));
	}
}

/**
 * sdhc_reset_card
 */
void sdhc_reset_card(void)
{
	hsmci_resp_t resp;

	hsmci_send_cmd(SDMMC_CMD7_SELECT_CARD_CMD, 0, &resp);
	hsmci_set_clock(HSMCI_400K_CLOCK, NULL, FALSE);
	hsmci_disable_hspeed();
	hsmci_set_bus_width(HSMCI_BUS_WIDTH_1);
	hsmci_soft_reset();
	if (hsmci_send_clock() != 0) {
		msg(INF, "sdhc.c: hsmci_send_clock() error\n");
	}
	ctx.state = SDHC_STATE_NO_CARD;
	ctx.card.type = SDHC_CARD_TYPE_UNKNOWN;
	ctx.card.flags = 0;
	msg(INF, "sdhc.c: sdhc_reset_card() done\n");
}

/**
 * sdhc_read_sectors
 */
int sdhc_read_sectors(size_t lba, unsigned int sector_cnt, void *buf)
{
	int ret = 0;

	if (!sector_cnt) {
		return (0);
	}
	if (buf == NULL || sector_cnt > SDHC_MAX_RW_SECTORS_CNT) {
		crit_err_exit(BAD_PARAMETER);
	}
	if (!(ctx.card.flags & SDHC_CARD_FLAG_INITIALIZED) || ctx.state != SDHC_STATE_TRANSFER) {
		msg(INF, "sdhc.c: sdhc_read_sectors(): card not ready\n");
		return (-ENRDY);
	}
	if (lba >= ctx.card.sector_count || sector_cnt > (ctx.card.sector_count - lba)) {
		msg(INF, "sdhc.c: sdhc_read_sectors(): LBA out of range\n");
		return (-EADDR);
	}
	if ((ret = hsmci_read_blocks(lba, sector_cnt, buf))) {
		ctx.card.flags |= SDHC_CARD_FLAG_ERROR;
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: sdhc_read_sectors(): hsmci_read_blocks() error\n");
		return (ret);
	}
	return (ret);
}

/**
 * sdhc_write_sectors
 */
int sdhc_write_sectors(size_t lba, unsigned int sector_cnt, const void *buf)
{
	int ret = 0;

	if (!sector_cnt) {
		return (0);
	}
	if (buf == NULL || sector_cnt > SDHC_MAX_RW_SECTORS_CNT) {
		crit_err_exit(BAD_PARAMETER);
	}
	if (!(ctx.card.flags & SDHC_CARD_FLAG_INITIALIZED) || ctx.state != SDHC_STATE_TRANSFER) {
		msg(INF, "sdhc.c: sdhc_write_sectors(): card not ready\n");
		return (-ENRDY);
	}
	if (lba >= ctx.card.sector_count || sector_cnt > (ctx.card.sector_count - lba)) {
		msg(INF, "sdhc.c: sdhc_write_sectors(): LBA out of range\n");
		return (-EADDR);
	}
	if ((ret = hsmci_write_blocks(lba, sector_cnt, buf))) {
		ctx.card.flags |= SDHC_CARD_FLAG_ERROR;
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: sdhc_write_sectors(): hsmci_write_blocks() error\n");
		return (ret);
	}
	return (ret);
}

/**
 * sdhc_sync
 */
int sdhc_sync(void)
{
	int ret = 0;
	unsigned int stat;

	if (!(ctx.card.flags & SDHC_CARD_FLAG_INITIALIZED) || ctx.state != SDHC_STATE_TRANSFER) {
		msg(INF, "sdhc.c: sdhc_sync(): card not ready\n");
		return (-ENRDY);
	}
	if ((ret = get_status(&stat))) {
		return (ret);
	}
	if (!(stat & CARD_STATUS_READY_FOR_DATA) ||
	    (stat & CARD_STATUS_STATE) != CARD_STATUS_STATE_TRAN) {
		ctx.card.flags |= SDHC_CARD_FLAG_ERROR;
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: sdhc_sync(): card not ready error\n");
		return (-EHW);
	}
	return (ret);
}

/**
 * sdhc_get_sector_size
 */
int sdhc_get_sector_size(uint32_t *sector_size)
{
	if (!sector_size) {
		crit_err_exit(BAD_PARAMETER);
	}
	if (!(ctx.card.flags & SDHC_CARD_FLAG_INITIALIZED)) {
		return (-ENRDY);
	}
	*sector_size = ctx.card.sector_size;
	return (0);
}

/**
 * sdhc_get_sector_count
 */
int sdhc_get_sector_count(uint32_t *sector_count)
{
	if (!sector_count) {
		crit_err_exit(BAD_PARAMETER);
	}
	if (!(ctx.card.flags & SDHC_CARD_FLAG_INITIALIZED)) {
		return (-ENRDY);
	}
	*sector_count = ctx.card.sector_count;
	return (0);
}

/**
 * sdhc_get_au_size_sectors
 */
int sdhc_get_au_size_sectors(uint32_t *au_sectors)
{
	if (!au_sectors) {
		crit_err_exit(BAD_PARAMETER);
	}
	if (!(ctx.card.flags & SDHC_CARD_FLAG_INITIALIZED)) {
		return (-ENRDY);
	}
	*au_sectors = ctx.card.au_size_sectors;
	return (0);
}

/**
 * sdhc_get_card_flags
 */
int sdhc_get_card_flags(unsigned int *flags)
{
	if (!flags) {
		crit_err_exit(BAD_PARAMETER);
	}
	*flags = ctx.card.flags;
	return (0);
}

/**
 * sdhc_calc_erase_timeout_ms
 */
int sdhc_calc_erase_timeout_ms(unsigned int au_cnt, unsigned int *timeout_ms)
{
	uint64_t t64;

	if (!timeout_ms) {
		crit_err_exit(BAD_PARAMETER);
	}
	if (!(ctx.card.flags & SDHC_CARD_FLAG_INITIALIZED)) {
		return (-ENRDY);
	}
	if (au_cnt == 0) {
		*timeout_ms = 0;
		return (0);
	}
	if (ctx.card.erase_sz && ctx.card.erase_tmo) {
		unsigned int per_au_ms = ((uint64_t) ctx.card.erase_tmo * 1000) / ctx.card.erase_sz;
		unsigned int offs_ms = (uint64_t) ctx.card.erase_offs * 1000;
		t64 = (uint64_t) per_au_ms * au_cnt + offs_ms;
	} else {
		t64 = (uint64_t) 250 * au_cnt;
	}
	if (t64 < 1000) {
		t64 = 1000;
	}
	if (t64 > UINT_MAX) {
		t64 = UINT_MAX;
	}
	*timeout_ms = t64;
	return (0);
}

/**
 * sdhc_trim
 */
int sdhc_trim(size_t lba, unsigned int sector_cnt)
{
	hsmci_resp_t resp;
	unsigned int stat;
	size_t end_lba;
	size_t start_au;
	size_t end_au;
	unsigned int au_sz;
	unsigned int au_cnt;
	unsigned int tmo_ms;
	int ret = 0;

	if (!sector_cnt) {
		return (0);
	}
	if (!(ctx.card.flags & SDHC_CARD_FLAG_INITIALIZED) || ctx.state != SDHC_STATE_TRANSFER) {
		msg(INF, "sdhc.c: sdhc_trim(): card not ready\n");
		return (-ENRDY);
	}
	if (lba >= ctx.card.sector_count || sector_cnt > (ctx.card.sector_count - lba)) {
		msg(INF, "sdhc.c: sdhc_trim(): LBA out of range\n");
		return (-EADDR);
	}
	end_lba = lba + sector_cnt - 1;
	if ((ret = get_status(&stat))) {
		return (ret);
	}
	if (!(stat & CARD_STATUS_READY_FOR_DATA) ||
	    (stat & CARD_STATUS_STATE) != CARD_STATUS_STATE_TRAN) {
		ctx.card.flags |= SDHC_CARD_FLAG_ERROR;
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: sdhc_trim(): card not ready error\n");
		return (-EHW);
	}
	/* CMD32: ERASE_WR_BLK_START [R1 48 bit]. */
	if ((ret = hsmci_send_cmd(SD_CMD32_ERASE_WR_BLK_START, lba, &resp))) {
		ctx.card.flags |= SDHC_CARD_FLAG_ERROR;
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: sdhc_trim(): CMD32 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_COM_CRC_ERROR | CARD_STATUS_ERR_RD_WR)) {
		ctx.card.flags |= SDHC_CARD_FLAG_ERROR;
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: sdhc_trim(): CMD32 error response\n");
		return (-EHW);
	}
	/* CMD33: ERASE_WR_BLK_END (inclusive) [R1 48 bit]. */
	if ((ret = hsmci_send_cmd(SD_CMD33_ERASE_WR_BLK_END, end_lba, &resp))) {
		ctx.card.flags |= SDHC_CARD_FLAG_ERROR;
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: sdhc_trim(): CMD33 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_COM_CRC_ERROR | CARD_STATUS_ERR_RD_WR)) {
		ctx.card.flags |= SDHC_CARD_FLAG_ERROR;
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: sdhc_trim(): CMD33 error response\n");
		return (-EHW);
	}
	au_sz = ctx.card.au_size_sectors;
	if (au_sz == 0) {
		au_sz = 1;
	}
	start_au = lba / au_sz;
	end_au = end_lba / au_sz;
	au_cnt = end_au - start_au + 1;
	if (au_cnt == 0) {
		au_cnt = 1;
	}
	if ((ret = sdhc_calc_erase_timeout_ms(au_cnt, &tmo_ms))) {
		return (ret);
	}
	if (tmo_ms) {
		hsmci_set_next_r1b_busy_tmo_ms(tmo_ms);
	}
	/* CMD38: ERASE [R1B 48 bit]. */
	if ((ret = hsmci_send_cmd(SDMMC_CMD38_ERASE,
	    (ctx.card.flags & SDHC_CARD_FLAG_DISCARD_SUP) ? 1 : 0, &resp))) {
		ctx.card.flags |= SDHC_CARD_FLAG_ERROR;
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: sdhc_trim(): CMD38 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_COM_CRC_ERROR | CARD_STATUS_ERR_RD_WR)) {
		ctx.card.flags |= SDHC_CARD_FLAG_ERROR;
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: sdhc_trim(): CMD38 error response\n");
		return (-EHW);
	}
	return (0);
}

/**
 * identify_and_read_csd
 */
static int identify_and_read_csd(void)
{
	hsmci_resp_t resp;
	int ret = 0;

	// CMD2: ALL_SEND_CID [R2 136 bit].
	if ((ret = hsmci_send_cmd(SDMMC_CMD2_ALL_SEND_CID, 0, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD2 error\n");
		return (ret);
	}
	memcpy(ctx.card.cid, resp.r2, sizeof(ctx.card.cid));
	ctx.state = SDHC_STATE_IDENT;
	// CMD3: SEND_RELATIVE_ADDR [R6 48 bit].
	if ((ret = hsmci_send_cmd(SD_CMD3_SEND_RELATIVE_ADDR, 0, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD3 error\n");
		return (ret);
	}
	if (resp.r1 & SD_R6_STATUS_ERR_MASK) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD3 error response\n");
		return (-EHW);
	}
	ctx.state = SDHC_STATE_STANDBY;
	ctx.card.rca = SD_R6_GET_RCA(resp.r1);
	// CMD9: SEND_CSD [R2 136 bit].
	if ((ret = hsmci_send_cmd(SDMMC_MCI_CMD9_SEND_CSD, (uint32_t) ctx.card.rca << 16, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD9 error\n");
		return (ret);
	}
	memcpy(ctx.card.csd, resp.r2, sizeof(ctx.card.csd));
	if ((ret = parse_csd())) {
		ctx.state = SDHC_STATE_ERROR;
		return (ret);
	}
	return (ret);
}

/**
 * set_bus_width
 */
static int set_bus_width(void)
{
	hsmci_resp_t resp;
	uint32_t busw;
	int ret = 0;

#if HSMCI_SD_DLINE_NUM == 1
	return (ret);
#else
	busw = SD_SCR_SD_BUS_WIDTHS(ctx.scr);
	if (!(busw & SD_SCR_SD_BUS_WIDTH_4BITS)) {
		msg(INF, "sdhc.c: data_bus_width=1\n");
		return (0);
	}
	// CMD55: APP_CMD (RCA != 0 v STANDBY/TRANSFER) [R1 48 bit].
	if ((ret = hsmci_send_cmd(SDMMC_CMD55_APP_CMD, (uint32_t) ctx.card.rca << 16, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD55 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_ILLEGAL_COMMAND | CARD_STATUS_COM_CRC_ERROR)) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD55 error response\n");
		return (-EHW);
	}
	if (!(resp.r1 & CARD_STATUS_APP_CMD)) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD55 no app_cmd response\n");
		return (-EHW);
	}
	// ACMD6: SET_BUS_WIDTH [R1 48 bit].
	if ((ret = hsmci_send_cmd(SD_ACMD6_SET_BUS_WIDTH, 2, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: ACMD6 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_ILLEGAL_COMMAND | CARD_STATUS_COM_CRC_ERROR)) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: ACMD6 error response\n");
		return (-EHW);
	}
	hsmci_set_bus_width(HSMCI_BUS_WIDTH_4);
	msg(INF, "sdhc.c: data_bus_width=4\n");
	ctx.card.flags |= SDHC_CARD_FLAG_WIDE_BUS;
	return (ret);
#endif
}

/**
 * read_scr
 */
static int read_scr(void)
{
	hsmci_resp_t resp;
	int ret = 0;

	// CMD55: APP_CMD (RCA != 0 in STANDBY/TRANSFER) [R1 48 bit].
	if ((ret = hsmci_send_cmd(SDMMC_CMD55_APP_CMD, (uint32_t) ctx.card.rca << 16, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD55 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_ILLEGAL_COMMAND | CARD_STATUS_COM_CRC_ERROR)) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD55 error response\n");
		return (-EHW);
	}
	if (!(resp.r1 & CARD_STATUS_APP_CMD)) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD55 no app_cmd response\n");
		return (-EHW);
	}
	// ACMD51: SEND_SCR [R1 48 bit].
	if ((ret = hsmci_send_data_cmd(SD_ACMD51_SEND_SCR, 0, ctx.scr, SD_SCR_REG_BSIZE, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: ACMD51 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_COM_CRC_ERROR | CARD_STATUS_ERR_RD_WR)) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: ACMD51 error response\n");
		return (-EHW);
	}
	return (ret);
}

/**
 * go_transfer_state
 */
static int go_transfer_state(void)
{
	hsmci_resp_t resp;
	int ret = 0;
	unsigned int stat;

	// CMD7: SELECT_CARD_CMD [R1B 48 bit].
	if ((ret = hsmci_send_cmd(SDMMC_CMD7_SELECT_CARD_CMD, (uint32_t) ctx.card.rca << 16, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD7 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_ILLEGAL_COMMAND | CARD_STATUS_COM_CRC_ERROR)) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD7 error response\n");
		return (-EHW);
	}
	if ((ret = get_status(&stat))) {
		return (ret);
	}
	if ((stat & CARD_STATUS_STATE) != CARD_STATUS_STATE_TRAN) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD7 state error: CARD_STATUS_STATE != STATE_TRAN\n");
		return (-EHW);
	}
	ctx.state = SDHC_STATE_TRANSFER;
	return (ret);
}

/**
 * switch_high_speed
 */
static int switch_high_speed(void)
{
	hsmci_resp_t resp;
	uint32_t func_info;
	uint32_t func_rc;
	uint32_t arg;
	int ret = 0;
	unsigned int clk;

	arg = SD_CMD6_MODE_CHECK | SD_CMD6_GRP1_HIGH_SPEED | SD_CMD6_GRP2_NO_INFLUENCE |
	      SD_CMD6_GRP3_NO_INFLUENCE | SD_CMD6_GRP4_NO_INFLUENCE | SD_CMD6_GRP5_NO_INFLUENCE |
	      SD_CMD6_GRP6_NO_INFLUENCE;
	// CMD6: SWITCH_FUNC [R1 48 bit].
	if ((ret = hsmci_send_data_cmd(SD_CMD6_SWITCH_FUNC, arg, ctx.switch_status, SD_SW_STATUS_BSIZE, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD6 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_COM_CRC_ERROR | CARD_STATUS_ERR_RD_WR)) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD6 error response\n");
		return (-EHW);
	}
	func_info = SD_SW_STATUS_FUN_GRP1_INFO(ctx.switch_status);
	if (!(func_info & (1 << 1))) {
		hsmci_set_clock(ctx.card.csd_max_sdclk_hz, &clk, FALSE);
		ctx.card.current_bus_clk_hz = clk;
		msg(INF, "sdhc.c: bus_clock=%u hispeed_mode=off\n", clk);
		return (0);
	}
	arg = SD_CMD6_MODE_SWITCH | SD_CMD6_GRP1_HIGH_SPEED | SD_CMD6_GRP2_NO_INFLUENCE |
	      SD_CMD6_GRP3_NO_INFLUENCE | SD_CMD6_GRP4_NO_INFLUENCE | SD_CMD6_GRP5_NO_INFLUENCE |
	      SD_CMD6_GRP6_NO_INFLUENCE;
	// CMD6: SWITCH_FUNC [R1 48 bit].
	if ((ret = hsmci_send_data_cmd(SD_CMD6_SWITCH_FUNC, arg, ctx.switch_status, SD_SW_STATUS_BSIZE, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD6 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_COM_CRC_ERROR | CARD_STATUS_ERR_RD_WR | CARD_STATUS_SWITCH_ERROR)) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD6 error response\n");
		return (-EHW);
	}
	func_rc = SD_SW_STATUS_FUN_GRP1_RC(ctx.switch_status);
	if (func_rc == SD_SW_STATUS_FUN_GRP_RC_ERROR || func_rc != 1) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD6 switch error\n");
		return (-EHW);
	}
	hsmci_enable_hspeed();
	hsmci_set_clock(HSMCI_SD_HISPEED_CLK, &clk, TRUE);
	ctx.card.current_bus_clk_hz = clk;
	msg(INF, "sdhc.c: bus_clock=%u hispeed_mode=on\n", clk);
	ctx.card.flags |= SDHC_CARD_FLAG_HIGH_SPEED;
	return (ret);
}

/**
 * read_sd_status
 */
static int read_sd_status(void)
{
	hsmci_resp_t resp;
	int ret = 0;

	// CMD55: APP_CMD (RCA != 0 in STANDBY/TRANSFER) [R1 48 bit].
	if ((ret = hsmci_send_cmd(SDMMC_CMD55_APP_CMD, (uint32_t) ctx.card.rca << 16, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD55 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_ILLEGAL_COMMAND | CARD_STATUS_COM_CRC_ERROR)) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD55 error response\n");
		return (-EHW);
	}
	if (!(resp.r1 & CARD_STATUS_APP_CMD)) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD55 no app_cmd response\n");
		return (-EHW);
	}
	// ACMD13: SD_STATUS [R1 48 bit].
	if ((ret = hsmci_send_data_cmd(SD_ACMD13_SD_STATUS, 0, ctx.sd_status, SD_STATUS_BSIZE, &resp))) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: ACMD13 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_COM_CRC_ERROR | CARD_STATUS_ERR_RD_WR)) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: ACMD13 error response\n");
		return (-EHW);
	}
	return (ret);
}

/**
 * get_status
 */
static int get_status(unsigned int *stat)
{
	hsmci_resp_t resp;
	int ret = 0;

	// CMD13: SEND_STATUS (RCA != 0 in STANDBY/TRANSFER) [R1 48 bit].
	if ((ret = hsmci_send_cmd(SDMMC_MCI_CMD13_SEND_STATUS, (uint32_t) ctx.card.rca << 16, &resp))) {
		ctx.card.flags |= SDHC_CARD_FLAG_ERROR;
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD13 error\n");
		return (ret);
	}
	if (resp.r1 & (CARD_STATUS_ILLEGAL_COMMAND | CARD_STATUS_COM_CRC_ERROR)) {
		ctx.card.flags |= SDHC_CARD_FLAG_ERROR;
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: CMD13 error response\n");
		return (-EHW);
	}
	*stat = resp.r1;
	return (ret);
}

/**
 * parse_csd
 */
static int parse_csd(void)
{
	uint8_t *csd = ctx.card.csd;
	uint32_t csd_ver;
	uint32_t c_size;
	uint32_t tran_speed;
	uint64_t sector_count64;

	csd_ver = CSD_STRUCTURE_VERSION(csd);
	if (csd_ver == SD_CSD_VER_2_0) {
		c_size = SD_CSD_2_0_C_SIZE(csd);
		ctx.card.sector_size  = 512;
		sector_count64 = ((uint64_t) c_size + 1) * 1024;
		if (sector_count64 > UINT32_MAX) {
			ctx.card.sector_count = UINT32_MAX;
		} else {
			ctx.card.sector_count = sector_count64;
		}
	} else {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: unsupported card type\n");
		return (-EHW);
	}
	ctx.card.capacity_bytes = (uint64_t) ctx.card.sector_size * ctx.card.sector_count;
	tran_speed = CSD_TRAN_SPEED(csd);
	ctx.card.csd_max_sdclk_hz = csd_tran_speed_to_hz(tran_speed);
	if (ctx.card.csd_max_sdclk_hz == 0) {
		ctx.state = SDHC_STATE_ERROR;
		msg(INF, "sdhc.c: csd error max_bus_clk_hz=0\n");
		return (-EHW);
	}
	ctx.card.current_bus_clk_hz = 400000;
	return (0);
}

/**
 * fill_card_info
 */
static void fill_card_info(void)
{
	const uint8_t *cid = ctx.card.cid;
	char pnm[6];
	uint8_t mid;
	uint16_t oid;
	uint8_t prv, prv_major, prv_minor;
	uint32_t psn;
	uint16_t mdt_raw, mdt;
	uint8_t mdt_month;
	uint16_t mdt_year;
	uint64_t cap_bytes;
	uint32_t cap_mib;

	mid = cid[0];
	oid = ((uint32_t) cid[1] << 8) | cid[2];
	memcpy(pnm, &cid[3], 5);
	pnm[5] = '\0';
	prv = cid[8];
	prv_major = prv >> 4;
	prv_minor = prv & 0x0F;
	psn = ((uint32_t) cid[9] << 24) | ((uint32_t) cid[10] << 16) | ((uint32_t) cid[11] << 8) | (uint32_t) cid[12];
	mdt_raw = ((uint32_t) cid[13] << 8) | cid[14];
	mdt = mdt_raw & 0x0FFF;
	mdt_month = mdt & 0x0F;
	mdt_year = 2000 + (mdt >> 4);
	cap_bytes = ctx.card.capacity_bytes;
	cap_mib = (cap_bytes / (1024 * 1024));
	ctx.card.manufacturer_id = mid;
	ctx.card.oem_id = oid;
	strncpy(ctx.card.product_name, pnm, sizeof(ctx.card.product_name));
	ctx.card.product_rev_major = prv_major;
	ctx.card.product_rev_minor = prv_minor;
	ctx.card.product_serial = psn;
	ctx.card.manuf_date = mdt;
	UBaseType_t pr = uxTaskPriorityGet(NULL);
	vTaskPrioritySet(NULL, TASK_PRIO_HIGH);
	msg(INF, "sdhc.c: Card> SIZE=%u MiB, MID=0x%02X, OID=%c%c, PNM=\"%s\"\n", (unsigned int) cap_mib, (unsigned int) mid,
	    (char) (oid >> 8), (char) (oid & 0xFF), pnm);
	msg(INF, "sdhc.c: Card> PRV=%u.%u, PSN=%lu, MDT=%u-%02u\n", (unsigned int) prv_major, (unsigned int) prv_minor,
	    (unsigned long) psn, (unsigned int) mdt_year, (unsigned int) mdt_month);
	vTaskPrioritySet(NULL, pr);
}

/**
 * parse_sd_status
 */
static void parse_sd_status(void)
{
	unsigned int au_sz;
	uint8_t *ps = &ctx.sd_status[0];
	static const char *au_sz_str[] = {"ndef", "16 KiB", "32 KiB", "64 KiB", "128 KiB", "256 KiB",
					  "512 KiB", "1 MiB", "2 MiB", "4 MiB", "8 MiB", "12 MiB",
					  "16 MiB", "24 MiB", "32 MiB", "64 MiB"};
	static const unsigned int au_sz_bytes[] = {0, 16 * 1024, 32 * 1024, 64 * 1024, 128 * 1024, 256 * 1024,
						   512 * 1024, 1 * 1024 * 1024, 2 * 1024 * 1024, 4 * 1024 * 1024,
						   8 * 1024 * 1024, 12 * 1024 * 1024, 16 * 1024 * 1024,
						   24 * 1024 * 1024, 32 * 1024 * 1024, 64 * 1024 * 1024};
	au_sz = SDMMC_UNSTUFF_BITS(ps, 512, 428, 4);
	if (au_sz > 9 && !SD_SCR_SD_SPEC3(ctx.scr)) {
		au_sz = 0;
	}
	if (au_sz > sizeof(au_sz_str) / sizeof(char *) - 1) {
		au_sz = 0;
	}
	ctx.card.erase_sz = SDMMC_UNSTUFF_BITS(ps, 512, 408, 16);
	ctx.card.erase_tmo = SDMMC_UNSTUFF_BITS(ps, 512, 402, 6);
	ctx.card.erase_offs = SDMMC_UNSTUFF_BITS(ps, 512, 400, 2);
	ctx.card.discard_support = SDMMC_UNSTUFF_BITS(ps, 512, 313, 1);
	if (ctx.card.discard_support) {
		ctx.card.flags |= SDHC_CARD_FLAG_DISCARD_SUP;
	}
	ctx.card.au_size_sectors = au_sz_bytes[au_sz] / SDHC_SECTOR_SIZE;
	if (ctx.card.au_size_sectors == 0) {
		ctx.card.au_size_sectors = 1;
	}
	msg(INF, "sdhc.c: Card> AU_SZ=%s ERASE(SZ=%u TMO=%u OFFS=%u) DISCARD_SUP=%u\n",
	    au_sz_str[au_sz], ctx.card.erase_sz, ctx.card.erase_tmo, ctx.card.erase_offs,
	    ctx.card.discard_support);
}

/**
 * csd_tran_speed_to_hz
 */
static unsigned int csd_tran_speed_to_hz(unsigned int tran_speed)
{
	static const uint32_t unit_tab[] = {
		100000,    // 100 kbit/s.
		1000000,   //   1 Mbit/s.
		10000000,  //  10 Mbit/s.
		100000000  // 100 Mbit/s.
	};
	static const uint8_t mult_x10[] = {0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80};
	unsigned int unit = tran_speed & 0x07;
	unsigned int tv = (tran_speed >> 3) & 0x0F;
	uint64_t hz;
	if (unit > 3 || tv == 0) {
		return (0);
	}
	hz = (uint64_t) unit_tab[unit] * mult_x10[tv];
	return (hz /= 10);
}
