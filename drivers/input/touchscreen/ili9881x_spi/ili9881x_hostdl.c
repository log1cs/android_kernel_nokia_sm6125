/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ili9881x.h"

#define UPDATE_PASS		0
#define UPDATE_FAIL		-1
#define TIMEOUT_SECTOR		500
#define TIMEOUT_PAGE		3500
#define TIMEOUT_PROGRAM		10

static struct touch_fw_data {
	u8 block_number;
	u32 start_addr;
	u32 end_addr;
	u32 new_fw_cb;
	u32 dl_ges_sa;
	int delay_after_upgrade;
	bool isCRC;
	bool isboot;
	bool is80k;
	int hex_tag;
} tfd;

static struct flash_block_info {
	char *name;
	u32 start;
	u32 end;
	u32 len;
	u32 mem_start;
	u32 fix_mem_start;
	u8 mode;
} fbi[FW_BLOCK_INFO_NUM];

static u8 *pfw = NULL;
static u8 *CTPM_FW = NULL;

void ili_fw_read_flash_info(void)
{
        return;
}

int ili_fw_dump_flash_data(u32 start, u32 end, bool user)
{
        return 0;
}

void ili_flash_clear_dma(void)
{
        return;
}

void ili_flash_dma_write(u32 start, u32 end, u32 len)
{
        return;
}

static int CalculateCRC32(u32 start_addr, u32 len, u8 *pfw)
{
	int i = 0, j = 0;
	int crc_poly = 0x04C11DB7;
	int tmp_crc = 0xFFFFFFFF;

	for (i = start_addr; i < start_addr + len; i++) {
		tmp_crc ^= (pfw[i] << 24);

		for (j = 0; j < 8; j++) {
			if ((tmp_crc & 0x80000000) != 0)
				tmp_crc = tmp_crc << 1 ^ crc_poly;
			else
				tmp_crc = tmp_crc << 1;
		}
	}
	return tmp_crc;
}

static u32 HexToDec(char *phex, s32 len)
{
	u32 ret = 0, temp = 0, i;
	s32 shift = (len - 1) * 4;

	for (i = 0; i < len; shift -= 4, i++) {
		if ((phex[i] >= '0') && (phex[i] <= '9'))
			temp = phex[i] - '0';
		else if ((phex[i] >= 'a') && (phex[i] <= 'f'))
			temp = (phex[i] - 'a') + 10;
		else if ((phex[i] >= 'A') && (phex[i] <= 'F'))
			temp = (phex[i] - 'A') + 10;
		else
			return -1;

		ret |= (temp << shift);
	}
	return ret;
}

static int ilitek_tddi_fw_iram_read(u8 *buf, u32 start, int len)
{
	int i, limit = SPI_RX_BUF_SIZE;
	int addr = 0, end = len - 1;
	u8 cmd[4] = {0};

	if (!buf) {
		ILI_ERR("buf in null\n");
		return -ENOMEM;
	}

	for (i = 0, addr = start; addr < end; i += limit, addr += limit) {
		if ((addr + limit) > len)
			limit = end % limit;

		cmd[0] = 0x25;
		cmd[3] = (char)((addr & 0x00FF0000) >> 16);
		cmd[2] = (char)((addr & 0x0000FF00) >> 8);
		cmd[1] = (char)((addr & 0x000000FF));

		if (ili9881x_ts->wrapper(cmd, 4, NULL, 0, OFF) < 0) {
			ILI_ERR("Failed to write iram data\n");
			return -ENODEV;
		}

		if (ili9881x_ts->wrapper(NULL, 0, buf + i, limit, OFF) < 0) {
			ILI_ERR("Failed to Read iram data\n");
			return -ENODEV;
		}
	}
	return 0;
}

static int calc_hw_dma_crc(u32 start_addr, u32 block_size)
{
	int count = 50;
	u32 busy = 0;

	/* dma1 src1 address */
	if (ili_ice_mode_write(0x072104, start_addr, 4) < 0)
		ILI_ERR("Write dma1 src1 address failed\n");
	/* dma1 src1 format */
	if (ili_ice_mode_write(0x072108, 0x80000001, 4) < 0)
		ILI_ERR("Write dma1 src1 format failed\n");
	/* dma1 dest address */
	if (ili_ice_mode_write(0x072114, 0x0002725C, 4) < 0)
		ILI_ERR("Write dma1 src1 format failed\n");
	/* dma1 dest format */
	if (ili_ice_mode_write(0x072118, 0x80000000, 4) < 0)
		ILI_ERR("Write dma1 dest format failed\n");
	/* Block size*/
	if (ili_ice_mode_write(0x07211C, block_size, 4) < 0)
		ILI_ERR("Write block size (%d) failed\n", block_size);
	/* crc off */
	if (ili_ice_mode_write(0x041016, 0x00, 1) < 0)
		ILI_INFO("Write crc of failed\n");
	/* dma crc */
	if (ili_ice_mode_write(0x041017, 0x03, 1) < 0)
		ILI_ERR("Write dma 1 crc failed\n");
	/* crc on */
	if (ili_ice_mode_write(0x041016, 0x01, 1) < 0)
		ILI_ERR("Write crc on failed\n");
	/* Dma1 stop */
	if (ili_ice_mode_write(0x072100, 0x00000000, 4) < 0)
		ILI_ERR("Write dma1 stop failed\n");
	/* clr int */
	if (ili_ice_mode_write(0x048006, 0x2, 1) < 0)
		ILI_ERR("Write clr int failed\n");
	/* Dma1 start */
	if (ili_ice_mode_write(0x072100, 0x01000000, 4) < 0)
		ILI_ERR("Write dma1 start failed\n");

	/* Polling BIT0 */
	while (count > 0) {
		mdelay(1);
		if (ili_ice_mode_read(0x048006, &busy, sizeof(u8)) < 0)
			ILI_ERR("Read busy error\n");
		ILI_DBG("busy = %x\n", busy);
		if ((busy & 0x02) == 2)
			break;
		count--;
	}

	if (count <= 0) {
		ILI_ERR("BIT0 is busy\n");
		return -1;
	}

	if (ili_ice_mode_read(0x04101C, &busy, sizeof(u32)) < 0) {
		ILI_ERR("Read dma crc error\n");
		return -1;
	}

	return busy;
}

void ili_fw_dump_iram_data(u32 start, u32 end, bool save)
{
	struct file *f = NULL;
	mm_segment_t old_fs;
	loff_t pos = 0;
	int wdt, i;
	int len, tmp = debug_en;
	bool ice = atomic_read(&ili9881x_ts->ice_stat);

	if (!ice) {
		if (ili_ice_mode_ctrl(ENABLE, OFF) < 0) {
			ILI_ERR("Enable ice mode failed\n");
			return;
		}
	}

	wdt = ili_ic_watch_dog_ctrl(ILI_READ, DISABLE);
	if (wdt) {
		if (ili_ic_watch_dog_ctrl(ILI_WRITE, DISABLE) < 0)
			ILI_ERR("Disable WDT failed during dumping iram\n");
	}

	len = end - start + 1;

	if (len > MAX_HEX_FILE_SIZE) {
		ILI_ERR("len is larger than buffer, abort\n");
		goto out;
	}

	for (i = 0; i < MAX_HEX_FILE_SIZE; i++)
		ili9881x_ts->update_buf[i] = 0xFF;

	if (ilitek_tddi_fw_iram_read(ili9881x_ts->update_buf, start, len) < 0)
		ILI_ERR("Read IRAM data failed\n");

	if (save) {
		f = filp_open(DUMP_IRAM_PATH, O_WRONLY | O_CREAT | O_TRUNC, 644);
		if (ERR_ALLOC_MEM(f)) {
			ILI_ERR("Failed to open the file at %ld.\n", PTR_ERR(f));
			goto out;
		}

		old_fs = get_fs();
		set_fs(get_ds());
		set_fs(KERNEL_DS);
		pos = 0;
		vfs_write(f, ili9881x_ts->update_buf, len, &pos);
		set_fs(old_fs);
		filp_close(f, NULL);
		ILI_INFO("Save iram data to %s\n", DUMP_IRAM_PATH);
	} else {
		debug_en = DEBUG_ALL;
		ili_dump_data(ili9881x_ts->update_buf, 8, len, 0, "IRAM");
		debug_en = tmp;
	}

out:
	if (wdt) {
		if (ili_ic_watch_dog_ctrl(ILI_WRITE, ENABLE) < 0)
			ILI_ERR("Enable WDT failed during dumping iram\n");
	}

	if (!ice) {
		if (ili_ice_mode_ctrl(DISABLE, OFF) < 0)
			ILI_ERR("Enable ice mode failed after code reset\n");
	}

	ILI_INFO("dump iram data completed\n");
}

static int ilitek_tddi_fw_iram_program(u32 start, u8 *w_buf, u32 w_len, u32 split_len)
{
	int i = 0, j = 0, addr = 0;
	u32 end = start + w_len;

	for (i = 0; i < MAX_HEX_FILE_SIZE; i++)
		ili9881x_ts->update_buf[i] = 0xFF;

	if (split_len != 0) {
		for (addr = start, i = 0; addr < end; addr += split_len, i += split_len) {
			if ((addr + split_len) > end)
				split_len = end - addr;

			ili9881x_ts->update_buf[0] = SPI_WRITE;
			ili9881x_ts->update_buf[1] = 0x25;
			ili9881x_ts->update_buf[2] = (char)((addr & 0x000000FF));
			ili9881x_ts->update_buf[3] = (char)((addr & 0x0000FF00) >> 8);
			ili9881x_ts->update_buf[4] = (char)((addr & 0x00FF0000) >> 16);

			for (j = 0; j < split_len; j++)
				ili9881x_ts->update_buf[5 + j] = w_buf[i + j];

			if (ili9881x_ts->spi_write_then_read(ili9881x_ts->spi, ili9881x_ts->update_buf, split_len + 5, NULL, 0)) {
				ILI_ERR("Failed to write data via SPI in host download (%x)\n", split_len + 5);
				return -EIO;
			}
		}
	} else {
		ili9881x_ts->update_buf[0] = SPI_WRITE;
		ili9881x_ts->update_buf[1] = 0x25;
		ili9881x_ts->update_buf[2] = (char)((start & 0x000000FF));
		ili9881x_ts->update_buf[3] = (char)((start & 0x0000FF00) >> 8);
		ili9881x_ts->update_buf[4] = (char)((start & 0x00FF0000) >> 16);

		memcpy(&ili9881x_ts->update_buf[5], w_buf, w_len);

		/* It must be supported by platforms that have the ability to transfer all data at once. */
		if (ili9881x_ts->spi_write_then_read(ili9881x_ts->spi, ili9881x_ts->update_buf, w_len + 5, NULL, 0) < 0) {
			ILI_ERR("Failed to write data via SPI in host download (%x)\n", w_len + 5);
			return -EIO;
		}
	}
	return 0;
}

static int ilitek_tddi_fw_iram_upgrade(u8 *pfw)
{
	int i, ret = UPDATE_PASS;
	u32 mode, crc, dma;
	u8 *fw_ptr = NULL;

	if (!ili9881x_ts->ddi_rest_done) {
		if (ili9881x_ts->actual_tp_mode != P5_X_FW_GESTURE_MODE)
			ili_reset_ctrl(ili9881x_ts->reset);

		ret = ili_ice_mode_ctrl(ENABLE, OFF);
		if (ret < 0)
			return -EFW_ICE_MODE;
	} else {
		/* Restore it if the wq of load_fw_ddi has been called. */
		ili9881x_ts->ddi_rest_done = false;
	}

	if (ili9881x_ts->actual_tp_mode != P5_X_FW_GESTURE_MODE)
		ili9881x_ts->ignore_first_irq = false;
	else
		ili9881x_ts->ignore_first_irq = false;
	ret = ili_ic_watch_dog_ctrl(ILI_WRITE, DISABLE);
	if (ret < 0)
		return -EFW_WDT;

	if (ili9881x_ts->actual_tp_mode != P5_X_FW_GESTURE_MODE) {
		if (ili9881x_ts->chip->dma_reset) {
			ILI_INFO("operate dma reset in reg after tp reset\n");
			if (ili_ice_mode_write(0x40040, 0x00800000, 4) < 0)
				ILI_INFO("Failed to open DMA reset\n");
			if (ili_ice_mode_write(0x40040, 0x00000000, 4) < 0)
				ILI_INFO("Failed to close DMA reset\n");
		}
	}

	/* Point to pfw with different addresses for getting its block data. */
	if (ili9881x_ts->actual_tp_mode == P5_X_FW_TEST_MODE) {
		fw_ptr = pfw;
		mode = MP;
	} else if (ili9881x_ts->actual_tp_mode == P5_X_FW_GESTURE_MODE) {
		fw_ptr = &pfw[tfd.dl_ges_sa];
		mode = GESTURE;
	} else {
		fw_ptr = pfw;
		mode = AP;
	}

	/* Program data to iram acorrding to each block */
	for (i = 0; i < ARRAY_SIZE(fbi); i++) {
		if (fbi[i].mode == mode && fbi[i].len != 0) {
			ILI_INFO("Download %s code from hex 0x%x to IRAM 0x%x, len = 0x%x\n",
					fbi[i].name, fbi[i].start, fbi[i].mem_start, fbi[i].len);

#if SPI_DMA_TRANSFER_SPLIT
			if (ilitek_tddi_fw_iram_program(fbi[i].mem_start, (fw_ptr + fbi[i].start), fbi[i].len, SPI_UPGRADE_LEN) < 0)
				ILI_ERR("IRAM program failed\n");
#else
			if (ilitek_tddi_fw_iram_program(fbi[i].mem_start, (fw_ptr + fbi[i].start), fbi[i].len, 0) < 0)
				ILI_ERR("IRAM program failed\n");
#endif

			crc = CalculateCRC32(fbi[i].start, fbi[i].len - 4, fw_ptr);
			dma = calc_hw_dma_crc(fbi[i].mem_start, fbi[i].len - 4);

			ILI_INFO("%s CRC is %s (%x) : (%x)\n",
				fbi[i].name, (crc != dma ? "Invalid !" : "Correct !"), crc, dma);

			if (crc != dma) {
				ILI_ERR("CRC Error! print iram data with first 16 bytes\n");
				ili_fw_dump_iram_data(0x0, 0xF, false);
				return -EFW_CRC;
			}
			ili9881x_ts->fw_update_stat = FW_UPDATING;
		}
	}

	if (ili9881x_ts->actual_tp_mode != P5_X_FW_GESTURE_MODE) {
		if (ili_reset_ctrl(TP_IC_CODE_RST) < 0) {
			ILI_ERR("TP Code reset failed during iram programming\n");
			ret = -EFW_REST;
		}
	}

	if (ili_ice_mode_ctrl(DISABLE, OFF) < 0) {
		ILI_ERR("Disable ice mode failed after code reset\n");
		ret = -EFW_ICE_MODE;
	}

	/* Waiting for fw ready sending first cmd */
	if (!ili9881x_ts->info_from_hex || (ili9881x_ts->chip->core_ver < CORE_VER_1410))
		mdelay(100);

	return ret;
}

static int ilitek_fw_calc_file_crc(u8 *pfw)
{
	int i;
	u32 ex_addr, data_crc, file_crc;

	for (i = 0; i < ARRAY_SIZE(fbi); i++) {
		if (fbi[i].end == 0)
			continue;
		ex_addr = fbi[i].end;
		data_crc = CalculateCRC32(fbi[i].start, fbi[i].len - 4, pfw);
		file_crc = pfw[ex_addr - 3] << 24 | pfw[ex_addr - 2] << 16 | pfw[ex_addr - 1] << 8 | pfw[ex_addr];
		ILI_INFO("data crc = %x, file crc = %x\n", data_crc, file_crc);
		if (data_crc != file_crc) {
			ILI_ERR("Content of fw file is broken. (%d, %x, %x)\n",
				i, data_crc, file_crc);
			return -1;
		}
	}

	ILI_INFO("Content of fw file is correct\n");
	return 0;
}

static int ilitek_tddi_fw_ili_convert(u8 *pfw)
{
	int i, size, blk_num = 0, blk_map = 0, num;
	int b0_addr = 0, b0_num = 0;

	if (ERR_ALLOC_MEM(ili9881x_ts->md_fw_ili))
		return -ENOMEM;

	CTPM_FW = ili9881x_ts->md_fw_ili;
	size = ili9881x_ts->md_fw_ili_size;

	if (size < ILI_FILE_HEADER || size > MAX_HEX_FILE_SIZE) {
		ILI_ERR("size of ILI file is invalid\n");
		return -EINVAL;
	}

	/* Check if it's old version of ILI format. */
	if (CTPM_FW[22] == 0xFF && CTPM_FW[23] == 0xFF &&
		CTPM_FW[24] == 0xFF && CTPM_FW[25] == 0xFF) {
		ILI_ERR("Invaild ILI format, abort!\n");
		return -EINVAL;
	}

	blk_num = CTPM_FW[131];
	blk_map = (CTPM_FW[129] << 8) | CTPM_FW[130];
	ILI_INFO("Parsing ILI file, block num = %d, block mapping = %x\n", blk_num, blk_map);

	if (blk_num > (FW_BLOCK_INFO_NUM - 1) || !blk_num || !blk_map) {
		ILI_ERR("Number of block or block mapping is invalid, abort!\n");
		return -EINVAL;
	}

	memset(fbi, 0x0, sizeof(fbi));

	tfd.start_addr = 0;
	tfd.end_addr = 0;
	tfd.hex_tag = BLOCK_TAG_AF;

	/* Parsing block info */
	for (i = 0; i < FW_BLOCK_INFO_NUM; i++) {
		/* B0 tag */
		b0_addr = (CTPM_FW[4 + i * 4] << 16) | (CTPM_FW[5 + i * 4] << 8) | (CTPM_FW[6 + i * 4]);
		b0_num = CTPM_FW[7 + i * 4];
		if ((b0_num != 0) && (b0_addr != 0x000000))
			fbi[b0_num].fix_mem_start = b0_addr;

		/* AF tag */
		num = i + 1;
		if (((blk_map >> i) & 0x01) == 0x01) {
			fbi[num].start = (CTPM_FW[132 + i * 6] << 16) | (CTPM_FW[133 + i * 6] << 8) | CTPM_FW[134 + i * 6];
			fbi[num].end = (CTPM_FW[135 + i * 6] << 16) | (CTPM_FW[136 + i * 6] << 8) |  CTPM_FW[137 + i * 6];

			if (fbi[num].fix_mem_start == 0)
				fbi[num].fix_mem_start = INT_MAX;

			fbi[num].len = fbi[num].end - fbi[num].start + 1;
			ILI_INFO("Block[%d]: start_addr = %x, end = %x, fix_mem_start = 0x%x\n", num, fbi[num].start,
								fbi[num].end, fbi[num].fix_mem_start);
			if (num == GESTURE)
				ili9881x_ts->gesture_load_code = true;
		}
	}

	memcpy(pfw, CTPM_FW + ILI_FILE_HEADER, size - ILI_FILE_HEADER);

	if (ilitek_fw_calc_file_crc(pfw) < 0)
		return -1;

	tfd.block_number = blk_num;
	tfd.end_addr = size - ILI_FILE_HEADER;
	return 0;
}

static int ilitek_tddi_fw_hex_convert(u8 *phex, int size, u8 *pfw)
{
	int block = 0;
	u32 i = 0, j = 0, k = 0, num = 0;
	u32 len = 0, addr = 0, type = 0;
	u32 start_addr = 0x0, end_addr = 0x0, ex_addr = 0;
	u32 offset;

	memset(fbi, 0x0, sizeof(fbi));

	/* Parsing HEX file */
	for (; i < size;) {
		len = HexToDec(&phex[i + 1], 2);
		addr = HexToDec(&phex[i + 3], 4);
		type = HexToDec(&phex[i + 7], 2);

		if (type == 0x04) {
			ex_addr = HexToDec(&phex[i + 9], 4);
		} else if (type == 0x02) {
			ex_addr = HexToDec(&phex[i + 9], 4);
			ex_addr = ex_addr >> 12;
		} else if (type == BLOCK_TAG_AF) {
			/* insert block info extracted from hex */
			tfd.hex_tag = type;
			if (tfd.hex_tag == BLOCK_TAG_AF)
				num = HexToDec(&phex[i + 9 + 6 + 6], 2);
			else
				num = 0xFF;

			if (num > (FW_BLOCK_INFO_NUM - 1)) {
				ILI_ERR("ERROR! block num is larger than its define (%d, %d)\n",
						num, FW_BLOCK_INFO_NUM - 1);
				return -EINVAL;
			}

			fbi[num].start = HexToDec(&phex[i + 9], 6);
			fbi[num].end = HexToDec(&phex[i + 9 + 6], 6);
			fbi[num].fix_mem_start = INT_MAX;
			fbi[num].len = fbi[num].end - fbi[num].start + 1;
			ILI_INFO("Block[%d]: start_addr = %x, end = %x", num, fbi[num].start, fbi[num].end);

			if (num == GESTURE)
				ili9881x_ts->gesture_load_code = true;

			block++;
		} else if (type == BLOCK_TAG_B0 && tfd.hex_tag == BLOCK_TAG_AF) {
			num = HexToDec(&phex[i + 9 + 6], 2);

			if (num > (FW_BLOCK_INFO_NUM - 1)) {
				ILI_ERR("ERROR! block num is larger than its define (%d, %d)\n",
						num, FW_BLOCK_INFO_NUM - 1);
				return -EINVAL;
			}

			fbi[num].fix_mem_start = HexToDec(&phex[i + 9], 6);
			ILI_INFO("Tag 0xB0: change Block[%d] to addr = 0x%x\n", num, fbi[num].fix_mem_start);
		}

		addr = addr + (ex_addr << 16);

		if (phex[i + 1 + 2 + 4 + 2 + (len * 2) + 2] == 0x0D)
			offset = 2;
		else
			offset = 1;

		if (addr > MAX_HEX_FILE_SIZE) {
			ILI_ERR("Invalid hex format %d\n", addr);
			return -1;
		}

		if (type == 0x00) {
			end_addr = addr + len;
			if (addr < start_addr)
				start_addr = addr;
			/* fill data */
			for (j = 0, k = 0; j < (len * 2); j += 2, k++)
				pfw[addr + k] = HexToDec(&phex[i + 9 + j], 2);
		}
		i += 1 + 2 + 4 + 2 + (len * 2) + 2 + offset;
	}

	if (ilitek_fw_calc_file_crc(pfw) < 0)
		return -1;

	tfd.start_addr = start_addr;
	tfd.end_addr = end_addr;
	tfd.block_number = block;
	return 0;
}

static int ilitek_tdd_fw_hex_open(u8 op, u8 *pfw)
{
	int ret =0, fsize = 0;
	const struct firmware *fw = NULL;
	struct file *f = NULL;
	mm_segment_t old_fs;
	loff_t pos = 0;

	ILI_INFO("Open file method = %s, path = %s\n",
		op ? "FILP_OPEN" : "REQUEST_FIRMWARE",
		op ? ili9881x_ts->md_fw_filp_path : ili9881x_ts->md_fw_rq_path);

	switch (op) {
	case REQUEST_FIRMWARE:
		if (request_firmware(&fw, ili9881x_ts->md_fw_rq_path, ili9881x_ts->dev) < 0) {
			ILI_ERR("Request firmware failed, try again\n");
			if (request_firmware(&fw, ili9881x_ts->md_fw_rq_path, ili9881x_ts->dev) < 0) {
				ILI_ERR("Request firmware failed after retry\n");
				ret = -1;
				goto out;
			}
		}

		fsize = fw->size;
		ILI_INFO("fsize = %d\n", fsize);
		if (fsize <= 0) {
			ILI_ERR("The size of file is zero\n");
			release_firmware(fw);
			ret = -1;
			goto out;
		}

		ili9881x_ts->tp_fw.size = 0;
		ili9881x_ts->tp_fw.data = vmalloc(fsize);
		if (ERR_ALLOC_MEM(ili9881x_ts->tp_fw.data)) {
			ILI_ERR("Failed to allocate tp_fw by vmalloc, try again\n");
			ili9881x_ts->tp_fw.data = vmalloc(fsize);
			if (ERR_ALLOC_MEM(ili9881x_ts->tp_fw.data)) {
				ILI_ERR("Failed to allocate tp_fw after retry\n");
				release_firmware(fw);
				ret = -ENOMEM;
				goto out;
			}
		}

		/* Copy fw data got from request_firmware to global */
		ipio_memcpy((u8 *)ili9881x_ts->tp_fw.data, fw->data, fsize * sizeof(*fw->data), fsize);
		ili9881x_ts->tp_fw.size = fsize;
		release_firmware(fw);
		break;
	case FILP_OPEN:
		f = filp_open(ili9881x_ts->md_fw_filp_path, O_RDONLY, 0644);
		if (ERR_ALLOC_MEM(f)) {
			ILI_ERR("Failed to open the file at %ld, try to load it from tp_fw\n", PTR_ERR(f));
			ret = -1;
			goto out;
		}

		fsize = f->f_inode->i_size;
		ILI_INFO("fsize = %d\n", fsize);
		if (fsize <= 0) {
			ILI_ERR("The size of file is invaild, try to load it from tp_fw\n");
			filp_close(f, NULL);
			ret = -1;
			goto out;
		}

		ipio_vfree((void **) & (ili9881x_ts->tp_fw.data));
		ili9881x_ts->tp_fw.size = 0;
		ili9881x_ts->tp_fw.data = vmalloc(fsize);
		if (ERR_ALLOC_MEM(ili9881x_ts->tp_fw.data)) {
			ILI_ERR("Failed to allocate tp_fw by vmalloc, try again\n");
			ili9881x_ts->tp_fw.data = vmalloc(fsize);
			if (ERR_ALLOC_MEM(ili9881x_ts->tp_fw.data)) {
				ILI_ERR("Failed to allocate tp_fw after retry\n");
				filp_close(f, NULL);
				ret = -ENOMEM;
				goto out;
			}
		}

		/* ready to map user's memory to obtain data by reading files */
		old_fs = get_fs();
		set_fs(get_ds());
		set_fs(KERNEL_DS);
		pos = 0;
		vfs_read(f, (u8 *)ili9881x_ts->tp_fw.data, fsize, &pos);
		set_fs(old_fs);
		filp_close(f, NULL);
		ili9881x_ts->tp_fw.size = fsize;
		break;
	default:
		ILI_ERR("Unknown open file method, %d\n", op);
		break;
	}

	if (ERR_ALLOC_MEM(ili9881x_ts->tp_fw.data) || ili9881x_ts->tp_fw.size <= 0) {
		ILI_ERR("fw data/size is invaild\n");
		ret = -1;
		goto out;
	}

	/* Convert hex and copy data from tp_fw.data to pfw */
	if (ilitek_tddi_fw_hex_convert((u8 *)ili9881x_ts->tp_fw.data, ili9881x_ts->tp_fw.size, pfw) < 0) {
		ILI_ERR("Convert hex file failed\n");
		ret = -1;
	}

out:
	ipio_vfree((void **)&(ili9881x_ts->tp_fw.data));
	return ret;
}

static void ilitek_tddi_fw_update_block_info(u8 *pfw)
{
	u32 ges_area_section = 0, ges_info_addr = 0, ges_fw_start = 0, ges_fw_end = 0;
	u32 ap_end = 0, ap_len = 0;
	u32 fw_info_addr = 0;

        if (tfd.hex_tag == BLOCK_TAG_AF) {
                fbi[AP].mem_start = (fbi[AP].fix_mem_start != INT_MAX) ? fbi[AP].fix_mem_start : 0;
                fbi[DATA].mem_start = (fbi[DATA].fix_mem_start != INT_MAX) ? fbi[DATA].fix_mem_start : DLM_START_ADDRESS;
                fbi[TUNING].mem_start = (fbi[TUNING].fix_mem_start != INT_MAX) ? fbi[TUNING].fix_mem_start :  fbi[DATA].mem_start + fbi[DATA].len;
                fbi[MP].mem_start = (fbi[MP].fix_mem_start != INT_MAX) ? fbi[MP].fix_mem_start : 0;
                fbi[GESTURE].mem_start = (fbi[GESTURE].fix_mem_start != INT_MAX) ? fbi[GESTURE].fix_mem_start :	0;
                fbi[TAG].mem_start = (fbi[TAG].fix_mem_start != INT_MAX) ? fbi[TAG].fix_mem_start : 0;
                fbi[PARA_BACKUP].mem_start = (fbi[PARA_BACKUP].fix_mem_start != INT_MAX) ? fbi[PARA_BACKUP].fix_mem_start : 0;
                fbi[DDI].mem_start = (fbi[DDI].fix_mem_start != INT_MAX) ? fbi[DDI].fix_mem_start : 0;

                /* Parsing gesture info form AP code */
                ges_info_addr = (fbi[AP].end + 1 - 60);
                ges_area_section = (pfw[ges_info_addr + 3] << 24) + (pfw[ges_info_addr + 2] << 16) + (pfw[ges_info_addr + 1] << 8) + pfw[ges_info_addr];
                fbi[GESTURE].mem_start = (pfw[ges_info_addr + 7] << 24) + (pfw[ges_info_addr + 6] << 16) + (pfw[ges_info_addr + 5] << 8) + pfw[ges_info_addr + 4];
                ap_end = (pfw[ges_info_addr + 11] << 24) + (pfw[ges_info_addr + 10] << 16) + (pfw[ges_info_addr + 9] << 8) + pfw[ges_info_addr + 8];

		if (ap_end != fbi[GESTURE].mem_start)
			ap_len = ap_end - fbi[GESTURE].mem_start + 1;

                ges_fw_start = (pfw[ges_info_addr + 15] << 24) + (pfw[ges_info_addr + 14] << 16) + (pfw[ges_info_addr + 13] << 8) + pfw[ges_info_addr + 12];
                ges_fw_end = (pfw[ges_info_addr + 19] << 24) + (pfw[ges_info_addr + 18] << 16) + (pfw[ges_info_addr + 17] << 8) + pfw[ges_info_addr + 16];

		if (ges_fw_end != ges_fw_start)
			fbi[GESTURE].len = ges_fw_end - ges_fw_start + 1;

                fbi[GESTURE].start = 0;
        }

        ILI_INFO("==== Gesture loader info ====\n");
        ILI_INFO("ap_start = 0x%x, ap_end = 0x%x, ap_len = 0x%x\n", fbi[GESTURE].mem_start, ap_end, ap_len);
        ILI_INFO("gesture_start = 0x%x, gesture_end = 0x%x, gesture_len = 0x%x\n", ges_fw_start, ges_fw_end, fbi[GESTURE].len);
        ILI_INFO("=============================\n");

	/* update gesture address */
	tfd.dl_ges_sa = ges_fw_start;

        fbi[AP].name = "AP";
        fbi[DATA].name = "DATA";
        fbi[TUNING].name = "TUNING";
        fbi[MP].name = "MP";
        fbi[GESTURE].name = "GESTURE";
        fbi[TAG].name = "TAG";
        fbi[PARA_BACKUP].name = "PARA_BACKUP";
        fbi[DDI].name = "DDI";

        /* upgrade mode define */
        fbi[DATA].mode = fbi[AP].mode = fbi[TUNING].mode = AP;
        fbi[MP].mode = MP;
        fbi[GESTURE].mode = GESTURE;

	/* Copy fw info  */
	fw_info_addr = fbi[AP].end - INFO_HEX_ST_ADDR;
	ILI_INFO("Parsing hex info start addr = 0x%x\n", fw_info_addr);
	ipio_memcpy(ili9881x_ts->fw_info, (pfw + fw_info_addr), sizeof(ili9881x_ts->fw_info), sizeof(ili9881x_ts->fw_info));

	ili9881x_ts->trans_xy = ((ili9881x_ts->fw_info[68] << 16 | ili9881x_ts->fw_info[69] << 8 | ili9881x_ts->fw_info[70]) >= CORE_VER_1430) ? ili9881x_ts->fw_info[0] : OFF;
	ILI_INFO("Transfer touch coordinate = %s\n", ili9881x_ts->trans_xy ? "ON" : "OFF");

	if (fbi[AP].end > (64*K))
		tfd.is80k = true;

	/* Get hex fw vers */
	tfd.new_fw_cb = (ili9881x_ts->fw_info[48] << 24) | (ili9881x_ts->fw_info[49] << 16) |
			(ili9881x_ts->fw_info[50] << 8) | ili9881x_ts->fw_info[51];

	/* Calculate update address */
	ILI_INFO("New FW ver = 0x%x\n", tfd.new_fw_cb);
	ILI_INFO("star_addr = 0x%06X, end_addr = 0x%06X, Block Num = %d\n", tfd.start_addr, tfd.end_addr, tfd.block_number);
}

int ili_fw_upgrade(int op)
{
	int i, ret = 0, retry = 3;

	if (!ili9881x_ts->boot || ili9881x_ts->force_fw_update || ERR_ALLOC_MEM(pfw)) {
		ili9881x_ts->gesture_load_code = false;

		if (ERR_ALLOC_MEM(pfw)) {
			ipio_vfree((void **)&pfw);
			pfw = vmalloc(MAX_HEX_FILE_SIZE * sizeof(u8));
			if (ERR_ALLOC_MEM(pfw)) {
				ILI_ERR("Failed to allocate pfw memory, %ld\n", PTR_ERR(pfw));
				ipio_vfree((void **)&pfw);
				ret = -ENOMEM;
				goto out;
			}
		}

		for (i = 0; i < MAX_HEX_FILE_SIZE; i++)
			pfw[i] = 0xFF;

		if (ilitek_tdd_fw_hex_open(op, pfw) < 0) {
			ILI_ERR("Open hex file fail, try upgrade from ILI file\n");

			/*
			 * They might not be aware of a broken hex file if we recover
			 * fw from ILI file. We should force them to check
			 * hex files they attempt to update via device node.
			 */
			if (ili9881x_ts->node_update) {
				ILI_ERR("Ignore update from ILI file\n");
				ipio_vfree((void **)&pfw);
				return -EFW_CONVERT_FILE;
			}

			if (ilitek_tddi_fw_ili_convert(pfw) < 0) {
				ILI_ERR("Convert ILI file error\n");
				ret = -EFW_CONVERT_FILE;
				goto out;
			}
		}
		ilitek_tddi_fw_update_block_info(pfw);
	}

	do {
		ret = ilitek_tddi_fw_iram_upgrade(pfw);
		if (ret == UPDATE_PASS)
			break;

		ILI_ERR("Upgrade failed, do retry!\n");
	} while (--retry > 0);

	if (ret != UPDATE_PASS) {
		ILI_ERR("Failed to upgrade fw %d times, erasing iram\n", retry);
		if (ili_reset_ctrl(ili9881x_ts->reset) < 0)
				ILI_ERR("TP reset failed while erasing data\n");
		ili9881x_ts->xch_num = 0;
		ili9881x_ts->ych_num = 0;
	}

out:
	ili_ic_get_core_ver();
	ili_ic_get_protocl_ver();
	ili_ic_get_fw_ver();
	ili_ic_get_tp_info();
	ili_ic_get_panel_info();
	return ret;
}
