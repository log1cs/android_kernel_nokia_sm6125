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

#include "firmware/ili9881x_fw.h"
#include "ili9881x.h"

/* Debug level */
bool debug_en = DEBUG_OUTPUT;
EXPORT_SYMBOL(debug_en);

static struct workqueue_struct *esd_wq;
static struct workqueue_struct *bat_wq;
static struct delayed_work esd_work;
static struct delayed_work bat_work;

#if RESUME_BY_DDI
static struct workqueue_struct	*resume_by_ddi_wq = NULL;
static struct work_struct	resume_by_ddi_work;

static void ilitek_resume_by_ddi_work(struct work_struct *work)
{
	mutex_lock(&ili9881x_ts->touch_mutex);

	if (ili9881x_ts->gesture)
		disable_irq_wake(ili9881x_ts->irq_num);

	/* Set tp as demo mode and reload code if it's iram. */
	ili9881x_ts->actual_tp_mode = P5_X_FW_AP_MODE;
	if (ili9881x_ts->fw_upgrade_mode == UPGRADE_IRAM)
		ili_fw_upgrade_handler(NULL);
	else
		ili_reset_ctrl(ili9881x_ts->reset);

	ili_irq_enable();
	ILI_INFO("TP resume end by wq\n");
	ili_wq_ctrl(WQ_ESD, ENABLE);
	ili_wq_ctrl(WQ_BAT, ENABLE);
	mutex_unlock(&ili9881x_ts->touch_mutex);
}

void ili_resume_by_ddi(void)
{
	if (!resume_by_ddi_wq) {
		ILI_INFO("resume_by_ddi_wq is null\n");
		return;
	}

	mutex_lock(&ili9881x_ts->touch_mutex);

	ILI_INFO("TP resume start called by ddi\n");
	ili9881x_ts->tp_suspend = false;
	ili9881x_ts->skip_wake = true;

	/*
	 * To match the timing of sleep out, the first of mipi cmd must be sent within 10ms
	 * after TP reset. Because of that, we create a wq doing host download for resume.
	 */
	atomic_set(&ili9881x_ts->fw_stat, ENABLE);
	ili_reset_ctrl(ili9881x_ts->reset);
	ili_ice_mode_ctrl(ENABLE, OFF);
	ili9881x_ts->ddi_rest_done = true;
	ili9881x_ts->resume_by_ddi = true;
	mdelay(5);
	queue_work(resume_by_ddi_wq, &(resume_by_ddi_work));

	mutex_unlock(&ili9881x_ts->touch_mutex);
}

#endif

int ili_mp_test_handler(char *apk, bool lcm_on)
{
	int ret = 0;

	if (atomic_read(&ili9881x_ts->fw_stat)) {
		ILI_ERR("fw upgrade processing, ignore\n");
		return 0;
	}

	atomic_set(&ili9881x_ts->mp_stat, ENABLE);

	if (ili9881x_ts->actual_tp_mode != P5_X_FW_TEST_MODE) {
		ret = ili_switch_tp_mode(P5_X_FW_TEST_MODE);
		if (ret < 0) {
			ILI_ERR("Switch MP mode failed\n");
			goto out;
		}
	}
	ili9881x_ts->i2c_cmd_int = ON;
	ret = ili_mp_test_main(apk, lcm_on);
	ili9881x_ts->i2c_cmd_int = OFF;

out:
	/*
	 * If there's running mp test with lcm off, we suspose that
	 * users will soon call resume from suspend. TP mode will be changed
	 * from MP to AP mode until resume finished.
	 */
	if (!lcm_on) {
		atomic_set(&ili9881x_ts->mp_stat, DISABLE);
		return ret;
	}

	ili9881x_ts->actual_tp_mode = P5_X_FW_AP_MODE;
	if (ili9881x_ts->fw_upgrade_mode == UPGRADE_IRAM) {
		if (ili_fw_upgrade_handler(NULL) < 0)
			ILI_ERR("FW upgrade failed during mp test\n");
	} else {
		if (ili_reset_ctrl(ili9881x_ts->reset) < 0)
			ILI_ERR("TP Reset failed during mp test\n");
	}

	atomic_set(&ili9881x_ts->mp_stat, DISABLE);
	return ret;
}

int ili_switch_tp_mode(u8 mode)
{
	int ret = 0;

	atomic_set(&ili9881x_ts->tp_sw_mode, START);

	ili9881x_ts->actual_tp_mode = mode;

	switch (ili9881x_ts->actual_tp_mode) {
	case P5_X_FW_AP_MODE:
		ILI_INFO("Switch to AP mode\n");
		if (ili9881x_ts->fw_upgrade_mode == UPGRADE_IRAM) {
			if (ili_fw_upgrade_handler(NULL) < 0)
				ILI_ERR("FW upgrade failed\n");
		} else {
			ret = ili_reset_ctrl(ili9881x_ts->reset);
		}
		if (ret < 0)
			ILI_ERR("TP Reset failed\n");

		break;
	case P5_X_FW_GESTURE_MODE:
		ret = ili9881x_ts->gesture_move_code(ili9881x_ts->gesture_mode);
		if (ret < 0)
			ILI_ERR("Move gesture code failed\n");
		break;
	case P5_X_FW_TEST_MODE:
		ILI_INFO("Switch to Test mode\n");
		ret = ili9881x_ts->mp_move_code();
		break;
	default:
		ILI_ERR("Unknown TP mode: %x\n", mode);
		ret = -1;
		break;
	}

	if (ret < 0)
		ILI_ERR("Switch TP mode (%d) failed \n", mode);

	ILI_DBG("Actual TP mode = %d\n", ili9881x_ts->actual_tp_mode);
	atomic_set(&ili9881x_ts->tp_sw_mode, END);
	return ret;
}

int ili_gesture_recovery(void)
{
	int ret = 0;
	bool lock = mutex_is_locked(&ili9881x_ts->touch_mutex);

	atomic_set(&ili9881x_ts->esd_stat, START);

	if (!lock)
		mutex_lock(&ili9881x_ts->touch_mutex);

	ILI_INFO("Doing gesture recovery\n");
	//ili9881x_ts->force_fw_update = ENABLE;
	ret = ili9881x_ts->ges_recover();
	//ili9881x_ts->force_fw_update = DISABLE;

	if (!lock)
		mutex_unlock(&ili9881x_ts->touch_mutex);

	atomic_set(&ili9881x_ts->esd_stat, END);
	return ret;
}

void ili_spi_recovery(void)
{
	bool lock = mutex_is_locked(&ili9881x_ts->touch_mutex);

	atomic_set(&ili9881x_ts->esd_stat, START);

	if (!lock)
		mutex_lock(&ili9881x_ts->touch_mutex);

	ILI_INFO("Doing spi recovery\n");
	//ili9881x_ts->force_fw_update = ENABLE;
	if (ili_fw_upgrade_handler(NULL) < 0)
		ILI_ERR("FW upgrade failed\n");
	//ili9881x_ts->force_fw_update = DISABLE;

	if (!lock)
		mutex_unlock(&ili9881x_ts->touch_mutex);

	atomic_set(&ili9881x_ts->esd_stat, END);
}

int ili_wq_esd_spi_check(void)
{
	int ret = 0;
	u8 tx = SPI_WRITE, rx = 0;

	ret = ili9881x_ts->spi_write_then_read(ili9881x_ts->spi, &tx, 1, &rx, 1);
	ILI_DBG("spi esd check = 0x%x\n", rx);
	if (ret == DO_SPI_RECOVER) {
		ILI_ERR("rx = 0x%x\n", rx);
		return -1;
	}
	return 0;
}

int ili_wq_esd_i2c_check(void)
{
	ILI_DBG("");
	return 0;
}

static void ilitek_tddi_wq_esd_check(struct work_struct *work)
{
	if (ili9881x_ts->esd_recover() < 0) {
		ILI_ERR("SPI ACK failed, doing spi recovery\n");
		ili_spi_recovery();
	}
	complete_all(&ili9881x_ts->esd_done);
	ili_wq_ctrl(WQ_ESD, ENABLE);
}

static int read_power_status(u8 *buf)
{
	struct file *f = NULL;
	mm_segment_t old_fs;
	ssize_t byte = 0;

	old_fs = get_fs();
	set_fs(get_ds());

	f = filp_open(POWER_STATUS_PATH, O_RDONLY, 0);
	if (ERR_ALLOC_MEM(f)) {
		ILI_ERR("Failed to open %s\n", POWER_STATUS_PATH);
		return -1;
	}

	f->f_op->llseek(f, 0, SEEK_SET);
	byte = f->f_op->read(f, buf, 20, &f->f_pos);

	ILI_DBG("Read %d bytes\n", (int)byte);

	set_fs(old_fs);
	filp_close(f, NULL);
	return 0;
}

static void ilitek_tddi_wq_bat_check(struct work_struct *work)
{
	u8 str[20] = {0};
	static int charge_mode;

	if (read_power_status(str) < 0)
		ILI_ERR("Read power status failed\n");

	ILI_DBG("Batter Status: %s\n", str);

	if (strstr(str, "Charging") != NULL || strstr(str, "Full") != NULL
		|| strstr(str, "Fully charged") != NULL) {
		if (charge_mode != 1) {
			ILI_DBG("Charging mode\n");
			if (ili_ic_func_ctrl("plug", DISABLE) < 0) // plug in
				ILI_ERR("Write plug in failed\n");
			charge_mode = 1;
		}
	} else {
		if (charge_mode != 2) {
			ILI_DBG("Not charging mode\n");
			if (ili_ic_func_ctrl("plug", ENABLE) < 0) // plug out
				ILI_ERR("Write plug out failed\n");
			charge_mode = 2;
		}
	}
	ili_wq_ctrl(WQ_BAT, ENABLE);
}

void ili_wq_ctrl(int type, int ctrl)
{
	switch (type) {
	case WQ_ESD:
		if (ENABLE_WQ_ESD || ili9881x_ts->wq_ctrl) {
			if (!esd_wq) {
				ILI_ERR("wq esd is null\n");
				break;
			}
			ili9881x_ts->wq_esd_ctrl = ctrl;
			if (ctrl == ENABLE) {
				ILI_DBG("execute esd check\n");
				if (!queue_delayed_work(esd_wq, &esd_work, msecs_to_jiffies(WQ_ESD_DELAY)))
					ILI_DBG("esd check was already on queue\n");
			} else {
				cancel_delayed_work_sync(&esd_work);
				flush_workqueue(esd_wq);
				ILI_DBG("cancel esd wq\n");
			}
		}
		break;
	case WQ_BAT:
		if (ENABLE_WQ_BAT || ili9881x_ts->wq_ctrl) {
			if (!bat_wq) {
				ILI_ERR("WQ BAT is null\n");
				break;
			}
			ili9881x_ts->wq_bat_ctrl = ctrl;
			if (ctrl == ENABLE) {
				ILI_DBG("execute bat check\n");
				if (!queue_delayed_work(bat_wq, &bat_work, msecs_to_jiffies(WQ_BAT_DELAY)))
					ILI_DBG("bat check was already on queue\n");
			} else {
				cancel_delayed_work_sync(&bat_work);
				flush_workqueue(bat_wq);
				ILI_DBG("cancel bat wq\n");
			}
		}
		break;
	default:
		ILI_ERR("Unknown WQ type, %d\n", type);
		break;
	}
}

static void ilitek_tddi_wq_init(void)
{
	esd_wq = alloc_workqueue("esd_check", WQ_MEM_RECLAIM, 0);
	bat_wq = alloc_workqueue("bat_check", WQ_MEM_RECLAIM, 0);

	WARN_ON(!esd_wq);
	WARN_ON(!bat_wq);

	INIT_DELAYED_WORK(&esd_work, ilitek_tddi_wq_esd_check);
	INIT_DELAYED_WORK(&bat_work, ilitek_tddi_wq_bat_check);

#if RESUME_BY_DDI
	resume_by_ddi_wq = create_singlethread_workqueue("resume_by_ddi_wq");
	WARN_ON(!resume_by_ddi_wq);
	INIT_WORK(&resume_by_ddi_work, ilitek_resume_by_ddi_work);
#endif
}

int ili_sleep_handler(int mode)
{
	int ret = 0;
	bool sense_stop = true;

	mutex_lock(&ili9881x_ts->touch_mutex);
	atomic_set(&ili9881x_ts->tp_sleep, START);

	if (atomic_read(&ili9881x_ts->fw_stat) ||
		atomic_read(&ili9881x_ts->mp_stat)) {
		ILI_INFO("fw upgrade or mp still running, ignore sleep requst\n");
		atomic_set(&ili9881x_ts->tp_sleep, END);
		mutex_unlock(&ili9881x_ts->touch_mutex);
		return 0;
	}

	ili_wq_ctrl(WQ_ESD, DISABLE);
	ili_wq_ctrl(WQ_BAT, DISABLE);
	ili_irq_disable();

	ILI_INFO("Sleep Mode = %d\n", mode);

	if (ili9881x_ts->ss_ctrl)
		sense_stop = true;
	else if ((ili9881x_ts->chip->core_ver >= CORE_VER_1430))
		sense_stop = false;
	else
		sense_stop = true;

	switch (mode) {
	case TP_SUSPEND:
		ILI_INFO("TP suspend start\n");
		if (sense_stop) {
			if (ili_ic_func_ctrl("sense", DISABLE) < 0)
				ILI_ERR("Write sense stop cmd failed\n");

			if (ili_ic_check_busy(50, 20) < 0)
				ILI_ERR("Check busy timeout during suspend\n");
		}

		if (ili9881x_ts->gesture) {
			if (ili9881x_ts->tp_data_format == DATA_FORMAT_DEBUG) {
				ILI_INFO("Enable gesture debug mode\n");
				ili9881x_ts->gesture_debug = ENABLE;
			}
			ili_switch_tp_mode(P5_X_FW_GESTURE_MODE);
			enable_irq_wake(ili9881x_ts->irq_num);
			ili_irq_enable();
		} else {
			if (ili_ic_func_ctrl("sleep", DEEP_SLEEP_IN) < 0)
				ILI_ERR("Write sleep in cmd failed\n");
		}
		ILI_INFO("TP suspend end\n");
		ili9881x_ts->tp_suspend = true;
		ili9881x_ts->skip_wake = false;
		break;
	case TP_DEEP_SLEEP:
		ILI_INFO("TP deep suspend start\n");
		if (sense_stop) {
			if (ili_ic_func_ctrl("sense", DISABLE) < 0)
				ILI_ERR("Write sense stop cmd failed\n");

			if (ili_ic_check_busy(50, 20) < 0)
				ILI_ERR("Check busy timeout during deep suspend\n");
		}

		if (ili9881x_ts->gesture) {
			ili_switch_tp_mode(P5_X_FW_GESTURE_MODE);
			enable_irq_wake(ili9881x_ts->irq_num);
			ili_irq_enable();
		} else {
			if (ili_ic_func_ctrl("sleep", DEEP_SLEEP_IN) < 0)
				ILI_ERR("Write deep sleep in cmd failed\n");
		}
		ILI_INFO("TP deep suspend end\n");
		ili9881x_ts->tp_suspend = true;
		ili9881x_ts->skip_wake = false;
		break;
	case TP_RESUME:
		if (!ili9881x_ts->resume_by_ddi) {
			ILI_INFO("TP resume start\n");
			ili9881x_ts->tp_suspend = false;
			ili9881x_ts->skip_wake = true;
			if (ili9881x_ts->gesture)
				disable_irq_wake(ili9881x_ts->irq_num);

			/* Set tp as demo mode and reload code if it's iram. */
			ili9881x_ts->actual_tp_mode = P5_X_FW_AP_MODE;
			if (ili9881x_ts->fw_upgrade_mode == UPGRADE_IRAM) {
				if (ili_fw_upgrade_handler(NULL) < 0)
					ILI_ERR("FW upgrade failed during resume\n");
			} else {
				if (ili_reset_ctrl(ili9881x_ts->reset) < 0)
					ILI_ERR("TP Reset failed during resume\n");
			}
			ili_wq_ctrl(WQ_ESD, ENABLE);
			ili_wq_ctrl(WQ_BAT, ENABLE);
			ILI_INFO("TP resume end\n");
		}
		ili_irq_enable();
		break;
	default:
		ILI_ERR("Unknown sleep mode, %d\n", mode);
		ret = -EINVAL;
		break;
	}

	ili_touch_release_all_point();
	atomic_set(&ili9881x_ts->tp_sleep, END);
	mutex_unlock(&ili9881x_ts->touch_mutex);
	return ret;
}

int ili_fw_upgrade_handler(void *data)
{
	int ret = 0;

	atomic_set(&ili9881x_ts->fw_stat, START);

	ili9881x_ts->fw_update_stat = FW_STAT_INIT;
	ret = ili_fw_upgrade(ili9881x_ts->fw_open);
	if (ret != 0) {
		ILI_INFO("FW upgrade fail\n");
		ili9881x_ts->fw_update_stat = FW_UPDATE_FAIL;
	} else {
		ILI_INFO("FW upgrade pass\n");
		ili9881x_ts->fw_update_stat = FW_UPDATE_PASS;
	}

	if (!ili9881x_ts->boot) {
		ili9881x_ts->boot = true;
		ILI_INFO("Registre touch to input subsystem\n");
		ili_input_register();
		ili_wq_ctrl(WQ_ESD, ENABLE);
		ili_wq_ctrl(WQ_BAT, ENABLE);
	}

	atomic_set(&ili9881x_ts->fw_stat, END);
	return ret;
}

int ili_set_tp_data_len(int format)
{
	u8 cmd[2] = {0}, ctrl = 0;
	u16 self_key = 2;
	int ret = 0;

	switch (format) {
	case DATA_FORMAT_DEMO:
		ili9881x_ts->tp_data_len = P5_X_DEMO_MODE_PACKET_LEN;
		ctrl = DATA_FORMAT_DEMO_CMD;
		break;
	case DATA_FORMAT_DEBUG:
		ili9881x_ts->tp_data_len = (2 * ili9881x_ts->xch_num * ili9881x_ts->ych_num) + (ili9881x_ts->stx * 2) + (ili9881x_ts->srx * 2);
		ili9881x_ts->tp_data_len += 2 * self_key + (8 * 2) + 1 + 35;
		ctrl = DATA_FORMAT_DEBUG_CMD;
		break;
	case DATA_FORMAT_DEMO_DEBUG_INFO:
		/*only suport SPI interface now, so defult use size 1024 buffer*/
		ili9881x_ts->tp_data_len = P5_X_DEMO_MODE_PACKET_LEN +
			P5_X_DEMO_DEBUG_INFO_ID0_LENGTH + P5_X_INFO_HEADER_LENGTH;
		ctrl = DATA_FORMAT_DEMO_DEBUG_INFO_CMD;
		break;
	case DATA_FORMAT_GESTURE_INFO:
		ili9881x_ts->tp_data_len = P5_X_GESTURE_INFO_LENGTH;
		ctrl = DATA_FORMAT_GESTURE_INFO_CMD;
		break;
	case DATA_FORMAT_GESTURE_NORMAL:
		ili9881x_ts->tp_data_len = P5_X_GESTURE_NORMAL_LENGTH;
		ctrl = DATA_FORMAT_GESTURE_NORMAL_CMD;
		break;
	case DATA_FORMAT_GESTURE_DEMO:
		if (ili9881x_ts->gesture_demo_ctrl == ENABLE) {
			if(ili9881x_ts->gesture_mode == DATA_FORMAT_GESTURE_INFO)
				ili9881x_ts->tp_data_len = P5_X_GESTURE_INFO_LENGTH + P5_X_INFO_HEADER_LENGTH + P5_X_INFO_CHECKSUM_LENGTH;
			else
				ili9881x_ts->tp_data_len = P5_X_DEMO_MODE_PACKET_LEN + P5_X_INFO_HEADER_LENGTH + P5_X_INFO_CHECKSUM_LENGTH;
		} else {
			if(ili9881x_ts->gesture_mode == DATA_FORMAT_GESTURE_INFO)
				ili9881x_ts->tp_data_len = P5_X_GESTURE_INFO_LENGTH;
			else
				ili9881x_ts->tp_data_len = P5_X_GESTURE_NORMAL_LENGTH;
		}
		ILI_INFO("Gesture demo mode control = %d\n",  ili9881x_ts->gesture_demo_ctrl);
		ili_ic_func_ctrl("gesture_demo_en", ili9881x_ts->gesture_demo_ctrl);
		ILI_INFO("knock_en setting\n");
		ili_ic_func_ctrl("knock_en", 0x8);
		break;
	default:
		ILI_ERR("Unknow TP data format\n");
		return -1;
	}

	ili9881x_ts->tp_data_format = format;
	ILI_INFO("Set TP data format = %d, len = %d\n", ili9881x_ts->tp_data_format, ili9881x_ts->tp_data_len);

	if (ili9881x_ts->actual_tp_mode == P5_X_FW_AP_MODE) {
		ili9881x_ts->tp_data_format = format;
		cmd[0] = P5_X_MODE_CONTROL;
		cmd[1] = ctrl;
		ret = ili9881x_ts->wrapper(cmd, 2, NULL, 0, ON);

		if (ret < 0) {
			ILI_ERR("switch to format %d failed\n", format);
			ili_switch_tp_mode(P5_X_FW_AP_MODE);
		}
	} else if (ili9881x_ts->actual_tp_mode == P5_X_FW_GESTURE_MODE) {
		ret = ili_ic_func_ctrl("lpwg", ctrl);
		if (ret < 0)
			ILI_ERR("write gesture mode failed\n");
	}

	return ret;
}

void ili_report_handler(void)
{
	int ret = 0, pid = 0;
	u8  checksum = 0;
	u8 *trdata = NULL;
	int rlen = 0;
	int tmp = debug_en;

	/* Just in case these stats couldn't be blocked in top half context */
	if (!ili9881x_ts->report || atomic_read(&ili9881x_ts->tp_reset) ||
		atomic_read(&ili9881x_ts->fw_stat) || atomic_read(&ili9881x_ts->tp_sw_mode) ||
		atomic_read(&ili9881x_ts->mp_stat) || atomic_read(&ili9881x_ts->tp_sleep)) {
		ILI_INFO("ignore report request\n");
		return;
	}

	if (ili9881x_ts->irq_after_recovery) {
		ILI_INFO("ignore int triggered by recovery\n");
		ili9881x_ts->irq_after_recovery = false;
		return;
	}

	ili_wq_ctrl(WQ_ESD, DISABLE);
	ili_wq_ctrl(WQ_BAT, DISABLE);

	if (ili9881x_ts->actual_tp_mode == P5_X_FW_GESTURE_MODE) {
		if (ili9881x_ts->gesture_debug) {
			if (ili_set_tp_data_len(DATA_FORMAT_DEBUG) < 0)
				ILI_ERR("Failed to set tp data length\n");
		}
		__pm_stay_awake(ili9881x_ts->ws);
		mdelay(40); //Waiting for pm resume completed
	}

	rlen = ili9881x_ts->tp_data_len;
	ILI_DBG("Packget length = %d\n", rlen);

	if (!rlen || rlen > TR_BUF_SIZE) {
		ILI_ERR("Length of packet is invaild\n");
		goto out;
	}

	memset(ili9881x_ts->tr_buf, 0x0, TR_BUF_SIZE);

	ret = ili9881x_ts->wrapper(NULL, 0, ili9881x_ts->tr_buf, rlen, OFF);
	if (ret < 0) {
		ILI_ERR("Read report packet failed, ret = %d\n", ret);
		if (ret == DO_SPI_RECOVER) {
			ili_ic_get_pc_counter_forwdt();
			if (ili9881x_ts->actual_tp_mode == P5_X_FW_GESTURE_MODE && ili9881x_ts->gesture && !ili9881x_ts->prox_near) {
				ILI_ERR("Gesture failed, doing gesture recovery\n");
				ili9881x_ts->irq_after_recovery = true;
				if (ili_gesture_recovery() < 0)
					ILI_ERR("Failed to recover gesture\n");
				ili9881x_ts->irq_after_recovery = false;
			} else {
				ILI_ERR("SPI ACK failed, doing spi recovery\n");
				ili9881x_ts->irq_after_recovery = true;
				ili_spi_recovery();
				ili9881x_ts->irq_after_recovery = false;
			}
		}
		goto out;
	}

	ili_dump_data(ili9881x_ts->tr_buf, 8, rlen, 0, "finger report");

	checksum = ili_calc_packet_checksum(ili9881x_ts->tr_buf, rlen - 1);

	if (checksum != ili9881x_ts->tr_buf[rlen-1] && ili9881x_ts->fw_uart_en == DISABLE) {
		ILI_ERR("Wrong checksum, checksum = %x, buf = %x, len = %d\n", checksum, ili9881x_ts->tr_buf[rlen-1], rlen);
		debug_en = DEBUG_ALL;
		ili_dump_data(ili9881x_ts->tr_buf, 8, rlen, 0, "finger report with wrong");
		debug_en = tmp;
		goto out;
	}

	pid = ili9881x_ts->tr_buf[0];
	ILI_DBG("Packet ID = %x\n", pid);
	trdata = ili9881x_ts->tr_buf;
	if (pid == P5_X_INFO_HEADER_PACKET_ID) {
		trdata = ili9881x_ts->tr_buf + P5_X_INFO_HEADER_LENGTH;
		pid = trdata[0];

	}

	switch (pid) {
	case P5_X_DEMO_PACKET_ID:
		ili_report_ap_mode(trdata, rlen);
		break;
	case P5_X_DEBUG_PACKET_ID:
		ili_report_debug_mode(trdata, rlen);
		break;
	case P5_X_I2CUART_PACKET_ID:
		ili_report_i2cuart_mode(trdata, rlen);
		break;
	case P5_X_GESTURE_PACKET_ID:
		ili_report_gesture_mode(trdata, rlen);
		break;
	case P5_X_GESTURE_FAIL_ID:
		ILI_INFO("gesture fail reason code = 0x%02x", trdata[1]);
		break;
	case P5_X_DEMO_DEBUG_INFO_PACKET_ID:
		ili_demo_debug_info_mode(trdata, rlen);
		break;
	default:
		ILI_ERR("Unknown packet id, %x\n", pid);
		break;
	}

out:
	if (ili9881x_ts->actual_tp_mode != P5_X_FW_GESTURE_MODE) {
		ili_wq_ctrl(WQ_ESD, ENABLE);
		ili_wq_ctrl(WQ_BAT, ENABLE);
	}

	if (ili9881x_ts->actual_tp_mode == P5_X_FW_GESTURE_MODE)
		__pm_relax(ili9881x_ts->ws);
}

int ili_reset_ctrl(int mode)
{
	int ret = 0;

	atomic_set(&ili9881x_ts->tp_reset, START);
	ili9881x_ts->skip_wake = true;

	if (mode != TP_IC_CODE_RST)
		ili_ic_check_otp_prog_mode();

	switch (mode) {
	case TP_IC_CODE_RST:
		ILI_INFO("TP IC Code RST \n");
		ret = ili_ic_code_reset();
		if (ret < 0)
			ILI_ERR("IC Code reset failed\n");
		break;
	case TP_IC_WHOLE_RST:
		ILI_INFO("TP IC whole RST\n");
		ret = ili_ic_whole_reset();
		if (ret < 0)
			ILI_ERR("IC whole reset failed\n");
		break;
	case TP_HW_RST_ONLY:
		ILI_INFO("TP HW RST\n");
		ili_tp_reset();
		break;
	default:
		ILI_ERR("Unknown reset mode, %d\n", mode);
		ret = -EINVAL;
		break;
	}

	/*
	 * Since OTP must be folloing with reset, except for code rest,
	 * the stat of ice mode should be set as 0.
	 */
	if (mode != TP_IC_CODE_RST)
		atomic_set(&ili9881x_ts->ice_stat, DISABLE);
	//ili9881x_ts->fw_uart_en = DISABLE;
	ili9881x_ts->tp_data_format = DATA_FORMAT_DEMO;
	ili9881x_ts->tp_data_len = P5_X_DEMO_MODE_PACKET_LEN;
	atomic_set(&ili9881x_ts->tp_reset, END);
	return ret;
}

static int ilitek_get_tp_module(void)
{
	/*
	 * TODO: users should implement this function
	 * if there are various tp modules been used in projects.
	 */

	return 0;
}

static void ili_update_tp_module_info(void)
{
	int module;

	module = ilitek_get_tp_module();

	switch(module) {
	case MODEL_CSOT:
		ili9881x_ts->md_name = "CSOT";
		ili9881x_ts->md_fw_filp_path = CSOT_FW_FILP_PATH;
		ili9881x_ts->md_fw_rq_path = CSOT_FW_REQUEST_PATH;
		ili9881x_ts->md_ini_path = CSOT_INI_NAME_PATH;
		ili9881x_ts->md_ini_rq_path = CSOT_INI_REQUEST_PATH;
		ili9881x_ts->md_fw_ili = CTPM_FW_CSOT;
		ili9881x_ts->md_fw_ili_size = sizeof(CTPM_FW_CSOT);
		break;
	case MODEL_AUO:
		ili9881x_ts->md_name = "AUO";
		ili9881x_ts->md_fw_filp_path = AUO_FW_FILP_PATH;
		ili9881x_ts->md_fw_rq_path = AUO_FW_REQUEST_PATH;
		ili9881x_ts->md_ini_path = AUO_INI_NAME_PATH;
		ili9881x_ts->md_ini_rq_path = AUO_INI_REQUEST_PATH;
		ili9881x_ts->md_fw_ili = CTPM_FW_AUO;
		ili9881x_ts->md_fw_ili_size = sizeof(CTPM_FW_AUO);
		break;
	case MODEL_BOE:
		ili9881x_ts->md_name = "BOE";
		ili9881x_ts->md_fw_filp_path = BOE_FW_FILP_PATH;
		ili9881x_ts->md_fw_rq_path = BOE_FW_REQUEST_PATH;
		ili9881x_ts->md_ini_path = BOE_INI_NAME_PATH;
		ili9881x_ts->md_ini_rq_path = BOE_INI_REQUEST_PATH;
		ili9881x_ts->md_fw_ili = CTPM_FW_BOE;
		ili9881x_ts->md_fw_ili_size = sizeof(CTPM_FW_BOE);
		break;
	case MODEL_INX:
		ili9881x_ts->md_name = "INX";
		ili9881x_ts->md_fw_filp_path = INX_FW_FILP_PATH;
		ili9881x_ts->md_fw_rq_path = INX_FW_REQUEST_PATH;
		ili9881x_ts->md_ini_path = INX_INI_NAME_PATH;
		ili9881x_ts->md_ini_rq_path = INX_INI_REQUEST_PATH;
		ili9881x_ts->md_fw_ili = CTPM_FW_INX;
		ili9881x_ts->md_fw_ili_size = sizeof(CTPM_FW_INX);
		break;
	case MODEL_DJ:
		ili9881x_ts->md_name = "DJ";
		ili9881x_ts->md_fw_filp_path = DJ_FW_FILP_PATH;
		ili9881x_ts->md_fw_rq_path = DJ_FW_REQUEST_PATH;
		ili9881x_ts->md_ini_path = DJ_INI_NAME_PATH;
		ili9881x_ts->md_ini_rq_path = DJ_INI_REQUEST_PATH;
		ili9881x_ts->md_fw_ili = CTPM_FW_DJ;
		ili9881x_ts->md_fw_ili_size = sizeof(CTPM_FW_DJ);
		break;
	case MODEL_TXD:
		ili9881x_ts->md_name = "TXD";
		ili9881x_ts->md_fw_filp_path = TXD_FW_FILP_PATH;
		ili9881x_ts->md_fw_rq_path = TXD_FW_REQUEST_PATH;
		ili9881x_ts->md_ini_path = TXD_INI_NAME_PATH;
		ili9881x_ts->md_ini_rq_path = TXD_FW_REQUEST_PATH;
		ili9881x_ts->md_fw_ili = CTPM_FW_TXD;
		ili9881x_ts->md_fw_ili_size = sizeof(CTPM_FW_TXD);
		break;
	case MODEL_TM:
		ili9881x_ts->md_name = "TM";
		ili9881x_ts->md_fw_filp_path = TM_FW_REQUEST_PATH;
		ili9881x_ts->md_fw_rq_path = TM_FW_REQUEST_PATH;
		ili9881x_ts->md_ini_path = TM_INI_NAME_PATH;
		ili9881x_ts->md_ini_rq_path = TM_INI_REQUEST_PATH;
		ili9881x_ts->md_fw_ili = CTPM_FW_TM;
		ili9881x_ts->md_fw_ili_size = sizeof(CTPM_FW_TM);
		break;
	default:
		break;
	}

	if (module == 0 || ili9881x_ts->md_fw_ili_size < ILI_FILE_HEADER) {
		ILI_ERR("Couldn't find any tp modules, applying default settings\n");
		ili9881x_ts->md_name = "DEF";
		ili9881x_ts->md_fw_filp_path = DEF_FW_FILP_PATH;
		ili9881x_ts->md_fw_rq_path = DEF_FW_REQUEST_PATH;
		ili9881x_ts->md_ini_path = DEF_INI_NAME_PATH;
		ili9881x_ts->md_ini_rq_path = DEF_INI_REQUEST_PATH;
		ili9881x_ts->md_fw_ili = CTPM_FW_DEF;
		ili9881x_ts->md_fw_ili_size = sizeof(CTPM_FW_DEF);
	}

	ILI_INFO("Found %s module: ini path = %s, fw path = (%s, %s, %d)\n",
			ili9881x_ts->md_name,
			ili9881x_ts->md_ini_path,
			ili9881x_ts->md_fw_filp_path,
			ili9881x_ts->md_fw_rq_path,
			ili9881x_ts->md_fw_ili_size);

	ili9881x_ts->tp_module = module;
}

bool ili9881x_reset_keep_high(void){
	return true;
}

int ili_tddi_init(void)
{
#if BOOT_FW_UPDATE
	struct task_struct *fw_boot_th;
#endif

	ILI_INFO("driver version = %s\n", DRIVER_VERSION);

	mutex_init(&ili9881x_ts->touch_mutex);
	mutex_init(&ili9881x_ts->debug_mutex);
	mutex_init(&ili9881x_ts->debug_read_mutex);
	init_waitqueue_head(&(ili9881x_ts->inq));
	spin_lock_init(&ili9881x_ts->irq_spin);
	init_completion(&ili9881x_ts->esd_done);

	atomic_set(&ili9881x_ts->irq_stat, DISABLE);
	atomic_set(&ili9881x_ts->ice_stat, DISABLE);
	atomic_set(&ili9881x_ts->tp_reset, END);
	atomic_set(&ili9881x_ts->fw_stat, END);
	atomic_set(&ili9881x_ts->mp_stat, DISABLE);
	atomic_set(&ili9881x_ts->tp_sleep, END);
	atomic_set(&ili9881x_ts->cmd_int_check, DISABLE);
	atomic_set(&ili9881x_ts->esd_stat, END);

	ili_ic_init();
	ilitek_tddi_wq_init();

	/* Must do hw reset once in first time for work normally if tp reset is avaliable */
	if (!TDDI_RST_BIND)
		if (ili_reset_ctrl(ili9881x_ts->reset) < 0)
			ILI_ERR("TP Reset failed during init\n");

	ili9881x_ts->do_otp_check = ENABLE;
	ili9881x_ts->fw_uart_en = DISABLE;
	ili9881x_ts->force_fw_update = DISABLE;
	ili9881x_ts->demo_debug_info[0] = ili_demo_debug_info_id0;
	ili9881x_ts->tp_data_format = DATA_FORMAT_DEMO;
	ili9881x_ts->boot = false;

	/*
	 * This status of ice enable will be reset until process of fw upgrade runs.
	 * it might cause unknown problems if we disable ice mode without any
	 * codes inside touch ic.
	 */
	if (ili_ice_mode_ctrl(ENABLE, OFF) < 0)
		ILI_ERR("Failed to enable ice mode failed during init\n");

	if (ili_ic_get_info() < 0) {
		ILI_ERR("Not found ilitek chips\n");
		return -ENODEV;
	}

	ili_update_tp_module_info();

	ili_node_init();

	ili_fw_read_flash_info();

#if BOOT_FW_UPDATE
	fw_boot_th = kthread_run(ili_fw_upgrade_handler, NULL, "ili_fw_boot");
	if (fw_boot_th == (struct task_struct *)ERR_PTR) {
		fw_boot_th = NULL;
		WARN_ON(!fw_boot_th);
		ILI_ERR("Failed to create fw upgrade thread\n");
	}
#else
	if (ili_ice_mode_ctrl(DISABLE, OFF) < 0)
		ILI_ERR("Failed to disable ice mode failed during init\n");

#if (TDDI_INTERFACE == BUS_I2C)
	ili9881x_ts->info_from_hex = DISABLE;
#endif

	ili_ic_get_core_ver();
	ili_ic_get_protocl_ver();
	ili_ic_get_fw_ver();
	ili_ic_get_tp_info();
	ili_ic_get_panel_info();

#if (TDDI_INTERFACE == BUS_I2C)
	ili9881x_ts->info_from_hex = ENABLE;
#endif

	ILI_INFO("Registre touch to input subsystem\n");
	ili_input_register();
	ili_wq_ctrl(WQ_ESD, ENABLE);
	ili_wq_ctrl(WQ_BAT, ENABLE);
	ili9881x_ts->boot = true;
#endif

	ili9881x_ts->ws = wakeup_source_register(NULL, "ili_wakelock");
	if (!ili9881x_ts->ws)
		ILI_ERR("wakeup source request failed\n");
	
#if ENABLE_GESTURE
	ili9881x_ts->gesture = DISABLE;
	if (init_lct_tp_gesture(ili9881x_tp_gesture_callback) < 0) {
		ILI_ERR("init_lct_tp_gesture Failed!\n");
	}
#endif

	return 0;
}

void ili_dev_remove(void)
{
	ILI_INFO("remove ilitek dev\n");

	if (!ili9881x_ts)
		return;

	gpio_free(ili9881x_ts->tp_int);
	gpio_free(ili9881x_ts->tp_rst);

	if (esd_wq != NULL) {
		cancel_delayed_work_sync(&esd_work);
		flush_workqueue(esd_wq);
		destroy_workqueue(esd_wq);
	}
	if (bat_wq != NULL) {
		cancel_delayed_work_sync(&bat_work);
		flush_workqueue(bat_wq);
		destroy_workqueue(bat_wq);
	}

	if (ili9881x_ts->ws)
		wakeup_source_unregister(ili9881x_ts->ws);

	kfree(ili9881x_ts->tr_buf);
	kfree(ili9881x_ts->gcoord);
	ili_interface_dev_exit(ili9881x_ts);
}

int ili_dev_init(struct ilitek_hwif_info *hwif)
{
	ILI_INFO("TP Interface: %s\n", (hwif->bus_type == BUS_I2C) ? "I2C" : "SPI");
	return ili_interface_dev_init(hwif);
}
