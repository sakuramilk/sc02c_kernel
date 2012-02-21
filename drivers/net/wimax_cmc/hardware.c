/*
 * hardware.c
 *
 * gpio control functions (power on/off, init/deinit gpios)
 * data tx and tx thread implemented.
 */
#include "headers.h"
#include "download.h"
#include "wimax_plat.h"

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/hardware.h>
#include <plat/sdhci.h>
#include <plat/devs.h>
#include <linux/spinlock.h>

#define WIMAX_POWER_SUCCESS		0
#define WIMAX_ALREADY_POWER_ON		-1
#define WIMAX_PROBE_FAIL		-2
#define WIMAX_ALREADY_POWER_OFF		-3

static void wimax_hostwake_task(unsigned long data)
{
	struct net_adapter    *adapter = (struct net_adapter *)data;
	struct wimax_cfg *g_cfg = adapter->pdata->g_cfg;

	wake_lock_timeout(&g_cfg->wimax_wake_lock, 1 * HZ);
}

static irqreturn_t wimax_hostwake_isr(int irq, void *dev)
{
	struct net_adapter *adapter = (struct net_adapter *)dev;
	tasklet_schedule(&adapter->hostwake_task);
	return IRQ_HANDLED;
}
static int cmc732_setup_wake_irq(struct net_adapter *adapter)
{
	int rc = -EIO;
	int irq;

	rc = gpio_request(WIMAX_INT, "gpio_wimax_int");
	if (rc < 0) {
		dump_debug("%s: gpio %d request failed (%d)\n",
			__func__, WIMAX_INT, rc);
		return rc;
	}

	rc = gpio_direction_input(WIMAX_INT);
	if (rc < 0) {
		dump_debug("%s: failed to set gpio %d as input (%d)\n",
			__func__, WIMAX_INT, rc);
		goto err_gpio_direction_input;
	}

	irq = gpio_to_irq(WIMAX_INT);

	rc = request_irq(irq, wimax_hostwake_isr, IRQF_TRIGGER_FALLING,
			 "wimax_int", adapter);
	if (rc < 0) {
		dump_debug("%s: request_irq(%d) failed for gpio %d (%d)\n",
			__func__, irq,
			WIMAX_INT, rc);
		goto err_request_irq;
	}

	rc = enable_irq_wake(irq);

	if (rc < 0) {
		dump_debug("%s: enable_irq_wake(%d) failed for gpio %d (%d)\n",
			__func__, irq,
			WIMAX_INT, rc);
		goto err_enable_irq_wake;
	}

	adapter->wake_irq = irq;

	tasklet_init(&adapter->hostwake_task,
		 wimax_hostwake_task, (unsigned long)adapter);

	goto done;
err_enable_irq_wake:
	free_irq(irq, adapter);
err_request_irq:
err_gpio_direction_input:
	gpio_free(WIMAX_INT);
done:
	return rc;

}
void cmc732_release_wake_irq(struct net_adapter *adapter)
{
	if (adapter->wake_irq)	{
		disable_irq_wake(adapter->wake_irq);
		free_irq(adapter->wake_irq, adapter);
		gpio_free(WIMAX_INT);
		tasklet_kill(&adapter->hostwake_task);
		}
}



int hw_start(struct net_adapter *adapter)
{
	struct wimax_cfg *g_cfg = adapter->pdata->g_cfg;

	if (load_wimax_image(g_cfg->wimax_mode))
		return STATUS_UNSUCCESSFUL;

	adapter->download_complete = FALSE;

	if (adapter->downloading) {
		sdio_claim_host(adapter->func);
		send_cmd_packet(adapter, MSG_DRIVER_OK_REQ);
		sdio_release_host(adapter->func);
		switch (wait_event_interruptible_timeout
				(adapter->download_event,
				(adapter->download_complete == TRUE),
				msecs_to_jiffies(FWDOWNLOAD_TIMEOUT))) {
		case 0:
			/* timeout */
			dump_debug("Error hw_start :"
				"F/W Download timeout failed");
			adapter->halted = TRUE;
			return STATUS_UNSUCCESSFUL;
		case -ERESTARTSYS:
			/* Interrupted by signal */
			dump_debug("Error hw_start :  -ERESTARTSYS retry");
			return STATUS_UNSUCCESSFUL;
		default:
			/* normal condition check */
			if (adapter->removed == TRUE
				|| adapter->halted == TRUE) {
				dump_debug("Error hw_start : "
					" F/W Download surprise removed");
				return STATUS_UNSUCCESSFUL;
			}

			/*Setup hostwake interrupt*/

			if (cmc732_setup_wake_irq(adapter) < 0)
				dump_debug("hw_start : "
					" Error setting up wimax_int");


			break;
		}
		adapter->downloading = FALSE;
	}

	return STATUS_SUCCESS;
}

int hw_stop(struct net_adapter *adapter)
{
	adapter->halted = TRUE;


	/* Stop Sdio Interface */
	sdio_claim_host(adapter->func);
	sdio_release_irq(adapter->func);
	sdio_disable_func(adapter->func);
	sdio_release_host(adapter->func);

	/*Remove wakeup  interrupt*/
	cmc732_release_wake_irq(adapter);

	return STATUS_SUCCESS;
}

int hw_init(struct net_adapter *adapter)
{

	/* set WIMAX_WAKEUP & WIMAX_IF_MODE0 */
	adapter->pdata->set_mode();

	/* initilize hardware info structure */
	memset(&adapter->hw, 0x0, sizeof(struct hardware_info));

	/* For sending data and control packets */
	INIT_LIST_HEAD(&adapter->hw.q_send.list);
	spin_lock_init(&adapter->hw.q_send.lock);

	init_waitqueue_head(&adapter->download_event);

	return STATUS_SUCCESS;
}

void hw_remove(struct net_adapter *adapter)
{
	struct buffer_descriptor *bufdsc;
	struct list_head	*pos, *nxt;

	/* Free the pending data packets and control packets */
	spin_lock(&adapter->hw.q_send.lock);
	list_for_each_safe(pos, nxt, &adapter->hw.q_send.list) {
		dump_debug("Freeing q_send");
		bufdsc = list_entry(pos, struct buffer_descriptor, list);
		list_del(pos);
		kfree(bufdsc->buffer);
		kfree(bufdsc);
	}
	spin_unlock(&adapter->hw.q_send.lock);

}

int con0_poll_thread(void *data)
{
	struct net_adapter *adapter = (struct net_adapter *)data;
	struct	wimax_cfg	*cfg = adapter->pdata->g_cfg;
	int prev_val = 0;
	int curr_val = 0;

	wake_lock(&cfg->wimax_tx_lock);
	while ((!adapter->halted)) {
		curr_val = gpio_get_value(GPIO_WIMAX_CON0);
		if ((prev_val && (!curr_val)) || (curr_val == GPIO_LEVEL_LOW)) {
			adapter->pdata->restore_uart_path();
			break;
		}
		prev_val = curr_val;
		msleep(40);
		}
	wake_unlock(&cfg->wimax_tx_lock);
	do_exit(0);
	return 0;
}





/* get MAC address from device */
void hw_get_mac_address(void *data)
{
	struct net_adapter *adapter = (struct net_adapter *)data;
	struct hw_private_packet	req;
	int				nResult = 0;
	int				retry = 5;
	req.id0 = 'W';
	req.id1 = 'P';
	req.code = HwCodeMacRequest;
	req.value = 0;
	do {
		if (adapter == NULL)
			break;

		if (retry == 2) /* odb backup takes 5.8sec */
			msleep(6000);

		nResult = sd_send(adapter, (u_char *)&req,
				sizeof(struct hw_private_packet));

		if (nResult != STATUS_SUCCESS)
			dump_debug("hw_get_mac_address: sd_send fail!!");
		msleep(300);
		retry--;
		/*in case we dont get MAC we need
				to release power lock and probe finsh */
		if (!retry) {
			adapter->download_complete = TRUE;
			wake_up_interruptible(&adapter->download_event);
			msleep(100);

		}
	} while ((!adapter->mac_ready) && (!adapter->halted) && retry);

	adapter->pdata->g_cfg->powerup_done = true ;
	dump_debug("MAC thread exit");
	return;
}

u_int hw_send_data(struct net_adapter *adapter,
			void *buffer , u_long length, bool control)
{
	struct buffer_descriptor	*dsc;
	struct hw_packet_header		*hdr;
	struct net_device		*net = adapter->net;
	u_char				*ptr;
	unsigned long flags ;
	struct wimax_cfg *g_cfg = adapter->pdata->g_cfg;

	dsc = kmalloc(sizeof(*dsc), GFP_ATOMIC | GFP_DMA);
	if (dsc == NULL)
		return STATUS_RESOURCES;

	dsc->buffer = kmalloc(BUFFER_DATA_SIZE , GFP_ATOMIC | GFP_DMA);
	if (dsc->buffer == NULL) {
		kfree(dsc);
		return STATUS_RESOURCES;
	}

	ptr = dsc->buffer;

	/* shift data pointer */
	ptr += sizeof(struct hw_packet_header);
#ifdef HARDWARE_USE_ALIGN_HEADER
	ptr += 2;
#endif
	hdr = (struct hw_packet_header *)dsc->buffer;

	if (control) {
		memcpy(ptr, buffer + (ETHERNET_ADDRESS_LENGTH * 2),
				length - (ETHERNET_ADDRESS_LENGTH * 2));

		/* add packet header */
		hdr->id0 = 'W';
		hdr->id1 = 'C';
		hdr->length = (u_short)length - (ETHERNET_ADDRESS_LENGTH * 2);

		/* set length */
		dsc->length = (u_short)length - (ETHERNET_ADDRESS_LENGTH * 2)
				+ sizeof(struct hw_packet_header);
	#ifdef HARDWARE_USE_ALIGN_HEADER
		dsc->length += 2;
	#endif

		/* dump control packet for debug */
		if (g_cfg->enable_dump_msg == 1)
			dump_buffer("Control Tx",
				(u_char *)dsc->buffer + 6, dsc->length - 6);
	} else {

		length -= (ETHERNET_ADDRESS_LENGTH * 2);
		buffer += (ETHERNET_ADDRESS_LENGTH * 2);

		memcpy(ptr, buffer, length);

		hdr->id0 = 'W';
		hdr->id1 = 'D';
		hdr->length = (u_short)length;

		dsc->length = length + sizeof(struct hw_packet_header);
	#ifdef HARDWARE_USE_ALIGN_HEADER
		dsc->length += 2;
	#endif
		adapter->netstats.tx_packets++;
		adapter->netstats.tx_bytes += dsc->length;

	/* add statistics */
		if (!netif_running(net))
			dump_debug("!netif_running");

	}
	spin_lock_irqsave(&adapter->hw.q_send.lock, flags);
	list_add_tail(&dsc->list, &adapter->hw.q_send.list);
	spin_unlock_irqrestore(&adapter->hw.q_send.lock, flags);

	schedule_work(&adapter->transmit_work);
	return STATUS_SUCCESS;
}

int sd_send_data(struct net_adapter *adapter, struct buffer_descriptor *dsc,
		bool *tx_pending)
{
	int	nRet = 0;
	int nWriteIdx = 0;
	dsc->length += (dsc->length & 1) ? 1 : 0;

#ifdef HARDWARE_USE_ALIGN_HEADER
	if (dsc->length > SDIO_MAX_BYTE_SIZE)
		dsc->length = (dsc->length + SDIO_MAX_BYTE_SIZE)
				& ~(SDIO_MAX_BYTE_SIZE);
#endif

	hwSdioWriteBankIndex(adapter, &nWriteIdx, &nRet);

	if (nWriteIdx < 0) {
		*tx_pending = true;
		return nRet;
	}

	if (nRet) {
		dump_debug("sd_send_data : "
			" error fetch bank index!! nRet = %d", nRet);
		return nRet;
	}

	sdio_writeb(adapter->func, (nWriteIdx + 1) % 15,
			 SDIO_H2C_WP_REG, NULL);

	nRet = sdio_memcpy_toio(adapter->func,
			SDIO_TX_BANK_ADDR+(SDIO_BANK_SIZE * nWriteIdx)+4,
			dsc->buffer, dsc->length);

	if (nRet) {
		pr_err("sd_send_data :"
			" error writing dsc packet!! nRet = %d", nRet);
		return nRet;
	}
	nRet = sdio_memcpy_toio(adapter->func,
			SDIO_TX_BANK_ADDR + (SDIO_BANK_SIZE * nWriteIdx),
			&(dsc->length), 4);

	if (nRet)
		pr_err("sd_send_data :"
			"error writing bank length info!! nRet = %d", nRet);
	return nRet;
}

int hw_device_wakeup(struct net_adapter *adapter)
{
	int	rc = 0;

	adapter->pdata->wakeup_assert(1);

	while (!adapter->pdata->is_modem_awake()) {
		if (rc == 0)
			dump_debug("hw_device_wakeup (CON0 status):"
				" waiting for modem awake");
		rc++;
		if (rc > WAKEUP_MAX_TRY) {
				dump_debug("hw_device_wakeup (CON0 status):"
					" modem wake up time out!!");
				break;
			}
		msleep(WAKEUP_TIMEOUT/2);
		adapter->pdata->wakeup_assert(0);
		msleep(WAKEUP_TIMEOUT/2);
		adapter->pdata->wakeup_assert(1);
		s3c_bat_use_wimax(1);
	}
	if (!adapter->pdata->is_modem_awake()) {
		pr_err("FATAL ERROR!! MODEM DOES NOT WAKEUP!!");
		adapter->halted = true;
		return STATUS_UNSUCCESSFUL;
	} else
		dump_debug("hw_device_wakeup (CON0 status): modem awake");
	adapter->pdata->wakeup_assert(0);

	return 0;
}

/*
This Work is responsible for Transmiting Both Control And Data packet
*/
void hw_transmit_thread(struct work_struct *work)
{
	struct buffer_descriptor        *dsc;
	struct net_adapter              *adapter;
	struct	list_head	*pos, *nxt;
	int				nRet = 0;
	struct wimax_cfg *g_cfg;
	bool	reset_modem = false;
	bool		tx_pending;

	adapter = container_of(work, struct net_adapter, transmit_work);
	g_cfg = adapter->pdata->g_cfg;

	if (!gpio_get_value(WIMAX_EN)) {
		dump_debug("WiMAX Power OFF!! (TX)");
		return;
	}

	wake_lock(&g_cfg->wimax_tx_lock);
	mutex_lock(&g_cfg->rx_lock);

	list_for_each_safe(pos, nxt, &adapter->hw.q_send.list) {

		tx_pending = false;
		if (adapter->halted) {
			dump_debug("Halted Already");
			break;
		}

		if (g_cfg->wimax_suspend_prepare) {
			dump_debug("cmc7xx received suspend prepare noti");
			break;
		}

		if (!adapter->pdata->is_modem_awake()) {
			if (hw_device_wakeup(adapter)) {
				reset_modem = true;
				break;
			}
		}
		dsc = list_entry(pos, struct buffer_descriptor, list);

		if (!dsc->buffer) {
			dump_debug("dsc->buffer is  NULL");
			break;
		}

		if (!dsc) {
			dump_debug("Fail...node is null");
			break;
		}

		sdio_claim_host(adapter->func);
		nRet = sd_send_data(adapter, dsc, &tx_pending);
		sdio_release_host(adapter->func);

		if (tx_pending)
			continue;

		list_del(pos);
		kfree(dsc->buffer);
		kfree(dsc);

		if (nRet != STATUS_SUCCESS) {
			sdio_claim_host(adapter->func);
			sdio_release_irq(adapter->func);
			sdio_release_host(adapter->func);
			dump_debug("SendData Fail******");
			++adapter->XmitErr;
			if (nRet == -ENOMEDIUM || nRet == /*-ETIMEOUT*/-110)
				pr_err("ENOMEDIUM | TIMEOUT");
			reset_modem = true;
			break;
		}
	}

	if (reset_modem) {
		pr_err("reset_modem!!");
		adapter->pdata->power(0);
	}

	mutex_unlock(&g_cfg->rx_lock);
	wake_unlock(&g_cfg->wimax_tx_lock);

	return ;
}
