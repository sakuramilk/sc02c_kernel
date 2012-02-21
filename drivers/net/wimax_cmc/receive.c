/*
 * receive.c
 *
 * handle download packet, private cmd and control/data packet
 */
#include "headers.h"
#include "wimax_plat.h"
#include "firmware.h"
#include "download.h"

#include <plat/sdhci.h>
#include <plat/devs.h>
#include <linux/mmc/host.h>
#include <linux/kthread.h>
extern struct image_data g_wimax_image;

void process_indicate_packet(struct net_adapter *adapter, u_char *buffer)
{
	struct wimax_msg_header	*packet;
	char			*tmp_byte;
	struct wimax_cfg *g_cfg = adapter->pdata->g_cfg;

	packet = (struct wimax_msg_header *)buffer;

	if (packet->type == be16_to_cpu(ETHERTYPE_DL)) {
		switch (be16_to_cpu(packet->id)) {
		case MSG_DRIVER_OK_RESP:
			dump_debug("process_indicate_packet: MSG_DRIVER_OK_RESP");
			send_image_info_packet(adapter, MSG_IMAGE_INFO_REQ);
			break;
		case MSG_IMAGE_INFO_RESP:
			dump_debug("process_indicate_packet: MSG_IMAGE_INFO_RESP");
			send_image_data_packet(adapter, MSG_IMAGE_DATA_REQ);
			break;
		case MSG_IMAGE_DATA_RESP:
			if (g_wimax_image.offset == g_wimax_image.size) {
				dump_debug("process_indicate_packet: Image Download Complete");
				send_cmd_packet(adapter, MSG_RUN_REQ); /* Run Message Send */
			} else
				send_image_data_packet(adapter, MSG_IMAGE_DATA_REQ);
			break;
		case MSG_RUN_RESP:
			tmp_byte = (char *)(buffer + sizeof(struct wimax_msg_header));

			if (*tmp_byte == 0x01) {
				dump_debug("process_indicate_packet: MSG_RUN_RESP");

				if (g_cfg->wimax_mode == SDIO_MODE || g_cfg->wimax_mode == DM_MODE
						|| g_cfg->wimax_mode == USB_MODE
						|| g_cfg->wimax_mode == USIM_RELAY_MODE) {
					dump_debug("%s: F/W Download Complete and Running ",__func__);
					dump_debug("Wait for SDIO ready...");
					msleep(1200);	/* IMPORTANT!! wait for cmc730 can handle mac req packet */

					kernel_thread((int (*)(void *))hw_get_mac_address, adapter, 0);
				} else if (g_cfg->wimax_mode == WTM_MODE) {
					adapter->download_complete = TRUE;
					wake_up_interruptible(&adapter->download_event);
					adapter->pdata->g_cfg->powerup_done = true ;
					adapter->wtm_task = kthread_create(
					con0_poll_thread,       adapter, "%s",
					"wimax_con0_poll_thread");
					if (adapter->wtm_task)
					  wake_up_process(
					  adapter->wtm_task);
				} else if (g_cfg->wimax_mode == AUTH_MODE) {
					adapter->download_complete = TRUE;
					wake_up_interruptible(&adapter->download_event);
					adapter->pdata->g_cfg->powerup_done = true ;
				}
			}
			break;
		default:
			dump_debug("process_indicate_packet: Unkown type");
			break;
		}
	}
	else
		dump_debug("process_indicate_packet - is not download pakcet");
}

u_long process_private_cmd(struct net_adapter *adapter, void *buffer)
{
	struct hw_private_packet	*cmd;
	u_char				*bufp;
	struct wimax_cfg *g_cfg = adapter->pdata->g_cfg;
	cmd = (struct hw_private_packet *)buffer;

	switch (cmd->code) {
	case HwCodeMacResponse: {
		u_char mac_addr[ETHERNET_ADDRESS_LENGTH];
		bufp = (u_char *)buffer;

		/* processing for mac_req request */
		dump_debug("MAC address = %02x:%02x:%02x:%02x:%02x:%02x",
				bufp[3], bufp[4], bufp[5], bufp[6], bufp[7], bufp[8]);
		memcpy(mac_addr, bufp + 3, ETHERNET_ADDRESS_LENGTH);

		/* create ethernet header */
		memcpy(adapter->hw.eth_header, mac_addr, ETHERNET_ADDRESS_LENGTH);
		memcpy(adapter->hw.eth_header + ETHERNET_ADDRESS_LENGTH, mac_addr, ETHERNET_ADDRESS_LENGTH);
		adapter->hw.eth_header[(ETHERNET_ADDRESS_LENGTH * 2) - 1] += 1;

		memcpy(adapter->net->dev_addr, bufp + 3, ETHERNET_ADDRESS_LENGTH);
		adapter->mac_ready = TRUE;

		if (adapter->downloading) {
			adapter->download_complete = TRUE;
			wake_up_interruptible(&adapter->download_event);
		}
		return (sizeof(struct hw_private_packet) + ETHERNET_ADDRESS_LENGTH - sizeof(u_char));
	}
	case HwCodeLinkIndication: {
		if ((cmd->value == HW_PROT_VALUE_LINK_DOWN)
					&& (adapter->media_state != MEDIA_DISCONNECTED)) {
			dump_debug("LINK_DOWN_INDICATION");
			s3c_bat_use_wimax(0);
			cmc7xx_tune_cpu(adapter, 0);

			/* set values */
			adapter->media_state = MEDIA_DISCONNECTED;

			/* indicate link down */
			netif_stop_queue(adapter->net);
			netif_carrier_off(adapter->net);
		} else if ((cmd->value == HW_PROT_VALUE_LINK_UP)
					&& (adapter->media_state != MEDIA_CONNECTED)) {
			dump_debug("LINK_UP_INDICATION");

			s3c_bat_use_wimax(1);
			cmc7xx_tune_cpu(adapter, 1);
			/* set values */
			adapter->media_state = MEDIA_CONNECTED;
			adapter->net->mtu = WIMAX_MTU_SIZE;

			/* indicate link up */
			netif_start_queue(adapter->net);
			netif_carrier_on(adapter->net);
		}
		break;
	}
	case HwCodeHaltedIndication: {
		dump_debug("Device is about to reset, stop driver");
		adapter->halted = TRUE;
		break;
	}
	case HwCodeRxReadyIndication: {
		dump_debug("Device RxReady");
		/* to start the data packet send queue again after stopping in xmit */
		if (adapter->media_state == MEDIA_CONNECTED)
			netif_wake_queue(adapter->net);
		break;
	}
	case HwCodeIdleNtfy: {
		/* set idle / vi */

		dump_debug("process_private_cmd: HwCodeIdleNtfy");

		s3c_bat_use_wimax(0);
		cmc7xx_tune_cpu(adapter, 0);
		break;
	}
	case HwCodeWakeUpNtfy: {
		/* IMPORTANT!! at least 4 sec is required after modem waked up */
		wake_lock_timeout(&g_cfg->wimax_wake_lock, 4 * HZ);
		dump_debug("process_private_cmd: HwCodeWakeUpNtfy");
		s3c_bat_use_wimax(1);
		cmc7xx_tune_cpu(adapter, 1);
		break;
	}
	default:
		dump_debug("Command = %04x", cmd->code);
		break;
	}

	return sizeof(struct hw_private_packet);
}

