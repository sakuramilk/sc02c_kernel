/**
 * wimax_sdio.c
 *
 * functions for device access
 * swmxctl (char device): gpio control
 * uwibro (char device): send/recv control packet
 * sdio device: sdio functions
 */
#include "headers.h"
#include "wimax_plat.h"
#include "ctl_types.h"
#include "wimax_i2c.h"
#include "download.h"
#include "firmware.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/miscdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio_func.h>
#include <asm/byteorder.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/fs.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/wimax/samsung/wimax732.h>
/* driver Information */
#define WIMAX_DRIVER_VERSION_STRING "3.0.0"
#define DRIVER_AUTHOR "Samsung"
#define DRIVER_DESC "Samsung WiMAX SDIO Device Driver"

#define UWBRBDEVMINOR	233
#define SWMXGPIOMINOR	234

struct net_adapter *g_adapter;
static u_char node_id[ETH_ALEN];
/* use ethtool to change the level for any given device */
static int msg_level = -1;
module_param(msg_level, int, 0);

/* test function */

void s3c_bat_use_wimax(int onoff)
{
	struct file     *fp;
	fp = klib_fopen(WIMAX_BAT_SYSPATH, O_RDWR, 0);

	if (!fp)
		dump_debug("open fail");
	if (onoff)
		klib_fwrite("1", 1, fp);
	else
		klib_fwrite("0", 1, fp);
	klib_fclose(fp);
}
EXPORT_SYMBOL(s3c_bat_use_wimax);

void cmc7xx_tune_cpu(struct net_adapter *adapter, int onoff)
{
	struct file		*fp2;

	fp2 = klib_fopen(WIMAX_CPU0SCALING_PATH, O_RDWR, 0);

	if (!fp2) {
		dump_debug("open fail");
		return;
	}

	if ((!onoff) && (adapter->cputunedone)) {
			adapter->cputunedone = false;
			klib_fwrite((char *)&adapter->cpu0scaling,
				strlen(adapter->cpu0scaling), fp2);
	} else if ((onoff) && (!adapter->cputunedone)) {
		adapter->cputunedone = true;
		klib_fread((char *)&adapter->cpu0scaling,
			sizeof(adapter->cpu0scaling), fp2);
		klib_fwrite("performance", strlen("performance"), fp2);
	}

	klib_fclose(fp2);
}

static const struct file_operations swmx_fops = {
owner:THIS_MODULE,
open	:	swmxdev_open,
release	:	swmxdev_release,
ioctl	:	swmxdev_ioctl,
read	:	swmxdev_read,
write	:	swmxdev_write,
};

void cmc7xx_prepare_skb(struct net_adapter *adapter, struct sk_buff *rx_skb)
{
	skb_reserve(rx_skb,
			(ETHERNET_ADDRESS_LENGTH * 2) +
			NET_IP_ALIGN);

	memcpy(skb_push(rx_skb,
			(ETHERNET_ADDRESS_LENGTH * 2)),
			adapter->hw.eth_header,
			(ETHERNET_ADDRESS_LENGTH * 2));

	rx_skb->dev = adapter->net;
	rx_skb->ip_summed = CHECKSUM_UNNECESSARY;
}
void cmc7xx_flush_skb(struct net_adapter *adapter)
{
	int i;
	for (i = 0; i < CMC_MP_SKB_POOL; i++) {
		if (adapter->rx_skb[i]) {
			dev_kfree_skb(adapter->rx_skb[i]);
			adapter->rx_skb[i] = NULL;
		}
	}
}

struct sk_buff *cmc7xx_fetch_skb(struct net_adapter *adapter)
{
	int i;
	struct sk_buff *ret_skb;
	for (i = 0; i < CMC_MP_SKB_POOL; i++) {
		if (adapter->rx_skb[i]) {
			ret_skb = adapter->rx_skb[i];
			adapter->rx_skb[i] = NULL;
			return ret_skb;
		}
	}
	ret_skb = dev_alloc_skb(WIMAX_MTU_SIZE + 2 +
					(ETHERNET_ADDRESS_LENGTH * 2) +
					NET_IP_ALIGN);
	if (!ret_skb) {
		pr_err("unable to allocate skb");
		return NULL;
	}
	cmc7xx_prepare_skb(adapter, ret_skb);
	return ret_skb;
}

void cmc7xx_pull_skb(struct net_adapter *adapter)
{
	int i;
	struct sk_buff *t_skb;
	for (i = 0; i < CMC_MP_SKB_POOL; i++) {
		if (adapter->rx_skb[i] == NULL) {
			t_skb = dev_alloc_skb(WIMAX_MTU_SIZE + 2 +
					(ETHERNET_ADDRESS_LENGTH * 2) +
					NET_IP_ALIGN);
			if (!t_skb) {
				pr_err("unable to allocate skb");
				break;
			}
			cmc7xx_prepare_skb(adapter, t_skb);
			adapter->rx_skb[i] = t_skb;
		}
	}
}

/*
   swmxctl functions
   (power on/off and factory function test)
 */
int swmxdev_open(struct inode *inode, struct file *file)
{
	struct wimax732_platform_data *pdata =
		container_of(file->private_data,
				struct wimax732_platform_data, swmxctl_dev);
	file->private_data = pdata;
	dump_debug("Device open by %d", current->tgid);
	return 0;
}

int swmxdev_release(struct inode *inode, struct file *file)
{
	dump_debug("Device close by %d", current->tgid);
	return 0;
}

int swmxdev_ioctl(struct inode *inode, struct file *file,
			 u_int cmd, u_long arg)
{
	int	ret = 0;
	u_int	val = ((u_char *)arg)[0];

	struct wimax732_platform_data *gpdata =
		(struct wimax732_platform_data *)(file->private_data);

	dump_debug("CMD: %x, PID: %d", cmd, current->tgid);

	switch (cmd) {
	case CONTROL_IOCTL_WIMAX_POWER_CTL: {
			dump_debug("CONTROL_IOCTL_WIMAX_POWER_CTL..");
			if (val == 0)
				ret = gpdata->power(0);
				else
				ret = gpdata->power(1);
				break;
			}
	case CONTROL_IOCTL_WIMAX_MODE_CHANGE: {
			dump_debug("CONTROL_IOCTL_WIMAX_MODE_CHANGE"
					" to %d..", val);
			if ((val < 0) || (val > AUTH_MODE)) {
				dump_debug("Wrong mode %d", val);
				return 0;
				}
				gpdata->power(0);
				gpdata->g_cfg->wimax_mode = val;
				msleep(500);/*gurantee bootloader initializing*/
				ret = gpdata->power(1);
				break;
			}
	case CONTROL_IOCTL_WIMAX_EEPROM_DOWNLOAD: {
				dump_debug("CNT_IOCTL_WIMAX_EEPROM_DOWNLOAD");
				gpdata->power(0);
				ret = eeprom_write_boot();
				break;
			}
	case CONTROL_IOCTL_WIMAX_WRITE_REV: {
			dump_debug("CONTROL_IOCTL_WIMAX_WRITE_REV");
			gpdata->power(0);
			ret = eeprom_write_rev();
			break;
			}
	case CONTROL_IOCTL_WIMAX_CHECK_CERT: {
			dump_debug("CONTROL_IOCTL_WIMAX_CHECK_CERT");
			gpdata->power(0);
			ret = eeprom_check_cert();
			break;
			}
	case CONTROL_IOCTL_WIMAX_CHECK_CAL: {
			dump_debug("CONTROL_IOCTL_WIMAX_CHECK_CAL");
			gpdata->power(0);
			ret = eeprom_check_cal();
			break;
			}
	}	/* switch (cmd) */

	return ret;
}

ssize_t swmxdev_read(struct file *file, char *buf,
			 size_t count, loff_t *ppos)
{
	return 0;
}

ssize_t swmxdev_write(struct file *file, const char *buf,
			 size_t count, loff_t *ppos)
{
	return 0;
}

/*
   uwibro functions
   (send and receive control packet with WiMAX modem)
 */
int uwbrdev_open(struct inode *inode, struct file *file)
{
	struct net_adapter		*adapter;
	struct process_descriptor	*process;

	if ((g_adapter == NULL) || g_adapter->halted) {
		dump_debug("can't find adapter or Device Removed");
		return -ENODEV;
	}

	file->private_data = (void *)g_adapter;
	adapter = (struct net_adapter *)(file->private_data);
	dump_debug("open: tgid=%d", current->tgid);

	if (adapter->mac_ready != TRUE || adapter->halted) {
		dump_debug("Device not ready Retry..");
		return -ENXIO;
	}

	process = process_by_id(adapter, current->tgid);
	if (process != NULL) {
		dump_debug("second open attemp from uid %d", current->tgid);
		return -EEXIST;
	} else {
		/* init new process descriptor */
		process = (struct process_descriptor *)
				kmalloc(sizeof(struct process_descriptor),
					 GFP_ATOMIC);
		if (process == NULL) {
			dump_debug("uwbrdev_open: kmalloc fail!!");
			return -ENOMEM;
		} else {
			process->id = current->tgid;
			process->irp = FALSE;
			process->type = 0;
			init_waitqueue_head(&process->read_wait);
			spin_lock(&adapter->ctl.apps.lock);
			list_add_tail(&process->list,
					&adapter->ctl.apps.process_list);
			spin_unlock(&adapter->ctl.apps.lock);
		}
	}

	return 0;
}

int uwbrdev_release(struct inode *inode, struct file *file)
{
	struct net_adapter		*adapter;
	struct process_descriptor	*process;
	int				current_tgid = 0;

	dump_debug("release: tgid=%d, pid=%d", current->tgid, current->pid);

	adapter = (struct net_adapter *)(file->private_data);
	if (adapter == NULL) {
		dump_debug("can't find adapter");
		return -ENODEV;
	}

	current_tgid = current->tgid;
	process = process_by_id(adapter, current_tgid);

	/* process is not exist. (open process != close process) */
	if (process == NULL) {
		current_tgid = adapter->pdata->g_cfg->temp_tgid;
		dump_debug("release: pid changed: %d", current_tgid);
		process = process_by_id(adapter, current_tgid);
	}

	if (process != NULL) {
		/* RELEASE READ THREAD */
		if (process->irp) {
			process->irp = FALSE;
			wake_up_interruptible(&process->read_wait);
		}
		spin_lock(&adapter->ctl.apps.lock);
		remove_process(adapter, current_tgid);
		spin_unlock(&adapter->ctl.apps.lock);
	} else {
		/*not found */
		dump_debug("process %d not found", current_tgid);
		return -ESRCH;
	}

	return 0;
}

static const struct file_operations uwbr_fops = {
owner:THIS_MODULE,
open	:	uwbrdev_open,
release	:	uwbrdev_release,
ioctl	:	uwbrdev_ioctl,
read	:	uwbrdev_read,
write	:	uwbrdev_write,
};

static struct miscdevice uwibro_dev = {
	.minor = UWBRBDEVMINOR,
	.name = "uwibro",
	.fops = &uwbr_fops,
};

/* buffer used in uwbrdev_ioctl routine */
struct control_tx_buffer {
	int	length;
	u_char	data[WIMAX_MAX_TOTAL_SIZE];
};

static struct control_tx_buffer g_tx_buffer;

int uwbrdev_ioctl(struct inode *inode, struct file *file, u_int cmd, u_long arg)
{
	struct net_adapter		*adapter;
	struct process_descriptor	*process;
	int				ret = 0;

	adapter = (struct net_adapter *)(file->private_data);

	if ((adapter == NULL) || adapter->halted) {
		dump_debug("can't find adapter or Device Removed");
		return -ENODEV;
	}

	switch (cmd) {
	case CONTROL_IOCTL_WRITE_REQUEST: {
			struct eth_header *ctlhdr;
			memset(&g_tx_buffer, 0x0,
				 sizeof(struct control_tx_buffer));
			if ((char *)arg == NULL) {
				dump_debug("arg == NULL: return -EFAULT");
				return -EFAULT;
			}
			g_tx_buffer.length =
				((struct control_tx_buffer *)arg)->length;
			if (g_tx_buffer.length < WIMAX_MAX_TOTAL_SIZE) {
				if (copy_from_user(g_tx_buffer.data,
						 (void *)(arg+sizeof(int)),
					 g_tx_buffer.length))
					return -EFAULT;
				} else
					return -EFBIG;
			spin_lock(&adapter->ctl.apps.lock);
			process = process_by_id(adapter, current->tgid);
			if (process == NULL) {
				dump_debug("process %d not found",\
				 current->tgid);
				ret = -EFAULT;
				spin_unlock(&adapter->ctl.apps.lock);
				break;
			}
			ctlhdr = (struct eth_header *)g_tx_buffer.data;
			process->type = ctlhdr->type;
			spin_unlock(&adapter->ctl.apps.lock);
			control_send(adapter, g_tx_buffer.data,
					 g_tx_buffer.length);
			break;
			}
	default:
			dump_debug("uwbrdev_ioctl: "
					"unknown ioctl cmd: 0x%x", cmd);
			break;
	}	/* switch (cmd) */

	return ret;
}

ssize_t uwbrdev_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	struct buffer_descriptor	*dsc;
	struct net_adapter		*adapter;
	struct process_descriptor	*procdsc;
	int				rlen = 0;

	adapter = (struct net_adapter *)(file->private_data);
	if ((adapter == NULL) || adapter->halted) {
		dump_debug("can't find adapter or Device Removed");
		return -ENODEV;
	}

	if (buf == NULL) {
		dump_debug("BUFFER is NULL");
		return -EFAULT; /* bad address */
	}

	procdsc = process_by_id(adapter, current->tgid);
	if (procdsc == NULL) {
		dump_debug("uwbrdev_read: "
				"process %d not exist", current->tgid);
		return -ESRCH;
	}

	if (procdsc->irp == FALSE) {
		dsc = buffer_by_type(adapter, procdsc->type);
		if (dsc == NULL) {
			procdsc->irp = TRUE;
			if (wait_event_interruptible(procdsc->read_wait,
				((procdsc->irp == FALSE) ||
				 (adapter->halted == TRUE)))) {
				procdsc->irp = FALSE;
				adapter->pdata->g_cfg->temp_tgid =
						 current->tgid;
				return -ERESTARTSYS;
			}
			if (adapter->halted == TRUE) {
				dump_debug("uwbrdev_read: "
						"Card Removed "
						"Indicated to Appln...");
				procdsc->irp = FALSE;
				adapter->pdata->g_cfg->temp_tgid =
							 current->tgid;
				return -ENODEV;
			}
		}

		if (count == 1500) {	/* app passes read count as 1500 */
			spin_lock(&adapter->ctl.apps.lock);
			dsc = buffer_by_type(adapter, procdsc->type);
			if (!dsc) {
				dump_debug("uwbrdev_read: Fail...node is null");
				spin_unlock(&adapter->ctl.apps.lock);
				return -1;
			}
			spin_unlock(&adapter->ctl.apps.lock);

			if (copy_to_user(buf, dsc->buffer, dsc->length)) {
				dump_debug("uwbrdev_read:copy_to_user failed"
						"len=%lu !!", dsc->length);
				return -EFAULT;
			}

			spin_lock(&adapter->ctl.apps.lock);
			rlen = dsc->length;
			list_del(&dsc->list);
			kfree(dsc->buffer);
			kfree(dsc);
			spin_unlock(&adapter->ctl.apps.lock);
		}
	} else {
		dump_debug("uwbrdev_read: Read was sent twice "
					"by process %d", current->tgid);
		return -EEXIST;
	}

	return rlen;
}

ssize_t uwbrdev_write(struct file *file, const char *buf,
			 size_t count, loff_t *ppos)
{
	return 0;
}

static struct net_device_ops wimax_net_ops = {
	.ndo_open =			adapter_open,
	.ndo_stop =			adapter_close,
	.ndo_get_stats =		adapter_netdev_stats,
	.ndo_do_ioctl =			adapter_ioctl,
	.ndo_start_xmit =		adapter_start_xmit,
	.ndo_set_mac_address =		NULL,
	.ndo_set_multicast_list =	adapter_set_multicast
};

static struct sdio_device_id adapter_table[] = {
	{ SDIO_DEVICE(0x98, 0x1) },
	{ }	/* Terminating entry */
};

static struct sdio_driver adapter_driver = {
	.name		= "C730SDIO",
	.probe		= adapter_probe,
	.remove		= adapter_remove,
	.id_table	= adapter_table,
};

static void create_char_name(u_char *str, u_long index)
{
	u_char	tempName[] = "uwbrdev";

	sprintf(str, "%s%lu", tempName, index);
	return;
}

static int netdev_ethtool_ioctl(struct net_device *dev, void *useraddr)
{
	u_long	ethcmd;

	if (copy_from_user(&ethcmd, useraddr, sizeof(ethcmd)))
		return -EFAULT;

	switch (ethcmd) {
	case ETHTOOL_GDRVINFO: {
		struct ethtool_drvinfo info = {ETHTOOL_GDRVINFO};
		strncpy(info.driver, "C730USB", sizeof(info.driver) - 1);
		if (copy_to_user(useraddr, &info, sizeof(info)))
			return -EFAULT;
		return 0;
		}
	default:
			break;
	}

	return -EOPNOTSUPP;
}

struct net_device_stats *adapter_netdev_stats(struct net_device *dev)
{
	return &((struct net_adapter *)netdev_priv(dev))->netstats;
}

int adapter_start_xmit(struct sk_buff *skb, struct net_device *net)
{
	struct net_adapter	*adapter = netdev_priv(net);
	int			len;

	if (!adapter->media_state || adapter->halted) {
		dump_debug("Driver already halted. Returning Failure...");
		dev_kfree_skb(skb);
		adapter->netstats.tx_dropped++;
		net->trans_start = jiffies;
		adapter->XmitErr += 1;
		return 0;
	}

	len = skb->len;
	hw_send_data(adapter, skb->data, len, DATA_PACKET);
	dev_kfree_skb(skb);

	return 0;
}

void adapter_set_multicast(struct net_device *net)
{
	return;
}

int adapter_open(struct net_device *net)
{
	struct net_adapter	*adapter;
	int			res = 0;

	adapter = netdev_priv(net);

	if (adapter == NULL || adapter->halted) {
		dump_debug("can't find adapter or halted");
		return -ENODEV;
	}

	if (adapter->media_state)
		netif_wake_queue(net);
	else
		netif_stop_queue(net);

	if (netif_msg_ifup(adapter))
		dump_debug("netif msg if up");

	res = 0;
	dump_debug("adapter driver open success!!!!!!!");

	return res;
}

int adapter_close(struct net_device *net)
{
	dump_debug("adapter driver close success!!!!!!!");
	netif_stop_queue(net);
	return 0;
}

int adapter_ioctl(struct net_device *net, struct ifreq *rq, int cmd)
{
	struct net_adapter	*adapter = netdev_priv(net);

	if (adapter->halted) {
		dump_debug("Driver already halted. Returning Failure...");
		return STATUS_UNSUCCESSFUL;
	}

	switch (cmd) {
	case SIOCETHTOOL:
		return netdev_ethtool_ioctl(net, (void *)rq->ifr_data);
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

void cmc7xx_sdio_reset(struct work_struct *work)
{
	struct net_adapter *adapter = container_of(work,
			struct net_adapter, wimax_reset);
	u8 data[100];

	sdio_claim_host(adapter->func);
	sdio_release_irq(adapter->func);
	sdio_release_host(adapter->func);

	if (wimax_cmc7xx_sdio_reset_comm(adapter->func->card))
		dump_debug("%s: cmc7xx_sdio_reset_comm fail", __func__);

	sdio_claim_host(adapter->func);
	if (sdio_enable_func(adapter->func))
		dump_debug("%s: sdio_enable_func fail", __func__);

	if (sdio_claim_irq(adapter->func, adapter_interrupt))
		dump_debug("%s: sdio_claim_irq fail", __func__);

	if (sdio_set_block_size(adapter->func, 512))
		dump_debug("%s: sdio_set_block_size fail", __func__);

	sdio_memcpy_toio(adapter->func,
		SDIO_TX_BANK_ADDR + 4,
		data, 32);

	sdio_release_host(adapter->func);
}

static void adapter_rx_packet(struct net_adapter *adapter,
		struct buffer_descriptor *bufdsc)
{
	struct hw_packet_header	*hdr;
	s32						rlen = bufdsc->length;
	u32						l;
	u8						*ofs;
	struct sk_buff				*rx_skb;
	ofs = (u8 *)bufdsc->buffer;

	while (rlen > 0) {
		hdr = (struct hw_packet_header *)ofs;

		/* "WD", "WC", "WP" or "WE" */
		if (unlikely(hdr->id0 != 'W')) {
			/*Ignore if it is the 4 byte allignment*/
			pr_warn("Wrong packet	\
			ID (%02x %02x) rlen = %d\n", hdr->id0, hdr->id1, rlen);
			/* skip rest of packets */
			break;
		}

		/* change offset */
		ofs += sizeof(*hdr);
		rlen -= sizeof(*hdr);

		/* check packet type */
		switch (hdr->id1) {
		case 'P': {
			/* revert offset */
			ofs -= sizeof(*hdr);
			rlen += sizeof(*hdr);
			/* process packet */
			l = process_private_cmd(adapter, ofs);
			/* shift */
			ofs += l;
			rlen -= l;

			/* process next packet */
			continue;
			}
		case 'C':
			if (!adapter->downloading) {
				ofs += 2;
				rlen -= 2;
				control_recv(adapter, (u8 *)ofs, hdr->length);
				break;
			} else {
				hdr->length -= sizeof(*hdr);
				process_indicate_packet(adapter, ofs);
				break;
			}
		case 'D':
			ofs += 2;
			rlen -= 2;

			if (hdr->length > BUFFER_DATA_SIZE) {
					pr_warn("Data packet too large");
					adapter->netstats.rx_dropped++;
					break;
				}

				if (likely(hdr->length <=
							(WIMAX_MTU_SIZE + 2))) {
					rx_skb = cmc7xx_fetch_skb(adapter);

					if (!rx_skb) {
						pr_err("unable to allocate skb");
						break;
					}
				} else {
					rx_skb = dev_alloc_skb(hdr->length +
						(ETHERNET_ADDRESS_LENGTH * 2) +
						NET_IP_ALIGN);
					if (!rx_skb) {
						pr_err("unable to allocate skb");
						break;
					}
					cmc7xx_prepare_skb(adapter, rx_skb);
				}

				memcpy(skb_put(rx_skb, hdr->length),
							(u8 *)ofs,
							hdr->length);
				rx_skb->protocol =
					eth_type_trans(rx_skb, adapter->net);
				if (netif_rx_ni(rx_skb) == NET_RX_DROP) {
					pr_debug("packet dropped!");
					adapter->netstats.rx_dropped++;
				}
				adapter->netstats.rx_packets++;
				adapter->netstats.rx_bytes +=
					(hdr->length +
					 (ETHERNET_ADDRESS_LENGTH * 2));

			break;
		case 'E':
			pr_warn("%s :Wrong packet Extended ID [%02x %02x]",
						__func__, hdr->id0, hdr->id1);
			/* skip rest of buffer */
			goto out;
		default:
			pr_warn("%s :Wrong packet ID [%02x %02x]",
						__func__, hdr->id0, hdr->id1);
			/* skip rest of buffer */
			goto out;
		}

		ofs += hdr->length;
		rlen -= hdr->length;
	}
out:
	if (adapter->mac_ready)
		cmc7xx_pull_skb(adapter);
	return;
}

static void rx_process_data(struct work_struct *rx_work)
{
	struct buffer_descriptor *bufdsc;
	struct net_adapter *adapter =
		container_of(rx_work, struct net_adapter, receive_work);

	while (1) {
		spin_lock(&adapter->recv_lock);
		if (unlikely(list_empty(&adapter->q_recv))) {
			spin_unlock(&adapter->recv_lock);
			return;
		}
		bufdsc = list_first_entry(&adapter->q_recv,
				struct buffer_descriptor, list);
		list_del(&bufdsc->list);
		spin_unlock(&adapter->recv_lock);
		adapter_rx_packet(adapter, bufdsc);
		kfree(bufdsc->buffer);
		bufdsc->buffer = NULL;
		kfree(bufdsc);
	};

}

static struct buffer_descriptor *rx_packet(struct net_adapter *adapter)
{
	int ret = 0;
	int read_idx;
	struct buffer_descriptor *bufdsc;
	s32							t_len;
	s32							t_index;
	s32							t_size;
	u8							*t_buff;

	read_idx = sdio_readb(adapter->func, SDIO_C2H_RP_REG, &ret);

	bufdsc = kmalloc(sizeof(*bufdsc), GFP_KERNEL);
	if (unlikely(!bufdsc)) {
		pr_err("%s bufdsc alloc fail", __func__);
		return NULL;
	}
	if (unlikely(ret)) {
		pr_err("%s sdio_readb error", __func__);
		schedule_work(&adapter->wimax_reset);
		goto err;
	}

#if 0
	/*check modem buffer overflow*/
	if (read_idx == sdio_readb(adapter->func, SDIO_C2H_WP_REG, &ret)) {
		read_idx = -1;
		goto err;
	}
#endif
#ifdef CMC7xx_MULTIPACKET_SUPPORT
	if (adapter->download_complete)
		t_len = sdio_readl(adapter->func, (SDIO_RX_BANK_ADDR +
					(read_idx * SDIO_RXBANK_SIZE)), &ret);
	else
#endif
		t_len = sdio_readl(adapter->func, (SDIO_RX_BANK_ADDR +
					(read_idx * SDIO_BANK_SIZE)), &ret);

	if (unlikely(ret)) {
		pr_err("%s sdio_readl error", __func__);
		schedule_work(&adapter->wimax_reset);
		goto err;
	}

#ifdef CMC7xx_MULTIPACKET_SUPPORT
	if (adapter->download_complete) {
		if (unlikely(t_len > (SDIO_RXBANK_SIZE -
						CMC732_PACKET_LENGTH_SIZE))) {
			pr_err("%s length out of bound", __func__);
			t_len = SDIO_RXBANK_SIZE - CMC732_PACKET_LENGTH_SIZE;
		}
		sdio_writeb(adapter->func, (read_idx + 1) % SDIO_RXBANK_COUNT,
			SDIO_C2H_RP_REG, NULL);
	}	else
#endif
	{
		if (unlikely(t_len > (SDIO_BANK_SIZE -
						CMC732_PACKET_LENGTH_SIZE))) {
			pr_err("%s length out of bound", __func__);
			t_len = SDIO_BANK_SIZE - CMC732_PACKET_LENGTH_SIZE;
		}
		sdio_writeb(adapter->func, (read_idx + 1) % 16,
			SDIO_C2H_RP_REG, NULL);
	}

	bufdsc->buffer = kmalloc(t_len, GFP_KERNEL);
	if (unlikely(!bufdsc->buffer)) {
		pr_err("%s bufdsc->buffer alloc fail", __func__);
		goto err;
	}

	bufdsc->length = (s32)t_len;
	t_buff = (u8 *)bufdsc->buffer;
#ifdef RX_SINGLE_BLOCK_MODE
#ifdef CMC7xx_MULTIPACKET_SUPPORT
	if (adapter->download_complete)
		t_index = (SDIO_RX_BANK_ADDR +
				(SDIO_RXBANK_SIZE * read_idx) + 4);
	else
#endif
		t_index = (SDIO_RX_BANK_ADDR + (SDIO_BANK_SIZE * read_idx) + 4);

	while (likely(t_len)) {
		t_size = (t_len > CMC_BLOCK_SIZE) ?
			(CMC_BLOCK_SIZE) : t_len;
		ret = sdio_memcpy_fromio(adapter->func, (void *)t_buff,
				t_index, t_size);

		if (unlikely(ret)) {
			pr_err("%s sdio_memcpy_fromio fail\n", __func__);
			schedule_work(&adapter->wimax_reset);
			goto err_2;
		}
		t_len -= t_size;
		t_buff += t_size;
		t_index += t_size;
	}
#else
		ret = sdio_memcpy_fromio(adapter->func, (void *)t_buff,
				t_index, t_len);

		if (unlikely(ret)) {
			pr_err("%s sdio_memcpy_fromio fail", __func__);
			schedule_work(&adapter->wimax_reset);
			goto err_2;
		}
#endif

	return bufdsc;

err_2:
	kfree(bufdsc->buffer);
err:
	kfree(bufdsc);
	adapter->netstats.rx_dropped++;
	return NULL;
}

void adapter_interrupt(struct sdio_func *func)
{
	struct net_adapter		*adapter = sdio_get_drvdata(func);
	int				intrd = 0;
	struct buffer_descriptor *bufdsc;

	wake_lock_timeout(&adapter->pdata->g_cfg->wimax_rxtx_lock, 0.2 * HZ);

	/* read interrupt identification register and clear the interrupt */
	intrd = sdio_readb(func, SDIO_INT_STATUS_REG, NULL);
	sdio_writeb(func, intrd, SDIO_INT_STATUS_CLR_REG, NULL);

	if (likely(intrd & SDIO_INT_DATA_READY)) {
		bufdsc = rx_packet(adapter);
		if (unlikely(!bufdsc))
			return;
		spin_lock(&adapter->recv_lock);
		list_add_tail(&bufdsc->list, &adapter->q_recv);
		spin_unlock(&adapter->recv_lock);
		schedule_work(&adapter->receive_work);
	} else if (unlikely(intrd & SDIO_INT_ERROR)) {
		adapter->netstats.rx_errors++;
		pr_err("%s intrd = SDIO_INT_ERROR occurred",
			__func__);
	}

}

int adapter_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
	struct net_adapter	*adapter;
	struct net_device	*net;
	u_char			charName[32];
	int			nRes = -ENOMEM;
	u_long			idx = 0;
	struct wimax732_platform_data	*pdata;

	dump_debug("Probe!!!!!!!!!");
	pdata = (struct wimax732_platform_data *) id->driver_data;
	net = alloc_etherdev(sizeof(struct net_adapter));
	if (!net) {
		dump_debug("adapter_probe: "
				"error can't allocate device");
		goto alloceth_fail;
	}

	adapter = netdev_priv(net);
	memset(adapter, 0, sizeof(struct net_adapter));
	g_adapter = adapter;

	adapter->pdata = (struct wimax732_platform_data *) id->driver_data;
	adapter->pdata->g_cfg->card_removed = false;
	adapter->pdata->g_cfg->powerup_done = false;

	/* Initialize control */
	control_init(adapter);

	/* initialize hardware */
	nRes = hw_init(adapter);

	if (nRes) {
		dump_debug("adapter_probe: error can't"
				"allocate receive buffer");
		goto hwInit_fail;
	}

	strcpy(net->name, "uwbr%d");

	adapter->func = func;
	adapter->net = net;
	net->netdev_ops = &wimax_net_ops;
	net->watchdog_timeo = ADAPTER_TIMEOUT;
	net->mtu = WIMAX_MTU_SIZE;
	adapter->msg_enable = netif_msg_init(msg_level, NETIF_MSG_DRV
			| NETIF_MSG_PROBE | NETIF_MSG_LINK);

	ether_setup(net);
	net->flags |= IFF_NOARP;

	adapter->media_state = MEDIA_DISCONNECTED;
	adapter->ready = FALSE;
	adapter->halted = FALSE;
	adapter->downloading = TRUE;
	adapter->removed = FALSE;
	adapter->mac_ready = FALSE;
	sdio_set_drvdata(func, adapter);

	SET_NETDEV_DEV(net, &func->dev);
	nRes = register_netdev(net);
	if (nRes)
		goto regdev_fail;

	netif_carrier_off(net);
	netif_tx_stop_all_queues(net);

	sdio_claim_host(adapter->func);
	nRes = sdio_enable_func(adapter->func);
	if (nRes < 0) {
		dump_debug("sdio_enable func error = %d", nRes);
		goto sdioen_fail;
	}

	nRes = sdio_claim_irq(adapter->func, adapter_interrupt);
	if (nRes < 0) {
		dump_debug("sdio_claim_irq = %d", nRes);
		goto sdioirq_fail;
	}
	sdio_set_block_size(adapter->func, 512);
	sdio_release_host(adapter->func);

	memset(charName, 0x00, sizeof(charName));
	create_char_name(charName, idx);
	if (misc_register(&uwibro_dev) != 0) {
		dump_debug("adapter_probe: misc_register() failed");
		goto regchar_fail;
	}

	/* Dummy value for "ifconfig up" for 2.6.24 */
	random_ether_addr(node_id);
	memcpy(net->dev_addr, node_id, sizeof(node_id));

	/* sangam dbg */
	INIT_WORK(&adapter->receive_work, rx_process_data);
	INIT_WORK(&adapter->transmit_work, hw_transmit_thread);
	INIT_WORK(&adapter->wimax_reset, cmc7xx_sdio_reset);

	if (hw_start(adapter)) {
		/* Free the resources and stop the driver processing */
		misc_deregister(&uwibro_dev);
		dump_debug("hw_start failed");
		goto regchar_fail;
	}

	adapter->ready = TRUE;

	return 0;

regchar_fail:
	sdio_claim_host(adapter->func);
	sdio_release_irq(adapter->func);
sdioirq_fail:
	sdio_disable_func(adapter->func);
sdioen_fail:
	sdio_release_host(adapter->func);
	unregister_netdev(adapter->net);
regdev_fail:
	sdio_set_drvdata(func, NULL);
	hw_remove(adapter);
hwInit_fail:
	free_netdev(net);
alloceth_fail:
	pdata->g_cfg->card_removed = true;
	pdata->g_cfg->powerup_done = true;
	pdata->power(0);
	return nRes;
}

void adapter_remove(struct sdio_func *func)
{
	struct net_adapter	*adapter = sdio_get_drvdata(func);

	dump_debug("%s", __func__);
	if (!adapter) {
		dump_debug("unregistering non-bound device?");
		return;
	}

	adapter->ready = FALSE;
	adapter->pdata->g_cfg->card_removed = TRUE;

	if (adapter->media_state == MEDIA_CONNECTED) {
		netif_stop_queue(adapter->net);
		adapter->media_state = MEDIA_DISCONNECTED;
	}

	/* remove adapter from adapters array */
	g_adapter = NULL;

	if (!adapter->removed)
		hw_stop(adapter);	/* free hw in and out buffer */

	if (adapter->downloading) {
		adapter->removed = TRUE;
		adapter->download_complete = TRUE;
		wake_up_interruptible(&adapter->download_event);
	}

	/* remove control process list */
	control_remove(adapter);

	/*remove hardware interface */
	hw_remove(adapter);

	misc_deregister(&uwibro_dev);
	if (adapter->net)
		unregister_netdev(adapter->net);
	cmc7xx_flush_skb(adapter);
	free_netdev(adapter->net);

	dump_debug("%s finished", __func__);
	return;
}



static ssize_t eeprom_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	dump_debug("Write EEPROM!!");

	eeprom_write_boot();
	eeprom_write_rev();
	return 0;

}

static ssize_t eeprom_store(struct device *dev,
		struct device_attribute *attr,
		const char *buffer, size_t count)
{
	struct wimax732_platform_data   *pdata = dev_get_drvdata(dev);

	if (count != 5)
		return count;

	if (strncmp(buffer, "wb00", 4) == 0) {
		dump_debug("Write EEPROM!!");
		eeprom_write_boot();
	} else if (strncmp(buffer, "rb00", 4) == 0) {
		dump_debug("Read Boot!!");
		eeprom_read_boot();
	} else if (strncmp(buffer, "re00", 4) == 0) {
		dump_debug("Read EEPROM!!");
		eeprom_read_all();
	} else if (strncmp(buffer, "rrdb", 4) == 0) {
		dump_debug("Read EEPROM(RDB)!!");
		eeprom_read_RDB();
	} else if (strncmp(buffer, "srdb", 4) == 0) {
		dump_debug("Store EEPROM(RDB)!!");
		eeprom_store_RDB();
	} else if (strncmp(buffer, "rcrt", 4) == 0) {
		dump_debug("Read EEPROM(CERTI)!!");
		eeprom_read_CERTI();
	} else if (strncmp(buffer, "ee00", 4) == 0) {
		dump_debug("Erase EEPROM!!");
		eeprom_erase_all();
	} else if (strncmp(buffer, "rcal", 4) == 0) {
		dump_debug("Check Cal!!");
		eeprom_check_cal();
	} else if (strncmp(buffer, "ecer", 4) == 0) {
		dump_debug("Erase Cert!!");
		eeprom_erase_cert();
	} else if (strncmp(buffer, "rcer", 4) == 0) {
		dump_debug("Check Cert!!");
		eeprom_check_cert();
	} else if (strncmp(buffer, "wrev", 4) == 0) {
		dump_debug("Write Rev!!");
		eeprom_write_rev();
	} else if (strncmp(buffer, "ons0", 4) == 0) {
		dump_debug("Power On - SDIO MODE!!");
		pdata->power(0);
		pdata->g_cfg->wimax_mode = SDIO_MODE ;
		pdata->power(1);
	} else if (strncmp(buffer, "off0", 4) == 0) {
		dump_debug("Power Off!!");
		pdata->power(0);
	} else if (strncmp(buffer, "wu00", 4) == 0) {
		dump_debug("WiMAX UART!!");
		pdata->uart_wimax();
	} else if (strncmp(buffer, "au00", 4) == 0) {
		dump_debug("AP UART!!");
		pdata->uart_ap();
	} else if (strncmp(buffer, "don0", 4) == 0) {
		dump_debug("Enable Dump!!");
		pdata->g_cfg->enable_dump_msg = 1;
	} else if (strncmp(buffer, "doff", 4) == 0) {
		dump_debug("Disable Dump!!");
		pdata->g_cfg->enable_dump_msg = 0;
	} else if (strncmp(buffer, "gpio", 4) == 0) {
		dump_debug("Display GPIOs!!");
		pdata->gpio_display();
	} else if (strncmp(buffer, "wake", 4) == 0) {
		dump_debug("WIMAX_WAKEUP!!");
		pdata->wakeup_assert(1);
		msleep(10);
		pdata->wakeup_assert(0);
	}

	return count - 1;


}

static ssize_t onoff_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct wimax732_platform_data   *pdata = dev_get_drvdata(dev);

	pdata->power(0);
	return 0;
}

static ssize_t onoff_store(struct device *dev,
		struct device_attribute *attr,
		const char *buffer, size_t count)
{
	struct wimax732_platform_data   *pdata = dev_get_drvdata(dev);

	if (buffer[0] == 's') {
		if (pdata->g_cfg->wimax_mode != SDIO_MODE ||
			 gpio_get_value(WIMAX_EN) == 0) {
			pdata->g_cfg->wimax_mode = SDIO_MODE;
			pdata->power(1);
		}
	} else if (buffer[0] == 'w') {
		if (pdata->g_cfg->wimax_mode != WTM_MODE ||
			 gpio_get_value(WIMAX_EN) == 0) {
			pdata->g_cfg->wimax_mode = WTM_MODE;
			pdata->power(1);
		}
	} else if (buffer[0] == 'u') {
		if (pdata->g_cfg->wimax_mode != USB_MODE ||
			 gpio_get_value(WIMAX_EN) == 0) {
			pdata->g_cfg->wimax_mode = USB_MODE;
			pdata->power(1);
		}
	} else if (buffer[0] == 'a') {
		if (pdata->g_cfg->wimax_mode != AUTH_MODE ||
			 gpio_get_value(WIMAX_EN) == 0) {
			pdata->g_cfg->wimax_mode = AUTH_MODE;
			pdata->power(1);
		}
	}

	return count - 1;


}

static ssize_t wmxuart_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct wimax732_platform_data   *pdata = dev_get_drvdata(dev);

	pdata->gpio_display();

	return 0;

}
static ssize_t wmxuart_store(struct device *dev,
		struct device_attribute *attr,
		const char *buffer, size_t count)
{
	struct wimax732_platform_data   *pdata = dev_get_drvdata(dev);

	if (buffer == NULL)
		return 0;

	if (buffer[0] == '0')
		pdata->uart_ap();
	else if (buffer[0] == '1')
		pdata->uart_wimax();

	return count - 1;


}

static ssize_t dump_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct wimax732_platform_data   *pdata = dev_get_drvdata(dev);
	pdata->power(0);
	eeprom_check_cal();

	return 0;


}
static ssize_t dump_store(struct device *dev,
		struct device_attribute *attr,
		const char *buffer, size_t count)
{
	struct wimax732_platform_data   *pdata = dev_get_drvdata(dev);

	if (buffer[0] == '0') {
		dump_debug("Control Dump Disabled.");
		pdata->g_cfg->enable_dump_msg = 0;
	} else if (buffer[0] == '1') {
		dump_debug("Control Dump Enabled.");
		pdata->g_cfg->enable_dump_msg = 1;
	}

	return count - 1;


}

static DEVICE_ATTR(eeprom, 0664, eeprom_show, eeprom_store);
static DEVICE_ATTR(onoff, 0664, onoff_show, onoff_store);
static DEVICE_ATTR(wmxuart, 0664, wmxuart_show, wmxuart_store);
static DEVICE_ATTR(dump, 0664, dump_show, dump_store);
static DEVICE_ATTR(sleepmode, 0664, NULL, NULL);

static int cmc7xx_wimax_pm_callback(struct notifier_block *nfb,
				unsigned long action, void *ignored)
{
	int ret = NOTIFY_DONE;
	struct wimax732_platform_data *pdata = container_of(nfb,
						struct wimax732_platform_data,
						pm_notifier);

	switch (action) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		pdata->g_cfg->wimax_suspend_prepare = true;;
		dump_debug("PM_SUSPEND_PREPARE: wimax");
		ret = NOTIFY_OK;
		break;
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		pdata->g_cfg->wimax_suspend_prepare = false;
		dump_debug("PM_POST_SUSPEND: wimax");
		ret = NOTIFY_OK;
		break;
	default:
		break;
	}
	return ret;
}

static void cmc7xx_register_pm_notifier(struct wimax732_platform_data *pdata)
{
	pdata->pm_notifier.notifier_call = cmc7xx_wimax_pm_callback;
	register_pm_notifier(&pdata->pm_notifier);
}

static void cmc7xx_unregister_pm_notifier(struct wimax732_platform_data *pdata)
{
	unregister_pm_notifier(&pdata->pm_notifier);
}

static int wimax_probe(struct platform_device *pdev)
{
	struct wimax732_platform_data   *pdata = pdev->dev.platform_data;
	struct device *dev_t;
	int	error = 0;
	int	err;
	int     i;

	dump_debug("SDIO driver installing... " WIMAX_DRIVER_VERSION_STRING);

	pdata->swmxctl_dev.minor = SWMXGPIOMINOR;
	pdata->swmxctl_dev.name = "swmxctl";
	pdata->swmxctl_dev.fops = &swmx_fops;

	mutex_init(&pdata->g_cfg->power_mutex);
	mutex_init(&pdata->g_cfg->rx_lock);
	misc_register(&pdata->swmxctl_dev);

	if (error < 0) {
		dump_debug("misc_register() failed");
		return error;
	}

	for (i = 0; i < ARRAY_SIZE(adapter_table); i++)
		adapter_table[i].driver_data =
			(unsigned long) pdev->dev.platform_data;

	/* register SDIO driver */
	error = sdio_register_driver(&adapter_driver);
	if (error < 0) {
		dump_debug("sdio_register_driver() failed");
		return error;
	}

	pdata->g_cfg->card_removed = true;
	pdata->g_cfg->wimax_suspend_prepare = false;
	cmc7xx_register_pm_notifier(pdata);
	pdata->power(0);
	/*Wimax sys entry*/

	pdata->wimax_class = class_create(THIS_MODULE, "wimax");
	if (IS_ERR(pdata->wimax_class))
		dump_debug("%s: class create"
			" failed\n", __func__);
	dev_t = device_create(pdata->wimax_class, NULL,
			MKDEV(SWMXGPIOMINOR, 0), "%s", "cmc732");
	if (IS_ERR(dev_t)) {
		dump_debug("%s: class create"
			" failed\n", __func__);
	}
	err = device_create_file(dev_t, &dev_attr_eeprom);
	if (err < 0) {
		dump_debug("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_eeprom.attr.name);
	}

	err = device_create_file(dev_t, &dev_attr_onoff);
	if (err < 0) {
		dump_debug("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_onoff.attr.name);
	}

	err = device_create_file(dev_t, &dev_attr_wmxuart);
	if (err < 0) {
		dump_debug("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_wmxuart.attr.name);
	}
	err = device_create_file(dev_t, &dev_attr_dump);
	if (err < 0) {
		dump_debug("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_dump.attr.name);
	}
	err = device_create_file(dev_t, &dev_attr_sleepmode);
	if (err < 0) {
		dump_debug("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_sleepmode.attr.name);
	}

	dev_set_drvdata(dev_t, pdata);

	/* End of Sysfs */

#ifndef DRIVER_BIT_BANG
	if (wmxeeprom_init())
		dump_debug("wmxeeprom_init() failed");
#endif
	/* initialize wake locks */
	wake_lock_init(&pdata->g_cfg->wimax_wake_lock,
			WAKE_LOCK_SUSPEND, "wimax_wakeup");
	wake_lock_init(&pdata->g_cfg->wimax_rxtx_lock,
			WAKE_LOCK_SUSPEND, "wimax_rxtx");
	wake_lock_init(&pdata->g_cfg->wimax_tx_lock,
			WAKE_LOCK_SUSPEND, "wimax_tx");


	return error;
}

static int wimax_remove(struct platform_device *pdev)
{
	struct wimax732_platform_data   *pdata = pdev->dev.platform_data;
	dump_debug("SDIO driver Uninstall");

	/* destroy wake locks */
	wake_lock_destroy(&pdata->g_cfg->wimax_wake_lock);
	wake_lock_destroy(&pdata->g_cfg->wimax_rxtx_lock);
	wake_lock_destroy(&pdata->g_cfg->wimax_tx_lock);
	class_destroy(pdata->wimax_class);
	cmc7xx_unregister_pm_notifier(pdata);
	sdio_unregister_driver(&adapter_driver);
	misc_deregister(&pdata->swmxctl_dev);
	mutex_destroy(&pdata->g_cfg->rx_lock);
	mutex_destroy(&pdata->g_cfg->power_mutex);
	return 0;
}
int wimax_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct wimax732_platform_data   *pdata;

	dump_debug("[wimax] %s", __func__);
	if (!g_adapter)
		return 0;

	pdata = pdev->dev.platform_data;

	/* AP active pin LOW */
	pdata->signal_ap_active(0);

	/* display WiMAX uart msg during AP suspend */
	if (pdata->g_cfg->enable_dump_msg == 1) {
		msleep(10);
		pdata->uart_wimax();
	}

	return 0;
}

/* wimax resume function */
int wimax_resume(struct platform_device *pdev)
{
	struct wimax732_platform_data   *pdata;

	dump_debug("[wimax] %s", __func__);

	if (!g_adapter)
		return 0;

	pdata = pdev->dev.platform_data;

	if (pdata->g_cfg->card_removed)
		return 0;

	/* AP active pin HIGH */
	pdata->signal_ap_active(1);

	/* wait wakeup noti for 1 sec otherwise suspend again */
	wake_lock_timeout(&pdata->g_cfg->wimax_wake_lock, 1 * HZ);

	return 0;
}

static struct platform_driver wimax_driver = {
	.probe          = wimax_probe,
	.remove         = wimax_remove,
	.suspend                = wimax_suspend,
	.resume                 = wimax_resume,
	.driver         = {
		.name   = "wimax732_driver",
	}
};

static int __init adapter_init_module(void)
{
	return platform_driver_register(&wimax_driver);
}

static void __exit adapter_deinit_module(void)
{
	platform_driver_unregister(&wimax_driver);
}


module_init(adapter_init_module);
module_exit(adapter_deinit_module);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_PARM_DESC(msg_level, "Override default message level");
MODULE_DEVICE_TABLE(sdio, adapter_table);
MODULE_VERSION(WIMAX_DRIVER_VERSION_STRING);
MODULE_LICENSE("GPL");
