/*
 * control.c
 *
 * send and receive control packet and handle it
 */
#include "headers.h"

u_int control_init(struct net_adapter *adapter)
{
	INIT_LIST_HEAD(&adapter->q_recv);
	spin_lock_init(&adapter->recv_lock);
	INIT_LIST_HEAD(&adapter->ctl.q_received.list);
	spin_lock_init(&adapter->ctl.q_received.lock);

	INIT_LIST_HEAD(&adapter->ctl.apps.process_list);
	spin_lock_init(&adapter->ctl.apps.lock);

	adapter->ctl.apps.ready = TRUE;

	return STATUS_SUCCESS;
}

void control_remove(struct net_adapter *adapter)
{
	struct list_head	*pos, *nxt;
	struct buffer_descriptor	*bufdsc;
	struct process_descriptor	*procdsc;
	unsigned long			flags;

	/* Free the received control packets queue */
	list_for_each_safe(pos, nxt, &adapter->ctl.q_received.list) {
		bufdsc = list_entry(pos, struct buffer_descriptor, list);
		list_del(pos);
		kfree(bufdsc->buffer);
		kfree(bufdsc);
	}

	if (!adapter->ctl.apps.ready)
		return;

	/* process list */
	list_for_each_safe(pos, nxt, &adapter->ctl.apps.process_list) {
		procdsc = list_entry(pos,
				struct process_descriptor, list);
		spin_lock_irqsave(&adapter->ctl.apps.lock, flags);
		if (procdsc->irp) {
			procdsc->irp = false;
			wake_up_interruptible
				(&procdsc->read_wait);
		}
		spin_unlock_irqrestore(&adapter->ctl.apps.lock, flags);
	}
	/* delay for the process release */
	msleep(100);

	adapter->ctl.apps.ready = FALSE;
}

/* add received packet to pending list */
static void control_enqueue_buffer(struct net_adapter *adapter,
					void *buffer, u32 length)
{
	struct buffer_descriptor	*bufdsc;
	struct process_descriptor	*procdsc;
	struct eth_header			hdr;

	/* get the packet type for the process check */
	memcpy(&hdr.type, (unsigned short *)buffer, sizeof(unsigned short));

	/* Queue and wake read only if process exist. */
	procdsc = process_by_type(adapter, hdr.type);
	if (!procdsc) {
		pr_debug("Waiting process not found skip the packet");
		return;
	}

	bufdsc = (struct buffer_descriptor *)
		kmalloc(sizeof(*bufdsc), GFP_ATOMIC);
	if (bufdsc == NULL) {
		pr_debug("bufdsc Memory Alloc Failure *****");
		return;
	}
	bufdsc->buffer = kmalloc(
		(length + (ETHERNET_ADDRESS_LENGTH * 2)), GFP_ATOMIC);
	if (bufdsc->buffer == NULL) {
		kfree(bufdsc);
		pr_debug("bufdsc->buffer Memory Alloc Failure *****");
		return;
	}

	/* add ethernet header to control packet */
	memcpy(bufdsc->buffer, adapter->hw.eth_header,
			(ETHERNET_ADDRESS_LENGTH * 2));
	memcpy(bufdsc->buffer + (ETHERNET_ADDRESS_LENGTH * 2),
			buffer, length);

	/* fill out descriptor */
	bufdsc->length = length + (ETHERNET_ADDRESS_LENGTH * 2);
	bufdsc->type = hdr.type;

	/* add to pending list */
	list_add_tail(&bufdsc->list, &adapter->ctl.q_received.list);

	if (procdsc->irp) {
		procdsc->irp = false;
		wake_up_interruptible(&procdsc->read_wait);
	}
}

/* receive control data */
void control_recv(struct net_adapter *adapter, void *buffer, u32 length)
{
	/* check halt flag */
	if (adapter->halted)
		return;

	/* not found, add to pending buffers list */
	spin_lock(&adapter->ctl.q_received.lock);
	control_enqueue_buffer(adapter, buffer, length);
	spin_unlock(&adapter->ctl.q_received.lock);
}

u_int control_send(struct net_adapter *adapter, void *buffer, u_long length)
{
	if ((length + sizeof(struct hw_packet_header)) >= WIMAX_MAX_TOTAL_SIZE)
		return STATUS_RESOURCES;
		/* changed from SUCCESS return status */

	return hw_send_data(adapter, buffer, length, CONTROL_PACKET);

}

struct process_descriptor *process_by_id(struct net_adapter *adapter, u32 id)
{
	struct process_descriptor	*procdsc;

	list_for_each_entry(procdsc, &adapter->ctl.apps.process_list, list) {
		if (procdsc->id == id)	/* process found */
			return procdsc;
	}
	return NULL;
}

struct process_descriptor
	*process_by_type(struct net_adapter *adapter, u16 type)
{
	struct process_descriptor	*procdsc;

	list_for_each_entry(procdsc, &adapter->ctl.apps.process_list, list) {
		if (procdsc->type == type)	/* process found */
			return procdsc;
	}
	return NULL;
}

/* find buffer by buffer type */
struct buffer_descriptor
*buffer_by_type(struct net_adapter *adapter, u16 type)
{
	struct buffer_descriptor	*bufdsc;

	list_for_each_entry(bufdsc, &adapter->ctl.q_received.list, list) {
		if (bufdsc->type == type)	/* process found */ {
			return bufdsc;
		}
	}
	return NULL;
}

void dump_buffer(const char *desc, u_char *buffer, u_int len)
{
	char	print_buf[256] = {0};
	char	chr[8] = {0};
	int	i;

	/* dump packets */
	u_char  *b = buffer;
	dump_debug("%s (%d) =>", desc, len);

	for (i = 0; i < len; i++) {
		sprintf(chr, " %02x", b[i]);
		strcat(print_buf, chr);
		if (((i + 1) != len) && (i % 16 == 15)) {
			dump_debug(print_buf);
			memset(print_buf, 0x0, 256);
		}
	}
	dump_debug(print_buf);
}

void remove_process(struct net_adapter *adapter, u32 id)
{
	struct process_descriptor	*procdsc;
	struct list_head	*pos, *nxt;

	list_for_each_safe(pos, nxt, &adapter->ctl.apps.process_list) {
		procdsc = list_entry(pos, struct process_descriptor, list);
		if (procdsc->id == id) {
			list_del(pos);
			kfree(procdsc);
		}
	}
}
