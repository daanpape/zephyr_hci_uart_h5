/*
 * Copyright (c) 2016 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <init.h>
#include <zephyr.h>
#include <device.h>
#include <net/buf.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <drivers/uart.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/buf.h>
#include <bluetooth/hci_raw.h>

#define LOG_MODULE_NAME hci_uart_h5
#include "common/log.h"

/*
 * HCI H:5 packet types
 */
#define HCI_3WIRE_ACK_PKT	0x00
#define HCI_COMMAND_PKT		0x01
#define HCI_ACLDATA_PKT		0x02
#define HCI_SCODATA_PKT		0x03
#define HCI_EVENT_PKT		0x04
#define HCI_ISODATA_PKT		0x05
#define HCI_3WIRE_LINK_PKT	0x0f
#define HCI_VENDOR_PKT		0xff

/* HCI command buffers */
#define CMD_BUF_SIZE BT_BUF_RX_SIZE
NET_BUF_POOL_DEFINE(cmd_tx_pool, CONFIG_BT_HCI_CMD_COUNT, CMD_BUF_SIZE,
		    4, NULL);

#define BT_L2CAP_MTU (CONFIG_BT_CTLR_TX_BUFFER_SIZE - BT_L2CAP_HDR_SIZE)

/** Data size needed for ACL buffers */
#define BT_BUF_ACL_SIZE BT_L2CAP_BUF_SIZE(BT_L2CAP_MTU)

#if defined(CONFIG_BT_CTLR_TX_BUFFERS)
#define TX_BUF_COUNT CONFIG_BT_CTLR_TX_BUFFERS
#else
#define TX_BUF_COUNT 6
#endif

static const struct device *hci_uart_dev;

NET_BUF_POOL_DEFINE(acl_tx_pool, TX_BUF_COUNT, BT_BUF_ACL_SIZE,
		    4, NULL);

// static K_THREAD_STACK_DEFINE(tx_stack, 256);
// static K_THREAD_STACK_DEFINE(unproc_stack, 256);
static K_KERNEL_STACK_DEFINE(tx_stack, 256);
static K_KERNEL_STACK_DEFINE(unproc_stack, 256);

static struct k_thread tx_thread_data;
static struct k_thread unproc_thread_data;

static struct k_delayed_work ack_work;
static struct k_delayed_work retx_work;



static bool reliable_packet(uint8_t type)
{
	switch (type) {
	case HCI_COMMAND_PKT:
	case HCI_ACLDATA_PKT:
	case HCI_EVENT_PKT:
	case HCI_ISODATA_PKT:
		return true;
	default:
		return false;
	}
}

/* FIXME: Correct timeout */
#define H5_RX_ACK_TIMEOUT	K_MSEC(250)
#define H5_TX_ACK_TIMEOUT	K_MSEC(250)

#define SLIP_DELIMITER	0xc0
#define SLIP_ESC		0xdb
#define SLIP_ESC_DELIM	0xdc
#define SLIP_ESC_ESC	0xdd

#define H5_RX_ESC		1
#define H5_TX_ACK_PEND	2

#define H5_HDR_SEQ(hdr)		((hdr)[0] & 0x07)
#define H5_HDR_ACK(hdr)		(((hdr)[0] >> 3) & 0x07)
#define H5_HDR_CRC(hdr)		(((hdr)[0] >> 6) & 0x01)
#define H5_HDR_RELIABLE(hdr)	(((hdr)[0] >> 7) & 0x01)
#define H5_HDR_PKT_TYPE(hdr)	((hdr)[1] & 0x0f)
#define H5_HDR_LEN(hdr)		((((hdr)[1] >> 4) & 0x0f) + ((hdr)[2] << 4))

#define H5_SET_SEQ(hdr, seq)	((hdr)[0] |= (seq))
#define H5_SET_ACK(hdr, ack)	((hdr)[0] |= (ack) << 3)
#define H5_SET_RELIABLE(hdr)	((hdr)[0] |= 1 << 7)
#define H5_SET_TYPE(hdr, type)	((hdr)[1] |= type)
#define H5_SET_LEN(hdr, len)	(((hdr)[1] |= ((len) & 0x0f) << 4), \
				 ((hdr)[2] |= (len) >> 4))

static struct h5 {
	struct net_buf		*rx_buf;

	struct k_fifo		controller_queue;
	struct k_fifo		host_queue;
	struct k_fifo		unack_queue;
	struct k_fifo		unprocessed_queue;

	uint8_t			tx_win;
	uint8_t			tx_ack;
	uint8_t			tx_seq;

	uint8_t			rx_ack;

	enum {
		UNINIT,
		INIT,
		ACTIVE,
	}			link_state;

	enum {
		START,
		HEADER,
		PAYLOAD,
		END,
	}			rx_state;
} h5;

static uint8_t unack_queue_len;

#define MAX_PACKETS_IN_FLIGHT (0x01)

static const uint8_t sync_req[] = { 0x01, 0x7e };
static const uint8_t sync_rsp[] = { 0x02, 0x7d };
/* Third byte may change */
static uint8_t conf_req[] = { 0x03, 0xfc };
static const uint8_t conf_rsp[] = { 0x04, 0x7b, MAX_PACKETS_IN_FLIGHT };

/* H5 signal buffers pool */
#define MAX_SIG_LEN	3
#define SIGNAL_COUNT 10
#define SIG_BUF_SIZE (BT_BUF_RESERVE + MAX_SIG_LEN)
NET_BUF_POOL_DEFINE(h5_pool, SIGNAL_COUNT, SIG_BUF_SIZE, 0, NULL);

/* H5 Packet Buf */
#define MAX_PACKET_LEN 255 // CMD Header + 255 max payload
#define PACKET_BUF_SIZE (BT_BUF_RESERVE +  MAX_PACKET_LEN)
NET_BUF_POOL_DEFINE(h5_pack_pool, MAX_PACKETS_IN_FLIGHT + 10, PACKET_BUF_SIZE, 0, NULL);

static inline void bt_uart_drain(const struct device *dev)
{
	uint8_t c;

	while (uart_fifo_read(dev, &c, 1)) {
		continue;
	}
}

/*
static void process_unack(void)
{
	uint8_t next_seq = h5.tx_seq;
	uint8_t number_removed = unack_queue_len;

	if (!unack_queue_len) {
		return;
	}

	LOG_DBG("rx_ack %u tx_ack %u tx_seq %u unack_queue_len %u",
	       h5.rx_ack, h5.tx_ack, h5.tx_seq, unack_queue_len);

	while (unack_queue_len > 0) {
		if (next_seq == h5.rx_ack) {
			// Next sequence number is the same as last received
			// ack number
			//
			break;
		}

		number_removed--;
		// Similar to (n - 1) % 8 with unsigned conversion
		next_seq = (next_seq - 1) & 0x07;
	}

	if (next_seq != h5.rx_ack) {
		LOG_ERR("Wrong sequence: rx_ack %u tx_seq %u next_seq %u",
		       h5.rx_ack, h5.tx_seq, next_seq);
	}

	LOG_DBG("Need to remove %u packet from the queue", number_removed);

	while (number_removed) {
		struct net_buf *buf = net_buf_get(&h5.unack_queue, K_NO_WAIT);

		if (!buf) {
			LOG_ERR("Unack queue is empty");
			break;
		}

		// TODO: print or do something with packet
		LOG_DBG("Remove buf from the unack_queue");

		//net_buf_unref(buf);
		unack_queue_len--;
		number_removed--;
	}
}

static void h5_print_header(const uint8_t *hdr, const char *str)
{
	if (H5_HDR_RELIABLE(hdr)) {
		LOG_DBG("%s REL: seq %u ack %u crc %u type %u len %u",
		       str, H5_HDR_SEQ(hdr), H5_HDR_ACK(hdr),
		       H5_HDR_CRC(hdr), H5_HDR_PKT_TYPE(hdr),
		       H5_HDR_LEN(hdr));
	} else {
		LOG_DBG("%s UNREL: ack %u crc %u type %u len %u",
		       str, H5_HDR_ACK(hdr), H5_HDR_CRC(hdr),
		       H5_HDR_PKT_TYPE(hdr), H5_HDR_LEN(hdr));
	}
}
*/

static uint8_t h5_slip_byte(uint8_t byte)
{
	switch (byte) {
	case SLIP_DELIMITER:
		uart_poll_out(hci_uart_dev, SLIP_ESC);
		uart_poll_out(hci_uart_dev, SLIP_ESC_DELIM);
		return 2;
	case SLIP_ESC:
		uart_poll_out(hci_uart_dev, SLIP_ESC);
		uart_poll_out(hci_uart_dev, SLIP_ESC_ESC);
		return 2;
	default:
		uart_poll_out(hci_uart_dev, byte);
		return 1;
	}
}

void h5_send(const uint8_t *payload, uint8_t type, int len)
{
	uint8_t hdr[4];
	int i;

	memset(hdr, 0, sizeof(hdr));

	/* Set ACK for outgoing packet and stop delayed work */
	H5_SET_ACK(hdr, h5.tx_ack);
	k_delayed_work_cancel(&ack_work);

	if (reliable_packet(type)) {
		H5_SET_RELIABLE(hdr);
		H5_SET_SEQ(hdr, h5.tx_seq);
		h5.tx_seq = (h5.tx_seq + 1) % 8;
	}

	H5_SET_TYPE(hdr, type);
	H5_SET_LEN(hdr, len);

	/* Calculate CRC */
	hdr[3] = ~((hdr[0] + hdr[1] + hdr[2]) & 0xff);

	//h5_print_header(hdr, "TX: <");

	uart_poll_out(hci_uart_dev, SLIP_DELIMITER);

	for (i = 0; i < 4; i++) {
		h5_slip_byte(hdr[i]);
	}

	for (i = 0; i < len; i++) {
		h5_slip_byte(payload[i]);
	}

	uart_poll_out(hci_uart_dev, SLIP_DELIMITER);
}

/* Delayed work taking care about retransmitting packets */
static void retx_timeout(struct k_work *work)
{
	ARG_UNUSED(work);

	LOG_DBG("unack_queue_len %u", unack_queue_len);

	if (unack_queue_len) {
		struct k_fifo tmp_queue;
		struct net_buf *buf;

		k_fifo_init(&tmp_queue);

		/* Queue to temperary queue */
		while ((buf = net_buf_get(&h5.host_queue, K_NO_WAIT))) {
			net_buf_put(&tmp_queue, buf);
		}

		/* Queue unack packets to the beginning of the queue */
		while ((buf = net_buf_get(&h5.unack_queue, K_NO_WAIT))) {
			/* include also packet type */
			net_buf_put(&h5.host_queue, buf);
			h5.tx_seq = (h5.tx_seq - 1) & 0x07;
			unack_queue_len--;
		}

		/* Queue saved packets from temp queue */
		while ((buf = net_buf_get(&tmp_queue, K_NO_WAIT))) {
			net_buf_put(&h5.host_queue, buf);
		}
	}
}

static void ack_timeout(struct k_work *work)
{
	ARG_UNUSED(work);

	LOG_DBG("");

	h5_send(NULL, HCI_3WIRE_ACK_PKT, 0);

	/* Analyze stacks */
	//STACK_ANALYZE("tx_stack", tx_stack);
	//STACK_ANALYZE("rx_stack", rx_stack);
}

int unslip_next_byte(struct net_buf *buf) {
	if (!buf->len) {
		return -1;
	}
	uint8_t next = net_buf_pull_u8(buf);
	if (next != SLIP_ESC) {
		return next;
	}
	if (!buf->len) {
		return -1;
	}
	next = net_buf_pull_u8(buf);
	if (next == SLIP_ESC_ESC) {
		return SLIP_ESC;
	}
	if (next == SLIP_ESC_DELIM) {
		return SLIP_DELIMITER;
	}
	LOG_WRN("Bad Escape Seqence: %02X %02X", SLIP_ESC, next);
	return -2;
}

static void bt_uart_isr(const struct device *unused, void *user_data)
{
	static uint8_t byte;
	static struct net_buf *buf = NULL;

	ARG_UNUSED(unused);
	ARG_UNUSED(user_data);

	while (uart_irq_update(hci_uart_dev) &&
	       uart_irq_is_pending(hci_uart_dev)) {

		if (!uart_irq_rx_ready(hci_uart_dev)) {
			/* Only the UART RX path is interrupt-enabled */
			break;
		}

		if (!buf) {
			buf = net_buf_alloc(&h5_pack_pool, K_NO_WAIT);
			if (!buf) {
				bt_uart_drain(hci_uart_dev);
				break;
			}
		}

		if (!uart_fifo_read(hci_uart_dev, &byte, sizeof(byte))) {
			continue;
		}
		if (byte == SLIP_DELIMITER) {
			if (buf->len > 0) {
				net_buf_put(&h5.unprocessed_queue, buf);
				buf = NULL;
			}
		} else {
			net_buf_add_u8(buf, byte);
		}
	}
}

int pull_header(struct net_buf *buf, uint8_t *hdr) {
	// Packet too short to contain an h5 header
	if (buf->len < 4) {
		return -1;
	}

	for (uint8_t i = 0; i < 4; i++) {
		int byte = unslip_next_byte(buf);
		if (byte < 0) {
			// Packet too short due to escaped bytes
			return -1;
		}
		hdr[i] = byte;
	}

	// Checksum
	if (((hdr[3] + hdr[0] + hdr[1] + hdr[2]) & 0xff) != 0xff) {
		LOG_WRN("Invalid Header Checksum\n");
	}

	return 0;
}

static void unproc_thread(void) {
	struct net_buf *buf;

	while (true) {
		buf = net_buf_get(&h5.unprocessed_queue, K_FOREVER);

		uint8_t hdr[4];
		if (pull_header(buf, hdr) < 0) {
			// Header is invalid
			goto next;
		}

		struct net_buf *rx_buf = NULL;

		switch (H5_HDR_PKT_TYPE(hdr)) {
        case HCI_ACLDATA_PKT:
            rx_buf = net_buf_alloc(&acl_tx_pool, K_NO_WAIT);
            bt_buf_set_type(rx_buf, BT_BUF_ACL_OUT);
            break;
        case HCI_COMMAND_PKT:
            rx_buf = net_buf_alloc(&cmd_tx_pool, K_NO_WAIT);
            bt_buf_set_type(rx_buf, BT_BUF_CMD);
            break;
        case HCI_3WIRE_ACK_PKT:
            h5.rx_ack = H5_HDR_ACK(hdr);
            goto next;
            break;
        case HCI_3WIRE_LINK_PKT:
            rx_buf = net_buf_alloc(&h5_pool, K_NO_WAIT);
            break;
        default:
            LOG_ERR("Wrong packet type from host: %u", H5_HDR_PKT_TYPE(hdr));
            goto next;
        }

		int byte;
		while ((byte = unslip_next_byte(buf)) >= 0) {
			net_buf_add_u8(rx_buf, (uint8_t) byte);
		}

		if (H5_HDR_LEN(hdr) != rx_buf->len) {
			LOG_ERR("Payload too short\n");
			goto next;
		}

        if (H5_HDR_RELIABLE(hdr) &&
	    	H5_HDR_SEQ(hdr) != h5.tx_ack) {
			LOG_ERR("Seq expected %u got %u. Drop packet", h5.tx_ack, H5_HDR_SEQ(hdr));
            goto next;
        }

		h5.rx_ack = H5_HDR_ACK(hdr);

		if (reliable_packet(H5_HDR_PKT_TYPE(hdr))) {
			/* For reliable packet increment next transmit ack number */
            h5.tx_ack = (h5.tx_ack + 1) % 8;
            /* Submit delayed work to ack the packet */
            k_delayed_work_submit(&ack_work, H5_RX_ACK_TIMEOUT);
		}

        switch (H5_HDR_PKT_TYPE(hdr)) {
        case HCI_3WIRE_ACK_PKT:
            // No further action required
            break;
        case HCI_3WIRE_LINK_PKT:
            net_buf_put(&h5.host_queue, rx_buf);
            break;
        case HCI_COMMAND_PKT:
        case HCI_ACLDATA_PKT:
            //LOG_DBG("Adding to controller queue\n");
            net_buf_put(&h5.controller_queue, rx_buf);
            break;
        default:
            LOG_WRN("Unknown packet type %u\n", H5_HDR_PKT_TYPE(hdr));
            break;
        }
next:
		net_buf_unref(buf);
	}
}

static void h5_send_sync(void);
static void h5_send_config(void);

static void tx_thread(void)
{
	LOG_DBG("TX Thread is alive.");

	while (true) {
		struct net_buf *buf;

		switch (h5.link_state) {
		case UNINIT:
			h5_send_sync();
			k_sleep(K_MSEC(250));
		case INIT:
			h5_send_config();
			k_sleep(K_MSEC(250));
			break;
		case ACTIVE:
			buf = net_buf_get(&h5.controller_queue, K_MSEC(250));
			if (!buf) {
				break;
			}
			//LOG_HEXDUMP_DBG(buf->data, buf->len, "TX_QUEUE -> CTRL");
            bt_send(buf);

			/* buf is dequeued from tx_queue and queued to unack
			 * queue.
			 */

			//net_buf_put(&h5.unack_queue, buf);
			//unack_queue_len++;

			//k_delayed_work_submit(&retx_work, H5_TX_ACK_TIMEOUT);

			break;
		}
	}
}

static void h5_init(void)
{
	LOG_DBG("");

	h5.link_state = UNINIT;
	h5.rx_state = START;
	h5.tx_win = 4;

	/* TX thread */
	k_fifo_init(&h5.controller_queue);
	k_thread_create(&tx_thread_data, tx_stack,
			K_KERNEL_STACK_SIZEOF(tx_stack),
			(k_thread_entry_t)tx_thread, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_HCI_TX_PRIO),
			0, K_NO_WAIT);

	k_fifo_init(&h5.host_queue);
	/*k_thread_create(&rx_thread_data, rx_stack,
			K_KERNEL_STACK_SIZEOF(rx_stack),
			(k_thread_entry_t)rx_thread, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_RX_PRIO),
			0, K_NO_WAIT);*/

	/* Unack queue */
	k_fifo_init(&h5.unack_queue);

	/* Thread & queue to un-slip and un-h5 incoming packets */
	k_fifo_init(&h5.unprocessed_queue);
	k_thread_create(&unproc_thread_data, unproc_stack,
			K_KERNEL_STACK_SIZEOF(unproc_stack),
			(k_thread_entry_t)unproc_thread, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_RX_PRIO),
			0, K_NO_WAIT);

	/* Init delayed work */
	k_delayed_work_init(&ack_work, ack_timeout);
	k_delayed_work_init(&retx_work, retx_timeout);
}

static int h5_open(const struct device *unused)
{
	LOG_DBG("Open");

	hci_uart_dev = device_get_binding(CONFIG_BT_CTLR_TO_HOST_UART_DEV_NAME);
	if (!hci_uart_dev) {
		return -EINVAL;
	}

	uart_irq_rx_disable(hci_uart_dev);
	uart_irq_tx_disable(hci_uart_dev);

	bt_uart_drain(hci_uart_dev);

	uart_irq_callback_set(hci_uart_dev, bt_uart_isr);

	h5_init();

	uart_irq_rx_enable(hci_uart_dev);

	return 0;
}

#if defined(CONFIG_BT_CTLR_ASSERT_HANDLER)
void bt_ctlr_assert_handle(char *file, uint32_t line)
{
	printk("Assert %s:%u", file, line);
}
#endif /* CONFIG_BT_CTLR_ASSERT_HANDLER */

DEVICE_INIT(hci_uart, "hci_uart", &h5_open, NULL, NULL,
	    APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

bool _link_ctrl_memcmp(struct net_buf const * const buf, uint8_t const * const ref) {
	return !memcmp(buf->data, ref, 2);
}

bool packet_is_sync(struct net_buf *buf) {
	return _link_ctrl_memcmp(buf, sync_req);
}

bool packet_is_sync_response(struct net_buf *buf) {
	return _link_ctrl_memcmp(buf, sync_rsp);
}

bool packet_is_config(struct net_buf *buf) {
	return _link_ctrl_memcmp(buf, conf_req);
}

static bool packet_is_config_response(struct net_buf *buf) {
	return _link_ctrl_memcmp(buf, conf_rsp);
}

static void _send_link_control(uint8_t const * const buf, uint8_t len) {
	h5_send(buf, HCI_3WIRE_LINK_PKT, len);
}

static void h5_send_sync(void) {
	_send_link_control(sync_req, sizeof(sync_req));
}
static void h5_send_sync_response(void) {
	_send_link_control(sync_rsp, sizeof(sync_rsp));
}
static void h5_send_config(void) {
	_send_link_control(conf_req, sizeof(conf_req));
}
static void h5_send_config_response(void) {
	_send_link_control(conf_rsp, sizeof(conf_rsp));
}

void main(void)
{
	LOG_DBG("Start");
	// Adds controller output to host output queue
	bt_enable_raw(&h5.host_queue);
	struct net_buf *buf = NULL;

	while (true) {
		buf = net_buf_get(&h5.host_queue, K_FOREVER);

		if (packet_is_sync(buf)) {
			h5.link_state = UNINIT;
			h5_send_sync_response();
			goto next;
		}

		if (h5.link_state == UNINIT) {
			if (packet_is_sync_response(buf)) {
				h5.link_state = INIT;
				h5_send_config();
			} else {
				/* SYNC is the answer to any non-SYNC_RESP packets in UNINIT
				   state */
				h5_send_sync();
			}
		} else if (h5.link_state == INIT) {
			if (packet_is_config(buf)) {
				h5_send_config_response();
			} else if (packet_is_config_response(buf)) {
				h5.link_state = ACTIVE;
				h5.tx_win = conf_rsp[2] & 0x7;
				h5.tx_seq = 0;
				h5.tx_ack = 0;
				LOG_DBG("Finished H5 configuration, tx_win %u", h5.tx_win);
			}
		} else if (h5.link_state == ACTIVE) {
			if (packet_is_config(buf)) {
				h5_send_config_response();
			} else if (packet_is_config_response(buf)) {
				goto next;
			} else if (packet_is_sync_response(buf) || packet_is_config(buf)) {
				h5.link_state = UNINIT;
				h5_send_sync();
			} else {
				// Presumably something from the controller
				uint8_t type = bt_buf_get_type(buf);
				if (type == BT_BUF_EVT) {
				    LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL -> HOST");
					h5_send(buf->data, HCI_EVENT_PKT, buf->len);
				} else {
				    LOG_HEXDUMP_ERR(buf->data, buf->len, "Unexpected buffer in host_queue");
				}
			}
		}
next:
		net_buf_unref(buf);
	}
}


