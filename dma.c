/*
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "mt7601u.h"
#include "dma.h"
#include "trace.h"

static void mt7601u_complete_rx(struct urb *urb);

void mt7601u_complete_urb(struct urb *urb)
{
	struct completion *cmpl = urb->context;

	/* TODO: handle errors */

	complete(cmpl);
}

static u16 mt7601u_rx_next_seg_len(u8 *data, u32 data_len)
{
	u32 min_seg_len = MT_DMA_HDR_LEN + MT_RX_INFO_LEN +
		sizeof(struct mt7601u_rxwi) + MT_FCE_INFO_LEN;
	u16 dma_len = get_unaligned_le16(data);

	if (data_len < min_seg_len)
		return 0;
	if (WARN_ON(!dma_len))
		return 0;
	if (WARN_ON(dma_len + MT_DMA_HDRS > data_len))
		return 0;
	if (WARN_ON(dma_len & 0x3))
		return 0;

	return MT_DMA_HDRS + dma_len;
}

static void
mt7601u_rx_process_seg(struct mt7601u_dev *dev, u8 *data, u32 seg_len)
{
	struct sk_buff *skb;
	struct mt7601u_rxwi *rxwi;
	u32 fce_info;

	/* TODO: drop this debug check */
	fce_info = get_unaligned_le32(data + seg_len - MT_FCE_INFO_LEN);
	if (seg_len - MT_DMA_HDRS != MT76_GET(MT_RX_FCE_INFO_LEN, fce_info))
		printk("Error: dma_len does not match fce_len\n");
	seg_len -= MT_FCE_INFO_LEN;

	data += MT_DMA_HDR_LEN;
	seg_len -= MT_DMA_HDR_LEN;

	rxwi = (struct mt7601u_rxwi *) data;
	data += sizeof(struct mt7601u_rxwi);
	seg_len -= sizeof(struct mt7601u_rxwi);

	/* TODO: make sure zero fields are zero */
	/* TODO: make sure it's a packet (fce->info_type == 0) */
	trace_mt_rx(rxwi, fce_info);

	skb = alloc_skb(seg_len, GFP_ATOMIC);
	if (!skb) {
		printk("Error: rx failed to allocate skb\n");
		return;
	}

	/* TODO: copy in a clever way - to avoid later moves */
	memcpy(skb_put(skb, seg_len), data, seg_len);

	memset(skb->cb, 0, sizeof(skb->cb));
	if (mt76_mac_process_rx(dev, skb, rxwi)) {
	    dev_kfree_skb(skb);
	    return;
	}

	ieee80211_rx_ni(dev->hw, skb);
}

static void
mt7601u_rx_process_entry(struct mt7601u_dev *dev, struct mt7601u_dma_buf *e)
{
	u32 seg_len, data_len = e->urb->actual_length;
	u8 *data = e->buf;
	int cnt = 0;

	while ((seg_len = mt7601u_rx_next_seg_len(data, data_len))) {
		mt7601u_rx_process_seg(dev, data, seg_len);

		data_len -= seg_len;
		data += seg_len;
		cnt++;
	}

	if (cnt > 1)
		trace_rx_dma_aggr(cnt);
}

static struct mt7601u_dma_buf *
mt7601u_rx_get_pending_entry(struct mt7601u_dev *dev)
{
	struct mt7601u_rx_queue *q = &dev->rx_q;
	struct mt7601u_dma_buf *buf = NULL;
	unsigned long flags;

	spin_lock_irqsave(&dev->rx_lock, flags);

	if (!q->pending) {
		if (q->start != q->end) /* TODO: remove this debug check */
			printk("Error: rx queue corrupted %d/%d\n",
			       q->start, q->end);
		goto out;
	}

	buf = &q->e[q->start];
	q->pending--;
	q->start = (q->start + 1) % q->entries;
out:
	spin_unlock_irqrestore(&dev->rx_lock, flags);

	return buf;
}

static int mt7601u_rx_submit_entry(struct mt7601u_dev *dev,
				   struct mt7601u_dma_buf *e, gfp_t gfp)
{
	struct usb_device *usb_dev = to_usb_device(dev->dev->parent);
	unsigned recv_pipe = usb_rcvbulkpipe(usb_dev, dev->in_eps[EP_IN_PKT]);
	int ret;

	usb_fill_bulk_urb(e->urb, usb_dev, recv_pipe, e->buf, e->len,
			  mt7601u_complete_rx, dev);
	e->urb->transfer_dma = e->dma;
	e->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	trace_submit_urb(e->urb);
	ret = usb_submit_urb(e->urb, gfp);
	if (ret)
		printk("Error: rx submit URB failed: %d\n", ret);

	return ret;
}

static int mt7601u_rx_entry_check(struct mt7601u_dma_buf *e)
{
	if (!e->urb->status)
		return 0;

	if (e->urb->status != -ESHUTDOWN)
		printk("Error: RX urb failed %d\n", e->urb->status);

	return 1;
}

static void mt7601u_rx_tasklet(unsigned long data)
{
	struct mt7601u_dev *dev = (struct mt7601u_dev *) data;
	struct mt7601u_dma_buf *e;

	while ((e = mt7601u_rx_get_pending_entry(dev))) {
		if (mt7601u_rx_entry_check(e))
			continue;

		mt7601u_rx_process_entry(dev, e);
		mt7601u_rx_submit_entry(dev, e, GFP_ATOMIC);
	}
}

static void mt7601u_complete_rx(struct urb *urb)
{
	struct mt7601u_dev *dev = urb->context;
	struct mt7601u_rx_queue *q = &dev->rx_q;
	unsigned long flags;

	spin_lock_irqsave(&dev->rx_lock, flags);

	if (WARN_ONCE(q->e[q->end].urb != urb, "rx urb mismatch"))
		goto out;

	q->end = (q->end + 1) % q->entries;
	q->pending++;
	tasklet_schedule(&dev->rx_tasklet);
out:
	spin_unlock_irqrestore(&dev->rx_lock, flags);
}

static void mt7601u_kill_rx(struct mt7601u_dev *dev)
{
	int i;

	for (i = 0; i < dev->rx_q.entries; i++)
		usb_poison_urb(dev->rx_q.e[i].urb);
}

static int mt7601u_submit_rx(struct mt7601u_dev *dev)
{
	int i, ret;

	for (i = 0; i < dev->rx_q.entries; i++) {
		ret = mt7601u_rx_submit_entry(dev, &dev->rx_q.e[i], GFP_KERNEL);
		if (ret)
			return ret;
	}

	return 0;
}

static void mt7601u_free_rx(struct mt7601u_dev *dev)
{
	struct usb_device *usb_dev = to_usb_device(dev->dev->parent);
	int i;

	for (i = 0; i < dev->rx_q.entries; i++) {
		usb_free_urb(dev->rx_q.e[i].urb);
		usb_free_coherent(usb_dev, dev->rx_q.e[i].len,
				  dev->rx_q.e[i].buf, dev->rx_q.e[i].dma);
	}
}

static int
mt7601u_alloc_rx_entry(struct mt7601u_dev *dev, struct mt7601u_dma_buf *e)
{
	struct usb_device *usb_dev = to_usb_device(dev->dev->parent);

	e->len = RX_URB_SIZE;
	e->urb = usb_alloc_urb(0, GFP_KERNEL);
	e->buf = usb_alloc_coherent(usb_dev, e->len, GFP_KERNEL, &e->dma);

	return !e->urb || !e->buf;
}

static int mt7601u_alloc_rx(struct mt7601u_dev *dev)
{
	int i;

	memset(&dev->rx_q, 0, sizeof(dev->rx_q));
	dev->rx_q.dev = dev;
	dev->rx_q.entries = N_RX_ENTRIES;

	for (i = 0; i < N_RX_ENTRIES; i++)
		if (mt7601u_alloc_rx_entry(dev, &dev->rx_q.e[i]))
			return -ENOMEM;

	return 0;
}

static void mt7601u_free_tx_queue(struct mt7601u_tx_queue *q)
{
	int i;

	WARN_ON(q->used);

	for (i = 0; i < q->entries; i++)  {
		usb_poison_urb(q->e[i].urb);
		usb_free_urb(q->e[i].urb);
	}
}

static void mt7601u_free_tx(struct mt7601u_dev *dev)
{
	int i;

	for (i = 0; i < MT7601U_N_PIPES_OUT; i++)
		mt7601u_free_tx_queue(&dev->tx_q[i]);
}

static int mt7601u_alloc_tx_queue(struct mt7601u_dev *dev,
				  struct mt7601u_tx_queue *q)
{
	int i;

	memset(q, 0, sizeof(*q));
	q->dev = dev;
	q->entries = N_TX_ENTRIES;

	for (i = 0; i < N_TX_ENTRIES; i++) {
		q->e[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!q->e[i].urb)
			return -ENOMEM;
	}

	return 0;
}

static int mt7601u_alloc_tx(struct mt7601u_dev *dev)
{
	int i;

	for (i = 0; i < MT7601U_N_PIPES_OUT; i++)
		if (mt7601u_alloc_tx_queue(dev, &dev->tx_q[i]))
			return -ENOMEM;

	return 0;
}

static void mt7601u_complete_tx(struct urb *urb)
{
	struct mt7601u_tx_queue *q = urb->context;
	struct mt7601u_dev *dev = q->dev;
	struct sk_buff *skb;
	unsigned long flags;

	spin_lock_irqsave(&dev->tx_lock, flags);

	if (WARN_ON(q->e[q->start].urb != urb))
		goto out;

	skb = q->e[q->start].skb;

	trace_tx_dma_done(skb);
	dev->tx_stat_quiting = false;
	queue_delayed_work(dev->stat_wq, &dev->stat_work, msecs_to_jiffies(10));

	dma_unmap_single(dev->dev, q->e[q->start].dma, skb->len, DMA_TO_DEVICE);
	mt7601u_tx_status(dev, skb);

	if (q->entries <= q->used)
		ieee80211_wake_queue(dev->hw, skb_get_queue_mapping(skb));

	q->start = (q->start + 1) % q->entries;
	q->used--;
out:
	spin_unlock_irqrestore(&dev->tx_lock, flags);
}

int usb_kick_out(struct mt7601u_dev *dev, struct sk_buff *skb, u8 ep)
{
	struct usb_device *usb_dev = to_usb_device(dev->dev->parent);
	unsigned snd_pipe = usb_sndbulkpipe(usb_dev, dev->out_eps[ep]);
	struct mt7601u_tx_queue *q = &dev->tx_q[ep];
	unsigned long flags;
	int e, ret;

	spin_lock_irqsave(&dev->tx_lock, flags);

	if (WARN_ON(q->entries <= q->used)) {
		ret = -ENOSPC;
		goto out;
	}

	e = q->end;

	q->e[e].skb = skb;
	q->e[e].dma = dma_map_single(dev->dev, skb->data, skb->len,
				     DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev->dev, q->e[e].dma))) {
		printk("Error: dma mapping\n");
		ret = -1;
		goto out;
	}

	usb_fill_bulk_urb(q->e[e].urb, usb_dev, snd_pipe, skb->data, skb->len,
			  mt7601u_complete_tx, q);
	q->e[e].urb->transfer_dma = q->e[e].dma;
	q->e[e].urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	ret = usb_submit_urb(q->e[e].urb, GFP_ATOMIC);
	if (ret) {
		printk("Error: submit %d\n", ret);
		goto out;
	}

	q->end = (q->end + 1) % q->entries;
	q->used++;

	if (q->entries <= q->used)
		ieee80211_stop_queue(dev->hw, skb_get_queue_mapping(skb));
out:
	spin_unlock_irqrestore(&dev->tx_lock, flags);

	return ret;
}

int mt7601u_dma_init(struct mt7601u_dev *dev)
{
	int ret = -ENOMEM;

	tasklet_init(&dev->rx_tasklet, mt7601u_rx_tasklet, (unsigned long) dev);

	ret = mt7601u_alloc_tx(dev);
	if (ret)
		goto err;
	ret = mt7601u_alloc_rx(dev);
	if (ret)
		goto err;

	ret = mt7601u_submit_rx(dev);
	if (ret)
		goto err;

	return 0;
err:
	mt7601u_dma_cleanup(dev);
	return ret;
}

void mt7601u_dma_cleanup(struct mt7601u_dev *dev)
{
	mt7601u_kill_rx(dev);

	tasklet_kill(&dev->rx_tasklet);

	mt7601u_free_rx(dev);
	mt7601u_free_tx(dev);
}
