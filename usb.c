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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>

#include "mt7601u.h"
#include "usb.h"
#include "trace.h"

static struct usb_device_id mt7601u_device_table[] = {
	{ USB_DEVICE(0x148f, 0x7601) },
	{ USB_DEVICE(0x148f, 0x760b) },
	{ 0, }
};

bool mt7601u_usb_alloc_buf(struct mt7601u_dev *dev, size_t len,
			   struct mt7601u_dma_buf *buf)
{
	struct usb_device *usb_dev = mt7601u_to_usb_dev(dev);

	buf->len = len;
	buf->urb = usb_alloc_urb(0, GFP_KERNEL);
	buf->buf = usb_alloc_coherent(usb_dev, buf->len, GFP_KERNEL, &buf->dma);

	return !buf->urb || !buf->buf;
}

void mt7601u_usb_free_buf(struct mt7601u_dev *dev, struct mt7601u_dma_buf *buf)
{
	struct usb_device *usb_dev = mt7601u_to_usb_dev(dev);

	usb_free_coherent(usb_dev, buf->len, buf->buf, buf->dma);
	usb_free_urb(buf->urb);
}

int mt7601u_usb_submit_buf(struct mt7601u_dev *dev, int dir, int ep_idx,
			   struct mt7601u_dma_buf *buf, gfp_t gfp,
			   usb_complete_t complete_fn, void *context)
{
	struct usb_device *usb_dev = mt7601u_to_usb_dev(dev);
	unsigned pipe;
	int ret;

	if (dir == USB_DIR_IN)
		pipe = usb_rcvbulkpipe(usb_dev, dev->in_eps[ep_idx]);
	else
		pipe = usb_sndbulkpipe(usb_dev, dev->out_eps[ep_idx]);

	usb_fill_bulk_urb(buf->urb, usb_dev, pipe, buf->buf, buf->len,
			  complete_fn, context);
	buf->urb->transfer_dma = buf->dma;
	buf->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	trace_submit_urb(buf->urb);
	ret = usb_submit_urb(buf->urb, gfp);
	if (ret)
		dev_err(dev->dev, "Error: submit URB dir:%d ep:%d failed:%d\n",
			dir, ep_idx, ret);
	return ret;
}

static int
__mt7601u_vendor_request(struct usb_device *usb_dev, unsigned int pipe,
			 const u8 req, const u8 direction, const u16 val,
			 const u16 offset, void *buf, const size_t buflen)
{
	int i, ret;
	const u8 req_type = direction | USB_TYPE_VENDOR | USB_RECIP_DEVICE;

	for (i = 0; i < MT7601U_VENDOR_REQ_MAX_RETRY; i++) {
		ret = usb_control_msg(usb_dev, pipe, req, req_type,
				      val, offset, buf, buflen,
				      MT7601U_VENDOR_REQ_TOUT_MS);
		trace_vend_req(pipe, req, req_type, val, offset,
			       buf, buflen, ret);

		if (ret >= 0 || ret == -ENODEV)
			return ret;

		msleep(5);
	}

	printk("Vendor request failed: %d [req:0x%02x offset:0x%04x]\n",
	       ret, req, offset);

	return ret;
}

int
mt7601u_vendor_request(struct mt7601u_dev *dev, const u8 req,
		       const u8 direction, const u16 val, const u16 offset,
		       void *buf, const size_t buflen)
{
	struct usb_device *usb_dev = mt7601u_to_usb_dev(dev);
	unsigned int pipe = (direction == USB_DIR_IN) ?
		usb_rcvctrlpipe(usb_dev, 0) : usb_sndctrlpipe(usb_dev, 0);
	int ret;

	mutex_lock(&dev->vendor_req_mutex);

	ret = __mt7601u_vendor_request(usb_dev, pipe, req, direction,
				       val, offset, buf, buflen);
	if (ret == -ENODEV)
		set_bit(MT7601U_STATE_REMOVED, &dev->state);

	mutex_unlock(&dev->vendor_req_mutex);

	return ret;
}

void mt7601u_vendor_reset(struct mt7601u_dev *dev)
{
	mt7601u_vendor_request(dev, VEND_DEV_MODE, USB_DIR_OUT,
			       VEND_DEV_MODE_RESET, 0, NULL, 0);
}

u32 mt7601u_rr(struct mt7601u_dev *dev, u32 offset)
{
	int ret;
	__le32 reg;
	u32 val;

	if (offset > 0xffff)
		printk("Error: high offset read: %08x\n", offset);

	ret = mt7601u_vendor_request(dev, VEND_MULTI_READ, USB_DIR_IN,
				     0, offset, &reg, sizeof(reg));
	val = le32_to_cpu(reg);
	if (ret != sizeof(reg)) {
		/* TODO: rt2k handles partial reads, but legacy doesn't */
		if (ret != -ENODEV)
			printk("RR of %08x - wrong size  %d!!\n", offset, ret);
		val = ~0;
	}

	trace_reg_read(dev, offset, val);
	return val;
}

int mt7601u_vendor_single_wr(struct mt7601u_dev *dev, const u8 req,
			     const u16 offset, const u32 val)
{
	int ret;
	ret = mt7601u_vendor_request(dev, req, USB_DIR_OUT,
				     val & 0xFFFF, offset, NULL, 0);
	if (ret)
		return ret;
	return mt7601u_vendor_request(dev, req, USB_DIR_OUT,
				      val >> 16, offset + 2, NULL, 0);
}

void mt7601u_wr(struct mt7601u_dev *dev, u32 offset, u32 val)
{
	if (offset > 0xffff)
		printk("Error: high offset write: %08x\n", offset);

	mt7601u_vendor_single_wr(dev, VEND_WRITE, offset, val);
	trace_reg_write(dev, offset, val);
}

u32 mt7601u_rmw(struct mt7601u_dev *dev, u32 offset, u32 mask, u32 val)
{
	val |= mt7601u_rr(dev, offset) & ~mask;
	mt7601u_wr(dev, offset, val);
	return val;
}

u32 mt7601u_rmc(struct mt7601u_dev *dev, u32 offset, u32 mask, u32 val)
{
	u32 reg = mt7601u_rr(dev, offset);
	val |= reg & ~mask;
	if (reg != val)
		mt7601u_wr(dev, offset, val);
	return val;
}

void mt7601u_wr_copy(struct mt7601u_dev *dev, u32 offset,
		     const void *data, int len)
{
	WARN_ON(len & 3);
	mt7601u_burst_write_regs(dev, offset, data, len / 4);
}

void mt7601u_addr_wr(struct mt7601u_dev *dev, const u32 offset, const u8 *addr)
{
	mt7601u_wr(dev, offset, get_unaligned_le32(addr));
	mt7601u_wr(dev, offset + 4, addr[4] | addr[5] << 8);
}

static int mt7601u_assign_pipes(struct usb_interface *usb_intf,
				struct mt7601u_dev *dev)
{
	struct usb_endpoint_descriptor *ep_desc;
	struct usb_host_interface *intf_desc = usb_intf->cur_altsetting;
	unsigned i, ep_i = 0, ep_o = 0;

	dev->in_eps = devm_kmalloc_array(dev->dev, __MT_EP_IN_MAX,
					 sizeof(*dev->in_eps), GFP_KERNEL);
	dev->out_eps = devm_kmalloc_array(dev->dev, __MT_EP_OUT_MAX,
					  sizeof(*dev->out_eps), GFP_KERNEL);

	for (i = 0; i < intf_desc->desc.bNumEndpoints; i++) {
		ep_desc = &intf_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_in(ep_desc) &&
		    ep_i++ < __MT_EP_IN_MAX) {
			/* EP0 - data in; EP1 - cmd resp */
			dev->in_eps[ep_i - 1] = usb_endpoint_num(ep_desc);
			/* TODO: drop the next line. */
			dev->in_eps[ep_i - 1] |= USB_ENDPOINT_DIR_MASK;
			dev->in_max_packet = usb_endpoint_maxp(ep_desc);
		} else  if (usb_endpoint_is_bulk_out(ep_desc) &&
			    ep_o++ < __MT_EP_OUT_MAX) {
			/* There are 6 bulk out EP. EP6 highest priority. */
			/* EP0 in-band cmd. EP1-4 is EDCA. EP5 is HCCA. */
			dev->out_eps[ep_o - 1] = usb_endpoint_num(ep_desc);
			dev->out_max_packet = usb_endpoint_maxp(ep_desc);
		}
	}

	if (ep_i != __MT_EP_IN_MAX || ep_o != __MT_EP_OUT_MAX) {
		printk("Error - wrong pipes!\n");
		return -EINVAL;
	}

	return 0;
}

static int mt7601u_probe(struct usb_interface *usb_intf,
			 const struct usb_device_id *id)
{
	struct usb_device *usb_dev = interface_to_usbdev(usb_intf);
	struct mt7601u_dev *dev;
	int ret;

	dev = mt7601u_alloc_device(&usb_intf->dev);
	if (!dev)
		return -ENOMEM;

	usb_dev = usb_get_dev(usb_dev);
	usb_reset_device(usb_dev);

	usb_set_intfdata(usb_intf, dev);

	ret = mt7601u_assign_pipes(usb_intf, dev);
	if (ret)
		goto err;

	ret = mt7601u_wait_asic_ready(dev);
	if (ret)
		goto err;

	dev->rev = dev->asic_rev = mt7601u_rr(dev, MT_ASIC_VERSION);
	dev->mac_rev = mt7601u_rr(dev, MT_MAC_CSR0);
	printk("ASIC revision: %08x  MAC revision: %08x\n",
	       dev->asic_rev, dev->mac_rev);

	/* TODO: vendor driver skips this check for MT7601U */
	if (!(mt7601u_rr(dev, MT_EFUSE_CTRL) & MT_EFUSE_CTRL_SEL))
		printk("Error: eFUSE not present\n");

	ret = mt7601u_init_hardware(dev);
	if (ret)
		goto err;

	ret = mt7601u_register_device(dev);
	if (ret)
		goto err_hw;

	set_bit(MT7601U_STATE_INITIALIZED, &dev->state);

	return 0;
err_hw:
	mt7601u_cleanup(dev);
err:
	usb_set_intfdata(usb_intf, NULL);
	usb_put_dev(interface_to_usbdev(usb_intf));

	destroy_workqueue(dev->stat_wq);
	ieee80211_free_hw(dev->hw);
	return ret;
}

static void mt7601u_disconnect(struct usb_interface *usb_intf)
{
	struct mt7601u_dev *dev = usb_get_intfdata(usb_intf);

	ieee80211_unregister_hw(dev->hw);
	mt7601u_cleanup(dev);

	usb_set_intfdata(usb_intf, NULL);
	usb_put_dev(interface_to_usbdev(usb_intf));

	destroy_workqueue(dev->stat_wq);
	ieee80211_free_hw(dev->hw);
}

static int mt7601u_suspend(struct usb_interface *usb_intf, pm_message_t state)
{
	return -1;
}

static int mt7601u_resume(struct usb_interface *usb_intf)
{
	return -1;
}

MODULE_DEVICE_TABLE(usb, mt7601u_device_table);
MODULE_FIRMWARE(MT7601U_FIRMWARE);
MODULE_LICENSE("GPL");

static struct usb_driver mt7601u_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= mt7601u_device_table,
	.probe		= mt7601u_probe,
	.disconnect	= mt7601u_disconnect,
	.suspend	= mt7601u_suspend,
	.resume		= mt7601u_resume,
	.reset_resume	= mt7601u_resume,
	.soft_unbind	= 1,
	.disable_hub_initiated_lpm = 1,
};
module_usb_driver(mt7601u_driver);
