/*
 * (c) Copyright 2002-2010, Ralink Technology, Inc.
 * Copyright (C) 2014 Felix Fietkau <nbd@openwrt.org>
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/skbuff.h>

#include "mt7601u.h"
#include "dma.h"
#include "mcu.h"
#include "usb.h"
#include "trace.h"

static inline int firmware_running(struct mt7601u_dev *dev)
{
	return mt7601u_rr(dev, MT_MCU_COM_REG0) == 1;
}

static inline void skb_put_le32(struct sk_buff *skb, u32 val)
{
	put_unaligned_le32(val, skb_put(skb, 4));
}

static inline void mt7601u_dma_skb_wrap_cmd(struct sk_buff *skb,
					    u8 seq, enum mcu_cmd cmd)
{
	mt7601u_dma_skb_wrap(skb, CPU_TX_PORT, DMA_COMMAND,
			     MT76_SET(MT_TXD_PKT_INFO_SEQ, seq) |
			     MT76_SET(MT_TXD_PKT_INFO_TYPE, cmd));
}

static inline void trace_mt_mcu_msg_send_cs(struct mt7601u_dev *dev,
					    struct sk_buff *skb, bool need_resp)
{
	u32 i, csum = 0;

	for (i = 0; i < skb->len / 4; i++)
		csum ^= get_unaligned_le32(skb->data + i * 4);

	trace_mt_mcu_msg_send(dev, skb, csum, need_resp);
}

static struct sk_buff *
mt7601u_mcu_msg_alloc(struct mt7601u_dev *dev, const void *data, int len)
{
	struct sk_buff *skb;

	WARN_ON(len % 4); /* if length is not divisible by 4 we need to pad */

	skb = alloc_skb(len + MT_DMA_HDR_LEN + 4, GFP_KERNEL);
	skb_reserve(skb, MT_DMA_HDR_LEN);
	memcpy(skb_put(skb, len), data, len);

	return skb;
}

static int mt7601u_mcu_wait_resp(struct mt7601u_dev *dev, u8 seq)
{
	u32 rxfce;
	int ret, i = 5;

	while (i--) {
		if (!wait_for_completion_timeout(&dev->mcu.resp_cmpl,
						 msecs_to_jiffies(300))) {
			dev_warn(dev->dev, "Warning: %s retrying\n", __func__);
			continue;
		}

		/* Make copies of important data before reusing the urb */
		rxfce = get_unaligned_le32(dev->mcu.resp.buf);

		ret = mt7601u_usb_submit_buf(dev, USB_DIR_IN, MT_EP_IN_CMD_RESP,
					     &dev->mcu.resp, GFP_KERNEL,
					     mt7601u_complete_urb,
					     &dev->mcu.resp_cmpl);
		if (ret)
			return ret;

		if (MT76_GET(MT_RX_FCE_INFO_CMD_SEQ, rxfce) == seq &&
		    MT76_GET(MT_RX_FCE_INFO_EVT_TYPE, rxfce) == CMD_DONE)
			return 0;

		dev_err(dev->dev, "Error: MCU resp evt:%hhx seq:%hhx-%hhx!\n",
			MT76_GET(MT_RX_FCE_INFO_EVT_TYPE, rxfce),
			seq, MT76_GET(MT_RX_FCE_INFO_CMD_SEQ, rxfce));
	}

	dev_err(dev->dev, "Error: %s timed out\n", __func__);
	return -ETIMEDOUT;
}

static int
mt7601u_mcu_msg_send(struct mt7601u_dev *dev, struct sk_buff *skb,
		     enum mcu_cmd cmd, bool wait_resp)
{
	struct usb_device *usb_dev = mt7601u_to_usb_dev(dev);
	unsigned cmd_pipe = usb_sndbulkpipe(usb_dev,
					    dev->out_eps[MT_EP_OUT_INBAND_CMD]);
	int sent, ret;
	u8 seq = 0;

	if (test_bit(MT7601U_STATE_REMOVED, &dev->state))
		return 0;

	mutex_lock(&dev->mcu.mutex);

	if (wait_resp)
		while (!seq)
			seq = ++dev->mcu.msg_seq & 0xf;

	mt7601u_dma_skb_wrap_cmd(skb, seq, cmd);

	if (dev->mcu.resp_cmpl.done)
		dev_err(dev->dev, "Error: MCU response pre-completed!\n");

	trace_mt_mcu_msg_send_cs(dev, skb, wait_resp);
	trace_submit_urb_sync(cmd_pipe, skb->len);
	ret = usb_bulk_msg(usb_dev, cmd_pipe, skb->data, skb->len, &sent, 500);
	if (ret) {
		dev_err(dev->dev, "Error: send MCU cmd failed:%d\n", ret);
		goto out;
	}
	if (sent != skb->len)
		dev_err(dev->dev, "Error: %s sent != skb->len\n", __func__);

	if (wait_resp)
		ret = mt7601u_mcu_wait_resp(dev, seq);
out:
	mutex_unlock(&dev->mcu.mutex);

	consume_skb(skb);

	return ret;
}

static int mt7601u_mcu_function_select(struct mt7601u_dev *dev,
				       enum mcu_function func, u32 val)
{
	struct sk_buff *skb;
	struct {
		__le32 id;
		__le32 value;
	} __packed __aligned(4) msg = {
		.id = cpu_to_le32(func),
		.value = cpu_to_le32(val),
	};

	skb = mt7601u_mcu_msg_alloc(dev, &msg, sizeof(msg));
	return mt7601u_mcu_msg_send(dev, skb, CMD_FUN_SET_OP, func == 5);
}

int mt7601u_mcu_tssi_read_kick(struct mt7601u_dev *dev, int use_hvga)
{
	int ret;

	if (!test_bit(MT7601U_STATE_MCU_RUNNING, &dev->state))
		return 0;

	ret = mt7601u_mcu_function_select(dev, ATOMIC_TSSI_SETTING,
					  use_hvga);
	if (ret) {
		dev_warn(dev->dev, "Warning: MCU TSSI read kick failed\n");
		return ret;
	}

	dev->tssi_read_trig = true;

	return 0;
}

int
mt7601u_mcu_calibrate(struct mt7601u_dev *dev, enum mcu_calibrate cal, u32 val)
{
	struct sk_buff *skb;
	struct {
		__le32 id;
		__le32 value;
	} __packed __aligned(4) msg = {
		.id = cpu_to_le32(cal),
		.value = cpu_to_le32(val),
	};

	skb = mt7601u_mcu_msg_alloc(dev, &msg, sizeof(msg));
	return mt7601u_mcu_msg_send(dev, skb, CMD_CALIBRATION_OP, true);
}

int mt7601u_write_reg_pairs(struct mt7601u_dev *dev, u32 base,
			    const struct mt76_reg_pair *data, int n)
{
	const int max_vals_per_cmd = INBAND_PACKET_MAX_LEN/8;
	struct sk_buff *skb;
	int cnt, i, ret;

	if (!n)
		return 0;

	cnt = min(max_vals_per_cmd, n);

	skb = alloc_skb(cnt * 8 + MT_DMA_HDR_LEN + 4, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;
	skb_reserve(skb, MT_DMA_HDR_LEN);

	for (i = 0; i < cnt; i++) {
		skb_put_le32(skb, base + data[i].reg);
		skb_put_le32(skb, data[i].value);
	}

	ret = mt7601u_mcu_msg_send(dev, skb, CMD_RANDOM_WRITE, cnt == n);
	if (ret)
		return ret;

	return mt7601u_write_reg_pairs(dev, base, data + cnt, n - cnt);
}

int mt7601u_burst_write_regs(struct mt7601u_dev *dev, u32 offset,
			     const u32 *data, int n)
{
	const int max_regs_per_cmd = INBAND_PACKET_MAX_LEN/4 - 1;
	struct sk_buff *skb;
	int cnt, i, ret;

	if (!n)
		return 0;

	cnt = min(max_regs_per_cmd, n);

	skb = alloc_skb(cnt * 4 + MT_DMA_HDR_LEN + 4, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;
	skb_reserve(skb, MT_DMA_HDR_LEN);

	skb_put_le32(skb, MT_MCU_MEMMAP_WLAN + offset);
	for (i = 0; i < cnt; i++)
		skb_put_le32(skb, data[i]);

	ret = mt7601u_mcu_msg_send(dev, skb, CMD_BURST_WRITE, cnt == n);
	if (ret)
		return ret;

	return mt7601u_burst_write_regs(dev, offset + cnt * 4,
					data + cnt, n - cnt);
}

struct mt76_fw_header {
	__le32 ilm_len;
	__le32 dlm_len;
	__le16 build_ver;
	__le16 fw_ver;
	u8 pad[4];
	char build_time[16];
};

struct mt76_fw {
	struct mt76_fw_header hdr;
	u8 ivb[MT_MCU_IVB_SIZE];
	u8 ilm[];
};

#define MCU_URB_MAX_PAYLOAD	0x3800
#define MCU_URB_SIZE		0x3900 /* TODO change to calc */

static int
__mt7601u_dma_fw(struct mt7601u_dev *dev, struct usb_device *usb_dev,
		 struct urb *urb, const void *data, u32 len,
		 void *buf, dma_addr_t dma, u32 dst_addr)
{
	DECLARE_COMPLETION_ONSTACK(cmpl);
	unsigned cmd_pipe = usb_sndbulkpipe(usb_dev, dev->out_eps[0]);
	__le32 reg;
	u32 val;
	int ret;

	/* Buffer layout:
	 *   |   4B   |  xfer len  |  4B  |    pad    |
	 *   | TXINFO | data/instr | zero | pad to 4B |
	 *
	 * lenghts:
	 *   txinfo: len
	 *   vend_req: (len + pad) << 16
	 *   urb: txinfo + len + 4 + pad
	 */

	/* TODO: make this skb so we can use common func */
	reg = cpu_to_le32(MT76_SET(MT_TXD_INFO_TYPE, DMA_PACKET) |
			  MT76_SET(MT_TXD_INFO_D_PORT, CPU_TX_PORT) |
			  MT76_SET(MT_TXD_INFO_LEN, len));
	memcpy(buf, &reg, sizeof(reg));
	memcpy(buf + sizeof(reg), data, len);
	memset(buf + sizeof(reg) + len, 0, 8);

	ret = mt7601u_vendor_single_wr(dev, VEND_WRITE_FCE,
				       MT_FCE_DMA_ADDR, dst_addr);
	if (ret)
		return ret;
	len = roundup(len, 4);
	ret = mt7601u_vendor_single_wr(dev, VEND_WRITE_FCE,
				       MT_FCE_DMA_LEN, len << 16);
	if (ret)
		return ret;

	usb_fill_bulk_urb(urb, usb_dev, cmd_pipe, buf, sizeof(reg) + len + 4,
			  mt7601u_complete_urb, &cmpl);
	urb->transfer_dma = dma;
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	trace_submit_urb(urb);
	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret) {
		printk("Submit urb failed %d\n", ret);
		return ret;
	}

	if (!wait_for_completion_timeout(&cmpl, msecs_to_jiffies(1000))) {
		usb_kill_urb(urb);
		trace_printk("URB timeout\n");
		return -ETIMEDOUT;
	}

	val = mt7601u_rr(dev, MT_TX_CPU_FROM_FCE_CPU_DESC_IDX);
	val++;
	mt7601u_wr(dev, MT_TX_CPU_FROM_FCE_CPU_DESC_IDX, val);

	return 0;
}

static int
mt7601u_dma_fw(struct mt7601u_dev *dev, struct usb_device *usb_dev,
	       struct urb *urb, const void *data, u32 len,
	       void *buf, dma_addr_t dma, u32 dst_addr)
{
	u32 done, size, val;
	int i, ret;

	for (done = 0; done < len; done += size) {
		size = min_t(u32, MCU_URB_MAX_PAYLOAD, len - done);

		ret = __mt7601u_dma_fw(dev, usb_dev, urb, data + done, size,
				       buf, dma, dst_addr + done);
		if (ret)
			return ret;

		for (i = 100; i; i--) {
			val = mt7601u_rr(dev, MT_MCU_COM_REG1);
			if (val & BIT(31))
				break;
			msleep(5);
		}
		if (!i)
			return -ETIMEDOUT;
	}

	return 0;
}

static int mt7601u_upload_firmware(struct mt7601u_dev *dev,
				   const struct mt76_fw *fw)
{
	struct usb_device *usb_dev = mt7601u_to_usb_dev(dev);
	struct mt7601u_dma_buf dma_buf;
	void *ivb;
	u32 ilm_len, dlm_len;
	int i, ret;

	ivb = kmalloc(MT_MCU_IVB_SIZE, GFP_KERNEL);
	if (!ivb || mt7601u_usb_alloc_buf(dev, MCU_URB_SIZE, &dma_buf)) {
		ret = -ENOMEM;
		goto error;
	}

	ilm_len = le32_to_cpu(fw->hdr.ilm_len) - MT_MCU_IVB_SIZE;
	dev_dbg(dev->dev, "loading FW - ILM %u + IVB %u\n",
		ilm_len, MT_MCU_IVB_SIZE);
	ret = mt7601u_dma_fw(dev, usb_dev, dma_buf.urb, fw->ilm, ilm_len,
			     dma_buf.buf, dma_buf.dma, MT_MCU_IVB_SIZE);
	if (ret)
		goto error;

	dlm_len = le32_to_cpu(fw->hdr.dlm_len);
	dev_dbg(dev->dev, "loading FW - DLM %u\n", dlm_len);
	ret = mt7601u_dma_fw(dev, usb_dev, dma_buf.urb,
			     fw->ilm + ilm_len, dlm_len,
			     dma_buf.buf, dma_buf.dma, MT_MCU_DLM_OFFSET);
	if (ret)
		goto error;

	memcpy(ivb, fw->ivb, MT_MCU_IVB_SIZE);
	ret = mt7601u_vendor_request(dev, VEND_DEV_MODE, USB_DIR_OUT,
				     0x12, 0, ivb, MT_MCU_IVB_SIZE);
	if (ret < 0)
		goto error;
	ret = 0;

	for (i = 100; i && !firmware_running(dev); i--)
		msleep(10);
	if (!i) {
		ret = -ETIMEDOUT;
		goto error;
	}

	dev_dbg(dev->dev, "Firmware running!\n");

error:
	kfree(ivb);
	usb_free_coherent(usb_dev, MCU_URB_SIZE, dma_buf.buf, dma_buf.dma);
	usb_free_urb(dma_buf.urb);

	return ret;
}

static int mt7601u_load_firmware(struct mt7601u_dev *dev)
{
	const struct firmware *fw;
	const struct mt76_fw_header *hdr;
	int len, ret;
	u32 val;

	mt7601u_wr(dev, MT_USB_DMA_CFG, (MT_USB_DMA_CFG_RX_BULK_EN |
					 MT_USB_DMA_CFG_TX_BULK_EN));

	if (firmware_running(dev))
		return 0;

	ret = request_firmware(&fw, MT7601U_FIRMWARE, dev->dev);
	if (ret)
		return ret;

	if (!fw || !fw->data || fw->size < sizeof(*hdr))
		goto err_inv_fw;

	hdr = (const struct mt76_fw_header *) fw->data;

	if (le32_to_cpu(hdr->ilm_len) <= MT_MCU_IVB_SIZE)
		goto err_inv_fw;

	len = sizeof(*hdr);
	len += le32_to_cpu(hdr->ilm_len);
	len += le32_to_cpu(hdr->dlm_len);

	if (fw->size != len)
		goto err_inv_fw;

	val = le16_to_cpu(hdr->fw_ver);
	printk("Firmware Version: %d.%d.%02d\n",
	       (val >> 12) & 0xf, (val >> 8) & 0xf, val & 0xf);

	val = le16_to_cpu(hdr->build_ver);
	printk("Build: %x\n", val);
	printk("Build Time: %16s\n", hdr->build_time);

	len = le32_to_cpu(hdr->ilm_len);

	mt7601u_wr(dev, 0x94c, 0);
	mt7601u_wr(dev, MT_FCE_PSE_CTRL, 0);

	mt7601u_vendor_reset(dev);
	msleep(5);

	mt7601u_wr(dev, 0xa44, 0);
	mt7601u_wr(dev, 0x230, 0x84210);
	mt7601u_wr(dev, 0x400, 0x80c00);
	mt7601u_wr(dev, 0x800, 1);

	mt7601u_rmw(dev, MT_PBF_CFG, 0, (MT_PBF_CFG_TX0Q_EN |
					 MT_PBF_CFG_TX1Q_EN |
					 MT_PBF_CFG_TX2Q_EN |
					 MT_PBF_CFG_TX3Q_EN));

	mt7601u_wr(dev, MT_FCE_PSE_CTRL, 1);

	mt7601u_wr(dev, MT_USB_DMA_CFG, (MT_USB_DMA_CFG_RX_BULK_EN |
					 MT_USB_DMA_CFG_TX_BULK_EN));
	val = mt7601u_rmw(dev, MT_USB_DMA_CFG, 0, MT_USB_DMA_CFG_TX_CLR);
	val &= ~MT_USB_DMA_CFG_TX_CLR;
	mt7601u_wr(dev, MT_USB_DMA_CFG, val);

	/* FCE tx_fs_base_ptr */
	mt7601u_wr(dev, MT_TX_CPU_FROM_FCE_BASE_PTR, 0x400230);

	/* FCE tx_fs_max_cnt */
	mt7601u_wr(dev, MT_TX_CPU_FROM_FCE_MAX_COUNT, 1);

	/* FCE pdma enable */
	mt7601u_wr(dev, MT_FCE_PDMA_GLOBAL_CONF, 0x44);

	/* FCE skip_fs_en */
	mt7601u_wr(dev, MT_FCE_SKIP_FS, 3);

	ret = mt7601u_upload_firmware(dev, (const struct mt76_fw *)fw->data);

	release_firmware(fw);

	return ret;

err_inv_fw:
	printk("Invalid firmware\n");
	release_firmware(fw);
	return -ENOENT;
}

int mt7601u_mcu_init(struct mt7601u_dev *dev)
{
	int ret;

	mutex_init(&dev->mcu.mutex);

	ret = mt7601u_load_firmware(dev);
	if (ret)
		return ret;

	set_bit(MT7601U_STATE_MCU_RUNNING, &dev->state);

	return 0;
}

static void mt7601u_mcu_resp_deinit(struct mt7601u_dev *dev)
{
	struct usb_device *usb_dev = mt7601u_to_usb_dev(dev);

	usb_free_urb(dev->mcu.resp.urb);
	usb_free_coherent(usb_dev, MCU_RESP_URB_SIZE,
			  dev->mcu.resp.buf, dev->mcu.resp.dma);
}

static int mt7601u_mcu_resp_init(struct mt7601u_dev *dev)
{
	struct usb_device *usb_dev = mt7601u_to_usb_dev(dev);

	init_completion(&dev->mcu.resp_cmpl);
	dev->mcu.resp.len = MCU_RESP_URB_SIZE;
	dev->mcu.resp.urb = usb_alloc_urb(0, GFP_KERNEL);
	dev->mcu.resp.buf = usb_alloc_coherent(usb_dev, MCU_RESP_URB_SIZE,
					       GFP_KERNEL, &dev->mcu.resp.dma);
	if (!dev->mcu.resp.urb || !dev->mcu.resp.buf) {
		mt7601u_mcu_resp_deinit(dev);
		return -ENOMEM;
	}

	return 0;
}

int mt7601u_mcu_cmd_init(struct mt7601u_dev *dev)
{
	int ret;

	ret = mt7601u_mcu_function_select(dev, Q_SELECT, 1);
	if (ret)
		return ret;

	ret = mt7601u_mcu_resp_init(dev);
	if (ret)
		return ret;
	ret = mt7601u_usb_submit_buf(dev, USB_DIR_IN, MT_EP_IN_CMD_RESP,
				     &dev->mcu.resp, GFP_KERNEL,
				     mt7601u_complete_urb, &dev->mcu.resp_cmpl);
	if (ret) {
		mt7601u_mcu_resp_deinit(dev);
		return ret;
	}

	return 0;
}

void mt7601u_mcu_cmd_deinit(struct mt7601u_dev *dev)
{
	usb_kill_urb(dev->mcu.resp.urb);
	mt7601u_mcu_resp_deinit(dev);
}
