/*
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

#if !defined(__MT7601U_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define __MT7601U_TRACE_H

#include <linux/tracepoint.h>
#include "mt7601u.h"

#include "mac.h" /* for txwi */ /* TODO: remove me */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM mt7601u

#define MAXNAME		32
#define DEV_ENTRY	__array(char, wiphy_name, 32)
#define DEV_ASSIGN	strlcpy(__entry->wiphy_name, wiphy_name(dev->hw->wiphy), MAXNAME)
#define DEV_PR_FMT	"%s"
#define DEV_PR_ARG	__entry->wiphy_name

#define REG_ENTRY	__field(u32, reg) __field(u32, val)
#define REG_ASSIGN	__entry->reg = reg; __entry->val = val
#define REG_PR_FMT	" %04x=%08x"
#define REG_PR_ARG	__entry->reg, __entry->val

DECLARE_EVENT_CLASS(dev_reg_evt,
	TP_PROTO(struct mt7601u_dev *dev, u32 reg, u32 val),
	TP_ARGS(dev, reg, val),
	TP_STRUCT__entry(
		DEV_ENTRY
		REG_ENTRY
	),
	TP_fast_assign(
		DEV_ASSIGN;
		REG_ASSIGN;
	),
	TP_printk(
		DEV_PR_FMT REG_PR_FMT,
		DEV_PR_ARG, REG_PR_ARG
	)
);

DEFINE_EVENT(dev_reg_evt, reg_read,
	TP_PROTO(struct mt7601u_dev *dev, u32 reg, u32 val),
	TP_ARGS(dev, reg, val)
);

DEFINE_EVENT(dev_reg_evt, reg_write,
	TP_PROTO(struct mt7601u_dev *dev, u32 reg, u32 val),
	TP_ARGS(dev, reg, val)
);

TRACE_EVENT(submit_urb,
	TP_PROTO(struct urb *u),

	TP_ARGS(u),

	TP_STRUCT__entry(
		__field(unsigned, pipe) __field(u32, len)
	),

	TP_fast_assign(
		__entry->pipe = u->pipe;
		__entry->len = u->transfer_buffer_length;
	),

	TP_printk("p:%08x len:%u", __entry->pipe, __entry->len)
);

TRACE_EVENT(mcu_msg_send,
	TP_PROTO(struct sk_buff *skb, u32 csum, bool resp),

	TP_ARGS(skb, csum, resp),

	TP_STRUCT__entry(
		__field(u32, info)
		__field(u32, csum)
		__field(bool, resp)
	),

	TP_fast_assign(
		__entry->info = *(u32 *)skb->data;
		__entry->csum = csum;
		__entry->resp = resp;
	),

	TP_printk("i:%08x c:%08x r:%d", __entry->info, __entry->csum,
		  __entry->resp)
);

#define trace_submit_urb_sync(__pipe, __len) ({	\
	struct urb u;				\
	u.pipe = __pipe;			\
	u.transfer_buffer_length = __len;	\
	trace_submit_urb(&u);			\
})

TRACE_EVENT(vend_req,
	TP_PROTO(unsigned pipe, u8 req, u8 req_type, u16 val, u16 offset, void *buf, size_t buflen, int ret),

	TP_ARGS(pipe, req, req_type, val, offset, buf, buflen, ret),

	TP_STRUCT__entry(
		__field(unsigned, pipe) __field(u8, req) __field(u8, req_type) __field(u16, val) __field(u16, offset) __field(void*, buf) __field(int, buflen) __field(int, ret)
	),

	TP_fast_assign(
		__entry->pipe = pipe;
		__entry->req = req;
		__entry->req_type = req_type;
		__entry->val = val;
		__entry->offset = offset;
		__entry->buf = buf;
		__entry->buflen = buflen;
		__entry->ret = ret;
	),

	TP_printk("%d p:%08x req:%02hhx %02hhx val:%04hx %04hx buf:%d %d", __entry->ret, __entry->pipe, __entry->req, __entry->req_type, __entry->val, __entry->offset, !!__entry->buf, __entry->buflen)
);

TRACE_EVENT(ee_read,
	TP_PROTO(int offset, u16 val),

	TP_ARGS(offset, val),

	TP_STRUCT__entry(
		__field(int, o) __field(u16, v)
	),

	TP_fast_assign(
		__entry->o = offset;
		__entry->v = val;
	),

	TP_printk("%04x=%04x", __entry->o, __entry->v)
);

TRACE_EVENT(read_temp,
	TP_PROTO(u8 temp),

	TP_ARGS(temp),

	TP_STRUCT__entry(
		__field(u8, temp)
	),

	TP_fast_assign(
		__entry->temp = temp;
	),

	TP_printk("%02hhx", __entry->temp)
);

TRACE_EVENT(freq_cal_offset,
	TP_PROTO(u8 phy_mode, s8 freq_off),

	TP_ARGS(phy_mode, freq_off),

	TP_STRUCT__entry(
		__field(u8, phy_mode)
		__field(s8, freq_off)
	),

	TP_fast_assign(
		__entry->phy_mode = phy_mode;
		__entry->freq_off = freq_off;
	),

	TP_printk("phy:%02hhx off:%02hhx",
		  __entry->phy_mode, __entry->freq_off)
);

TRACE_EVENT(freq_cal_adjust,
	TP_PROTO(u8 freq_off),

	TP_ARGS(freq_off),

	TP_STRUCT__entry(
		__field(u8, freq_off)
	),

	TP_fast_assign(
		__entry->freq_off = freq_off;
	),

	TP_printk("%02hhx", __entry->freq_off)
);

TRACE_EVENT(bbp_read,
	TP_PROTO(u8 offset, u8 value),

	TP_ARGS(offset, value),

	TP_STRUCT__entry(
		__field(u8, offset)	__field(u8, value)
	),

	TP_fast_assign(
		__entry->offset = offset;
		__entry->value = value;
	),

	TP_printk("%02hhx=%02hhx", __entry->offset, __entry->value)
);

TRACE_EVENT(bbp_write,
	TP_PROTO(u8 offset, u8 value),

	TP_ARGS(offset, value),

	TP_STRUCT__entry(
		__field(u8, offset)	__field(u8, value)
	),

	TP_fast_assign(
		__entry->offset = offset;
		__entry->value = value;
	),

	TP_printk("%02hhx=%02hhx", __entry->offset, __entry->value)
);

TRACE_EVENT(mt_rx,
	TP_PROTO(struct mt7601u_rxwi *rxwi, u32 f),

	TP_ARGS(rxwi, f),

	TP_STRUCT__entry(
		__field_struct(struct mt7601u_rxwi, rxwi)
		__field(u32, fce_info)
	),

	TP_fast_assign(
		__entry->rxwi = *rxwi;
		__entry->fce_info = f;
	),

	TP_printk("rxi:%08x ctl:%08x frag_sn:%04hx rate:%04hx "
		  "uknw:%02hhx z:%02hhx%02hhx%02hhx snr:%02hhx "
		  "ant:%02hhx gain:%02hhx freq_o:%02hhx "
		  "r:%08x ea:%08x fce:%08x",
		  le32_to_cpu(__entry->rxwi.rxinfo),
		  le32_to_cpu(__entry->rxwi.ctl),
		  le16_to_cpu(__entry->rxwi.frag_sn),
		  le16_to_cpu(__entry->rxwi.rate),
		  __entry->rxwi.unknown,
		  __entry->rxwi.zero[0], __entry->rxwi.zero[1],
		  __entry->rxwi.zero[2],
		  __entry->rxwi.snr, __entry->rxwi.ant,
		  __entry->rxwi.gain, __entry->rxwi.freq_off,
		  __entry->rxwi.resv2, __entry->rxwi.expert_ant,
		  __entry->fce_info)
);

DECLARE_EVENT_CLASS(dev_rf_reg_evt,
	TP_PROTO(u8 bank, u8 offset, u8 value),
	TP_ARGS(bank, offset, value),
	TP_STRUCT__entry(
		__field(u8, bank)
		__field(u8, offset)
		__field(u8, value)
	),
	TP_fast_assign(
		__entry->bank = bank;
		__entry->offset = offset;
		__entry->value = value;
	),
	TP_printk(
		"%02hhx:%02hhx=%02hhx", __entry->bank, __entry->offset, __entry->value
	)
);

DEFINE_EVENT(dev_rf_reg_evt, rf_read,
	TP_PROTO(u8 bank, u8 offset, u8 value),
	TP_ARGS(bank, offset, value)
);

DEFINE_EVENT(dev_rf_reg_evt, rf_write,
	TP_PROTO(u8 bank, u8 offset, u8 value),
	TP_ARGS(bank, offset, value)
);

TRACE_EVENT(tx_dma_done,
	TP_PROTO(struct sk_buff *skb),

	TP_ARGS(skb),

	TP_STRUCT__entry(
		__field(struct sk_buff *, skb)
	),

	TP_fast_assign(
		__entry->skb = skb;
	),

	TP_printk("%p", __entry->skb)
);

TRACE_EVENT(tx_status_cleaned,
	TP_PROTO(int cleaned),

	TP_ARGS(cleaned),

	TP_STRUCT__entry(
		__field(int, cleaned)
	),

	TP_fast_assign(
		__entry->cleaned = cleaned;
	),

	TP_printk("%d", __entry->cleaned)
);

TRACE_EVENT(tx_status,
	TP_PROTO(u32 stat1, u32 stat2),

	TP_ARGS(stat1, stat2),

	TP_STRUCT__entry(
		__field(u32, stat1)	__field(u32, stat2)
	),

	TP_fast_assign(
		__entry->stat1 = stat1;
		__entry->stat2 = stat2;
	),

	TP_printk("%08x %08x", __entry->stat1, __entry->stat2)
);

TRACE_EVENT(mt_tx,
	TP_PROTO(struct sk_buff *skb, struct mt76_sta *sta, struct mt76_txwi *h),

	TP_ARGS(skb, sta, h),

	TP_STRUCT__entry(
		__field_struct(struct mt76_txwi, h)
		__field(struct sk_buff *, skb)
		__field(struct mt76_sta *, sta)
	),

	TP_fast_assign(
		__entry->h = *h;
		__entry->skb = skb;
		__entry->sta = sta;
	),

	TP_printk("skb:%p sta:%p  flg:%04hx rate_ctl:%04hx ack:%02hhx "
		  "wcid:%02hhx len_ctl:%05hx",
		  __entry->skb, __entry->sta,
		  le16_to_cpu(__entry->h.flags),
		  le16_to_cpu(__entry->h.rate_ctl),
		  __entry->h.ack_ctl, __entry->h.wcid,
		  le16_to_cpu(__entry->h.len_ctl))
);

TRACE_EVENT(rx_dma_aggr,
	TP_PROTO(int cnt),

	TP_ARGS(cnt),

	TP_STRUCT__entry(
		__field(u8, cnt)
	),

	TP_fast_assign(
		__entry->cnt = cnt;
	),

	TP_printk("%d", __entry->cnt)
);

#endif

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

#include <trace/define_trace.h>
