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

#ifndef __MT76_MAC_H
#define __MT76_MAC_H

struct mt76_dev;
struct mt76_sta;
struct mt76_vif;
struct mt76_wcid;

struct mt76_txwi;
struct mt76_queue;
struct mt76_txwi_cache;

struct mt76_tx_status {
	u8 valid:1;
	u8 success:1;
	u8 aggr:1;
	u8 ack_req:1;
	u8 is_probe:1;
	u8 wcid;
	u8 pktid;
	u8 retry;
	u16 rate;
} __packed __aligned(2);

struct mt76_tx_info {
	unsigned long jiffies;
	u8 tries;

	u8 wcid;
	u8 pktid;
	u8 retry;
};

/* Note: values in original "RSSI" and "SNR" fields are not actually what they
 *	 are called for MT7601U, names used by this driver are educated guesses
 *	 (see vendor mac/ral_omac.c).
 */
struct mt7601u_rxwi {
	__le32 rxinfo;

	__le32 ctl;

	__le16 frag_sn;
	__le16 rate;

	u8 unknown;
	u8 zero[3];

	u8 snr;
	u8 ant;
	u8 gain;
	u8 freq_off;

	__le32 resv2;
	__le32 expert_ant;
} __packed __aligned(4);

#define MT_RXINFO_BA			BIT(0)
#define MT_RXINFO_DATA			BIT(1)
#define MT_RXINFO_NULL			BIT(2)
#define MT_RXINFO_FRAG			BIT(3)
#define MT_RXINFO_U2M			BIT(4)
#define MT_RXINFO_MULTICAST		BIT(5)
#define MT_RXINFO_BROADCAST		BIT(6)
#define MT_RXINFO_MYBSS			BIT(7)
#define MT_RXINFO_CRCERR		BIT(8)
#define MT_RXINFO_ICVERR		BIT(9)
#define MT_RXINFO_MICERR		BIT(10)
#define MT_RXINFO_AMSDU			BIT(11)
#define MT_RXINFO_HTC			BIT(12)
#define MT_RXINFO_RSSI			BIT(13)
#define MT_RXINFO_L2PAD			BIT(14)
#define MT_RXINFO_AMPDU			BIT(15)
#define MT_RXINFO_DECRYPT		BIT(16)
#define MT_RXINFO_BSSIDX3		BIT(17)
#define MT_RXINFO_WAPI_KEY		BIT(18)
#define MT_RXINFO_PN_LEN		GENMASK(21, 19)
#define MT_RXINFO_SW_PKT_80211		BIT(22)
#define MT_RXINFO_TCP_SUM_BYPASS	BIT(28)
#define MT_RXINFO_IP_SUM_BYPASS		BIT(29)
#define MT_RXINFO_TCP_SUM_ERR		BIT(30)
#define MT_RXINFO_IP_SUM_ERR		BIT(31)

#define MT_RXWI_CTL_WCID		GENMASK(7, 0)
#define MT_RXWI_CTL_KEY_IDX		GENMASK(9, 8)
#define MT_RXWI_CTL_BSS_IDX		GENMASK(12, 10)
#define MT_RXWI_CTL_UDF			GENMASK(15, 13)
#define MT_RXWI_CTL_MPDU_LEN		GENMASK(27, 16)
#define MT_RXWI_CTL_TID			GENMASK(31, 28)

#define MT_RXWI_FRAG			GENMASK(3, 0)
#define MT_RXWI_SN			GENMASK(15, 4)

/* TODO: *XWI_RATE_* defs are the same for RX/TX/stat */
#define MT_RXWI_RATE_MCS		GENMASK(6, 0)
#define MT_RXWI_RATE_BW			BIT(7)
#define MT_RXWI_RATE_SGI		BIT(8)
#define MT_RXWI_RATE_STBC		GENMASK(10, 9)
#define MT_RXWI_RATE_ETXBF		BIT(11)
#define MT_RXWI_RATE_SND		BIT(12)
#define MT_RXWI_RATE_ITXBF		BIT(13)
#define MT_RXWI_RATE_PHY		GENMASK(15, 14)

#define MT_RXWI_GAIN_RSSI_VAL		GENMASK(5, 0)
#define MT_RXWI_GAIN_RSSI_LNA_ID	GENMASK(7, 6)
#define MT_RXWI_ANT_AUX_LNA		BIT(7)

#define MT_RXWI_EANT_ENC_ANT_ID		GENMASK(7, 0)

enum mt76_phy_type {
	MT_PHY_TYPE_CCK,
	MT_PHY_TYPE_OFDM,
	MT_PHY_TYPE_HT,
	MT_PHY_TYPE_HT_GF,
};

enum mt76_phy_bandwith {
	MT_PHY_BW_20,
	MT_PHY_BW_40,
	MT_PHY_BW_80,
};


/*  */
/* 00000000 5049ff1e */
/* phy:1 txop:3 */
/* 40000300 ff49001c */
/* 00 00 00 00 1e ff 49 50 */
/*              nseq ba:7 wicd:ff len:49 pkt_id:5 */
/* 00 03 00 40 1c ff 49 00 */
/* txop:3 phy:1      ba:7 wicd:ff len:49 pkt_id:0 */
#define MT_TXWI_FLAGS_FRAG		BIT(0)
#define MT_TXWI_FLAGS_MMPS		BIT(1)
#define MT_TXWI_FLAGS_CFACK		BIT(2)
#define MT_TXWI_FLAGS_TS		BIT(3)
#define MT_TXWI_FLAGS_AMPDU		BIT(4)
#define MT_TXWI_FLAGS_MPDU_DENSITY	GENMASK(7, 5)
#define MT_TXWI_FLAGS_TXOP		GENMASK(9, 8)
#define MT_TXWI_FLAGS_CWMIN		GENMASK(12, 10)
#define MT_TXWI_FLAGS_NO_RATE_FALLBACK	BIT(13)
#define MT_TXWI_FLAGS_TX_RPT		BIT(14)
#define MT_TXWI_FLAGS_TX_RATE_LUT	BIT(15)

#define MT_TXWI_RATE_MCS		GENMASK(6, 0)
#define MT_TXWI_RATE_BW			BIT(7)
#define MT_TXWI_RATE_SGI		BIT(8)
#define MT_TXWI_RATE_STBC		GENMASK(10, 9)
#define MT_TXWI_RATE_PHY_MODE		GENMASK(15, 14)

#define MT_TXWI_ACK_CTL_REQ		BIT(0)
#define MT_TXWI_ACK_CTL_NSEQ		BIT(1)
#define MT_TXWI_ACK_CTL_BA_WINDOW	GENMASK(7, 2)

#define MT_TXWI_LEN_BYTE_CNT		GENMASK(11, 0)
#define MT_TXWI_LEN_PKTID		GENMASK(15, 12)

#define MT_TXWI_PKTID_PROBE		BIT(2)

#define MT_TXWI_CTL_TX_POWER_ADJ	GENMASK(3, 0)
#define MT_TXWI_CTL_CHAN_CHECK_PKT	BIT(4)
#define MT_TXWI_CTL_PIFS_REV		BIT(6)

struct mt76_txwi {
	__le16 flags;
	__le16 rate_ctl;

	u8 ack_ctl;
	u8 wcid;
	__le16 len_ctl;

	__le32 iv;

	__le32 eiv;

	u8 aid;
	u8 txstream;
	__le16 ctl;
} __packed __aligned(4);


static inline struct mt76_tx_info *
mt76_skb_tx_info(struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	return (void *) info->status.status_driver_data;
}

int mt76_mac_reset(struct mt7601u_dev *dev, bool hard);
int mt76_mac_start(struct mt7601u_dev *dev);
void mt76_mac_stop(struct mt7601u_dev *dev, bool force);
void mt76_mac_resume(struct mt7601u_dev *dev);
void mt76_mac_set_bssid(struct mt7601u_dev *dev, u8 idx, const u8 *addr);

int
mt76_mac_process_rx(struct mt7601u_dev *dev, struct sk_buff *skb, void *rxwi);
void mt76_mac_write_txwi(struct mt7601u_dev *dev, struct mt76_txwi *txwi,
			 struct sk_buff *skb, struct mt76_wcid *wcid,
			 struct ieee80211_sta *sta);
void mt76_mac_wcid_setup(struct mt7601u_dev *dev, u8 idx, u8 vif_idx, u8 *mac);
int mt76_mac_wcid_set_key(struct mt7601u_dev *dev, u8 idx,
			  struct ieee80211_key_conf *key);
void mt76_mac_wcid_set_rate(struct mt7601u_dev *dev, struct mt76_wcid *wcid,
			    const struct ieee80211_tx_rate *rate);

int mt76_mac_shared_key_setup(struct mt7601u_dev *dev, u8 vif_idx, u8 key_idx,
			      struct ieee80211_key_conf *key);

int mt76_insert_hdr_pad(struct sk_buff *skb);
void mt76_remove_hdr_pad(struct sk_buff *skb);
int mt76_mac_skb_tx_overhead(struct mt7601u_dev *dev, struct sk_buff *skb);

int
mt76_mac_set_beacon(struct mt7601u_dev *dev, u8 vif_idx, struct sk_buff *skb);
void mt76_mac_set_beacon_enable(struct mt7601u_dev *dev, u8 vif_idx, bool val);

void mt76_mac_queue_txdone(struct mt7601u_dev *dev, struct sk_buff *skb,
			   struct mt76_txwi *txwi);

void mt76_mac_poll_tx_status(struct mt7601u_dev *dev, bool irq);
void mt76_mac_process_tx_status_fifo(struct mt7601u_dev *dev);

void mt76_mac_work(struct work_struct *work);

u16 mt76_mac_tx_rate_val(struct mt7601u_dev *dev,
			 const struct ieee80211_tx_rate *rate, u8 *nss_val);
struct mt76_tx_status
mt7601u_mac_fetch_tx_status(struct mt7601u_dev *dev);
void mt76_send_tx_status(struct mt7601u_dev *dev, struct mt76_tx_status *stat);

#endif
