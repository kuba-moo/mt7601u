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

#ifndef MT7601U_H
#define MT7601U_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/usb.h>
#include <linux/completion.h>
#include <net/mac80211.h>
#include <linux/debugfs.h>

#define MT7601U_FIRMWARE "mt7601u.bin"

#define MT_FREQ_CAL_INIT_DELAY		(30 * HZ)
#define MT_FREQ_CAL_CHECK_INTERVAL	(10 * HZ)
#define MT_FREQ_CAL_ADJ_INTERVAL	(HZ / 2)

#define MT7601U_VENDOR_REQ_MAX_RETRY	10
#define MT7601U_VENDOR_REQ_TOUT_MS	300


#define INBAND_PACKET_MAX_LEN		192

#define MT_BBP_REG_VERSION		0x00

enum mt_vendor_req {
	VEND_DEV_MODE = 1,
	VEND_WRITE = 2,
	VEND_MULTI_READ = 7,
	VEND_WRITE_FCE = 0x42,
};

#define VEND_DEV_MODE_RESET		1

struct mt7601u_pipes {
	u8 ep;
	u8 max_packet;
};

#define MT7601U_N_PIPES_OUT	6
#define MT7601U_N_PIPES_IN	2

struct mt7601u_dma_buf {
	struct urb *urb;
	void *buf;
	dma_addr_t dma;
	size_t len;
};

struct mt7601u_mcu {
	struct mutex mutex;

	u8 msg_seq;

#define MCU_RESP_URB_SIZE	1024
	struct mt7601u_dma_buf resp;
	struct completion resp_cmpl;
};

struct mt76_hw_cap {
	bool has_2ghz;
	bool has_5ghz;
};

enum mt7601u_flags {
	MT7601U_WLAN_RUNNING = 0,
	MT7601U_MCU_RUNNING = 1,
	MT7601U_SCANNING = 2,
};

struct power_per_rate {
	u8 raw;  /* validated s6 value */
	s8 bw20; /* sign-extended int */
	s8 bw40; /* sign-extended int */
};

struct mac_stats {
	u64 rx_stat[6];
	u64 tx_stat[6];
	u64 aggr_stat[2];
	u64 aggr_n[32];
	u64 zero_len_del[2];
};

#define N_RX_ENTRIES	64
struct mt7601u_rx_queue {
	struct mt7601u_dev *dev;

	struct mt7601u_dma_buf e[N_RX_ENTRIES];

	unsigned int start;
	unsigned int end;
	unsigned int entries;
	unsigned int pending;
};

struct reg_channel_bounds {
	u8 start;
	u8 num;
};

#define N_TX_ENTRIES	64

struct mt7601u_tx_queue {
	struct mt7601u_dev *dev;

	struct {
		struct urb *urb;
		struct sk_buff *skb;
		dma_addr_t dma;
	} e[N_TX_ENTRIES];

	unsigned int start;
	unsigned int end;
	unsigned int entries;
	unsigned int used;
	unsigned int fifo_seq;
};

#define N_WCIDS		128
#define GROUP_WCID(idx)	(N_WCIDS - 2 - idx)

struct mt7601u_dev {
	struct ieee80211_hw *hw;
	struct device *dev;
	u8 macaddr[ETH_ALEN];

	struct mutex mutex;

	unsigned long wcid_mask[N_WCIDS / BITS_PER_LONG];

	struct cfg80211_chan_def chandef;
	struct ieee80211_supported_band *sband_2g;

	struct mt7601u_mcu mcu;

	struct delayed_work cal_work;
	struct delayed_work mac_work;
	struct delayed_work stat_work;

	struct mt76_wcid *mon_wcid;
	struct mt76_wcid __rcu *wcid[N_WCIDS];

	spinlock_t lock;

	u32 rev;
	u32 rxfilter;

	struct debugfs_blob_wrapper eeprom;
	struct mt76_hw_cap cap;

	u32 debugfs_reg;

	/* TX */
	spinlock_t tx_lock;
	struct mt7601u_tx_queue tx_q[MT7601U_N_PIPES_OUT];
	bool tx_stat_quiting;

	/* RX */
	spinlock_t rx_lock;
	struct tasklet_struct rx_tasklet;
	struct mt7601u_rx_queue rx_q;
	atomic_t avg_ampdu_len;

	/* Beacon monitoring stuff */
	u8 bssid[ETH_ALEN];
	struct {
		spinlock_t lock;
		s8 freq_off;
		u8 phy_mode;
	} last_beacon;
#define MT7601U_FREQ_OFFSET_INVALID -128

	struct {
		u8 freq;
		bool enabled;
		bool adjusting;
		struct delayed_work work;
	} freq_cal;

	struct reg_channel_bounds reg;

	/***** Mine *****/
	unsigned long flags;
	u32 asic_rev;
	u32 mac_rev;

	u32 wlan_ctrl;

	struct mac_stats stats;

	u8 rx_stream;
	u8 tx_stream;
	u16 ant;
#define MT_ANT_RX_PATH		GENMASK(3, 0)
#define MT_ANT_TX_PATH		GENMASK(7, 4)
#define MT_ANT_RF_IC_TYPE	GENMASK(11, 8)
#define MT_ANT_BOARD_TYPE	GENMASK(13, 12)
#define MT_ANT_RSSI_IND_MODE	BIT(15)
	u16 cfg2;
	u8 rf_freq_off;

	s8 avg_rssi;
	s8 rssi_offset[2];
	s8 lna_gain;

	u8 chan_pwr[14];

	s8 tssi0;
	s8 tssi0_hvga;
	s16 tssi0_db;
	s16 tssi0_hvga_db;

	int prev_pwr_diff;

	/* TSSI stuff - only with internal TX ALC */
	struct tssi_data {
		int tx0_delta_offset;

		u8 slope;

		u8 offset_low;
		u8 offset_mid;
		u8 offset_high;
	} tssi_data;

	bool tssi_read_trig;

	const u16 *beacon_offsets;

	struct mutex vendor_req_mutex;
	struct mutex reg_atomic_mutex;
	/* TODO: Is this needed? dev->mutex should suffice */
	struct mutex hw_atomic_mutex;

	u8 out_eps[MT7601U_N_PIPES_OUT];
	u8 in_eps[MT7601U_N_PIPES_IN];
	u16 out_max_packet;
	u16 in_max_packet;
#define RX_URB_SIZE		(12 * 2048)
	struct mt7601u_dma_buf fake_rx;

#define MT7601_E2_TEMPERATURE_SLOPE		39
	s8 b49_temp;
	int curr_temp;
	int dpd_temp;

	bool pll_lock_protect;

	u8 agc_save;

	/* TODO: use nl80211 enums for this */
#define MT_BW_20				0
#define MT_BW_40				1
#define MT_BW_80				2
	u8 bw;
	bool chan_ext_below;

	/* Power per rate - one value per two rates */
	struct power_rate_table {
		struct power_per_rate cck[2];
		struct power_per_rate ofdm[4];
		struct power_per_rate ht[4];
	} power_rate_table;
	s8 real_cck_bw20[2];

	/* PA mode */
	struct pa_mode {
		s16 cck[4];
		s16 ofdm[8];
		s16 ht[16];
	} pa_mode;

#define MT_TEMP_MODE_NORMAL			0
#define MT_TEMP_MODE_HIGH			1
#define MT_TEMP_MODE_LOW			2
	u8 temp_mode;
};

struct mt7601u_tssi_params {
	char tssi0;
	int trgt_power;
};

#define mt76_dev	mt7601u_dev
#define mt76_rr		mt7601u_rr
#define mt76_wr		mt7601u_wr
#define mt76_rmw	mt7601u_rmw

#include "mt76.h"

extern const struct ieee80211_ops mt7601u_ops;

/* Core */
u32 mt7601u_rmc(struct mt7601u_dev *dev, u32 offset, u32 mask, u32 val);
int mt7601u_wait_asic_ready(struct mt7601u_dev *dev);

/* USB */
int mt7601u_vendor_request(struct mt7601u_dev *mt7601u, const u8 req,
			   const u8 direction, const u16 val, const u16 offset,
			   void *buf, const size_t buflen);
void mt7601u_vendor_reset(struct mt7601u_dev *mt7601u);
int mt7601u_vendor_single_wr(struct mt7601u_dev *dev, const u8 req,
			     const u16 offset, const u32 val);
int mt7601u_write_reg_pairs(struct mt7601u_dev *dev, u32 base,
			    const struct mt76_reg_pair *data, int len);
struct sk_buff *mt7601u_read_reg_pairs(struct mt7601u_dev *dev, u32 base,
				       const u32 *addr, u32 n);
int mt7601u_burst_write_regs(struct mt7601u_dev *dev, u32 offset,
			     const u32 *data, int n);
void mt7601u_addr_wr(struct mt7601u_dev *dev, const u32 offset, const u8 *addr);

/* Init */
struct mt7601u_dev *mt7601u_alloc_device(struct device *dev);
int mt7601u_init_hardware(struct mt7601u_dev *dev);
int mt7601u_register_device(struct mt76_dev *dev);
void mt7601u_cleanup(struct mt7601u_dev *dev);

u8 mt7601u_bbp_rr(struct mt7601u_dev *dev, u8 offset);
void mt7601u_bbp_wr(struct mt7601u_dev *dev, u8 offset, u8 val);
u8 mt7601u_bbp_rmw(struct mt7601u_dev *dev, u8 offset, u8 mask, u8 val);
u8 mt7601u_bbp_rmc(struct mt7601u_dev *dev, u8 offset, u8 mask, u8 val);

int mt7601u_mac_start(struct mt7601u_dev *dev);
void mt7601u_mac_stop(struct mt7601u_dev *dev);

/* MCU */
int mt7601u_mcu_init(struct mt7601u_dev *dev);
int mt7601u_mcu_cmd_init(struct mt7601u_dev *dev);
void mt7601u_mcu_cmd_deinit(struct mt7601u_dev *dev);
int mt7601u_mcu_tssi_read_kick(struct mt7601u_dev *dev, int use_hvga);

/* DMA */
void mt7601u_complete_urb(struct urb *urb);
int usb_kick_out(struct mt7601u_dev *dev, struct sk_buff *skb, u8 ep);
int mt7601u_dma_init(struct mt7601u_dev *dev);
void mt7601u_dma_cleanup(struct mt7601u_dev *dev);

/* PHY */
void set_power_rate(struct power_per_rate *rate, s8 delta, u8 value);
int mt7601u_phy_init(struct mt7601u_dev *dev);
void mt7601u_set_rx_path(struct mt7601u_dev *dev, u8 path);
void mt7601u_set_tx_dac(struct mt7601u_dev *dev, u8 path);
int mt7601u_bbp_set_bw(struct mt7601u_dev *dev, int bw);
void mt7601u_init_tssi_table(struct mt7601u_dev *dev);
void mt7601u_agc_save(struct mt7601u_dev *dev);
void mt7601u_agc_restore(struct mt7601u_dev *dev);
int mt7601u_phy_set_channel(struct mt76_dev *dev,
			    struct cfg80211_chan_def *chandef);
void mt7601u_rxdc_cal(struct mt7601u_dev *dev);
int
mt7601u_phy_get_rssi(struct mt76_dev *dev, struct mt7601u_rxwi *rxwi, u16 rate);
void mt7601u_phy_freq_cal_onoff(struct mt76_dev *dev,
				struct ieee80211_bss_conf *info);

/* MAC */
void mt7601u_mac_work(struct work_struct *work);
void mt7601u_mac_stat(struct work_struct *work);
void mt7601u_mac_set_protection(struct mt7601u_dev *dev, bool legacy_prot,
				int ht_mode);
void mt7601u_mac_set_short_preamble(struct mt7601u_dev *dev, bool short_preamb);
void mt7601u_mac_config_tsf(struct mt7601u_dev *dev, bool enable, int interval);
void mt7601u_mac_wcid_setup(struct mt76_dev *dev, u8 idx, u8 vif_idx, u8 *mac);
void mt7601u_mac_set_ampdu_factor(struct mt76_dev *dev,
				  struct ieee80211_sta_ht_cap *cap);

/* TX */
void mt7601u_tx(struct ieee80211_hw *hw, struct ieee80211_tx_control *control,
		struct sk_buff *skb);
int mt7601u_conf_tx(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		    u16 queue, const struct ieee80211_tx_queue_params *params);
void mt7601u_tx_status(struct mt7601u_dev *dev, struct sk_buff *skb);

static inline u32 mt7601u_bbp_set_ctrlch(struct mt7601u_dev *dev, bool below)
{
	return mt7601u_bbp_rmc(dev, 3, 0x20, below ? 0x20 : 0);
}

static inline u32 mt7601u_mac_set_ctrlch(struct mt7601u_dev *dev, bool below)
{
	return mt7601u_rmc(dev, MT_TX_BAND_CFG, 1, below);
}

#endif
