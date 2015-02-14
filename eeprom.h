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

#ifndef __MT7601U_EEPROM_H
#define __MT7601U_EEPROM_H

#include "mt7601u.h"
#include "trace.h"

#define MT7601U_EE_MAX_VER			0x0c
#define MT7601U_DEFAULT_TX_POWER		6

enum mt76_eeprom_field {
	MT_EE_CHIP_ID =				0x00,
	MT_EE_VERSION =				0x02,
	MT_EE_MAC_ADDR =			0x04,
	MT_EE_NIC_CONF_0 =			0x34,
	MT_EE_NIC_CONF_1 =			0x36,
	MT_EE_COUNTRY_REGION =			0x38,
	MT_EE_FREQ_OFFSET =			0x3a,
	MT_EE_NIC_CONF_2 =			0x42,

	MT_EE_LNA_GAIN =			0x44,
	MT_EE_RSSI_OFFSET =			0x46,

	MT_EE_TX_POWER_DELTA_BW40 =		0x50,
	MT_EE_TX_POWER_OFFSET =			0x52,

	MT_EE_TX_TSSI_SLOPE =			0x6e,
	MT_EE_TX_TSSI_OFFSET_GROUP =		0x70,
	MT_EE_TX_TSSI_OFFSET =			0x76,

	MT_EE_TX_TSSI_TARGET_POWER =		0xd0,
	MT_EE_FREQ_OFFSET_COMPENSATION =	0xda,
	MT_EE_TX_POWER_BYRATE =			0xde,

	__MT_EE_MAX = 				0xf0
};

#define MT_EE_VERSION_FAE			GENMASK(7, 0)
#define MT_EE_VERSION_EE			GENMASK(15, 8)

#define MT_EE_NIC_CONF_0_PA_INT_2G		BIT(8)
#define MT_EE_NIC_CONF_0_PA_INT_5G		BIT(9)
#define MT_EE_NIC_CONF_0_BOARD_TYPE		GENMASK(13, 12)

#define MT_EE_NIC_CONF_1_HW_RF_CTRL		BIT(0)
#define MT_EE_NIC_CONF_1_TEMP_TX_ALC		BIT(1)
#define MT_EE_NIC_CONF_1_LNA_EXT_2G		BIT(2)
#define MT_EE_NIC_CONF_1_LNA_EXT_5G		BIT(3)
#define MT_EE_NIC_CONF_1_TX_ALC_EN		BIT(13)

#define MT_EE_NIC_CONF_2_RX_STREAM		GENMASK(3, 0)
#define MT_EE_NIC_CONF_2_TX_STREAM		GENMASK(7, 4)
#define MT_EE_NIC_CONF_2_HW_ANTDIV		BIT(8)
#define MT_EE_NIC_CONF_2_XTAL_OPTION		GENMASK(10, 9)
#define MT_EE_NIC_CONF_2_TEMP_DISABLE		BIT(11)
#define MT_EE_NIC_CONF_2_COEX_METHOD		GENMASK(15, 13)

enum mt76_board_type {
	BOARD_TYPE_2GHZ = 1,
	BOARD_TYPE_5GHZ = 2,
};

enum mt76_cal_channel_group {
	MT_CH_5G_JAPAN,
	MT_CH_5G_UNII_1,
	MT_CH_5G_UNII_2,
	MT_CH_5G_UNII_2E_1,
	MT_CH_5G_UNII_2E_2,
	MT_CH_5G_UNII_3,
	__MT_CH_MAX
};

struct mt76_rate_power {
	s8 cck[2];
	s8 ofdm[4];
	s8 ht[8];
	s8 vht[5];
};

struct mt76_tx_power_info {
	u8 target_power;

	s8 delta_bw40;
	s8 delta_bw80;

	struct {
		s8 tssi_slope;
		s8 tssi_offset;
		s8 target_power;
		s8 delta;
	} chain[MT_MAX_CHAINS];
};

struct mt76_temp_comp {
	u8 temp_25_ref;
	int lower_bound; /* J */
	int upper_bound; /* J */
	unsigned int high_slope; /* J / dB */
	unsigned int low_slope; /* J / dB */
};

static inline int
mt76_eeprom_get(struct mt76_dev *dev, enum mt76_eeprom_field field)
{
	int ret = -1;

	if (!(field & 1) && field < __MT_EE_MAX)
		ret = get_unaligned_le16(dev->eeprom.data + field);

	trace_ee_read(field, ret);
	return ret;
}

void mt76_get_rate_power(struct mt76_dev *dev, struct mt76_rate_power *t);
void mt76_get_power_info(struct mt76_dev *dev, struct mt76_tx_power_info *t);
int mt76_get_temp_comp(struct mt76_dev *dev, struct mt76_temp_comp *t);
bool mt76_ext_pa_enabled(struct mt76_dev *dev, enum ieee80211_band band);
void mt76_read_rx_gain(struct mt76_dev *dev);

/* AKA dynamic TX AGC control */
static inline bool
mt76_temp_tx_alc_enabled(struct mt76_dev *dev)
{
	return mt76_eeprom_get(dev, MT_EE_NIC_CONF_1) &
	       MT_EE_NIC_CONF_1_TEMP_TX_ALC;
}

/* AKA internal TX ALC */
static inline bool
mt76_has_tssi(struct mt76_dev *dev)
{
	return ~mt76_eeprom_get(dev, MT_EE_NIC_CONF_1) &&
		mt76_eeprom_get(dev, MT_EE_NIC_CONF_1) &
		MT_EE_NIC_CONF_1_TX_ALC_EN;
}

static inline bool
mt76_tssi_enabled(struct mt76_dev *dev)
{
	return !mt76_temp_tx_alc_enabled(dev) &&
		mt76_has_tssi(dev);
}

/*
static inline bool
mt76_has_ext_lna(struct mt76_dev *dev)
{
	u32 val = mt76_eeprom_get(dev, MT_EE_NIC_CONF_1);

	if (dev->chandef.chan->band == IEEE80211_BAND_2GHZ)
		return val & MT_EE_NIC_CONF_1_LNA_EXT_2G;
	else
		return val & MT_EE_NIC_CONF_1_LNA_EXT_5G;
}
*/

/* TODO: this should be per-chip */
#define MT_EFUSE_USAGE_MAP_START	0x1e0
#define MT_EFUSE_USAGE_MAP_END		0x1fc
#define MT_EFUSE_USAGE_MAP_SIZE		29

enum mt7601u_eeprom_access_modes {
	MT_EE_READ = 0,
	MT_EE_PHYSICAL_READ = 1,
};

#endif
