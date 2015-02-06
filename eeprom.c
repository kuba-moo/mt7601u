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

#include <linux/of.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/etherdevice.h>
#include <asm/unaligned.h>
#include "mt7601u.h"
#include "eeprom.h"

static int
mt76_eeprom_get_macaddr(struct mt76_dev *dev)
{
	void *src = dev->eeprom.data + MT_EE_MAC_ADDR;

	memcpy(dev->macaddr, src, ETH_ALEN);
	return 0;
}

static int mt76_get_of_eeprom(struct mt76_dev *dev, int len)
{
	int ret = -ENOENT;
#ifdef CONFIG_OF
	struct device_node *np = dev->dev->of_node;
	struct mtd_info *mtd;
	const __be32 *list;
	const char *part;
	phandle phandle;
	int offset = 0;
	int size;
	size_t retlen;

	if (!np)
		return -ENOENT;

	list = of_get_property(np, "mediatek,mtd-eeprom", &size);
	if (!list)
		return -ENOENT;

	phandle = be32_to_cpup(list++);
	if (!phandle)
		return -ENOENT;

	np = of_find_node_by_phandle(phandle);
	if (!np)
		return -EINVAL;

	part = of_get_property(np, "label", NULL);
	if (!part)
		part = np->name;

	mtd = get_mtd_device_nm(part);
	if (IS_ERR(mtd))
		return PTR_ERR(mtd);

	if (size <= sizeof(*list))
		return -EINVAL;

	offset = be32_to_cpup(list);
	ret = mtd_read(mtd, offset, len, &retlen, dev->eeprom.data);
	put_mtd_device(mtd);
	if (ret)
		return ret;

	if (retlen < len)
		return -EINVAL;
#endif
	return ret;
}

/*< Diff: select mode; don't do the useless sleep.
 *        read all 4 registers */
static int
mt7601u_efuse_read(struct mt76_dev *dev, u16 addr, u8 *data,
		   enum mt7601u_eeprom_access_modes mode)
{
	u32 val;
	int i;

	val = mt76_rr(dev, MT_EFUSE_CTRL);
	val &= ~(MT_EFUSE_CTRL_AIN |
		 MT_EFUSE_CTRL_MODE);
	val |= MT76_SET(MT_EFUSE_CTRL_AIN, addr & ~0xf) |
		MT76_SET(MT_EFUSE_CTRL_MODE, mode) |
		MT_EFUSE_CTRL_KICK;
	mt76_wr(dev, MT_EFUSE_CTRL, val);

	if (!mt76_poll(dev, MT_EFUSE_CTRL, MT_EFUSE_CTRL_KICK, 0, 1000))
		return -ETIMEDOUT;

	val = mt76_rr(dev, MT_EFUSE_CTRL);
	if ((val & MT_EFUSE_CTRL_AOUT) == MT_EFUSE_CTRL_AOUT) {
		memset(data, 0xff, 16);
		return 0;
	}

	for (i = 0; i < 4; i++) {
	    val = mt76_rr(dev, MT_EFUSE_DATA(i));
	    put_unaligned_le32(val, data + 4 * i);
	}

	return 0;
}

static int
mt76_efuse_read(struct mt76_dev *dev, u16 addr, u8 *data)
{
	return mt7601u_efuse_read(dev, addr, data, MT_EE_READ);
}

/* TODO: this relies heavily on the MT7601U MT_EFUSE_USAGE_MAP_* */
static int
mt7601u_efuse_physical_size_check(struct mt76_dev *dev)
{
	u8 data[32];
	int ret, i;
	u32 start = 0, end = 0;

	WARN_ON(MT_EFUSE_USAGE_MAP_SIZE >= sizeof(data));
	WARN_ON(MT_EFUSE_USAGE_MAP_START & 0xf);

	for (i = 0; i + 16 <= sizeof(data); i += 16) {
		ret = mt7601u_efuse_read(dev, MT_EFUSE_USAGE_MAP_START + i,
					 data + i, MT_EE_PHYSICAL_READ);
		if (ret)
			return ret;
	}

	for (i = 0; i < MT_EFUSE_USAGE_MAP_SIZE; i++)
		if (!data[i]) {
			start = MT_EFUSE_USAGE_MAP_START + i;
			break;
		}
	for (i = 0; i < MT_EFUSE_USAGE_MAP_SIZE; i++)
		if (!data[i])
			end = MT_EFUSE_USAGE_MAP_START + i;

	trace_printk("I recon phy efuse free s:%04x e:%04x l:%04x\n",
		     start, end, end - start + 1);

	if (end - start + 1 + 5 > MT_EFUSE_USAGE_MAP_SIZE) {
		printk("Error: your device needs default EEPROM file and this driver doesn't support it!\n");
		return -EINVAL;
	}

	return 0;
}

static int
mt76_get_efuse_data(struct mt76_dev *dev, int len)
{
	int ret, i;

	ret = mt7601u_efuse_physical_size_check(dev);
	if (ret)
		return ret;

	for (i = 0; i + 16 <= len; i += 16) {
		ret = mt76_efuse_read(dev, i, dev->eeprom.data + i);
		if (ret)
			return ret;
	}

	return 0;
}

static int
mt76_eeprom_load(struct mt76_dev *dev)
{
	int len = MT7662_EEPROM_SIZE; /* TODO: this should be ok, but still.. */

	dev->eeprom.size = len;
	dev->eeprom.data = devm_kzalloc(dev->dev, len, GFP_KERNEL);
	if (!dev->eeprom.data)
		return -ENOMEM;

	if (!mt76_get_of_eeprom(dev, len))
		return 0;

	if (!mt76_get_efuse_data(dev, len))
		return 0;

	return -ENOENT;
}

static bool
field_valid(u8 val)
{
	return val != 0 && val != 0xff;
}

static s8
mt76_rate_power_val(u8 val)
{
	if (!field_valid(val))
		return 0;

	return val;
}

static void
mt7601u_set_channel_power(struct mt76_dev *dev)
{
	u32 i, val;
	u8 max_pwr, trgt_pwr;
	s8 p1, p2;

	val = mt7601u_rr(dev, MT_TX_ALC_CFG_0);
	max_pwr = MT76_GET(MT_TX_ALC_CFG_0_LIMIT_0, val);

	/* TODO: only read trgt power when TSSI is enabled. */
	val = mt76_eeprom_get(dev, MT_EE_G_TARGET_POWER);
	trgt_pwr = val & 0xff;
	trace_printk("trgt power (eo:0x0d): %02hhx\n", trgt_pwr);

	if (!trgt_pwr || trgt_pwr > max_pwr) {
		printk("Error: EEPROM trgt power invalid %hhx!\n", trgt_pwr);
		trgt_pwr = 0x20;
	}

	if (mt76_has_tssi(dev)) {
		memset(dev->chan_pwr, trgt_pwr, sizeof(dev->chan_pwr));
		goto out;
	}

	for (i = 0; i < 7; i++) {
		val = mt76_eeprom_get(dev, MT_EE_G_TX_PWR_OFFSET + i * 2);

		p1 = mt76_rate_power_val(val);
		p2 = mt76_rate_power_val(val >> 8);

 		if (p1 > max_pwr || p1 < 0)
			p1 = MT7601U_DEFAULT_TX_POWER;
		/* Note: vendor driver does && for the second value... */
		if (p2 > max_pwr || p2 < 0)
			p2 = MT7601U_DEFAULT_TX_POWER;

		dev->chan_pwr[i * 2] = p1;
		dev->chan_pwr[i * 2 + 1] = p2;
	}

	/* TODO: move this to debugfs? */
out:
	for (i = 0; i < 7; i++) {
		trace_printk("tx_power  ch%u:%02hhx ch%u:%02hhx\n",
			     i * 2 + 1, dev->chan_pwr[i * 2],
			     i * 2 + 2, dev->chan_pwr[i * 2 + 1]);
	}
}

static void
mt7601u_set_ant_word(struct mt76_dev *dev)
{
	dev->ant = mt76_eeprom_get(dev, MT_EE_NIC_CONF_0);

	if ((dev->ant & 0xff00) == 0xff00)
		dev->ant = MT76_SET(MT_ANT_RX_PATH, 1) |
			MT76_SET(MT_ANT_TX_PATH, 1) |
			MT76_SET(MT_ANT_RF_IC_TYPE, 0xf);

	/* TODO: drop this once I'm sure there are no multi-stream devs */
	dev->rx_stream = MT76_GET(MT_ANT_RX_PATH, dev->ant);
	dev->tx_stream = MT76_GET(MT_ANT_TX_PATH, dev->ant);

	if (dev->rx_stream > 1 || dev->tx_stream > 1)
		printk("Error: your device has more RX/TX streams than is supported by this driver!\n");
}

static void
mt7601u_set_cfg2_word(struct mt76_dev *dev)
{
	dev->cfg2 = mt76_eeprom_get(dev, MT_EE_NIC_CONF_1);

	if (!field_valid(dev->cfg2))
		dev->cfg2 &= 0xff00;
	if (!field_valid(dev->cfg2 >> 8))
		dev->cfg2 &= 0x00ff;
}

static void
mt7601u_set_rf_freq_off(struct mt76_dev *dev)
{
	u16 val;

	dev->rf_freq_off = mt76_eeprom_get(dev, MT_EE_XTAL_TRIM_1) & 0xff;
	if ((dev->rf_freq_off & 0xff) == 0xff)
		dev->rf_freq_off = 0;

	val = mt76_eeprom_get(dev, MT_EE_FREQ_OFFSET_COMPENSATION) >> 8;
	if (val == 0xff)
		return;

	/* Note: does EEPROM really hold non-U2 values? */
	if (val & BIT(7))
		dev->rf_freq_off -= val & 0x7f;
	else
		dev->rf_freq_off += val;

	trace_printk("RF freq off: %hhx\n", dev->rf_freq_off);
}

static void
mt7601u_set_rssi_offset(struct mt76_dev *dev)
{
	u16 val = mt76_eeprom_get(dev, MT_EE_RSSI_OFFSET_2G_0);
	int i;

	for (i = 0; i < 2; i++) {
		dev->rssi_offset[i] = (val >> i * 8) & 0xff;

		if (dev->rssi_offset[i] < -10 || dev->rssi_offset[i] > 10) {
			printk("Warning: RSSI from EEPROM is invalid %02hhx\n",
			       dev->rssi_offset[i]);
			dev->rssi_offset[i] = 0;
		}
	}
}

static void
mt7601u_extra_power_over_mac(struct mt76_dev *dev)
{
	u32 val;

	val = ((mt7601u_rr(dev, MT_TX_PWR_CFG_1) & 0x0000ff00) >> 8);
	val |= ((mt7601u_rr(dev, MT_TX_PWR_CFG_2) & 0x0000ff00) << 8);
	mt7601u_wr(dev, MT_TX_PWR_CFG_7, val);

	val = ((mt7601u_rr(dev, MT_TX_PWR_CFG_4) & 0x0000ff00) >> 8);
	mt7601u_wr(dev, MT_TX_PWR_CFG_9, val);
}

static void
mt7601u_save_power_rate(struct mt76_dev *dev, s8 delta, u32 val, int i)
{
	struct power_rate_table *t = &dev->power_rate_table;

	switch (i) {
	case 0:
		set_power_rate(&t->cck[0], delta, (val >> 0) & 0xff);
		set_power_rate(&t->cck[1], delta, (val >> 8) & 0xff);
		/* Save cck bw20 for fixups of channel 14 */
		dev->real_cck_bw20[0] = t->cck[0].bw20;
		dev->real_cck_bw20[1] = t->cck[1].bw20;

		set_power_rate(&t->ofdm[0], delta, (val >> 16) & 0xff);
		set_power_rate(&t->ofdm[1], delta, (val >> 24) & 0xff);
		break;
	case 1:
		set_power_rate(&t->ofdm[2], delta, (val >> 0) & 0xff);
		set_power_rate(&t->ofdm[3], delta, (val >> 8) & 0xff);
		set_power_rate(&t->ht[0], delta, (val >> 16) & 0xff);
		set_power_rate(&t->ht[1], delta, (val >> 24) & 0xff);
		break;
	case 2:
		set_power_rate(&t->ht[2], delta, (val >> 0) & 0xff);
		set_power_rate(&t->ht[3], delta, (val >> 8) & 0xff);
		break;
	}
}

static s8 get_delta(u8 val)
{
	s8 ret;

	if (!field_valid(val) || !(val & BIT(7)))
		return 0;

	ret = val & 0x1f;
	if (ret > 8)
		ret = 8;
	if (val & BIT(6))
		ret = -ret;

	return ret;
}

static void
mt7601u_config_tx_power_per_rate(struct mt76_dev *dev)
{
	u32 val;
	int i;
	s8 bw40_delta;

	val = mt76_eeprom_get(dev, MT_EE_TX_POWER_DELTA_BW40);
	bw40_delta = get_delta(val);
	trace_printk("g_delta: %hhx\n", bw40_delta);

	for (i = 0; i < 5; i++) {
		val = mt76_eeprom_get(dev, MT_EE_TX_POWER_BYRATE_20MHZ_2_4G
				      + i * 4);
		val |= mt76_eeprom_get(dev, MT_EE_TX_POWER_BYRATE_20MHZ_2_4G
				       + i * 4 + 2) << 16;

		mt7601u_save_power_rate(dev, bw40_delta, val, i);

		if (~val)
			mt7601u_wr(dev, MT_TX_PWR_CFG_0 + i * 4, val);
	}

	mt7601u_extra_power_over_mac(dev);
}

int mt76_eeprom_init(struct mt76_dev *dev)
{
	int ret;
	u8 eeprom_ver;

	ret = mt76_eeprom_load(dev);
	if (ret)
		return ret;

	mt76_eeprom_get_macaddr(dev);

	if (!is_valid_ether_addr(dev->macaddr)) {
		eth_random_addr(dev->macaddr);
		dev_printk(KERN_INFO, dev->dev,
			   "Invalid MAC address, using random address %pM\n",
			   dev->macaddr);
	}

	/* TODO: move this out of here. */
	mt76_wr(dev, MT_MAC_ADDR_DW0, get_unaligned_le32(dev->macaddr));
	mt76_wr(dev, MT_MAC_ADDR_DW1, get_unaligned_le16(dev->macaddr + 4) |
		MT76_SET(MT_MAC_ADDR_DW1_U2ME_MASK, 0xff));

	ret = mt76_eeprom_get(dev, MT_EE_VERSION);
	eeprom_ver = MT76_GET(MT_EE_VERSION_EE, ret);
	trace_printk("EEPROM ver:%02hhx fae:%02hhx\n", eeprom_ver,
		     MT76_GET(MT_EE_VERSION_FAE, ret));
	if (eeprom_ver > MT7601U_EE_MAX_VER)
		printk("Warning: unsupported EEPROM version %d\n", eeprom_ver);

	mt7601u_set_channel_power(dev);

	trace_printk("Country region is %04x\n",
		    mt76_eeprom_get(dev, MT_EE_COUNTRY_REGION));

	mt7601u_set_ant_word(dev);
	mt7601u_set_cfg2_word(dev);
	mt7601u_set_rf_freq_off(dev);

	mt76_eeprom_get(dev, MT_EE_COUNTRY_REGION); /* TODO: drop */

	mt7601u_set_rssi_offset(dev);
	dev->lna_gain =	mt76_eeprom_get(dev, MT_EE_LNA_GAIN) & 0xff;

	mt7601u_config_tx_power_per_rate(dev);

	if (dev->cfg2 & MT_EE_NIC_CONF_1_HW_RF_CTRL)
		printk("Error: this driver does not support HW RF ctrl\n");

	mt76_eeprom_get(dev, MT_EE_G_TARGET_POWER); /* temp comp */

#ifdef TODO
	mt76_eeprom_parse_hw_cap(dev);
	mt76_get_of_overrides(dev);
#endif
	return 0;
}
