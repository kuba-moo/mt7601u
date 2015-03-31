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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "mt7601u.h"
#include "mac.h"
#include <linux/etherdevice.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
#error Your kernel is too old, please upgrade to at least 3.19
#endif

static int mt7601u_start(struct ieee80211_hw *hw)
{
	struct mt7601u_dev *dev = hw->priv;
	int ret;

	mutex_lock(&dev->mutex);

	ret = mt7601u_mac_start(dev);
	if (ret)
		goto out;

	ieee80211_queue_delayed_work(dev->hw, &dev->mac_work,
				     MT_CALIBRATE_INTERVAL);
	ieee80211_queue_delayed_work(dev->hw, &dev->cal_work,
				     MT_CALIBRATE_INTERVAL);
out:
	mutex_unlock(&dev->mutex);
	return ret;
}

static void mt7601u_stop(struct ieee80211_hw *hw)
{
	struct mt7601u_dev *dev = hw->priv;

	mutex_lock(&dev->mutex);

	cancel_delayed_work_sync(&dev->cal_work);
	cancel_delayed_work_sync(&dev->mac_work);
	mt7601u_mac_stop(dev);

	mutex_unlock(&dev->mutex);
}

static int mt7601u_add_interface(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif)
{
	struct mt76_vif *mvif = (struct mt76_vif *) vif->drv_priv;
	unsigned int idx = 0;

	/* TODO: do the AP-STA things mt76 does:
	 *	- beacon offsets
	 *	- do mac address tricks
	 *	- shift vif idx
	 */
	mvif->idx = idx; /* TODO: wcid idx should be reserved in the map */
	mvif->group_wcid.idx = GROUP_WCID(idx);
	mvif->group_wcid.hw_key_idx = -1;

	printk("%s idx:%d\n", __func__, idx);

	return 0;
}

static void mt7601u_remove_interface(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif)
{
	printk("%s\n", __func__);
}

static int mt7601u_config(struct ieee80211_hw *hw, u32 changed)
{
	struct mt7601u_dev *dev = hw->priv;
	int ret = 0;

	printk("%s %08x ch:%d\n", __func__, changed,
	       hw->conf.chandef.chan->hw_value);
	mutex_lock(&dev->mutex);

/*	if (changed & IEEE80211_CONF_CHANGE_POWER) {
		dev->txpower_conf = hw->conf.power_level;

		if (test_bit(MT76_STATE_RUNNING, &dev->state))
			mt76_phy_set_txpower(dev);
	}*/

	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
		ieee80211_stop_queues(hw);
		/* TODO: mt76 stops mac while new channel is being set - we
		 *       stop it only during BW change.
		 * Note: there is no need to stop cal though 'cause we protect
		 *       cal with dev->mutex.
		 */
		ret = mt7601u_phy_set_channel(dev, &hw->conf.chandef);
		ieee80211_wake_queues(hw);
	}

	mutex_unlock(&dev->mutex);

	return ret;
}

static void
mt76_configure_filter(struct ieee80211_hw *hw, unsigned int changed_flags,
		      unsigned int *total_flags, u64 multicast)
{
	struct mt7601u_dev *dev = hw->priv;
	u32 flags = 0;

#define MT76_FILTER(_flag, _hw) do { \
		flags |= *total_flags & FIF_##_flag;			\
		dev->rxfilter &= ~(_hw);				\
		dev->rxfilter |= !(flags & FIF_##_flag) * (_hw);	\
	} while (0)

	printk("%s changed:%x total:%08x\n", __func__,
	       changed_flags, *total_flags);

	mutex_lock(&dev->mutex);

	dev->rxfilter &= ~MT_RX_FILTR_CFG_OTHER_BSS;

	MT76_FILTER(PROMISC_IN_BSS, MT_RX_FILTR_CFG_PROMISC);
	MT76_FILTER(FCSFAIL, MT_RX_FILTR_CFG_CRC_ERR);
	MT76_FILTER(PLCPFAIL, MT_RX_FILTR_CFG_PHY_ERR);
	MT76_FILTER(CONTROL, MT_RX_FILTR_CFG_ACK |
			     MT_RX_FILTR_CFG_CTS |
			     MT_RX_FILTR_CFG_CFEND |
			     MT_RX_FILTR_CFG_CFACK |
			     MT_RX_FILTR_CFG_BA |
			     MT_RX_FILTR_CFG_CTRL_RSV);
	MT76_FILTER(PSPOLL, MT_RX_FILTR_CFG_PSPOLL);

	*total_flags = flags;
	mt76_wr(dev, MT_RX_FILTR_CFG, dev->rxfilter);

	mutex_unlock(&dev->mutex);
}

static void
mt7601u_bss_info_changed(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			 struct ieee80211_bss_conf *info, u32 changed)
{
	struct mt7601u_dev *dev = hw->priv;

	/* 00004000 - BSS_CHANGED_IDLE
	 * 000000e0 - BSS_CHANGED_BASIC_RATES
		      BSS_CHANGED_BEACON_INT
		      BSS_CHANGED_BSSID
	 * 0010200d - BSS_CHANGED_ASSOC
		      BSS_CHANGED_ERP_PREAMBLE
		      BSS_CHANGED_ERP_SLOT
		      BSS_CHANGED_CQM
		      BSS_CHANGED_BEACON_INFO
	   ====================================
	 * 0000209f - 1e
		      BSS_CHANGED_ASSOC
		      BSS_CHANGED_BSSID
		      BSS_CHANGED_BEACON_INFO
	 * 00004000 - BSS_CHANGED_IDLE
	 */
	printk("%s %08x\n", __func__, changed);

	mutex_lock(&dev->mutex);

	if (changed & BSS_CHANGED_ASSOC)
		mt7601u_phy_con_cal_onoff(dev, info);

	if (changed & BSS_CHANGED_BSSID) {
		mt7601u_addr_wr(dev, MT_MAC_BSSID_DW0, info->bssid);

		/* TODO: this is a hack but beacon_int is not changed on leave,
		 *	 nor is any more appropriate event generated.
		 *	 rt2x00 doesn't seem to bother though.
		 */
		if (is_zero_ether_addr(info->bssid))
			mt7601u_mac_config_tsf(dev, false, 0);
	}

	if (changed & BSS_CHANGED_BASIC_RATES) {
		printk("basic rates: %08x\n", info->basic_rates);
		/* TODO: make sure those are ok - vendor does 0x15f. */
		mt7601u_wr(dev, MT_LEGACY_BASIC_RATE, info->basic_rates);
		/* TODO: properly configure fallback - values are ok */
		mt7601u_wr(dev, MT_HT_FBK_CFG0, 0x65432100);
		mt7601u_wr(dev, MT_HT_FBK_CFG1, 0xedcba980);
		mt7601u_wr(dev, MT_LG_FBK_CFG0, 0xedcba988);
		mt7601u_wr(dev, MT_LG_FBK_CFG1, 0x00002100);
	}

	if (changed & BSS_CHANGED_BEACON_INT)
		mt7601u_mac_config_tsf(dev, true, info->beacon_int);

	if (changed & BSS_CHANGED_HT || changed & BSS_CHANGED_ERP_CTS_PROT)
		mt7601u_mac_set_protection(dev, info->use_cts_prot,
					   info->ht_operation_mode);

	if (changed & BSS_CHANGED_ERP_PREAMBLE)
		mt7601u_mac_set_short_preamble(dev, info->use_short_preamble);

	if (changed & BSS_CHANGED_ERP_SLOT) {
		int slottime = info->use_short_slot ? 9 : 20;

		mt76_rmw_field(dev, MT_BKOFF_SLOT_CFG,
			       MT_BKOFF_SLOT_CFG_SLOTTIME, slottime);
	}

	if (changed & BSS_CHANGED_ASSOC)
		mt7601u_phy_recalibrate_after_assoc(dev);

	mutex_unlock(&dev->mutex);
}

static int
mt76_wcid_alloc(struct mt7601u_dev *dev)
{
	int i, idx = 0;

	for (i = 0; i < ARRAY_SIZE(dev->wcid_mask); i++) {
		idx = ffs(~dev->wcid_mask[i]);
		if (!idx)
			continue;

		idx--;
		dev->wcid_mask[i] |= BIT(idx);
		break;
	}

	idx = i * BITS_PER_LONG + idx;
	if (idx > 119)
		return -1;

	return idx;
}

static int
mt7601u_sta_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		struct ieee80211_sta *sta)
{
	struct mt7601u_dev *dev = hw->priv;
	struct mt76_sta *msta = (struct mt76_sta *) sta->drv_priv;
	struct mt76_vif *mvif = (struct mt76_vif *) vif->drv_priv;
	int ret = 0;
	int idx = 0;

	printk("%s\n", __func__);

	mutex_lock(&dev->mutex);

	idx = mt76_wcid_alloc(dev);
	if (idx < 0) {
		ret = -ENOSPC;
		goto out;
	}

	msta->wcid.idx = idx;
	msta->wcid.hw_key_idx = -1;
	mt7601u_mac_wcid_setup(dev, idx, mvif->idx, sta->addr);
	//mt76_clear(dev, MT_WCID_DROP(idx), MT_WCID_DROP_MASK(idx)); \/
	/* TODO: for TKIP set MT_RX_PARSER_RX_SET_NAV_ALL
	 *	 (see connect.c:2093).
	 */

	rcu_assign_pointer(dev->wcid[idx], &msta->wcid);
	mt7601u_mac_set_ampdu_factor(dev);

out:
	mutex_unlock(&dev->mutex);

	return ret;
}

static int
mt7601u_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		   struct ieee80211_sta *sta)
{
	struct mt7601u_dev *dev = hw->priv;
	struct mt76_sta *msta = (struct mt76_sta *) sta->drv_priv;
	int idx = msta->wcid.idx;

	printk("%s\n", __func__);

	mutex_lock(&dev->mutex);
	rcu_assign_pointer(dev->wcid[idx], NULL);
	//mt76_set(dev, MT_WCID_DROP(idx), MT_WCID_DROP_MASK(idx)); ^
	dev->wcid_mask[idx / BITS_PER_LONG] &= ~BIT(idx % BITS_PER_LONG);
	mt7601u_mac_wcid_setup(dev, idx, 0, NULL);
	mt7601u_mac_set_ampdu_factor(dev);
	mutex_unlock(&dev->mutex);

	return 0;
}

static void
mt7601u_sta_notify(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		   enum sta_notify_cmd cmd, struct ieee80211_sta *sta)
{
	printk("%s\n", __func__);
}

static void
mt7601u_sw_scan(struct ieee80211_hw *hw,
		struct ieee80211_vif *vif,
		const u8 *mac_addr)
{
	struct mt7601u_dev *dev = hw->priv;

	mt7601u_agc_save(dev); /* TODO: this will race with agc tune */
	set_bit(MT7601U_STATE_SCANNING, &dev->state);
}

static void
mt7601u_sw_scan_complete(struct ieee80211_hw *hw,
			 struct ieee80211_vif *vif)
{
	struct mt7601u_dev *dev = hw->priv;

	mt7601u_agc_restore(dev);
	clear_bit(MT7601U_STATE_SCANNING, &dev->state);
}

static int
mt7601u_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
		struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		struct ieee80211_key_conf *key)
{
	struct mt7601u_dev *dev = hw->priv;
	struct mt76_vif *mvif = (struct mt76_vif *) vif->drv_priv;
	struct mt76_sta *msta = sta ? (struct mt76_sta *) sta->drv_priv : NULL;
	struct mt76_wcid *wcid = msta ? &msta->wcid : &mvif->group_wcid;
	int idx = key->keyidx;
	int ret;

	printk("%s cmd:%x flg:%x kid:%x wid:%x\n",
	       __func__, cmd, key->flags, idx, wcid->idx);

	if (cmd == SET_KEY) {
		key->hw_key_idx = wcid->idx;
		wcid->hw_key_idx = idx;
	} else {
		if (idx == wcid->hw_key_idx)
			wcid->hw_key_idx = -1;

		key = NULL;
	}

	if (!msta) {
		if (key || wcid->hw_key_idx == idx) {
			ret = mt76_mac_wcid_set_key(dev, wcid->idx, key);
			if (ret)
				return ret;
		}

		return mt76_mac_shared_key_setup(dev, mvif->idx, idx, key);
	}

	return mt76_mac_wcid_set_key(dev, msta->wcid.idx, key);
}

static int mt7601u_set_rts_threshold(struct ieee80211_hw *hw, u32 value)
{
	struct mt7601u_dev *dev = hw->priv;

	mt76_rmw_field(dev, MT_TX_RTS_CFG, MT_TX_RTS_CFG_THRESH, value);

	return 0;
}

static int
mt76_ampdu_action(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		  enum ieee80211_ampdu_mlme_action action,
		  struct ieee80211_sta *sta, u16 tid, u16 *ssn, u8 buf_size)
{
	struct mt7601u_dev *dev = hw->priv;
	struct mt76_sta *msta = (struct mt76_sta *) sta->drv_priv;

	WARN_ON(msta->wcid.idx > GROUP_WCID(0));

	switch (action) {
	case IEEE80211_AMPDU_RX_START:
		mt76_set(dev, MT_WCID_ADDR(msta->wcid.idx)+4, BIT(16 + tid));
		break;
	case IEEE80211_AMPDU_RX_STOP:
		mt76_clear(dev, MT_WCID_ADDR(msta->wcid.idx)+4, BIT(16 + tid));
		break;
	case IEEE80211_AMPDU_TX_OPERATIONAL:
		ieee80211_send_bar(vif, sta->addr, tid, msta->agg_ssn[tid]);
		break;
	case IEEE80211_AMPDU_TX_STOP_FLUSH:
	case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
		break;
	case IEEE80211_AMPDU_TX_START:
		msta->agg_ssn[tid] = *ssn << 4;
		ieee80211_start_tx_ba_cb_irqsafe(vif, sta->addr, tid);
		break;
	case IEEE80211_AMPDU_TX_STOP_CONT:
		ieee80211_stop_tx_ba_cb_irqsafe(vif, sta->addr, tid);
		break;
	}

	return 0;
}

static void
mt76_sta_rate_tbl_update(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			 struct ieee80211_sta *sta)
{
	struct mt7601u_dev *dev = hw->priv;
	struct mt76_sta *msta = (struct mt76_sta *) sta->drv_priv;
	struct ieee80211_sta_rates *rates;
	struct ieee80211_tx_rate rate = {};

	rcu_read_lock();
	rates = rcu_dereference(sta->rates);

	if (!rates)
		goto out;

	rate.idx = rates->rate[0].idx;
	rate.flags = rates->rate[0].flags;
	mt76_mac_wcid_set_rate(dev, &msta->wcid, &rate);

out:
	rcu_read_unlock();
}

const struct ieee80211_ops mt7601u_ops = {
	.tx = mt7601u_tx,
	.start = mt7601u_start,
	.stop = mt7601u_stop,
	.add_interface = mt7601u_add_interface,
	.remove_interface = mt7601u_remove_interface,
	.config = mt7601u_config,
	.configure_filter = mt76_configure_filter,
	.bss_info_changed = mt7601u_bss_info_changed,
	.sta_add = mt7601u_sta_add,
	.sta_remove = mt7601u_sta_remove,
	.sta_notify = mt7601u_sta_notify,
	.set_key = mt7601u_set_key,
	.conf_tx = mt7601u_conf_tx,
	.sw_scan_start = mt7601u_sw_scan,
	.sw_scan_complete = mt7601u_sw_scan_complete,
/*	.flush = mt7601u_flush,*/
	.ampdu_action = mt76_ampdu_action,
/*	.get_txpower = mt7601u_get_txpower,
	.wake_tx_queue = mt7601u_wake_tx_queue,*/
	.sta_rate_tbl_update = mt76_sta_rate_tbl_update,
	.set_rts_threshold = mt7601u_set_rts_threshold,
};
