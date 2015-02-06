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

#include <linux/debugfs.h>
#include "mt7601u.h"

static int
mt76_reg_set(void *data, u64 val)
{
	struct mt76_dev *dev = data;

	mt76_wr(dev, dev->debugfs_reg, val);
	return 0;
}

static int
mt76_reg_get(void *data, u64 *val)
{
	struct mt76_dev *dev = data;

	*val = mt76_rr(dev, dev->debugfs_reg);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_regval, mt76_reg_get, mt76_reg_set, "0x%08llx\n");


static int
mt76_ampdu_stat_read(struct seq_file *file, void *data)
{
	struct mt76_dev *dev = file->private;
	int i, j;

#define stat_printf(name)  seq_printf(file, #name ":\t%llu\n", dev->stats.name)
	stat_printf(rx_crc_err);
	stat_printf(rx_phy_err);
	stat_printf(rx_false_cca);
	stat_printf(rx_plcp_err);
	stat_printf(rx_fifo_overflow);
	stat_printf(rx_duplicate);

	stat_printf(tx_fail_cnt);
	stat_printf(tx_bcn_cnt);
	stat_printf(tx_success);
	stat_printf(tx_retransmit);
	stat_printf(tx_zero_len);
	stat_printf(tx_underflow);

	stat_printf(non_aggr_tx);
	stat_printf(aggr_tx);

	stat_printf(tx_zero_len_del);
	stat_printf(rx_zero_len_del);
#undef stat_printf

	seq_puts(file, "Aggregations stats:\n");
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 8; j++)
			seq_printf(file, "%08llx ",
				   dev->stats.aggr_size[i * 8 + j]);
		seq_puts(file, "\n");
	}

	return 0;
}

static int
mt76_ampdu_stat_open(struct inode *inode, struct file *f)
{
	return single_open(f, mt76_ampdu_stat_read, inode->i_private);
}

static const struct file_operations fops_ampdu_stat = {
	.open = mt76_ampdu_stat_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void mt76_init_debugfs(struct mt76_dev *dev)
{
	struct dentry *dir;

	dir = debugfs_create_dir("mt76", dev->hw->wiphy->debugfsdir);
	if (!dir)
		return;

	debugfs_create_blob("eeprom", S_IRUSR, dir, &dev->eeprom);
	debugfs_create_u8("temperature", S_IRUSR, dir, &dev->b49_temp);

	debugfs_create_u32("regidx", S_IRUSR | S_IWUSR, dir, &dev->debugfs_reg);
	debugfs_create_file("regval", S_IRUSR | S_IWUSR, dir, dev, &fops_regval);
	debugfs_create_file("ampdu_stat", S_IRUSR, dir, dev, &fops_ampdu_stat);
}
