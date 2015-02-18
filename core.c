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

#include "mt7601u.h"

int mt7601u_wait_asic_ready(struct mt7601u_dev *dev)
{
	int i = 100;
	u32 val;

	do {
		val = mt7601u_rr(dev, MT_MAC_CSR0);
		if (val && ~val)
			return 0;
	} while (i--);

	return -EIO;
}

bool mt76_poll(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
	       int timeout)
{
	u32 cur;

	timeout /= 10;
	do {
		cur = mt76_rr(dev, offset) & mask;
		if (cur == val)
			return true;

		if (test_bit(MT7601U_STATE_REMOVED, &dev->state))
			return false;

		udelay(10);
	} while (timeout-- > 0);

	printk("Error: Time out with reg %08x\n", offset);

	return false;
}

bool mt76_poll_msec(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
		    int timeout)
{
	u32 cur;

	timeout /= 10;
	do {
		cur = mt76_rr(dev, offset) & mask;
		if (cur == val)
			return true;

		if (test_bit(MT7601U_STATE_REMOVED, &dev->state))
			return false;

		msleep(10);
	} while (timeout-- > 0);

	printk("Error: Time out with reg %08x\n", offset);

	return false;
}
