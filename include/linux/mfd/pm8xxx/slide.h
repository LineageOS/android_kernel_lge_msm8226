/* include/linux/mfd/pm8xxx/slide.h
 *
 * Copyright (c) 2011-2012, LG Electronics Inc, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PM8XXX_SLIDE_H__
#define __PM8XXX_SLIDE_H__

#define HALL_IC_DEV_NAME "bu52061nvx"


/* SMART COVER Support */
#define SMARTCOVER_SLIDE_CLOSED		1
#define SMARTCOVER_SLIDE_OPENED		0
#define SMARTCOVER_SLIDE_HALF		2
#define SMARTCOVER_SLIDE_TOP		3

struct pm8xxx_slide_platform_data {
	int hallic_top_detect_pin;
	int hallic_bottom_detect_pin;
	unsigned int hallic_top_irq;
	unsigned int hallic_bottom_irq;
	unsigned long irq_flags;
};

#endif /* __PM8XXX_SLIDE_H__ */
