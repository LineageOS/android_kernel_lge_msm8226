/***************************************************************************
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *    File  	: lge_log.h
 *    Author(s)   : D3 BSP Sensor Team < d3-bsp-sensor@lge.com >
 *    Description :
 *
 ***************************************************************************/

#ifndef __LGE_LOG_H
#define __LGE_LOG_H

#include <linux/kernel.h>

#define DEBUG	1

#ifdef DEBUG
   #define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
   #define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt "\n", __FUNCTION__, __LINE__, ##args)
   #define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt "\n", __FUNCTION__,##args)
   #define SENSOR_DBG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[D]""%s : "fmt "\n", __FUNCTION__,##args)
#else
   #define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
   #define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt "\n", __FUNCTION__, __LINE__, ##args)
   #define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt "\n", __FUNCTION__,##args)
   #define SENSOR_DBG(fmt, args...)    NULL
#endif

#endif/*__LGE_LOG_H*/
