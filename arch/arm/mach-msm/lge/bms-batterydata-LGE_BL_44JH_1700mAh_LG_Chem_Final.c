/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/batterydata-lib.h>

static struct single_row_lut fcc_temp = {
	.x		= {-20,0,25,40,60},
	.y		= {1616,1634,1626,1617,1602},
	.cols	= 5
};

static struct single_row_lut fcc_sf = {
	.x		= {0},
	.y		= {100},
	.cols	= 1
};

static struct sf_lut rbatt_sf = {
	.rows		= 30,
	.cols		= 5,
	.row_entries		= {-20,0,25,40,60},
	.percent	= {100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,16,13,11,10,9,8,7,6,5,4,3,2,1},
	.sf		= {
				{1071,273,99,81,74},
				{1071,273,99,81,74},
				{1258,281,104,84,77},
				{1461,287,108,88,79},
				{1301,295,114,91,81},
				{1196,299,121,95,84},
				{1172,286,127,100,87},
				{1171,280,120,101,90},
				{1198,279,108,92,84},
				{1248,280,106,86,79},
				{1326,285,108,89,81},
				{1473,290,112,93,84},
				{1701,294,118,94,86},
				{1966,302,119,94,83},
				{2289,322,114,91,81},
				{2780,363,109,86,79},
				{3796,423,112,87,78},
				{5438,484,129,98,84},
				{7828,530,147,109,93},
				{10133,568,161,117,101},
				{11980,588,169,121,103},
				{13826,607,177,123,106},
				{16001,627,186,129,109},
				{18523,623,195,136,114},
				{21027,616,197,146,120},
				{24215,675,220,158,127},
				{28394,824,245,172,137},
				{35612,1254,276,192,149},
				{46724,2030,356,230,175},
				{54410,3535,578,353,260}
	}
};

static struct pc_temp_ocv_lut pc_temp_ocv = {
	.rows		= 31,
	.cols		= 5,
	.temp		= {-20,0,25,40,60},
	.percent	= {100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,16,13,11,10,9,8,7,6,5,4,3,2,1,0},
	.ocv		= {
				{4310,4310,4305,4300,4292},
				{4218,4242,4244,4241,4234},
				{4148,4186,4190,4188,4182},
				{4094,4134,4138,4138,4132},
				{4037,4084,4090,4090,4086},
				{3966,4030,4044,4044,4040},
				{3912,3976,4000,4002,3998},
				{3869,3932,3950,3956,3956},
				{3837,3894,3904,3908,3909},
				{3814,3860,3869,3872,3871},
				{3794,3832,3842,3845,3844},
				{3776,3807,3820,3821,3822},
				{3756,3784,3801,3800,3798},
				{3736,3762,3780,3777,3770},
				{3710,3743,3753,3748,3739},
				{3675,3724,3718,3713,3705},
				{3618,3702,3689,3681,3670},
				{3547,3678,3678,3675,3664},
				{3472,3641,3646,3640,3632},
				{3424,3592,3610,3608,3602},
				{3404,3566,3588,3584,3578},
				{3384,3536,3564,3559,3550},
				{3358,3502,3531,3530,3523},
				{3327,3468,3499,3496,3492},
				{3296,3431,3462,3462,3459},
				{3272,3394,3427,3426,3424},
				{3252,3354,3390,3386,3383},
				{3224,3308,3340,3336,3332},
				{3190,3252,3274,3266,3260},
				{3096,3175,3180,3170,3158},
				{3000,3000,3000,3000,3000}
	}
};

struct bms_battery_data LGE_BL_44JH_1700mAh_LG_Chem_data = {
	.fcc				= 1700,
	.fcc_temp_lut			= &fcc_temp,
	.fcc_sf_lut				= &fcc_sf,
	.pc_temp_ocv_lut		= &pc_temp_ocv,
	.rbatt_sf_lut			= &rbatt_sf,
	.default_rbatt_mohm	= 192
};
// Charge voltage in mV
// CHARGE_VOLTAGE=4350
// Charge complete current in mA
// CHARGE_COMPLETE_CURRENT=150
// Discharge voltage in mV
// DISCHARGE_VOLTAGE=3000
// Battery ID resistance in kOhm
// BATTERY_ID_RESISTANCE=10
// Rbat capacitive in mOhm
// RBAT_CAPACITIVE=5000000
