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
	.x		= {-20, 0, 25, 40, 65},
	.y		= {1601, 1645, 1656, 1666, 1673},
	.cols	= 5
};

static struct single_row_lut fcc_sf = {
	.x		= {0},
	.y		= {100},
	.cols	= 1
};

static struct sf_lut rbatt_sf = { 
        .rows       = 28, 
        .cols       = 5,
        /* row_entries are temperature */
        .row_entries    = {-20, 0, 25, 40, 60},
        .percent   = {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1},
        .sf   = {
					{1203,284,100,84,77},
					{1149,291,103,86,78},
					{1210,299,107,88,80},
					{1243,309,111,91,82},
					{1187,324,118,94,85},
					{1172,309,127,100,88},
					{1162,300,135,109,92},
					{1166,296,112,95,84},
					{1203,300,106,86,80},
					{1255,305,107,88,82},
					{1318,315,111,92,86},
					{1392,331,115,95,89},
					{1485,355,118,93,83},
					{1620,387,112,88,82},
					{1963,418,111,88,82},
					{2939,477,114,90,84},
					{5102,594,140,105,105},
					{9857,740,192,138,117},
					{9955,648,182,141,124},
					{10822,679,198,151,129},
					{11689,715,217,163,135},
					{13486,761,237,175,140},
					{15470,831,261,189,147},
					{17455,912,290,204,154},
					{22980,1185,336,224,166},
					{27335,1700,422,267,189},
					{38982,2702,582,363,253},
					{53388,5331,2201,1861,380},                                   
        }
};

static struct pc_temp_ocv_lut pc_temp_ocv = {
	.rows		= 29,
	.cols		= 5,
	.temp		= {-20, 0, 25, 40, 65},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
	.ocv		= {
				{4329, 4326, 4321, 4314, 4301},
				{4238, 4254, 4255, 4251, 4239},
				{4173, 4196, 4200, 4196, 4184},
				{4112, 4143, 4147, 4143, 4132},
				{4059, 4093, 4097, 4093, 4082},
				{3990, 4044, 4050, 4046, 4035},
				{3937, 3989, 4006, 4003, 3993},
				{3893, 3941, 3963, 3962, 3952},
				{3856, 3899, 3912, 3910, 3901},
				{3831, 3864, 3872, 3870, 3864},
				{3813, 3836, 3843, 3842, 3837},
				{3798, 3811, 3820, 3819, 3814},
				{3783, 3792, 3800, 3798, 3793},
				{3768, 3776, 3782, 3776, 3761},
				{3751, 3758, 3760, 3750, 3729},
				{3733, 3732, 3728, 3714, 3690},
				{3708, 3705, 3696, 3685, 3667},
				{3671, 3681, 3674, 3661, 3637},
				{3600, 3609, 3600, 3582, 3545},
				{3567, 3583, 3576, 3560, 3523},
				{3529, 3557, 3553, 3535, 3495},
				{3491, 3531, 3519, 3499, 3464},
				{3453, 3487, 3480, 3464, 3434},
				{3415, 3442, 3441, 3428, 3402},
				{3363, 3393, 3398, 3388, 3364},
				{3296, 3340, 3347, 3338, 3313},
				{3233, 3273, 3277, 3268, 3243},
				{3151, 3187, 3173, 3167, 3143},
				{3000, 3000, 3000, 3000, 3000}
	}
};

static struct sf_lut pc_sf = {
	.rows		= 1,
	.cols		= 1,
	/* row_entries are chargecycles */
	.row_entries	= {0},
	.percent	= {100},
	.sf		= {
			{100}
	},
};

struct bms_battery_data LGC_BL44JH_1700_data = {
	.fcc				= 1700,
	.fcc_temp_lut		= &fcc_temp,
	.fcc_sf_lut			= &fcc_sf,
	.pc_temp_ocv_lut	= &pc_temp_ocv,
	.pc_sf_lut			= &pc_sf,
	.rbatt_sf_lut		= &rbatt_sf,
	.default_rbatt_mohm		=189,

};
