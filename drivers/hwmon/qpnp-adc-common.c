/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/spmi.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/platform_device.h>
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
#include <mach/board_lge.h>
#endif


/* Min ADC code represets 0V */
#define QPNP_VADC_MIN_ADC_CODE			0x6000
/* Max ADC code represents full-scale range of 1.8V */
#define QPNP_VADC_MAX_ADC_CODE			0xA800
#define KELVINMIL_DEGMIL	273160

/* Units for temperature below (on x axis) is in 0.1DegC as
   required by the battery driver. Note the resolution used
   here to compute the table was done for DegC to milli-volts.
   In consideration to limit the size of the table for the given
   temperature range below, the result is linearly interpolated
   and provided to the battery driver in the units desired for
   their framework which is 0.1DegC. True resolution of 0.1DegC
   will result in the below table size to increase by 10 times */

/* LGE_CHANGE_S; apply battery thermistor adc value for W7 */
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
static const struct qpnp_vadc_map_pt adcmap_btm_threshold_for_rev0[] = {
	{-300,	1668},
	{-200,	1571},
	{-100,	1431},
	{0,	1255},
	{10,	1236},
	{20,	1217},
	{30,	1197},
	{40,	1177},
	{50,	1159},
	{60,	1139},
	{70,	1120},
	{80,	1100},
	{90,	1080},
	{100,	1061},
	{110,	1042},
	{120,	1023},
	{130,	1003},
	{140,	984},
	{150,	966},
	{160,	947},
	{170,	928},
	{180,	910},
	{190,	892},
	{200,	874},
	{210,	857},
	{220,	840},
	{230,	823},
	{240,	806},
	{250,	790},
	{260,	774},
	{270,	758},
	{280,	743},
	{290,	728},
	{300,	713},
	{310,	699},
	{320,	685},
	{330,	671},
	{340,	658},
	{350,	645},
	{360,	632},
	{370,	620},
	{380,	608},
	{390,	597},
	{400,	585},
	{410,	574},
	{420,	564},
	{430,	554},
	{440,	544},
	{450,	533},
	{460,	525},
	{470,	516},
	{480,	507},
	{490,	499},
	{500,	490},
	{510,	482},
	{520,	475},
	{530,	467},
	{540,	460},
	{550,	453},
	{560,	446},
	{570,	440},
	{580,	434},
	{590,	428},
	{600,	422},
	{610,	416},
	{620,	411},
	{630,	405},
	{640,	400},
	{650,	395},
	{660,	391},
	{670,	386},
	{680,	382},
	{690,	378},
	{700,	373},
	{710,	369},
	{720,	366},
	{730,	362},
	{740,	358},
	{750,	355},
	{760,	352},
	{770,	348},
	{780,	345},
	{790,	342}
};

#ifdef CONFIG_MACH_MSM8926_X5_SPR
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-300, 1627},
	{-200, 1507},
	{-100, 1326},
	{0, 1139},
	{10, 1120},
	{20, 1101},
	{30, 1082},
	{40, 1064},
	{50, 1045},
	{60, 1026},
	{70, 1008},
	{80, 989},
	{90, 972},
	{100, 963},
	{110, 947},
	{120, 930},
	{130, 914},
	{140, 898},
	{150, 882},
	{160, 865},
	{170, 849},
	{180, 833},
	{190, 816},
	{200, 800},
	{210, 787},
	{220, 774},
	{230, 761},
	{240, 748},
	{250, 735},
	{260, 721},
	{270, 708},
	{280, 695},
	{290, 682},
	{300, 669},
	{310, 658},
	{320, 646},
	{330, 635},
	{340, 624},
	{350, 613},
	{360, 603},
	{370, 596},
	{380, 587},
	{390, 579},
	{400, 570},
	{410, 562},
	{420, 555},
	{430, 547},
	{440, 540},
	{450, 536},
	{460, 530},
	{470, 524},
	{480, 518},
	{490, 512},
	{500, 506},
	{510, 500},
	{520, 494},
	{530, 487},
	{540, 481},
	{550, 475},
	{560, 472},
	{570, 469},
	{580, 461},
	{590, 457},
	{600, 453},
	{610, 449},
	{620, 446},
	{630, 442},
	{640, 439},
	{650, 436},
	{660, 433},
	{670, 430},
	{680, 427},
	{690, 425},
	{700, 422},
	{710, 420},
	{720, 417},
	{730, 415},
	{740, 413},
	{750, 411},
	{760, 409},
	{770, 407},
	{780, 405},
	{790, 403}

};
#elif defined (CONFIG_MACH_MSM8926_X10_VZW) || defined (CONFIG_MACH_MSM8926_B2L_ATT) || defined(CONFIG_MACH_MSM8926_B2LN_KR)
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-300, 1544},
	{-200, 1408},
	{-100, 1218},
	{0, 1045},
	{10, 1025},
	{20, 1010},
	{30, 1000},
	{40, 984},
	{50, 968},
	{60, 952},
	{70, 934},
	{80, 921},
	{90, 906},
	{100, 889},
	{110, 874},
	{120, 861},
	{130, 846},
	{140, 829},
	{150, 814},
	{160, 798},
	{170, 784},
	{180, 772},
	{190, 755},
	{200, 743},
	{210, 729},
	{220, 716},
	{230, 705},
	{240, 694},
	{250, 680},
	{260, 669},
	{270, 660},
	{280, 649},
	{290, 639},
	{300, 628},
	{310, 618},
	{320, 609},
	{330, 601},
	{340, 592},
	{350, 581},
	{360, 580},
	{370, 578},
	{380, 576},
	{390, 574},
	{400, 572},
	{410, 565},
	{420, 559},
	{430, 551},
	{440, 545},
	{450, 537},
	{460, 530},
	{470, 523},
	{480, 517},
	{490, 510},
	{500, 504},
	{510, 499},
	{520, 493},
	{530, 488},
	{540, 483},
	{550, 478},
	{560, 473},
	{570, 469},
	{580, 464},
	{590, 460},
	{600, 456},
	{610, 452},
	{620, 448},
	{630, 445},
	{640, 441},
	{650, 438},
	{660, 434},
	{670, 431},
	{680, 428},
	{690, 425},
	{700, 420},
	{710, 417},
	{720, 414},
	{730, 412},
	{740, 410},
	{750, 408},
	{760, 406},
	{770, 404},
	{780, 401},
	{790, 400}

};

#elif defined (CONFIG_MACH_MSM8926_JAGC_SPR) || defined (CONFIG_MACH_MSM8926_JAGN_KR) || defined (CONFIG_MACH_MSM8926_VFP_KR) ||  defined (CONFIG_MACH_MSM8926_AKA_CN) || defined(CONFIG_MACH_MSM8926_AKA_KR)
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-400, 1710},
	{-350, 1670},
	{-300, 1630},
	{-250, 1570},
	{-200, 1510},
	{-150, 1430},
	{-100, 1350},
	{-50, 1250},
	{0, 1160},
	{10, 1140},
	{20, 1120},
	{30, 1100},
	{40, 1080},
	{50, 1060},
	{60, 1040},
	{70, 1020},
	{80, 1000},
	{90, 980},
	{100, 970},
	{150, 890},
	{200, 810},
	{250, 740},
	{300, 680},
	{350, 620},
	{400, 580},
	{430, 560},
	{450, 540},
	{460, 530},
	{500, 510},
	{550, 480},
	{570, 470},
	{600, 460},
	{620, 440},
	{650, 430},
	{700, 430},
	{750, 420},
	{800, 410},
	{850, 400}
};

#elif defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGNM_GLOBAL_COM) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM8926_JAGNM_TLS) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL)
static const struct qpnp_vadc_map_pt adcmap_btm_threshold_A[] = {
	{-400,	1729},
	{-350,	1600},
	{-300,	1550},
	{-250,	1500},
	{-200,	1400},
	{-150,	1300},
	{-100,	1150},
	{-50,	1100},
	{0,	1000},
	{10, 	980},
	{20, 	960},
	{30, 	940},
	{40, 	920},
	{50,	900},
	{60, 	880},
	{70, 	860},
	{80, 	840},
	{90, 	820},
	{100,	800},
	{150,	700},
	{200,	600},
	{250,	500},
	{300,	450},
	{350,	370},
	{400,	310},
	{420,	300},
	{450,	280},
	{500,	230},
	{550,	200},
	{570,	180},
	{600,	170},
	{650,	150},
	{700,	118},
	{750,	109},
	{800,	90}
};

static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-400, 1710},
	{-350, 1670},
	{-300, 1630},
	{-250, 1570},
	{-200, 1510},
	{-150, 1430},
	{-100, 1350},
	{-50, 1250},
	{0, 1160},
	{10, 1140},
	{20, 1120},
	{30, 1100},
	{40, 1080},
	{50, 1060},
	{60, 1040},
	{70, 1020},
	{80, 1000},
	{90, 980},
	{100, 970},
	{150, 890},
	{200, 810},
	{250, 740},
	{300, 680},
	{350, 620},
	{400, 580},
	{430, 560},
	{450, 540},
	{460, 530},
	{500, 510},
	{550, 480},
	{570, 470},
	{600, 460},
	{620, 440},
	{650, 430},
	{700, 430},
	{750, 420},
	{800, 410},
	{850, 400}
};
#elif defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8226_E9WIFI) || defined (CONFIG_MACH_MSM8926_E7LTE_ATT_US) || defined(CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E7LTE_VZW_US) || defined (CONFIG_MACH_MSM8926_E7LTE_USC_US) || defined (CONFIG_MACH_MSM8926_T8LTE) || defined (CONFIG_MACH_MSM8926_E9LTE_VZW_US)
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-300,	1673},
	{-290,	1661},
	{-280,	1649},
	{-270,	1637},
	{-260,	1625},
	{-250,	1614},
	{-240,	1607},
	{-230,	1600},
	{-220,	1593},
	{-210,	1586},
	{-200,	1579},
	{-190,	1565},
	{-180,	1552},
	{-170,	1539},
	{-160,	1526},
	{-150,	1513},
	{-140,	1493},
	{-130,	1473},
	{-120,	1454},
	{-110,	1435},
	{-100,	1416},
	{-90,	1389},
	{-80,	1382},
	{-70,	1365},
	{-60,	1349},
	{-50,	1333},
	{-40,	1314},
	{-30,	1296},
	{-20,	1278},
	{-10,	1260},
	{0,	1242},
	{10,	1225},
	{20,	1208},
	{30,	1192},
	{40,	1176},
	{50,	1160},
	{60,	1141},
	{70,	1122},
	{80,	1104},
	{90,	1086},
	{100,	1068},
	{110,	1053},
	{120,	1035},
	{130,	1017},
	{140,	997},
	{150,	974},
	{160,	956},
	{170,	939},
	{180,	921},
	{190,	904},
	{200,	887},
	{210,	869},
	{220,	851},
	{230,	834},
	{240,	817},
	{250,	800},
	{260,	784},
	{270,	767},
	{280,	752},
	{290,	736},
	{300,	721},
	{310,	706},
	{320,	692},
	{330,	678},
	{340,	664},
	{350,	650},
	{360,	637},
	{370,	625},
	{380,	612},
	{390,	600},
	{400,	588},
	{410,	577},
	{420,	566},
	{430,	555},
	{440,	544},
	{450,	534},
	{460,	525},
	{470,	516},
	{480,	507},
	{490,	499},
	{500,	491},
	{510,	482},
	{520,	473},
	{530,	464},
	{540,	455},
	{550,	446},
	{560,	439},
	{570,	431},
	{580,	425},
	{590,	421},
	{600,	415},
	{610,	410},
	{620,	405},
	{630,	400},
	{640,	394},
	{650,	389},
	{660,	384},
	{670,	379},
	{680,	375},
	{690,	370},
	{700,	366},
	{710,	362},
	{720,	357},
	{730,	354},
	{740,	350},
	{750,	346},
	{760,	343},
	{770,	339},
	{780,	336},
	{790,	333}
};

#else
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-300,	1627},
	{-200,	1507},
	{-100,	1345},
	{0,	1159},
	{10,	1139},
	{20,	1120},
	{30,	1101},
	{40,	1082},
	{50,	1064},
	{60,	1045},
	{70,	1026},
	{80,	1008},
	{90,	989},
	{100,	972},
	{110,	954},
	{120,	936},
	{130,	919},
	{140,	902},
	{150,	885},
	{160,	869},
	{170,	853},
	{180,	837},
	{190,	822},
	{200,	807},
	{210,	792},
	{220,	777},
	{230,	763},
	{240,	750},
	{250,	737},
	{260,	724},
	{270,	711},
	{280,	699},
	{290,	687},
	{300,	675},
	{310,	664},
	{320,	653},
	{330,	643},
	{340,	633},
	{350,	623},
	{360,	613},
	{370,	604},
	{380,	595},
	{390,	586},
	{400,	578},
	{410,	570},
	{420,	562},
	{430,	555},
	{440,	547},
	{450,	540},
	{460,	533},
	{470,	527},
	{480,	521},
	{490,	515},
	{500,	509},
	{510,	503},
	{520,	498},
	{530,	492},
	{540,	487},
	{550,	482},
	{560,	478},
	{570,	473},
	{580,	469},
	{590,	465},
	{600,	461},
	{610,	457},
	{620,	453},
	{630,	449},
	{640,	446},
	{650,	442},
	{660,	439},
	{670,	436},
	{680,	433},
	{690,	430},
	{700,	427},
	{710,	425},
	{720,	422},
	{730,	420},
	{740,	417},
	{750,	415},
	{760,	413},
	{770,	411},
	{780,	409},
	{790,	407}
};
#endif
#else
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-300,	1642},
	{-200,	1544},
	{-100,	1414},
	{0, 1260},
	{10,	1244},
	{20,	1228},
	{30,	1212},
	{40,	1195},
	{50,	1179},
	{60,	1162},
	{70,	1146},
	{80,	1129},
	{90,	1113},
	{100,	1097},
	{110,	1080},
	{120,	1064},
	{130,	1048},
	{140,	1032},
	{150,	1016},
	{160,	1000},
	{170,	985},
	{180,	969},
	{190,	954},
	{200,	939},
	{210,	924},
	{220,	909},
	{230,	894},
	{240,	880},
	{250,	866},
	{260,	852},
	{270,	838},
	{280,	824},
	{290,	811},
	{300,	798},
	{310,	785},
	{320,	773},
	{330,	760},
	{340,	748},
	{350,	736},
	{360,	725},
	{370,	713},
	{380,	702},
	{390,	691},
	{400,	681},
	{410,	670},
	{420,	660},
	{430,	650},
	{440,	640},
	{450,	631},
	{460,	622},
	{470,	613},
	{480,	604},
	{490,	595},
	{500,	587},
	{510,	579},
	{520,	571},
	{530,	563},
	{540,	556},
	{550,	548},
	{560,	541},
	{570,	534},
	{580,	527},
	{590,	521},
	{600,	514},
	{610,	508},
	{620,	502},
	{630,	496},
	{640,	490},
	{650,	485},
	{660,	281},
	{670,	274},
	{680,	267},
	{690,	260},
	{700,	254},
	{710,	247},
	{720,	241},
	{730,	235},
	{740,	229},
	{750,	224},
	{760,	218},
	{770,	213},
	{780,	208},
	{790,	203}
};
#endif
/* LGE_CHANGE_E */

static const struct qpnp_vadc_map_pt adcmap_qrd_btm_threshold[] = {
	{-200,	1540},
	{-180,	1517},
	{-160,	1492},
	{-140,	1467},
	{-120,	1440},
	{-100,	1412},
	{-80,	1383},
	{-60,	1353},
	{-40,	1323},
	{-20,	1292},
	{0,	1260},
	{20,	1228},
	{40,	1196},
	{60,	1163},
	{80,	1131},
	{100,	1098},
	{120,	1066},
	{140,	1034},
	{160,	1002},
	{180,	971},
	{200,	941},
	{220,	911},
	{240,	882},
	{260,	854},
	{280,	826},
	{300,	800},
	{320,	774},
	{340,	749},
	{360,	726},
	{380,	703},
	{400,	681},
	{420,	660},
	{440,	640},
	{460,	621},
	{480,	602},
	{500,	585},
	{520,	568},
	{540,	552},
	{560,	537},
	{580,	523},
	{600,	510},
	{620,	497},
	{640,	485},
	{660,	473},
	{680,	462},
	{700,	452},
	{720,	442},
	{740,	433},
	{760,	424},
	{780,	416},
	{800,	408},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skuaa_btm_threshold[] = {
	{-200,	1476},
	{-180,	1450},
	{-160,	1422},
	{-140,	1394},
	{-120,	1365},
	{-100,	1336},
	{-80,	1306},
	{-60,	1276},
	{-40,	1246},
	{-20,	1216},
	{0,	1185},
	{20,	1155},
	{40,	1126},
	{60,	1096},
	{80,	1068},
	{100,	1040},
	{120,	1012},
	{140,	986},
	{160,	960},
	{180,	935},
	{200,	911},
	{220,	888},
	{240,	866},
	{260,	844},
	{280,	824},
	{300,	805},
	{320,	786},
	{340,	769},
	{360,	752},
	{380,	737},
	{400,	722},
	{420,	707},
	{440,	694},
	{460,	681},
	{480,	669},
	{500,	658},
	{520,	648},
	{540,	637},
	{560,	628},
	{580,	619},
	{600,	611},
	{620,	603},
	{640,	595},
	{660,	588},
	{680,	582},
	{700,	575},
	{720,	569},
	{740,	564},
	{760,	559},
	{780,	554},
	{800,	549},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skug_btm_threshold[] = {
	{-200,	1338},
	{-180,	1307},
	{-160,	1276},
	{-140,	1244},
	{-120,	1213},
	{-100,	1182},
	{-80,	1151},
	{-60,	1121},
	{-40,	1092},
	{-20,	1063},
	{0,	1035},
	{20,	1008},
	{40,	982},
	{60,	957},
	{80,	933},
	{100,	910},
	{120,	889},
	{140,	868},
	{160,	848},
	{180,	830},
	{200,	812},
	{220,	795},
	{240,	780},
	{260,	765},
	{280,	751},
	{300,	738},
	{320,	726},
	{340,	714},
	{360,	704},
	{380,	694},
	{400,	684},
	{420,	675},
	{440,	667},
	{460,	659},
	{480,	652},
	{500,	645},
	{520,	639},
	{540,	633},
	{560,	627},
	{580,	622},
	{600,	617},
	{620,	613},
	{640,	608},
	{660,	604},
	{680,	600},
	{700,	597},
	{720,	593},
	{740,	590},
	{760,	587},
	{780,	585},
	{800,	582},
};

/* Voltage to temperature */
static const struct qpnp_vadc_map_pt adcmap_100k_104ef_104fb[] = {
	{1758,	-40},
	{1742,	-35},
	{1719,	-30},
	{1691,	-25},
	{1654,	-20},
	{1608,	-15},
	{1551,	-10},
	{1483,	-5},
	{1404,	0},
	{1315,	5},
	{1218,	10},
	{1114,	15},
	{1007,	20},
	{900,	25},
	{795,	30},
	{696,	35},
	{605,	40},
	{522,	45},
	{448,	50},
	{383,	55},
	{327,	60},
	{278,	65},
	{237,	70},
	{202,	75},
	{172,	80},
	{146,	85},
	{125,	90},
	{107,	95},
	{92,	100},
	{79,	105},
	{68,	110},
	{59,	115},
	{51,	120},
	{44,	125}
};

/* Voltage to temperature */
static const struct qpnp_vadc_map_pt adcmap_150k_104ef_104fb[] = {
	{1738,	-40},
	{1714,	-35},
	{1682,	-30},
	{1641,	-25},
	{1589,	-20},
	{1526,	-15},
	{1451,	-10},
	{1363,	-5},
	{1266,	0},
	{1159,	5},
	{1048,	10},
	{936,	15},
	{825,	20},
	{720,	25},
	{622,	30},
	{533,	35},
	{454,	40},
	{385,	45},
	{326,	50},
	{275,	55},
	{232,	60},
	{195,	65},
	{165,	70},
	{139,	75},
	{118,	80},
	{100,	85},
	{85,	90},
	{73,	95},
	{62,	100},
	{53,	105},
	{46,	110},
	{40,	115},
	{34,	120},
	{30,	125}
};

static int32_t qpnp_adc_map_voltage_temp(const struct qpnp_vadc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if (pts == NULL)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].x < pts[1].x)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].x < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) &&
				(pts[i].x > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0)
		*output = pts[0].y;
	else if (i == tablesize)
		*output = pts[tablesize-1].y;
	else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].y - pts[i-1].y)*
			(input - pts[i-1].x))/
			(pts[i].x - pts[i-1].x))+
			pts[i-1].y);
	}

	return 0;
}

static int32_t qpnp_adc_map_temp_voltage(const struct qpnp_vadc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if (pts == NULL)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].y < pts[1].y)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].y < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) && (pts[i].y > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0) {
		*output = pts[0].x;
	} else if (i == tablesize) {
		*output = pts[tablesize-1].x;
	} else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].x - pts[i-1].x)*
			(input - pts[i-1].y))/
			(pts[i].y - pts[i-1].y))+
			pts[i-1].x);
	}

	return 0;
}

static int64_t qpnp_adc_scale_ratiometric_calib(int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties)
{
	int64_t adc_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties)
		return -EINVAL;

	adc_voltage = (adc_code -
		chan_properties->adc_graph[CALIB_RATIOMETRIC].adc_gnd)
		* adc_properties->adc_vdd_reference;
	if (adc_voltage < 0) {
		negative_offset = 1;
		adc_voltage = -adc_voltage;
	}
	do_div(adc_voltage,
		chan_properties->adc_graph[CALIB_RATIOMETRIC].dy);
	if (negative_offset)
		adc_voltage = -adc_voltage;

	return adc_voltage;
}

int32_t qpnp_adc_scale_pmic_therm(struct qpnp_vadc_chip *vadc,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t pmic_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result
		|| !chan_properties->adc_graph[CALIB_ABSOLUTE].dy)
		return -EINVAL;

	pmic_voltage = (adc_code -
		chan_properties->adc_graph[CALIB_ABSOLUTE].adc_gnd)
		* chan_properties->adc_graph[CALIB_ABSOLUTE].dx;
	if (pmic_voltage < 0) {
		negative_offset = 1;
		pmic_voltage = -pmic_voltage;
	}
	do_div(pmic_voltage,
		chan_properties->adc_graph[CALIB_ABSOLUTE].dy);
	if (negative_offset)
		pmic_voltage = -pmic_voltage;
	pmic_voltage += chan_properties->adc_graph[CALIB_ABSOLUTE].dx;

	if (pmic_voltage > 0) {
		/* 2mV/K */
		adc_chan_result->measurement = pmic_voltage*
			chan_properties->offset_gain_denominator;

		do_div(adc_chan_result->measurement,
			chan_properties->offset_gain_numerator * 2);
	} else {
		adc_chan_result->measurement = 0;
	}
	/* Change to .001 deg C */
	adc_chan_result->measurement -= KELVINMIL_DEGMIL;
	adc_chan_result->physical = (int32_t)adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_pmic_therm);

int32_t qpnp_adc_scale_millidegc_pmic_voltage_thr(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0, sign = 0;

	rc = qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_ABSOLUTE);
	if (rc < 0) {
		pr_err("Could not acquire gain and offset\n");
		return rc;
	}

	/* Convert to Kelvin and account for voltage to be written as 2mV/K */
	low_output = (param->low_temp + KELVINMIL_DEGMIL) * 2;
	/* Convert to voltage threshold */
	low_output = (low_output - QPNP_ADC_625_UV) * btm_param.dy;
	if (low_output < 0) {
		sign = 1;
		low_output = -low_output;
	}
	do_div(low_output, QPNP_ADC_625_UV);
	if (sign)
		low_output = -low_output;
	low_output += btm_param.adc_gnd;

	sign = 0;
	/* Convert to Kelvin and account for voltage to be written as 2mV/K */
	high_output = (param->high_temp + KELVINMIL_DEGMIL) * 2;
	/* Convert to voltage threshold */
	high_output = (high_output - QPNP_ADC_625_UV) * btm_param.dy;
	if (high_output < 0) {
		sign = 1;
		high_output = -high_output;
	}
	do_div(high_output, QPNP_ADC_625_UV);
	if (sign)
		high_output = -high_output;
	high_output += btm_param.adc_gnd;

	*low_threshold = (uint32_t) low_output;
	*high_threshold = (uint32_t) high_output;
	pr_debug("high_temp:%d, low_temp:%d\n", param->high_temp,
				param->low_temp);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_millidegc_pmic_voltage_thr);

/* Scales the ADC code to degC using the mapping
 * table for the XO thermistor.
 */
int32_t qpnp_adc_tdkntcg_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t xo_thm = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	xo_thm = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		xo_thm, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tdkntcg_therm);

int32_t qpnp_adc_scale_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);
	return qpnp_adc_map_temp_voltage(
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
#if defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGNM_GLOBAL_COM) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM8926_JAGNM_TLS) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL)
			(lge_get_board_revno() == HW_REV_A) ? adcmap_btm_threshold_A : adcmap_btm_threshold,

#elif defined (CONFIG_MACH_MSM8926_JAGC_SPR)
			adcmap_btm_threshold,

#else
			(lge_get_board_revno() == HW_REV_0) ? adcmap_btm_threshold_for_rev0 : adcmap_btm_threshold,
#endif
#else
			adcmap_btm_threshold,
#endif
			ARRAY_SIZE(adcmap_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_batt_therm);

int32_t qpnp_adc_scale_qrd_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_batt_therm);

int32_t qpnp_adc_scale_qrd_skuaa_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skuaa_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skuaa_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skuaa_batt_therm);

int32_t qpnp_adc_scale_qrd_skug_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skug_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skug_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skug_batt_therm);
int32_t qpnp_adc_scale_therm_pu1(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t therm_voltage = 0;

	therm_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_150k_104ef_104fb,
		ARRAY_SIZE(adcmap_150k_104ef_104fb),
		therm_voltage, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_therm_pu1);

int32_t qpnp_adc_scale_therm_pu2(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t therm_voltage = 0;

	therm_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		therm_voltage, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_therm_pu2);

int32_t qpnp_adc_tm_scale_voltage_therm_pu2(struct qpnp_vadc_chip *chip,
					uint32_t reg, int64_t *result)
{
	int64_t adc_voltage = 0;
	struct qpnp_vadc_linear_graph param1;
	int negative_offset;

	qpnp_get_vadc_gain_and_offset(chip, &param1, CALIB_RATIOMETRIC);

	adc_voltage = (reg - param1.adc_gnd) * param1.adc_vref;
	if (adc_voltage < 0) {
		negative_offset = 1;
		adc_voltage = -adc_voltage;
	}

	do_div(adc_voltage, param1.dy);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		adc_voltage, result);
	if (negative_offset)
		adc_voltage = -adc_voltage;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tm_scale_voltage_therm_pu2);

int32_t qpnp_adc_tm_scale_therm_voltage_pu2(struct qpnp_vadc_chip *chip,
				struct qpnp_adc_tm_config *param)
{
	struct qpnp_vadc_linear_graph param1;
	int rc;

	qpnp_get_vadc_gain_and_offset(chip, &param1, CALIB_RATIOMETRIC);

	rc = qpnp_adc_map_temp_voltage(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		param->low_thr_temp, &param->low_thr_voltage);
	if (rc)
		return rc;

	param->low_thr_voltage *= param1.dy;
	do_div(param->low_thr_voltage, param1.adc_vref);
	param->low_thr_voltage += param1.adc_gnd;

	rc = qpnp_adc_map_temp_voltage(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		param->high_thr_temp, &param->high_thr_voltage);
	if (rc)
		return rc;

	param->high_thr_voltage *= param1.dy;
	do_div(param->high_thr_voltage, param1.adc_vref);
	param->high_thr_voltage += param1.adc_gnd;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tm_scale_therm_voltage_pu2);

int32_t qpnp_adc_scale_batt_id(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t batt_id_voltage = 0;

	batt_id_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);
	adc_chan_result->physical = batt_id_voltage;
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_batt_id);

int32_t qpnp_adc_scale_default(struct qpnp_vadc_chip *vadc,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	bool negative_rawfromoffset = 0, negative_offset = 0;
	int64_t scale_voltage = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	scale_voltage = (adc_code -
		chan_properties->adc_graph[chan_properties->calib_type].adc_gnd)
		* chan_properties->adc_graph[chan_properties->calib_type].dx;
	if (scale_voltage < 0) {
		negative_offset = 1;
		scale_voltage = -scale_voltage;
	}
	do_div(scale_voltage,
		chan_properties->adc_graph[chan_properties->calib_type].dy);
	if (negative_offset)
		scale_voltage = -scale_voltage;

	if (chan_properties->calib_type == CALIB_ABSOLUTE)
		scale_voltage +=
		chan_properties->adc_graph[chan_properties->calib_type].dx;
	else
		scale_voltage *= 1000;

	if (scale_voltage < 0) {
		if (adc_properties->bipolar) {
			scale_voltage = -scale_voltage;
			negative_rawfromoffset = 1;
		} else {
		#ifdef CONFIG_LGE_PM
			pr_err("adc_result is negative! ->  %lld uV \n", scale_voltage);
			pr_err("adc code = 0x%x, gnd = %lld, dx = %lld, dy = %lld \n",
			   adc_code, chan_properties->adc_graph[CALIB_ABSOLUTE].adc_gnd,
			   chan_properties->adc_graph[CALIB_ABSOLUTE].dx,
			   chan_properties->adc_graph[CALIB_ABSOLUTE].dy);
		#endif
			scale_voltage = 0;
		}
	}

	adc_chan_result->measurement = scale_voltage *
				chan_properties->offset_gain_denominator;

	/* do_div only perform positive integer division! */
	do_div(adc_chan_result->measurement,
				chan_properties->offset_gain_numerator);

	if (negative_rawfromoffset)
		adc_chan_result->measurement = -adc_chan_result->measurement;

	/*
	 * Note: adc_chan_result->measurement is in the unit of
	 * adc_properties.adc_reference. For generic channel processing,
	 * channel measurement is a scale/ratio relative to the adc
	 * reference input
	 */
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_default);

int32_t qpnp_adc_usb_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph usb_param;

	qpnp_get_vadc_gain_and_offset(chip, &usb_param, CALIB_RATIOMETRIC);

	*low_threshold = param->low_thr * usb_param.dy;
	do_div(*low_threshold, usb_param.adc_vref);
	*low_threshold += usb_param.adc_gnd;

	*high_threshold = param->high_thr * usb_param.dy;
	do_div(*high_threshold, usb_param.adc_vref);
	*high_threshold += usb_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_usb_scaler);

int32_t qpnp_adc_vbatt_rscaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph vbatt_param;
	int rc = 0, sign = 0;
	int64_t low_thr = 0, high_thr = 0;

	rc = qpnp_get_vadc_gain_and_offset(chip, &vbatt_param, CALIB_ABSOLUTE);
	if (rc < 0)
		return rc;

	low_thr = (((param->low_thr/3) - QPNP_ADC_625_UV) *
				vbatt_param.dy);
	if (low_thr < 0) {
		sign = 1;
		low_thr = -low_thr;
	}
	do_div(low_thr, QPNP_ADC_625_UV);
	if (sign)
		low_thr = -low_thr;
	*low_threshold = low_thr + vbatt_param.adc_gnd;

	sign = 0;
	high_thr = (((param->high_thr/3) - QPNP_ADC_625_UV) *
				vbatt_param.dy);
	if (high_thr < 0) {
		sign = 1;
		high_thr = -high_thr;
	}
	do_div(high_thr, QPNP_ADC_625_UV);
	if (sign)
		high_thr = -high_thr;
	*high_threshold = high_thr + vbatt_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_vbatt_rscaler);

int32_t qpnp_adc_btm_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0;

	qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_RATIOMETRIC);

	pr_debug("warm_temp:%d and cool_temp:%d\n", param->high_temp,
				param->low_temp);
	rc = qpnp_adc_map_voltage_temp(
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		(lge_get_board_revno() == HW_REV_0) ? adcmap_btm_threshold_for_rev0 : adcmap_btm_threshold,
#else
		adcmap_btm_threshold,
#endif
		ARRAY_SIZE(adcmap_btm_threshold),
		(param->low_temp),
		&low_output);
	if (rc) {
		pr_debug("low_temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("low_output:%lld\n", low_output);
	low_output *= btm_param.dy;
	do_div(low_output, btm_param.adc_vref);
	low_output += btm_param.adc_gnd;

	rc = qpnp_adc_map_voltage_temp(
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		(lge_get_board_revno() == HW_REV_0) ? adcmap_btm_threshold_for_rev0 : adcmap_btm_threshold,
#else
		adcmap_btm_threshold,
#endif
		ARRAY_SIZE(adcmap_btm_threshold),
		(param->high_temp),
		&high_output);
	if (rc) {
		pr_debug("high temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("high_output:%lld\n", high_output);
	high_output *= btm_param.dy;
	do_div(high_output, btm_param.adc_vref);
	high_output += btm_param.adc_gnd;

	/* btm low temperature correspondes to high voltage threshold */
	*low_threshold = high_output;
	/* btm high temperature correspondes to low voltage threshold */
	*high_threshold = low_output;

	pr_debug("high_volt:%d, low_volt:%d\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_btm_scaler);

int32_t qpnp_vadc_check_result(int32_t *data)
{
	if (*data < QPNP_VADC_MIN_ADC_CODE)
		*data = QPNP_VADC_MIN_ADC_CODE;
	else if (*data > QPNP_VADC_MAX_ADC_CODE)
		*data = QPNP_VADC_MAX_ADC_CODE;

	return 0;
}
EXPORT_SYMBOL(qpnp_vadc_check_result);

/*Qualcomm patch -BMS tuning eunho.yang@lge.com
  *Issue : Battery soc didn't match battery voltage*/
#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8226_E9WIFI) || defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E7LTE_ATT_US) || defined (CONFIG_MACH_MSM8926_E7LTE_VZW_US) || defined (CONFIG_MACH_MSM8926_E7LTE_USC_US) || defined (CONFIG_MACH_MSM8926_T8LTE) || defined (CONFIG_MACH_MSM8926_E9LTE_VZW_US)
int qpnp_adc_get_revid_version(struct device *dev)
{
	struct pmic_revid_data *revid_data;
	struct device_node *revid_dev_node;

	revid_dev_node = of_parse_phandle(dev->of_node,
						"qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_debug("Missing qcom,pmic-revid property\n");
		return -EINVAL;
	}

	revid_data = get_revid_data(revid_dev_node);
	if (IS_ERR(revid_data)) {
		pr_err("revid error rc = %ld\n", PTR_ERR(revid_data));
		return -EINVAL;
	}

	if ((revid_data->rev1 == PM8941_V3P1_REV1) &&
		(revid_data->rev2 == PM8941_V3P1_REV2) &&
		(revid_data->rev3 == PM8941_V3P1_REV3) &&
		(revid_data->rev4 == PM8941_V3P1_REV4) &&
		(revid_data->pmic_type == PM8941_V3P1_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V3P1_SUBTYPE))
			return QPNP_REV_ID_8941_3_1;
#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8226_E9WIFI) || defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E7LTE_ATT_US) || defined (CONFIG_MACH_MSM8926_E7LTE_VZW_US) || defined (CONFIG_MACH_MSM8926_E7LTE_USC_US) || defined (CONFIG_MACH_MSM8926_T8LTE) || defined (CONFIG_MACH_MSM8926_E9LTE_VZW_US)
	else if ((revid_data->rev1 == PM8941_V3P0_REV1) &&
		(revid_data->rev2 == PM8941_V3P0_REV2) &&
		(revid_data->rev3 == PM8941_V3P0_REV3) &&
		(revid_data->rev4 == PM8941_V3P0_REV4) &&
		(revid_data->pmic_type == PM8941_V3P0_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V3P0_SUBTYPE))
			return QPNP_REV_ID_8941_3_0;
	else if ((revid_data->rev1 == PM8941_V2P0_REV1) &&
		(revid_data->rev2 == PM8941_V2P0_REV2) &&
		(revid_data->rev3 == PM8941_V2P0_REV3) &&
		(revid_data->rev4 == PM8941_V2P0_REV4) &&
		(revid_data->pmic_type == PM8941_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V2P0_SUBTYPE))
			return QPNP_REV_ID_8941_2_0;
	else if ((revid_data->rev1 == PM8226_V2P2_REV1) &&
		(revid_data->rev2 == PM8226_V2P2_REV2) &&
		(revid_data->rev3 == PM8226_V2P2_REV3) &&
		(revid_data->rev4 == PM8226_V2P2_REV4) &&
		(revid_data->pmic_type == PM8226_V2P2_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P2_SUBTYPE))
			return QPNP_REV_ID_8026_2_2;
#endif
	else if ((revid_data->rev1 == PM8226_V2P1_REV1) &&
		(revid_data->rev2 == PM8226_V2P1_REV2) &&
		(revid_data->rev3 == PM8226_V2P1_REV3) &&
		(revid_data->rev4 == PM8226_V2P1_REV4) &&
		(revid_data->pmic_type == PM8226_V2P1_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P1_SUBTYPE))
			return QPNP_REV_ID_8026_2_1;
	else if ((revid_data->rev1 == PM8226_V2P0_REV1) &&
		(revid_data->rev2 == PM8226_V2P0_REV2) &&
		(revid_data->rev3 == PM8226_V2P0_REV3) &&
		(revid_data->rev4 == PM8226_V2P0_REV4) &&
		(revid_data->pmic_type == PM8226_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P0_SUBTYPE))
			return QPNP_REV_ID_8026_2_0;
	else if ((revid_data->rev1 == PM8226_V1P0_REV1) &&
		(revid_data->rev2 == PM8226_V1P0_REV2) &&
		(revid_data->rev3 == PM8226_V1P0_REV3) &&
		(revid_data->rev4 == PM8226_V1P0_REV4) &&
		(revid_data->pmic_type == PM8226_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V1P0_SUBTYPE))
			return QPNP_REV_ID_8026_1_0;
	else if ((revid_data->rev1 == PM8110_V1P0_REV1) &&
		(revid_data->rev2 == PM8110_V1P0_REV2) &&
		(revid_data->rev3 == PM8110_V1P0_REV3) &&
		(revid_data->rev4 == PM8110_V1P0_REV4) &&
		(revid_data->pmic_type == PM8110_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8110_V1P0_SUBTYPE))
			return QPNP_REV_ID_8110_1_0;
/*Qualcomm patch -BMS tuning eunho.yang@lge.com
  *Issue : Battery soc didn't match battery voltage*/
#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8226_E9WIFI) || defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E7LTE_ATT_US) || defined (CONFIG_MACH_MSM8926_E7LTE_VZW_US) || defined (CONFIG_MACH_MSM8926_E7LTE_USC_US) || defined (CONFIG_MACH_MSM8926_T8LTE) || defined (CONFIG_MACH_MSM8926_E9LTE_VZW_US)
	else if ((revid_data->rev1 == PM8110_V2P0_REV1) &&
		(revid_data->rev2 == PM8110_V2P0_REV2) &&
		(revid_data->rev3 == PM8110_V2P0_REV3) &&
		(revid_data->rev4 == PM8110_V2P0_REV4) &&
		(revid_data->pmic_type == PM8110_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8110_V2P0_SUBTYPE))
			return QPNP_REV_ID_8110_2_0;
#endif
	else
		return -EINVAL;
}
EXPORT_SYMBOL(qpnp_adc_get_revid_version);
#else

int qpnp_adc_get_revid_version(struct device *dev)
{
	struct pmic_revid_data *revid_data;
	struct device_node *revid_dev_node;

	revid_dev_node = of_parse_phandle(dev->of_node,
						"qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_debug("Missing qcom,pmic-revid property\n");
		return -EINVAL;
	}

	revid_data = get_revid_data(revid_dev_node);
	if (IS_ERR(revid_data)) {
		pr_debug("revid error rc = %ld\n", PTR_ERR(revid_data));
		return -EINVAL;
	}

	if ((revid_data->rev1 == PM8941_V3P1_REV1) &&
		(revid_data->rev2 == PM8941_V3P1_REV2) &&
		(revid_data->rev3 == PM8941_V3P1_REV3) &&
		(revid_data->rev4 == PM8941_V3P1_REV4) &&
		(revid_data->pmic_type == PM8941_V3P1_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V3P1_SUBTYPE))
			return QPNP_REV_ID_8941_3_1;
	else if ((revid_data->rev1 == PM8941_V3P0_REV1) &&
		(revid_data->rev2 == PM8941_V3P0_REV2) &&
		(revid_data->rev3 == PM8941_V3P0_REV3) &&
		(revid_data->rev4 == PM8941_V3P0_REV4) &&
		(revid_data->pmic_type == PM8941_V3P0_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V3P0_SUBTYPE))
			return QPNP_REV_ID_8941_3_0;
	else if ((revid_data->rev1 == PM8941_V2P0_REV1) &&
		(revid_data->rev2 == PM8941_V2P0_REV2) &&
		(revid_data->rev3 == PM8941_V2P0_REV3) &&
		(revid_data->rev4 == PM8941_V2P0_REV4) &&
		(revid_data->pmic_type == PM8941_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V2P0_SUBTYPE))
			return QPNP_REV_ID_8941_2_0;
	else if ((revid_data->rev1 == PM8226_V2P2_REV1) &&
		(revid_data->rev2 == PM8226_V2P2_REV2) &&
		(revid_data->rev3 == PM8226_V2P2_REV3) &&
		(revid_data->rev4 == PM8226_V2P2_REV4) &&
		(revid_data->pmic_type == PM8226_V2P2_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P2_SUBTYPE))
			return QPNP_REV_ID_8026_2_2;
	else if ((revid_data->rev1 == PM8226_V2P1_REV1) &&
		(revid_data->rev2 == PM8226_V2P1_REV2) &&
		(revid_data->rev3 == PM8226_V2P1_REV3) &&
		(revid_data->rev4 == PM8226_V2P1_REV4) &&
		(revid_data->pmic_type == PM8226_V2P1_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P1_SUBTYPE))
			return QPNP_REV_ID_8026_2_1;
	else if ((revid_data->rev1 == PM8226_V2P0_REV1) &&
		(revid_data->rev2 == PM8226_V2P0_REV2) &&
		(revid_data->rev3 == PM8226_V2P0_REV3) &&
		(revid_data->rev4 == PM8226_V2P0_REV4) &&
		(revid_data->pmic_type == PM8226_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P0_SUBTYPE))
			return QPNP_REV_ID_8026_2_0;
	else if ((revid_data->rev1 == PM8226_V1P0_REV1) &&
		(revid_data->rev2 == PM8226_V1P0_REV2) &&
		(revid_data->rev3 == PM8226_V1P0_REV3) &&
		(revid_data->rev4 == PM8226_V1P0_REV4) &&
		(revid_data->pmic_type == PM8226_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V1P0_SUBTYPE))
			return QPNP_REV_ID_8026_1_0;
	else if ((revid_data->rev1 == PM8110_V1P0_REV1) &&
		(revid_data->rev2 == PM8110_V1P0_REV2) &&
		(revid_data->rev3 == PM8110_V1P0_REV3) &&
		(revid_data->rev4 == PM8110_V1P0_REV4) &&
		(revid_data->pmic_type == PM8110_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8110_V1P0_SUBTYPE))
			return QPNP_REV_ID_8110_1_0;
	else if ((revid_data->rev1 == PM8110_V2P0_REV1) &&
		(revid_data->rev2 == PM8110_V2P0_REV2) &&
		(revid_data->rev3 == PM8110_V2P0_REV3) &&
		(revid_data->rev4 == PM8110_V2P0_REV4) &&
		(revid_data->pmic_type == PM8110_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8110_V2P0_SUBTYPE))
			return QPNP_REV_ID_8110_2_0;
	else
		return -EINVAL;
}
EXPORT_SYMBOL(qpnp_adc_get_revid_version);
#endif

int32_t qpnp_adc_get_devicetree_data(struct spmi_device *spmi,
			struct qpnp_adc_drv *adc_qpnp)
{
	struct device_node *node = spmi->dev.of_node;
	struct resource *res;
	struct device_node *child;
	struct qpnp_adc_amux *adc_channel_list;
	struct qpnp_adc_properties *adc_prop;
	struct qpnp_adc_amux_properties *amux_prop;
	int count_adc_channel_list = 0, decimation, rc = 0, i = 0;

	if (!node)
		return -EINVAL;

	for_each_child_of_node(node, child)
		count_adc_channel_list++;

	if (!count_adc_channel_list) {
		pr_err("No channel listing\n");
		return -EINVAL;
	}

	adc_qpnp->spmi = spmi;

	adc_prop = devm_kzalloc(&spmi->dev, sizeof(struct qpnp_adc_properties),
					GFP_KERNEL);
	if (!adc_prop) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	adc_channel_list = devm_kzalloc(&spmi->dev,
		((sizeof(struct qpnp_adc_amux)) * count_adc_channel_list),
				GFP_KERNEL);
	if (!adc_channel_list) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	amux_prop = devm_kzalloc(&spmi->dev,
		sizeof(struct qpnp_adc_amux_properties) +
		sizeof(struct qpnp_vadc_chan_properties), GFP_KERNEL);
	if (!amux_prop) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	adc_qpnp->adc_channels = adc_channel_list;
	adc_qpnp->amux_prop = amux_prop;

	for_each_child_of_node(node, child) {
		int channel_num, scaling, post_scaling, hw_settle_time;
		int fast_avg_setup, calib_type = 0, rc;
		const char *calibration_param, *channel_name;

		channel_name = of_get_property(child,
				"label", NULL) ? : child->name;
		if (!channel_name) {
			pr_err("Invalid channel name\n");
			return -EINVAL;
		}

		rc = of_property_read_u32(child, "reg", &channel_num);
		if (rc) {
			pr_err("Invalid channel num\n");
			return -EINVAL;
		}
		rc = of_property_read_u32(child, "qcom,decimation",
								&decimation);
		if (rc) {
			pr_err("Invalid channel decimation property\n");
			return -EINVAL;
		}
		if (!of_device_is_compatible(node, "qcom,qpnp-iadc")) {
			rc = of_property_read_u32(child,
				"qcom,hw-settle-time", &hw_settle_time);
			if (rc) {
				pr_err("Invalid channel hw settle time property\n");
				return -EINVAL;
			}
			rc = of_property_read_u32(child,
				"qcom,pre-div-channel-scaling", &scaling);
			if (rc) {
				pr_err("Invalid channel scaling property\n");
				return -EINVAL;
			}
			rc = of_property_read_u32(child,
				"qcom,scale-function", &post_scaling);
			if (rc) {
				pr_err("Invalid channel post scaling property\n");
				return -EINVAL;
			}
			rc = of_property_read_string(child,
				"qcom,calibration-type", &calibration_param);
			if (rc) {
				pr_err("Invalid calibration type\n");
				return -EINVAL;
			}
			if (!strcmp(calibration_param, "absolute"))
				calib_type = CALIB_ABSOLUTE;
			else if (!strcmp(calibration_param, "ratiometric"))
				calib_type = CALIB_RATIOMETRIC;
			else {
				pr_err("%s: Invalid calibration property\n",
						__func__);
				return -EINVAL;
			}
		}
		rc = of_property_read_u32(child,
				"qcom,fast-avg-setup", &fast_avg_setup);
		if (rc) {
			pr_err("Invalid channel fast average setup\n");
			return -EINVAL;
		}
		/* Individual channel properties */
		adc_channel_list[i].name = (char *)channel_name;
		adc_channel_list[i].channel_num = channel_num;
		adc_channel_list[i].adc_decimation = decimation;
		adc_channel_list[i].fast_avg_setup = fast_avg_setup;
		if (!of_device_is_compatible(node, "qcom,qpnp-iadc")) {
			adc_channel_list[i].chan_path_prescaling = scaling;
			adc_channel_list[i].adc_scale_fn = post_scaling;
			adc_channel_list[i].hw_settle_time = hw_settle_time;
			adc_channel_list[i].calib_type = calib_type;
		}
		i++;
	}

	/* Get the ADC VDD reference voltage and ADC bit resolution */
	rc = of_property_read_u32(node, "qcom,adc-vdd-reference",
			&adc_prop->adc_vdd_reference);
	if (rc) {
		pr_err("Invalid adc vdd reference property\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,adc-bit-resolution",
			&adc_prop->bitresolution);
	if (rc) {
		pr_err("Invalid adc bit resolution property\n");
		return -EINVAL;
	}
	adc_qpnp->adc_prop = adc_prop;

	/* Get the peripheral address */
	res = spmi_get_resource(spmi, 0, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("No base address definition\n");
		return -EINVAL;
	}

	adc_qpnp->slave = spmi->sid;
	adc_qpnp->offset = res->start;

	/* Register the ADC peripheral interrupt */
	adc_qpnp->adc_irq_eoc = spmi_get_irq_byname(spmi, NULL,
						"eoc-int-en-set");
	if (adc_qpnp->adc_irq_eoc < 0) {
		pr_err("Invalid irq\n");
		return -ENXIO;
	}

	init_completion(&adc_qpnp->adc_rslt_completion);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_get_devicetree_data);
