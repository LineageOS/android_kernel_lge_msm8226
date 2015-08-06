/*
 * SX9500 Cap Touch 
 * Currently Supports:
 *  SX9500
 *
 * Copyright 2012 Semtech Corp.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _SX9500_I2C_REG_H_
#define _SX9500_I2C_REG_H_

/*
 *  I2C Registers
 */
#define SX9500_IRQSTAT_REG    0x00
#define SX9500_TCHCMPSTAT_REG 0x01
#define SX9500_IRQ_ENABLE_REG 0x03
#define SX9500_CPS_CTRL0_REG  0x06
#define SX9500_CPS_CTRL1_REG  0x07
#define SX9500_CPS_CTRL2_REG  0x08
#define SX9500_CPS_CTRL3_REG  0x09
#define SX9500_CPS_CTRL4_REG  0x0A
#define SX9500_CPS_CTRL5_REG  0x0B
#define SX9500_CPS_CTRL6_REG  0x0C
#define SX9500_CPS_CTRL7_REG  0x0D
#define SX9500_CPS_CTRL8_REG  0x0E
#define SX9500_SOFTRESET_REG  0x7F

/* Sensor Readback */
#define SX9500_SENSORSEL_REG  0x20

#define SX9500_USEMSB_REG     0x21
#define SX9500_USELSB_REG     0x22

#define SX9500_AVGMSB_REG     0x23
#define SX9500_AVGLSB_REG     0x24

#define SX9500_DIFFMSB_REG    0x25
#define SX9500_DIFFLSB_REG    0x26

#define SX9500_OFFSETMSB_REG  0x27
#define SX9500_OFFSETLSB_REG  0x28

/* IrqStat 0:Inactive 1:Active     */
#define SX9500_IRQSTAT_RESET_FLAG      0x80
#define SX9500_IRQSTAT_TOUCH_FLAG      0x40
#define SX9500_IRQSTAT_RELEASE_FLAG    0x20
#define SX9500_IRQSTAT_COMPDONE_FLAG   0x10
#define SX9500_IRQSTAT_CONV_FLAG       0x08
#define SX9500_IRQSTAT_TXENSTAT_FLAG   0x01


/* CpsStat  */
#define SX9500_TCHCMPSTAT_TCHSTAT3_FLAG   0x80
#define SX9500_TCHCMPSTAT_TCHSTAT2_FLAG   0x40
#define SX9500_TCHCMPSTAT_TCHSTAT1_FLAG   0x20
#define SX9500_TCHCMPSTAT_TCHSTAT0_FLAG   0x10

/* SoftReset */
#define SX9500_SOFTRESET  0xDE

/* CSx PIN */
#define SENSORSEL_CS0  0x00
#define SENSORSEL_CS1  0x01
#define SENSORSEL_CS2  0x02
#define SENSORSEL_CS3  0x03

#define SENSOREN_DISABLE_ALL 0x00
#define SENSOREN_CS0   0x01
#define SENSOREN_CS1   0x02
#define SENSOREN_CS2   0x04
#define SENSOREN_CS3   0x08

/* Input Capacitance Range */
#define REG_RANGE_LARGE        0x00
#define REG_RANGE_MEDIUM_LARGE 0x01
#define REG_RANGE_MEDIUM_SMALL 0x02
#define REG_RANGE_SMALL        0x03

#define RANGE_LARGE            146000
#define RANGE_MEDIUM_LARGE     74000
#define RANGE_MEDIUM_SMALL     60000
#define RANGE_SMALL            50000

/* Input Capacitance Range */
#define REG_GAIN_1X  0x00
#define REG_GAIN_2X  0x01
#define REG_GAIN_4X  0x02
#define REG_GAIN_8X  0x03

#define GAIN_1X      1
#define GAIN_2X      2
#define GAIN_4X      4
#define GAIN_8X      8

#endif /* _SX9500_I2C_REG_H_*/
