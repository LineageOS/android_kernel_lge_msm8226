/*! \file sx9500.c
 * \brief  SX9500 Driver
 *
 * Driver for the SX9500 
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

//#define DEBUG /* for dev_dbg function */

#define DRIVER_NAME "sx9500"

#define MAX_WRITE_ARRAY_SIZE 32
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>

#include <sx86xx.h> /* main struct, interrupt,init,pointers */
#include <sx9500_i2c_reg.h>
#include <sx9500_platform_data.h> /* platform data */
//#include <sx9500-specifics.h>

#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/sort.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif

#define ACTIVE              0
#define IDLE                1
#define PROX_STATUS_NEAR    0 /*ACTIVE*/
#define PROX_STATUS_FAR     1 /*IDLE*/

#define ENABLE_IRQ_MASK     0x10
#define DISABLE_IRQ_MASK    0x20
#define ENABLE_SENSOR_PINS  0x01
#define DISABLE_SENSOR_PINS 0x02

/* IO Used for NIRQ */
#define GPIO_SX9500_NIRQ    54//114

#define CS2_DISABLE         0x00
#define CS2_ENABLE          0x05

/* For Calibration & Startup function */
#define ON_SENSOR           1
#define OFF_SENSOR          2
#define COLLECT_NUM         20
#define FILTER_NUM          2
#define PATH_CAPSENSOR_CAL  "/sns/capsensor_cal.dat"
#define PATH_CAPSENSOR_CAL2 "/sns/capsensor_cal2.dat"
#define PATH_CAL_BACKUP     "/sns/capsensor_cal_backup.dat"
#define SCANPERIOD_CAL      0x0
#define PATH_XO_THERM       "/sys/class/hwmon/hwmon0/device/xo_therm_pu2"

#ifdef CONFIG_OF
enum sensor_dt_entry_type {
    DT_U8,
    DT_U8_ARRAY,
    DT_U32,
    DT_GPIO,
};

struct sensor_dt_to_platformdata {
    const char                  *dt_name;
    void                        *ptr_data;
    enum sensor_dt_entry_type   type;
};
#endif

struct smtc_cal_data {
    s32 Calculate_CSx;
    s16 Useful_CSx;
    u16 Offset_CSx; /* fullbyte */
};

static struct _buttoninfo psmtcButtons[] = {
    { .keycode = KEY_0,    .mask = SX9500_TCHCMPSTAT_TCHSTAT0_FLAG, },
    { .keycode = KEY_1,    .mask = SX9500_TCHCMPSTAT_TCHSTAT1_FLAG, },
    { .keycode = KEY_2,    .mask = SX9500_TCHCMPSTAT_TCHSTAT2_FLAG, },
    { .keycode = KEY_3,    .mask = SX9500_TCHCMPSTAT_TCHSTAT3_FLAG, },
};

static bool running_cal_or_reset = false;
static bool skip_startup = false;
static bool check_allnear = false;
static bool on_sensor = false;

typedef int (*compfn)(const void*, const void*);

static int initialize_device(struct sx86XX *this);

/*! \fn static int write_register(struct sx86XX *this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct 
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(struct sx86XX *this, u8 address, u8 value)
{
    struct i2c_client *i2c = NULL;
    /*char*/u8 buffer[2];
    int returnValue = 0;

    buffer[0] = address;
    buffer[1] = value;
    returnValue = -ENOMEM;
    if (this && this->bus) {
        i2c = this->bus;

        returnValue = i2c_master_send(i2c, buffer, 2);
        dev_dbg(&i2c->dev,
                "write reg Add=0x%02x Val=0x%02x Ret=%d\n",
                address, value, returnValue);
    }

    return returnValue;
}

/*! \fn static int read_register(struct sx86XX *this, u8 address, u8 *value) 
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct 
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to 
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(struct sx86XX *this, u8 address, u8 *value)
{
    struct i2c_client *i2c = NULL;
    s32 returnValue = 0;

    if (this && value && this->bus) {
        i2c = this->bus;
        returnValue = i2c_smbus_read_byte_data(i2c, address);

        if (address != SX9500_IRQSTAT_REG) //temp 2014.03.24 gooni.shim
            dev_dbg(&i2c->dev, "read reg Add=0x%02x Ret=0x%02x\n", address, returnValue);

        if (returnValue >= 0) {
            *value = returnValue;
            return 0;
        } 
        else {
            return returnValue;
        }
    }

    return -ENOMEM;
}

static void onoff_sensor(struct sx86XX *this, int onoff_mode)
{
    struct sx9500 *pDevice = NULL;
    struct sx9500_platform_data *pdata = NULL;

    unsigned char val_regproxctrl0;
    unsigned char val_regirqmask;

    int nparse_mode;
    int i = 0;

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
        while ( i < pdata->i2c_reg_num) {
            if (pdata->pi2c_reg[i].reg == SX9500_IRQ_ENABLE_REG)
                val_regirqmask = pdata->pi2c_reg[i].val;
            else if (pdata->pi2c_reg[i].reg == SX9500_CPS_CTRL0_REG)
                val_regproxctrl0 = pdata->pi2c_reg[i].val;
            i++;
        }

        nparse_mode = onoff_mode & ENABLE_SENSOR_PINS;
        dev_dbg(this->pdev, "nparse_mode=0x%02x, on_sensor=%d\n", nparse_mode, (int)on_sensor);
        if (nparse_mode == ENABLE_SENSOR_PINS) {
            write_register(this, SX9500_CPS_CTRL0_REG, val_regproxctrl0);

            if (!on_sensor)
                enable_irq_wake(this->irq);

            on_sensor = true;
        }

        nparse_mode = onoff_mode & DISABLE_SENSOR_PINS;
        if (nparse_mode == DISABLE_SENSOR_PINS) {
            for (i = 0; i < pDevice->pButtonInformation->buttonSize; i++) {
                pDevice->pButtonInformation->buttons[i].state = IDLE;
            }

            // Clear flags because sensor is disabled.
            this->inStartupTouch = false;
            running_cal_or_reset = false;
            check_allnear = false;

            write_register(this, SX9500_CPS_CTRL0_REG, ((val_regproxctrl0 >> 4) << 4) | 0x00);

            if (on_sensor)
                disable_irq_wake(this->irq);

            on_sensor = false;
        }

        nparse_mode = onoff_mode & ENABLE_IRQ_MASK;
        if (nparse_mode == ENABLE_IRQ_MASK)
            write_register(this, SX9500_IRQ_ENABLE_REG, val_regirqmask);

        nparse_mode = onoff_mode & DISABLE_IRQ_MASK;
        if (nparse_mode == DISABLE_IRQ_MASK)
            write_register(this, SX9500_IRQ_ENABLE_REG, 0x00);

        msleep(100); /* make sure everything is running */

    }
}

static bool valid_multiple_input_pins(struct sx86XX *this)
{
    struct sx9500 *pDevice = NULL;
    struct sx9500_platform_data *pdata = NULL;

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
        if (pdata->input_pins_num > 1) {
            if ((pdata->input_main2sensor >= SENSORSEL_CS0) && 
                (pdata->input_main2sensor <= SENSORSEL_CS3) &&
                (pdata->input_main2sensor != pdata->input_refsensor)) {
                return true;
            }
        }
    }

    return false;
}

static int write_calibration_data(struct sx86XX *this,
                                s32 nMargin, u16 Offset_CSx, s32 TotalCap_CSx,
                                bool isMultiInput)
{
    int fd;
    int ret = 0;
    char buf[50];
    char path_cal_file[50];
    mm_segment_t old_fs = get_fs();

    memset(buf, 0, sizeof(buf));
    memset(path_cal_file, 0, sizeof(path_cal_file));

    if (isMultiInput)
        strcpy(path_cal_file, PATH_CAPSENSOR_CAL2);
    else
        strcpy(path_cal_file, PATH_CAPSENSOR_CAL);

    sprintf(buf, "%d %d %d", nMargin, Offset_CSx, TotalCap_CSx);

    dev_info(this->pdev, "buf = %s\n", buf);

    set_fs(KERNEL_DS);
    fd = sys_open(path_cal_file, O_WRONLY|O_CREAT, 0664);

    if (fd >= 0) {
        sys_write(fd, buf, sizeof(buf));
        sys_write(fd, "\n", 1);
        sys_fsync(fd); //ensure calibration data write to file system 
        sys_close(fd);
        sys_chmod(path_cal_file, 0664);
        set_fs(old_fs);
    }
    else {
        ret++;
        sys_close(fd);
        set_fs(old_fs);
        return ret;
    }

    return ret;
}

static int write_calibration_backup_data(struct sx86XX *this,
                                s32 TotalCap_CSx, s16 Useful_CSx, u16 Offset_CSx)
{
    int fd;
    int ret = 0;
    char buf[30];
    char path_cal_backup_file[50];
    mm_segment_t old_fs = get_fs();

    memset(buf, 0, sizeof(buf));
    memset(path_cal_backup_file, 0, sizeof(path_cal_backup_file));

    strcpy(path_cal_backup_file, PATH_CAL_BACKUP);

    sprintf(buf, "%d %d %d", TotalCap_CSx, Useful_CSx, Offset_CSx);

    dev_info(this->pdev, "buf = %s\n", buf);

    set_fs(KERNEL_DS);
    fd = sys_open(path_cal_backup_file, O_WRONLY|O_CREAT|O_APPEND, 0664);

    if (fd >= 0) {
        sys_write(fd, buf, sizeof(buf));
        sys_write(fd, "\n", 1);
        sys_fsync(fd); //ensure calibration backup data write to file system 
        sys_close(fd);
        sys_chmod(path_cal_backup_file, 0664);
        set_fs(old_fs);

        
    }
    else {
        ret++;
        sys_close(fd);
        set_fs(old_fs);
        return ret;
    }

    return ret;
}

static void read_calibration_data(struct sx86XX *this,
                                s32 *nMargin, u16 *Offset_CSx, s32 *TotalCap_CSx,
                                bool isMultiInput)
{
    int fd;
    //int ret = 0;
    int len = 0;
    char read_buf[50];
    char path_cal_file[50];
    mm_segment_t old_fs = get_fs();

    s32 nCalMargin = -1;
    s32 unOffset_CSx = 0;
    s32 nTotalCap_CSx = 0;

    *nMargin = (s32) nCalMargin;
    *Offset_CSx = (u16) unOffset_CSx;
    *TotalCap_CSx = (s32) nTotalCap_CSx;

    memset(read_buf, 0, sizeof(read_buf));
    memset(path_cal_file, 0, sizeof(path_cal_file));

    if (isMultiInput)
        strcpy(path_cal_file, PATH_CAPSENSOR_CAL2);
    else
        strcpy(path_cal_file, PATH_CAPSENSOR_CAL);

    set_fs(KERNEL_DS);

    fd = sys_open(path_cal_file, O_RDONLY, 0);
    if (fd >= 0) {
        len = sys_read(fd, read_buf, sizeof(read_buf));
        dev_dbg(this->pdev, "cap sensor calibration file size is = %d\n", len);
        if (len <= 0) {
            //ret = -1;
            sys_close(fd);
            set_fs(old_fs);
            //return ret;
            return;
        }

        sscanf(read_buf, "%d %d %d", &nCalMargin, &unOffset_CSx, &nTotalCap_CSx);

        *nMargin = (s32) nCalMargin;
        *Offset_CSx = (u16) unOffset_CSx;
        *TotalCap_CSx = (s32) nTotalCap_CSx;

        sys_close(fd);
        set_fs(old_fs);
    }
    else {
        dev_err(this->pdev, "Read cap sensor cal data. Error[%d]!!!\n",  fd);
        //ret = -1;
        sys_close(fd);
        set_fs(old_fs);
        //return ret;
        return;
    }
}

static void read_xo_therm_data(struct sx86XX *this, int *xo_therm)
{
    int fd;
    //int ret = 0;
    int len = 0;
    char read_buf[32];
    mm_segment_t old_fs = get_fs();

    int nresult = 0;
    int nraw = 0;

    *xo_therm = (int) nresult;

    memset(read_buf, 0, sizeof(read_buf));
    set_fs(KERNEL_DS);

    fd = sys_open(PATH_XO_THERM, O_RDONLY, 0);
    if (fd >= 0) {
        len = sys_read(fd, read_buf, sizeof(read_buf));
        dev_dbg(this->pdev, "current xo_thermal data file size is = %d\n", len);
        if (len <= 0) {
            //ret = -1;
            sys_close(fd);
            set_fs(old_fs);
            //return ret;
            return;
        }

        sscanf(read_buf, "Result:%d Raw:%d\n", &nresult, &nraw);

        *xo_therm = (int) nresult;

        sys_close(fd);
        set_fs(old_fs);
    }
    else {
        dev_err(this->pdev, "Read xo_therm data. Error[%d]!!!\n",  fd);
        //ret = -1;
        sys_close(fd);
        set_fs(old_fs);
        //return ret;
        return;
    }
}

static void read_sensor_regdata(struct sx86XX *this, unsigned char nSensorSel,
                             s16 *pUseful_CSx, s16 *pAvg_CSx, s16 *pDiff_CSx, u16 *pOffset_CSx)
{
    u8 msByte = 0;
    u8 lsByte = 0;

    s16 Useful_CSx = 0;
    s16 Avg_CSx = 0;
    s16 Diff_CSx = 0;
    u16 Offset_CSx = 0;

    if (this) {
        write_register(this, SX9500_SENSORSEL_REG, nSensorSel);

        read_register(this, SX9500_USEMSB_REG, &msByte);
        read_register(this, SX9500_USELSB_REG, &lsByte);
        Useful_CSx = (s16)((msByte << 8) | lsByte);

        msByte = 0; lsByte = 0;
        read_register(this, SX9500_AVGMSB_REG, &msByte);
        read_register(this, SX9500_AVGLSB_REG, &lsByte);
        Avg_CSx = (s16)((msByte << 8) | lsByte);

        msByte = 0; lsByte = 0;
        read_register(this, SX9500_DIFFMSB_REG, &msByte);
        read_register(this, SX9500_DIFFLSB_REG, &lsByte);
        Diff_CSx = (s16)((msByte << 8) | lsByte);
        if (Diff_CSx > 4095)
            Diff_CSx -= 8192;

        msByte = 0; lsByte = 0;
        read_register(this, SX9500_OFFSETMSB_REG, &msByte);
        read_register(this, SX9500_OFFSETLSB_REG, &lsByte);
        Offset_CSx = (u16)((msByte << 8) | lsByte);

        *pUseful_CSx = (s16) Useful_CSx;
        *pAvg_CSx = (s16) Avg_CSx;
        *pDiff_CSx = (s16) Diff_CSx;
        *pOffset_CSx = (u16) Offset_CSx;
    }

    return;
}

static void calculate_CSx_rawdata(struct sx86XX *this, unsigned char channel_num,
                                s16 *pUseful_CSx, u16 *pOffset_CSx, s32 *pCalculate_CSx)
{
    struct sx9500 *pDevice = NULL;
    struct sx9500_platform_data *pdata = NULL;

    u8 msByte = 0;
    u8 lsByte = 0;

    s16 Useful_CSx = 0;
    u16 Offset_CSx = 0; /* fullbyte */
    s32 Calculate_CSx = 0;

    s32 ncapacitance_range = RANGE_MEDIUM_SMALL;
    s32 ngain_factor = GAIN_8X;

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
        ncapacitance_range = pdata->capacitance_range;
        ngain_factor = pdata->gain_factor;
    }

    // Calculate out the CSx Cap information //
    write_register(this, SX9500_SENSORSEL_REG, channel_num);
    read_register(this, SX9500_USEMSB_REG, &msByte);
    read_register(this, SX9500_USELSB_REG, &lsByte);
    Useful_CSx = (s16) ((msByte << 8) | lsByte);

    msByte = 0;  lsByte = 0;
    read_register(this, SX9500_OFFSETMSB_REG, &msByte);
    read_register(this, SX9500_OFFSETLSB_REG, &lsByte);
    Offset_CSx = (u16)((msByte << 8) | lsByte);

    msByte = 0; lsByte = 0;
    msByte = (u8)(Offset_CSx >> 6);
    lsByte = (u8)(Offset_CSx - (((u16) msByte) << 6));

    // Calculate total capacitance.
    // Use offset value, capacitance range, gain_factor.
    Calculate_CSx = 2 * (((s32) msByte * 3600) + ((s32) lsByte * 225)) +
                            (((s32) Useful_CSx * ncapacitance_range) / (ngain_factor * 65536));

    dev_info(this->pdev, "CS[%02x] Useful = %6d, Offset = %6d, Calculate = %6d\n", 
                            channel_num, Useful_CSx, Offset_CSx, Calculate_CSx);

    if (pUseful_CSx)
        *pUseful_CSx = (s16) Useful_CSx;

    if (pOffset_CSx)
        *pOffset_CSx = (u16) Offset_CSx;

    if (pCalculate_CSx)
        *pCalculate_CSx = (s32) Calculate_CSx;

    return;
}

/***********************************************************/
/*! \brief Perform a manual offset calibration
* \param this Pointer to main parent struct 
* \return Value return value from the write register
 */
static int manual_offset_calibration(struct sx86XX *this)
{
    s32 returnValue = 0;

    returnValue = write_register(this, SX9500_IRQSTAT_REG, 0xFF);

    return returnValue;
}

/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    dev_dbg(this->pdev, "Reading IRQSTAT_REG\n");
    read_register(this, SX9500_IRQSTAT_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

/*! \brief sysfs store function for manual calibration
 */
static ssize_t manual_offset_calibration_store(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    dev_dbg( this->pdev, "Performing manual_offset_calibration()\n");
    manual_offset_calibration(this);

    return count;
}

static u32 conv_regval_to_range(unsigned char regval)
{
    unsigned char val_1bit = (((1 << 1) & regval) >> 1);
    unsigned char val_0bit = (((1 << 0) & regval) >> 0);
    unsigned char val_rangebit = ((val_1bit << 1) | val_0bit);

    u32 ncapacitancerange = RANGE_LARGE;

    switch (val_rangebit) {
        case REG_RANGE_LARGE:        ncapacitancerange = RANGE_LARGE;        break;
        case REG_RANGE_MEDIUM_LARGE: ncapacitancerange = RANGE_MEDIUM_LARGE; break;
        case REG_RANGE_MEDIUM_SMALL: ncapacitancerange = RANGE_MEDIUM_SMALL; break;
        case REG_RANGE_SMALL:        ncapacitancerange = RANGE_SMALL;        break;
    }

    return ncapacitancerange;
}

static u32 conv_regval_to_gain(unsigned char regval)
{
    unsigned char val_6bit = (((1 << 6) & regval) >> 6);
    unsigned char val_5bit = (((1 << 5) & regval) >> 5);
    unsigned char val_gainbit = ((val_6bit << 1) | val_5bit);

    u32 ngainfactor = GAIN_1X;

    switch (val_gainbit) {
        case REG_GAIN_1X: ngainfactor = GAIN_1X; break;
        case REG_GAIN_2X: ngainfactor = GAIN_2X; break;
        case REG_GAIN_4X: ngainfactor = GAIN_4X; break;
        case REG_GAIN_8X: ngainfactor = GAIN_8X; break;
    }

    return ngainfactor;
}

static unsigned char conv_sensorsel_to_sensoren(unsigned char csx_pin)
{
    unsigned char sensoren_csx;

    switch (csx_pin) {
        case SENSORSEL_CS0: sensoren_csx = SENSOREN_CS0; break;
        case SENSORSEL_CS1: sensoren_csx = SENSOREN_CS1; break;
        case SENSORSEL_CS2: sensoren_csx = SENSOREN_CS2; break;
        case SENSORSEL_CS3: sensoren_csx = SENSOREN_CS3; break;
    }

    return sensoren_csx;
}

static int cmp_rawdata(struct smtc_cal_data *elem1, struct smtc_cal_data *elem2)
{
    // Ascending sort by Capacitance value.
    if (elem1->Calculate_CSx > elem2->Calculate_CSx)
        return 1;
    else if (elem1->Calculate_CSx < elem2->Calculate_CSx)
        return -1;
    else
        return 0;
}

static ssize_t sx9500_show_skipstartup(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", (int) skip_startup);
}

static ssize_t sx9500_store_skipstartup(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    dev_dbg(this->pdev,"skipstartup set val = %d\n", (int) val);
    if (val == 0)
        skip_startup = false;
    else if (val == 1)
        skip_startup = true;

    return count;
}

static ssize_t sx9500_store_onoffsensor(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    dev_info(this->pdev,"request onoff val = %d\n", (int) val);
    if (val == ON_SENSOR)
        onoff_sensor(this, ENABLE_IRQ_MASK | ENABLE_SENSOR_PINS);
    else if (val == OFF_SENSOR)
        onoff_sensor(this, DISABLE_IRQ_MASK | DISABLE_SENSOR_PINS);

    return count;
}

static ssize_t sx9500_store_docalibration(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    struct sx9500 *pDevice = NULL;

    unsigned char cal_scanperiod = SCANPERIOD_CAL;// Set SCANPERIOD[6:4] - 000(30ms).
    unsigned char write_RegProxCtrl0 = 0x00;

    unsigned char mainSensor = 0;
    unsigned char main2Sensor = 0;
    unsigned char refSensor = 0;

    struct smtc_cal_data capMain_cal_data[COLLECT_NUM];
    struct smtc_cal_data capMain2_cal_data[COLLECT_NUM];
    struct smtc_cal_data capRef_cal_data[COLLECT_NUM];

    s32 avg_Capacitance_Main = 0;
    s32 avg_Useful_Main = 0;
    s32 avg_Offset_Main = 0;

    s32 avg_Capacitance_Main2 = 0;
    s32 avg_Useful_Main2 = 0;
    s32 avg_Offset_Main2 = 0;

    s32 avg_Capacitance_Ref = 0;
    s32 avg_Useful_Ref = 0;
    s32 avg_Offset_Ref = 0;
    s32 avg_Capacitance_Ref_dynamicthreshold = 0;

    s32 nhysteresis = 0;

    s32 capMargin = 0;
    s32 capMargin2 = 0;

    bool is_different_offset = false;
    bool is_over_gap_useful = false;
    bool is_over_useful = false;

    int ret = 0;

    if (this && (pDevice = this->pDevice) && !running_cal_or_reset) {
        int i = 0;

        u8 old_RegIrqMsk_val = 0;
        u8 old_RegCtrl0_val = 0;

        unsigned char conv_mainSensor = 0;
        unsigned char conv_main2Sensor = 0;
        unsigned char conv_refSensor = 0;

        running_cal_or_reset = true;

        dev_info(this->pdev, "calibration start!!!\n");

        mainSensor = (unsigned char)pDevice->hw->input_mainsensor;
        if (valid_multiple_input_pins(this))
            main2Sensor = (unsigned char)pDevice->hw->input_main2sensor;
        refSensor = (unsigned char)pDevice->hw->input_refsensor;

        conv_mainSensor = conv_sensorsel_to_sensoren(mainSensor);
        if (valid_multiple_input_pins(this))
            conv_main2Sensor = conv_sensorsel_to_sensoren(main2Sensor);
        conv_refSensor = conv_sensorsel_to_sensoren(refSensor);

        write_RegProxCtrl0 = (unsigned char) 
                ((cal_scanperiod << 4) | (conv_mainSensor | conv_main2Sensor | conv_refSensor));

        // Backup RegIrqMask & RegProxCtrl0 value.
        // And, Change RegIrqMask & RegProxCtrl0(scanperiod) value for calibration.
        read_register(this, SX9500_IRQ_ENABLE_REG, &old_RegIrqMsk_val);
        read_register(this, SX9500_CPS_CTRL0_REG, &old_RegCtrl0_val);

        write_register(this, SX9500_IRQ_ENABLE_REG, 0x70);
        write_register(this, SX9500_CPS_CTRL0_REG, write_RegProxCtrl0);

        msleep(100); /* make sure everything is running */

        manual_offset_calibration(this);

        // This should cover most of the scan periods, if extremely large may
        // want to set this to 500 
        msleep(500); /* make sure manual offset has been fully done */

        // Clear flags because sensor is calibrationing.
        this->inStartupTouch = false;
        check_allnear = false;

        // Calculate out the Cap information for Main/(Main2/)Ref Pins.
        for (i = 0; i < COLLECT_NUM; i++) {
            calculate_CSx_rawdata(this, mainSensor,
                                &capMain_cal_data[i].Useful_CSx,
                                &capMain_cal_data[i].Offset_CSx,
                                &capMain_cal_data[i].Calculate_CSx);

            if (valid_multiple_input_pins(this)) {
                calculate_CSx_rawdata(this, main2Sensor,
                                    &capMain2_cal_data[i].Useful_CSx,
                                    &capMain2_cal_data[i].Offset_CSx,
                                    &capMain2_cal_data[i].Calculate_CSx);
            }

            calculate_CSx_rawdata(this, refSensor,
                                &capRef_cal_data[i].Useful_CSx,
                                &capRef_cal_data[i].Offset_CSx,
                                &capRef_cal_data[i].Calculate_CSx);

            msleep(50);
        }

        // To sort the collected value. 
        sort((void *) &capMain_cal_data, COLLECT_NUM, sizeof(struct smtc_cal_data),
                (compfn) cmp_rawdata, NULL);
        if (valid_multiple_input_pins(this)) {
            sort((void *) &capMain2_cal_data, COLLECT_NUM, sizeof(struct smtc_cal_data),
                    (compfn) cmp_rawdata, NULL);
        }
        sort((void *) &capRef_cal_data, COLLECT_NUM, sizeof(struct smtc_cal_data),
                (compfn) cmp_rawdata, NULL);

        // Check readback sensor raw data.
        // 1. Check Offset value.(is equal values?)
        // 2. Check Useful value.(is over min/max gap? is over init range?)
        for (i = 0; i < COLLECT_NUM; i++) {
            if (capMain_cal_data[i].Offset_CSx != capMain_cal_data[COLLECT_NUM/2].Offset_CSx) {
                is_different_offset = true;
                dev_info(this->pdev, "Calibration Fail! different(offset)! [%d] %d\n",
                            i, capMain_cal_data[i].Offset_CSx);
            }
        }

        if (is_different_offset == false) {
            if ((capMain_cal_data[COLLECT_NUM-1].Useful_CSx - capMain_cal_data[0].Useful_CSx) > 2000) {
                is_over_gap_useful = true;
                dev_info(this->pdev, "Calibration Fail! over gap(useful)! [gap = %d(%d - %d)]\n", 
                        ((capMain_cal_data[COLLECT_NUM-1].Useful_CSx) - (capMain_cal_data[0].Useful_CSx)),
                        capMain_cal_data[COLLECT_NUM-1].Useful_CSx, capMain_cal_data[0].Useful_CSx);
            }

            for (i = 0; i < COLLECT_NUM; i++) {
                if ((capMain_cal_data[i].Useful_CSx < -8000) || (capMain_cal_data[i].Useful_CSx > 8000)) {
                    is_over_useful = true;
                    dev_info(this->pdev, "Calibration Fail! over range(useful)! [%d] %d\n",
                                i, capMain_cal_data[i].Useful_CSx);
                }
            }
        }

        if (is_different_offset || is_over_gap_useful || is_over_useful) {
            ret = write_calibration_data(this, 0, 0, 0, false);
            //if (valid_multiple_input_pins(this))
            //    ret = write_calibration_data(this, 0, 0, 0, true);
            running_cal_or_reset = false;

            return count;
        }

        // Calculate the average capacitance value about the Main/(Main2/)Ref Pins.
        // 2 max & 2 min values are excluded.
        for (i = FILTER_NUM; i < COLLECT_NUM - FILTER_NUM; i++) {
            avg_Capacitance_Main += capMain_cal_data[i].Calculate_CSx;
            avg_Useful_Main += capMain_cal_data[i].Useful_CSx;
            avg_Offset_Main += capMain_cal_data[i].Offset_CSx;
        }
        avg_Capacitance_Main = avg_Capacitance_Main / (COLLECT_NUM - (FILTER_NUM * 2));
        avg_Useful_Main = avg_Useful_Main / (COLLECT_NUM - (FILTER_NUM * 2));
        avg_Offset_Main = avg_Offset_Main / (COLLECT_NUM - (FILTER_NUM * 2));
        dev_info(this->pdev, 
                    "avg_Capacitance_Main = %d, avg_Useful_Main = %d, avg_Offset_Main = %d\n", 
                    avg_Capacitance_Main, avg_Useful_Main, avg_Offset_Main);

        if (valid_multiple_input_pins(this)) {
            for (i = FILTER_NUM; i < COLLECT_NUM - FILTER_NUM; i++) {
                avg_Capacitance_Main2 += capMain2_cal_data[i].Calculate_CSx;
                avg_Useful_Main2 += capMain2_cal_data[i].Useful_CSx;
                avg_Offset_Main2 += capMain2_cal_data[i].Offset_CSx;
            }
            avg_Capacitance_Main2 = avg_Capacitance_Main2 / (COLLECT_NUM - (FILTER_NUM * 2));
            avg_Useful_Main2 = avg_Useful_Main2 / (COLLECT_NUM - (FILTER_NUM * 2));
            avg_Offset_Main2 = avg_Offset_Main2 / (COLLECT_NUM - (FILTER_NUM * 2));
            dev_info(this->pdev,
                    "avg_Capacitance_Main2 = %d, avg_Useful_Main2 = %d, avg_Offset_Main2 = %d\n",
                    avg_Capacitance_Main2, avg_Useful_Main2, avg_Offset_Main2);
        }

        for (i = FILTER_NUM; i < COLLECT_NUM - FILTER_NUM; i++) {
            avg_Capacitance_Ref += capRef_cal_data[i].Calculate_CSx;
            avg_Useful_Ref += capRef_cal_data[i].Useful_CSx;
            avg_Offset_Ref += capRef_cal_data[i].Offset_CSx;
        }
        avg_Capacitance_Ref = avg_Capacitance_Ref / (COLLECT_NUM - (FILTER_NUM * 2));
        avg_Useful_Ref = avg_Useful_Ref / (COLLECT_NUM - (FILTER_NUM * 2));
        avg_Offset_Ref = avg_Offset_Ref / (COLLECT_NUM - (FILTER_NUM * 2));
        dev_info(this->pdev, 
                    "avg_Capacitance_Ref = %d, avg_Useful_Ref = %d, avg_Offset_Ref = %d\n",
                    avg_Capacitance_Ref, avg_Useful_Ref, avg_Offset_Ref);

        // Calculate the dynamic thershold through the ref sensor.
        avg_Capacitance_Ref_dynamicthreshold = pDevice->pStartupCheckParameters->dynamicthreshold_offset + 
                ((pDevice->pStartupCheckParameters->dynamicthreshold_temp_slope * avg_Capacitance_Ref) / 10);

        // Calculate capacitance margin value.
        // And, Save margin & offset values.
        nhysteresis = pDevice->pStartupCheckParameters->dynamicthreshold_hysteresis;
        capMargin = avg_Capacitance_Main - avg_Capacitance_Ref_dynamicthreshold + nhysteresis;
        ret = write_calibration_data(this,
                                    capMargin,
                                    capMain_cal_data[COLLECT_NUM/2].Offset_CSx,
                                    avg_Capacitance_Main,
                                    false);

        if (valid_multiple_input_pins(this)) {
            capMargin2 = avg_Capacitance_Main2 - avg_Capacitance_Ref_dynamicthreshold + nhysteresis;
            ret = write_calibration_data(this,
                                        capMargin2,
                                        capMain2_cal_data[COLLECT_NUM/2].Offset_CSx,
                                        avg_Capacitance_Main2,
                                        true);
        }

        write_calibration_backup_data(this, 
                            avg_Capacitance_Main, (s16) avg_Useful_Main, (u16) avg_Offset_Main);
        if (valid_multiple_input_pins(this)) {
            write_calibration_backup_data(this, 
                            avg_Capacitance_Main2, (s16)avg_Useful_Main2, (u16) avg_Offset_Main2);
        }
        write_calibration_backup_data(this,
                            avg_Capacitance_Ref, (s16) avg_Useful_Ref, (u16) avg_Offset_Ref);

        // Restore RegIrqMask & RegProxCtrl0(scanperiod) value.
        write_register(this, SX9500_IRQ_ENABLE_REG, old_RegIrqMsk_val);
        write_register(this, SX9500_CPS_CTRL0_REG, old_RegCtrl0_val);

        // Must be sure to set the Main CS pin.
        write_register(this, SX9500_SENSORSEL_REG, mainSensor);

        dev_info(this->pdev, "calibration end!!!\n");

        running_cal_or_reset = false;

    }

    return count;
}

static ssize_t sx9500_store_checkallnear(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    dev_info(this->pdev,"checkallnear set val = %d\n", (int) val);
    if (val == 0)
        check_allnear = false;
    else if (val == 1)
        check_allnear = true;

    return count;
}

static ssize_t sx9500_show_count_inputpins(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    int count_inputpins = 0;

    struct sx86XX *this = dev_get_drvdata(dev);
    struct sx9500 *pDevice = NULL;
    struct sx9500_platform_data *pdata = NULL;

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
        count_inputpins = pdata->input_pins_num;
        if (count_inputpins > 1) {
            if (valid_multiple_input_pins(this) == false)
                count_inputpins = 1;
        }
    }

    dev_info(this->pdev,"count input pins = %d\n", count_inputpins);

    return sprintf(buf, "%d\n", count_inputpins);
}

static ssize_t sx9500_show_proxstatus(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    struct sx9500 *pDevice = NULL;
    struct sx86XX *this = dev_get_drvdata(dev);
    struct _buttoninfo *buttons = NULL;
    u8 mainSensor, main2Sensor;

    int prox_status = -1;

    if (this && (pDevice = this->pDevice)) {
        buttons = pDevice->pButtonInformation->buttons;
        mainSensor = pDevice->hw->input_mainsensor;

        if (valid_multiple_input_pins(this)) {
            main2Sensor = pDevice->hw->input_main2sensor;
            if (check_allnear) {
                if ((buttons[mainSensor].state == IDLE) || (buttons[main2Sensor].state == IDLE))
                   prox_status = IDLE;

                if ((buttons[mainSensor].state == ACTIVE) && (buttons[main2Sensor].state == ACTIVE))
                    prox_status = ACTIVE;
            }
            else {
                if ((buttons[mainSensor].state == IDLE) && (buttons[main2Sensor].state == IDLE))
                    prox_status = IDLE;

                if ((buttons[mainSensor].state == ACTIVE) || (buttons[main2Sensor].state == ACTIVE))
                    prox_status = ACTIVE;
            }
        }
        else {
            prox_status = buttons[mainSensor].state;
        }
    }

    return sprintf(buf, "%d\n", prox_status);
}

static ssize_t sx9500_show_regproxdata(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    s16 main_Useful  = 0, main_Avg  = 0, main_Diff  = 0; u16 main_Offset  = 0;
    s16 main2_Useful = 0, main2_Avg = 0, main2_Diff = 0; u16 main2_Offset = 0;
    s16 ref_Useful   = 0, ref_Avg   = 0, ref_Diff   = 0; u16 ref_Offset   = 0;

    u8 mainSensor, main2Sensor, refSensor;

    int current_xo_therm = 0;

    char buf_bstate_main[5] = "";
    char buf_bstate_main2[5] = "";
    char buf_line[64] = "";
    char buf_regproxdata[256] = "";
    int nlength = 0;
    struct sx9500 *pDevice = NULL;
    struct sx86XX *this = dev_get_drvdata(dev);
    struct _buttoninfo *buttons = NULL;

    memset(buf_bstate_main, 0, sizeof(buf_bstate_main));
    memset(buf_bstate_main2, 0, sizeof(buf_bstate_main2));
    memset(buf_line, 0, sizeof(buf_line));
    memset(buf_regproxdata, 0, sizeof(buf_regproxdata));
    if (this && (pDevice = this->pDevice) && !running_cal_or_reset) {
        buttons = pDevice->pButtonInformation->buttons;

        // Select Main Sensor, Readback Reg Sensor Data
        mainSensor = pDevice->hw->input_mainsensor;
        read_sensor_regdata(this, mainSensor, &main_Useful, &main_Avg, &main_Diff, &main_Offset);
        (buttons[mainSensor].state) ? strcpy(buf_bstate_main, "F"):strcpy(buf_bstate_main, "N");
        dev_info(this->pdev,"capMain = %6d %6d %6d %6d %s\n",
                            main_Useful, main_Avg, main_Diff, main_Offset, buf_bstate_main);

        // Select Main2 Sensor, Readback Reg Sensor Data
        if (valid_multiple_input_pins(this)) {
            main2Sensor = pDevice->hw->input_main2sensor;
            read_sensor_regdata(this, main2Sensor, &main2_Useful, &main2_Avg, &main2_Diff, &main2_Offset);
            (buttons[main2Sensor].state) ? strcpy(buf_bstate_main2, "F"):strcpy(buf_bstate_main2, "N");
            dev_info(this->pdev,"capMain2 = %6d %6d %6d %6d %s\n",
                            main2_Useful, main2_Avg, main2_Diff, main2_Offset, buf_bstate_main2);
        }

        // Select Reference Sensor, Readback Reg Sensor Data
        refSensor = pDevice->hw->input_refsensor;
        read_sensor_regdata(this, refSensor, &ref_Useful, &ref_Avg, &ref_Diff, &ref_Offset);
        dev_info(this->pdev,"capRef = %6d %6d %6d %6d\n", ref_Useful, ref_Avg, ref_Diff, ref_Offset);

        // Please Select Main Sensor again.
        write_register(this, SX9500_SENSORSEL_REG, mainSensor);

        read_xo_therm_data(this, &current_xo_therm);
    }

    sprintf(buf_line, "[M]%6d %6d %6d %6d %s %3d\n", 
                main_Useful, main_Avg, main_Diff, main_Offset, buf_bstate_main, current_xo_therm);
    strcpy(buf_regproxdata, buf_line);

    if (this && valid_multiple_input_pins(this)) {
        sprintf(buf_line, "[S]%6d %6d %6d %6d %s\n",
                            main2_Useful, main2_Avg, main2_Diff, main2_Offset, buf_bstate_main2);
        nlength = strlen(buf_regproxdata);
        strcpy(&buf_regproxdata[nlength], buf_line);
    }

    sprintf(buf_line, "[R]%6d %6d %6d %6d\n", ref_Useful, ref_Avg, ref_Diff, ref_Offset);
    nlength = strlen(buf_regproxdata);
    strcpy(&buf_regproxdata[nlength], buf_line);

    return sprintf(buf, "%s", buf_regproxdata);
}

static ssize_t sx9500_show_regproxctrl0(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL0_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl0(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx9500 *pDevice = NULL;
    struct sx86XX *this = dev_get_drvdata(dev);

    struct input_dev *input = NULL;

    unsigned long val;
    unsigned long val_check;

    int i = 0;
    int ret = 0;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    ret = write_register(this, SX9500_CPS_CTRL0_REG, val);
    if (ret > 0) {
        val_check = val & SENSOREN_CS2;
        if (val_check == SENSOREN_DISABLE_ALL) {
            /* Initialize prox status : default FAR */
            if (this && (pDevice = this->pDevice)) {
                for (i = 0; i < pDevice->pButtonInformation->buttonSize; i++) {
                    pDevice->pButtonInformation->buttons[i].state = IDLE;
                }

                input = pDevice->pButtonInformation->input;
                input_report_abs(input, ABS_DISTANCE, PROX_STATUS_FAR);
                input_sync(input);
            }
        }
    }

    return count;
}

static ssize_t sx9500_show_regproxctrl1(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL1_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl1(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL1_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrl2(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL2_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl2(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL2_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrl3(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL3_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl3(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL3_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrl4(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL4_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl4(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL4_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrl5(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL5_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl5(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL5_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrl6(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL6_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl6(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL6_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrl7(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL7_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl7(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL7_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrl8(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL8_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl8(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL8_REG, val);

    return count;
}

static ssize_t sx9500_show_regirqmask(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_IRQ_ENABLE_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regirqmask(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_IRQ_ENABLE_REG, val);

    return count;
}

static ssize_t sx9500_store_regreset(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx9500 *pDevice = NULL;
    struct sx86XX *this = dev_get_drvdata(dev);
    struct input_dev *input = NULL;

    unsigned long val;
    int i = 0;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    if (this && (pDevice = this->pDevice) && !running_cal_or_reset) {
        running_cal_or_reset = true;

        if (val == SX9500_SOFTRESET) {
            dev_info(this->pdev, "reset start!!!\n");
            for (i = 0; i < pDevice->pButtonInformation->buttonSize; i++) {
                pDevice->pButtonInformation->buttons[i].state = IDLE;
            }

            input = pDevice->pButtonInformation->input;
            input_report_abs(input, ABS_DISTANCE, PROX_STATUS_FAR);
            input_sync(input);

            //sx86XX_suspend(this);
            //sx86XX_resume(this);
            //disable_irq(this->irq);

        #ifdef USE_THREADED_IRQ
            mutex_lock(&this->mutex);
            /* Just in case need to reset any uncaught interrupts */
            sx86XX_process_interrupt(this,0);
            mutex_unlock(&this->mutex);
        #else
            sx86XX_schedule_work(this,0);
        #endif
            initialize_device(this);

            //enable_irq(this->irq);

            dev_info(this->pdev, "reset end!!!\n");
        }

        running_cal_or_reset = false;
    }

    return count;
}

static ssize_t sx9500_store_regoffset(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);

    unsigned int msb_val;
    unsigned int lsb_val;

    if (sscanf(buf, "%02x,%02x", &msb_val, &lsb_val) != 2)
        return -EINVAL;

    dev_dbg(this->pdev, "regoffset = %02x, %02x\n", msb_val, lsb_val);

    write_register(this, SX9500_OFFSETMSB_REG, msb_val);
    write_register(this, SX9500_OFFSETLSB_REG, lsb_val);

    return count;
}

static DEVICE_ATTR(calibrate,    0664, manual_offset_calibration_show, manual_offset_calibration_store);
static DEVICE_ATTR(skipstartup,  0664, sx9500_show_skipstartup, sx9500_store_skipstartup);
static DEVICE_ATTR(onoff,        0664, NULL, sx9500_store_onoffsensor);
static DEVICE_ATTR(docalibration,0664, NULL, sx9500_store_docalibration);
static DEVICE_ATTR(checkallnear, 0664, NULL, sx9500_store_checkallnear);
static DEVICE_ATTR(cntinputpins, 0664, sx9500_show_count_inputpins, NULL);
static DEVICE_ATTR(proxstatus,   0664, sx9500_show_proxstatus, NULL);
static DEVICE_ATTR(regproxdata,  0664, sx9500_show_regproxdata, NULL);
static DEVICE_ATTR(regproxctrl0, 0664, sx9500_show_regproxctrl0, sx9500_store_regproxctrl0);
static DEVICE_ATTR(regproxctrl1, 0664, sx9500_show_regproxctrl1, sx9500_store_regproxctrl1);
static DEVICE_ATTR(regproxctrl2, 0664, sx9500_show_regproxctrl2, sx9500_store_regproxctrl2);
static DEVICE_ATTR(regproxctrl3, 0664, sx9500_show_regproxctrl3, sx9500_store_regproxctrl3);
static DEVICE_ATTR(regproxctrl4, 0664, sx9500_show_regproxctrl4, sx9500_store_regproxctrl4);
static DEVICE_ATTR(regproxctrl5, 0664, sx9500_show_regproxctrl5, sx9500_store_regproxctrl5);
static DEVICE_ATTR(regproxctrl6, 0664, sx9500_show_regproxctrl6, sx9500_store_regproxctrl6);
static DEVICE_ATTR(regproxctrl7, 0664, sx9500_show_regproxctrl7, sx9500_store_regproxctrl7);
static DEVICE_ATTR(regproxctrl8, 0664, sx9500_show_regproxctrl8, sx9500_store_regproxctrl8);
static DEVICE_ATTR(regirqmask,   0664, sx9500_show_regirqmask, sx9500_store_regirqmask);
static DEVICE_ATTR(regreset,     0664, NULL, sx9500_store_regreset);
static DEVICE_ATTR(regoffset,    0664, NULL, sx9500_store_regoffset);

static struct attribute *sx9500_attributes[] = {
    &dev_attr_calibrate.attr,
    &dev_attr_skipstartup.attr,
    &dev_attr_onoff.attr,
    &dev_attr_docalibration.attr,
    &dev_attr_checkallnear.attr,
    &dev_attr_cntinputpins.attr,
    &dev_attr_proxstatus.attr,
    &dev_attr_regproxdata.attr,
    &dev_attr_regproxctrl0.attr,
    &dev_attr_regproxctrl1.attr,
    &dev_attr_regproxctrl2.attr,
    &dev_attr_regproxctrl3.attr,
    &dev_attr_regproxctrl4.attr,
    &dev_attr_regproxctrl5.attr,
    &dev_attr_regproxctrl6.attr,
    &dev_attr_regproxctrl7.attr,
    &dev_attr_regproxctrl8.attr,
    &dev_attr_regirqmask.attr,
    &dev_attr_regreset.attr,
    &dev_attr_regoffset.attr,
    NULL,
};

static struct attribute_group sx9500_attr_group = {
    .attrs = sx9500_attributes,
};
/*********************************************************************/

/*! \fn static int read_regIrqStat(struct sx86XX *this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s) 
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct 
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regIrqStat(struct sx86XX *this)
{
    u8 data = 0;

    if (this) {
        if (read_register(this, SX9500_IRQSTAT_REG, &data) == 0){
            dev_info(this->pdev, "SX9500_IRQSTAT_REG[0x00] = 0x%02x\n", data);
            return (data & 0x00FF);
        }
    }

    return 0;
}

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct 
 */
static void hw_init(struct sx86XX *this)
{
    struct sx9500 *pDevice = NULL;
    struct sx9500_platform_data *pdata = NULL;
    int i = 0;

    /* configure device */
    dev_dbg(this->pdev, "Setup I2C Registers\n");
    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
        while ( i < pdata->i2c_reg_num) {
            /* Write all registers/values contained in i2c_reg */
            dev_dbg(this->pdev,
                    "Write Reg: 0x%02x, Value: 0x%02x\n",
                    pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
            //msleep(3);        
            write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
            i++;
        }
    }
    else {
        dev_err(this->pdev, "ERROR! platform data 0x%p\n", pDevice->hw);
    }
}
/*********************************************************************/




/*! \fn static int initialize_device(struct sx86XX *this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct 
 * \return Last used command's return value (negative if error)
 */
static int initialize_device(struct sx86XX *this)
{
    s16 Useful_CSx = 0, Avg_CSx = 0, Diff_CSx = 0; u16 Offset_CSx = 0;

    unsigned char mainSensor = 0x00, main2Sensor = 0x00, refSensor = 0x00;

#if defined(CONFIG_MACH_MSM8926_E8LTE)
	int cnt= 0;
#endif
    struct sx9500 *pDevice = NULL;

    if (this) {
        int i = 0;

        /* Make sure we initialize that we are not in startup detection */
        this->inStartupTouch = false;
        /* prepare reset by disabling any irq handling */
        this->irq_disabled = 1;
        disable_irq(this->irq);
        /* perform a reset */
        write_register(this, SX9500_SOFTRESET_REG, SX9500_SOFTRESET);
        /* wait until the reset has finished by monitoring NIRQ */
        dev_info(this->pdev,
                "Sent Software Reset. Waiting until device is back from reset to continue.\n");

        /* just sleep for awhile instead of using a loop with reading irq status */
    #if defined(CONFIG_MACH_MSM8926_E8LTE)
        do {
            msleep(100);
            read_regIrqStat(this);
			
            if (++cnt >= 3) break;
        } while(this->get_nirq_low && this->get_nirq_low());
    #else	
        msleep(300);
    #endif

        dev_info(this->pdev, "Device is back from the reset, continuing. NIRQ = %d\n", this->get_nirq_low());

        hw_init(this);

        /* make sure everything is running */
        msleep(100);

        manual_offset_calibration(this);

        // This should cover most of the scan periods, if extremely large may
        // want to set this to 500ms. Make sure manual offset has been fully done
        msleep(500); 

        if (likely((pDevice = this->pDevice) != NULL)) {
            mainSensor = (unsigned char) pDevice->hw->input_mainsensor;
            refSensor = (unsigned char) pDevice->hw->input_refsensor;
            if (valid_multiple_input_pins(this))
                main2Sensor = (unsigned char) pDevice->hw->input_main2sensor; 

            for (i = 0; i < 5 ; i++) {
                read_sensor_regdata(this, mainSensor, &Useful_CSx, &Avg_CSx, &Diff_CSx, &Offset_CSx);
                dev_info(this->pdev,
                            "[startup] capMain[%d] = %6d %6d %6d %6d\n",
                            i, Useful_CSx, Avg_CSx, Diff_CSx, Offset_CSx);
            }

            if (valid_multiple_input_pins(this)) {
                Useful_CSx = 0, Avg_CSx = 0, Diff_CSx = 0; Offset_CSx = 0;
                for (i = 0; i < 5 ; i++) {
                    read_sensor_regdata(this, main2Sensor, &Useful_CSx, &Avg_CSx, &Diff_CSx, &Offset_CSx);
                    dev_info(this->pdev,
                                "[startup] capMain2[%d] = %6d %6d %6d %6d\n",
                                i, Useful_CSx, Avg_CSx, Diff_CSx, Offset_CSx);
                }
            }

            StartupTouchCheckWithReferenceSensor(this, mainSensor, refSensor);
            /* Howevet use multiple inputs, the startup is only one input is used.
               (Temporary Exception Handling) */
            //if (valid_multiple_input_pins(this))
            //    StartupTouchCheckWithReferenceSensor(this, main2Sensor, refSensor);
        }
        else {
            dev_err(this->pdev,
                "Couldn't open platform data containing main and ref sensors, using fallback\n");

            return -ENOMEM;
        }

        /* re-enable interrupt handling */
        enable_irq(this->irq);
        this->irq_disabled = 0;

        /* make sure no interrupts are pending since enabling irq will only
         * work on next falling edge */
        read_regIrqStat(this);
        dev_info(this->pdev, "Exiting initialize(). NIRQ = %d\n", this->get_nirq_low());

        return 0;
    }

    return -ENOMEM;
}

/*!
 * \brief   Check if a touch(near) is on the specified mainSensor
 * \details This uses a reference sensor (refSensor) to check whether a touch
 *      (=near) is being performed. You normally would use this type of check 
 *      after a compensation has been made (especially during power up).
 * \param   this Pointer to main parent struct 
 * \param   mainSensor Main sensor that may or may not have a touch
 * \param   refSensor A known sensor that will never have a touch
 * \return  Whether a touch was detected
 */
void StartupTouchCheckWithReferenceSensor(struct sx86XX *this,
                                        unsigned char mainSensor,
                                        unsigned char refSensor) 
{
    s32 cal_Margin = -65535;
    u16 cal_Offset = 0; /* fullbyte */
    s32 cal_TotalCap = 0;

    s32 capMain = 0;
    s32 capRef = 0;
    s32 capStartup = 0;

    int counter = 0;
    u8 CompareButtonMask = 0;
    int numberOfButtons = 0;
    struct sx9500 *pDevice = NULL;
    struct _buttoninfo *buttons = NULL;
    struct input_dev *input = NULL;

    struct _buttoninfo *pCurrentButton  = NULL;

    u8 ucTouchAvgThresh = 0;
    u8 ucReleaseAvgThresh = 0;

    if (unlikely((this==NULL) || ((pDevice = this->pDevice)==NULL)))
        return; // ERROR!!

    if (mainSensor == (pDevice->hw->input_mainsensor))
        read_calibration_data(this, &cal_Margin, &cal_Offset, &cal_TotalCap, false);
    else if (mainSensor == (pDevice->hw->input_main2sensor))
        read_calibration_data(this, &cal_Margin, &cal_Offset, &cal_TotalCap, true);

    dev_info(this->pdev,
                "read_calibration_data : %d, %d, %d\n",
                cal_Margin, cal_Offset, cal_TotalCap);

    if (skip_startup == true)
        return;

    if (cal_Margin == -65535/* || cal_Offset < 500 || cal_Offset > 4000*/) {
        dev_err(this->pdev, "Fail!!! Check Calibration Data!!!\n");
        return;
    }

    pDevice->pStartupCheckParameters->calibration_margin = cal_Margin;

    // Calculate out the Main Cap information //
    calculate_CSx_rawdata(this, mainSensor, NULL, NULL, &capMain);

    // Calculate out the Reference Cap information //
    calculate_CSx_rawdata(this, refSensor, NULL, NULL, &capRef);

    // Calculate Dynamic Threshold value.
    capStartup = pDevice->pStartupCheckParameters->dynamicthreshold_offset + 
                ((pDevice->pStartupCheckParameters->dynamicthreshold_temp_slope * capRef) / 10) +
                       pDevice->pStartupCheckParameters->calibration_margin;

    dev_info(this->pdev,
                "Main[%ld] - Startup[%ld] = %ld",
                (long int) capMain, (long int) capStartup, (long int) (capMain - capStartup));

    // Must be sure to set the Main CS pin.
    write_register(this, SX9500_SENSORSEL_REG, mainSensor);

    // capMain is now the diff to check with startupThreshold..
    // The code below is what will send the key event.
    // We need to shift 4 more as the lower bits are the compensation.
    //  TODO: Change this to use a define index
    CompareButtonMask = 1 << (mainSensor + 4); 

    buttons = pDevice->pButtonInformation->buttons;
    input = pDevice->pButtonInformation->input;
    numberOfButtons = pDevice->pButtonInformation->buttonSize;

    if (unlikely( (buttons==NULL) || (input==NULL) )) {
        dev_err(this->pdev, "ERROR!! buttons or input NULL!!!\n");
        return;
    }

    ucTouchAvgThresh = pDevice->pStartupCheckParameters->startup_touch_regavgthresh;
    ucReleaseAvgThresh = pDevice->pStartupCheckParameters->startup_release_regavgthresh;

    // If the buttons are added to the array from 0 to max sensor,
    // then we could just skip the for and change this to have..
    // pCurrentButton = &buttons[mainSensor]
    // But since they could be added out of here this is here just in case..
    for (counter = 0; counter < numberOfButtons; counter++) {
        pCurrentButton = &buttons[counter];
        if (pCurrentButton==NULL) {
            dev_err(this->pdev, "ERROR!! current button at index: %d NULL!!!\n", counter);
            return; // ERRORR!!!!
        }

        if ((CompareButtonMask & pCurrentButton->mask) != pCurrentButton->mask) {
            dev_dbg(this->pdev, "Mask: 0x%02x Looking For: 0x%02x Counter: %d\n",
                                       pCurrentButton->mask, CompareButtonMask, counter);
            continue; // Not the correct one so continue to next
        }

        switch (pCurrentButton->state) {
            case IDLE: /* Button is not being touched! */
                if (capMain > capStartup) {
                    /* User pressed button */
                    dev_info(this->pdev, "[startup]cap button %d touched\n", counter);
                    //input_report_key(input, pCurrentButton->keycode, 1);
                    input_report_abs(input, ABS_DISTANCE, PROX_STATUS_NEAR);
                    input_sync(input);
                    pCurrentButton->state = ACTIVE;

                    /* Set the flag since touch is detected during startup */
                    this->inStartupTouch = true;
                    write_register(this, SX9500_CPS_CTRL4_REG, ucTouchAvgThresh);
                }
                else {
                    /* Clear the flag since no touch is detected during startup */
                    this->inStartupTouch = false;
                    write_register(this, SX9500_CPS_CTRL4_REG, ucReleaseAvgThresh);

                    dev_info(this->pdev, "[startup]Button %d already released.\n",counter);
                }
                break;

            case ACTIVE: /* Button is being touched! */ 
                if (capMain <= capStartup) {
                    /* User released button */
                    dev_info(this->pdev, "[startup]cap button %d released\n",counter);
                    //input_report_key(input, pCurrentButton->keycode, 0);
                    input_report_abs(input, ABS_DISTANCE, PROX_STATUS_FAR);
                    input_sync(input);
                    pCurrentButton->state = IDLE;

                    /* Clear the flag since no touch is detected during startup */
                    this->inStartupTouch = false;
                    write_register(this, SX9500_CPS_CTRL4_REG, ucReleaseAvgThresh);
                }
                else {
                    /* Set the flag since touch is detected during startup */
                    this->inStartupTouch = true;
                    write_register(this, SX9500_CPS_CTRL4_REG, ucTouchAvgThresh);

                    dev_info(this->pdev, "[startup]Button %d still touched.\n",counter);
                }
                break;

            default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
                dev_err(this->pdev, "ERROR! state: %d unknown!\n",pCurrentButton->state);
                break;
        };

        break; // We only want to do this once as we only are checking one sensor

    }
}
 
/*! 
 * \brief Handle what to do when a Close/Far occurs
 * \param this Pointer to main parent struct 
 */
static void Irq_Process_Close_Far(struct sx86XX *this)
{
    int counter = 0;
    u8 regstat = 0;
    int numberOfButtons = 0;
    struct sx9500 *pDevice = NULL;
    struct _buttoninfo *buttons = NULL;
    struct input_dev *input = NULL;

    struct _buttoninfo *pCurrentButton  = NULL;

    u8 mainSensor = 0, main2Sensor = 0;

    if (this && (pDevice = this->pDevice)) {
        dev_dbg(this->pdev, "Inside Irq_Process_Close_Far()\n");

        read_register(this, SX9500_TCHCMPSTAT_REG, &regstat);
        dev_info(this->pdev, "SX9500_TCHCMPSTAT_REG[0x01] = 0x%02x\n", regstat);

        buttons = pDevice->pButtonInformation->buttons;
        input = pDevice->pButtonInformation->input;
        numberOfButtons = pDevice->pButtonInformation->buttonSize;

        if (unlikely((buttons == NULL) || (input == NULL))) {
            dev_err(this->pdev, "ERROR!! buttons or input NULL!!!\n");
            return;
        }

        mainSensor = pDevice->hw->input_mainsensor;
        if (valid_multiple_input_pins(this))
            main2Sensor = pDevice->hw->input_main2sensor;

        for (counter = 0; counter < numberOfButtons; counter++) {
            pCurrentButton = &buttons[counter];
            if (pCurrentButton == NULL) {
                dev_err(this->pdev, "ERROR!! current button at index: %d NULL!!!\n", counter);
                return; // ERRORR!!!!
            }

            switch (pCurrentButton->state) {
                case IDLE: /* Button is not being touched! */
                    if (((regstat & pCurrentButton->mask) == pCurrentButton->mask)) {
                        /* User pressed button */
                        dev_info(this->pdev, "cap button %d touched\n", counter);
                        /*input_report_key(input, pCurrentButton->keycode, 1);*/
                        if (valid_multiple_input_pins(this)) {
                            if ((buttons[mainSensor].state == ACTIVE) || 
                                    (buttons[main2Sensor].state == ACTIVE)) {
                                pCurrentButton->state = ACTIVE;
                                dev_info(this->pdev, "active set only\n");
                            }
                            else {
                                pCurrentButton->state = ACTIVE;
                                input_report_abs(input, ABS_DISTANCE, PROX_STATUS_NEAR);
                                input_sync(input);
                                dev_info(this->pdev, "active set and PROX_STATUS_NEAR\n");
                            }
                        }
                        else {
                            input_report_abs(input, ABS_DISTANCE, PROX_STATUS_NEAR);
                            input_sync(input);
                            pCurrentButton->state = ACTIVE;
                        }
                    }
                    else {
                        dev_info(this->pdev, "Button %d already released.\n",counter);
                    }
                    break;

                case ACTIVE: /* Button is being touched! */ 
                    if (((regstat & pCurrentButton->mask) != pCurrentButton->mask)) {
                        /* User released button */
                        dev_info(this->pdev, "cap button %d released\n",counter);
                        /*input_report_key(input, pCurrentButton->keycode, 0);*/
                        if (valid_multiple_input_pins(this)) {
                            pCurrentButton->state = IDLE;
                            if ((buttons[mainSensor].state == IDLE) && 
                                    (buttons[main2Sensor].state == IDLE)) {
                                input_report_abs(input, ABS_DISTANCE, PROX_STATUS_FAR);
                                input_sync(input);
                                dev_info(this->pdev, "idle set and PROX_STATUS_FAR\n");
                            }
                        }
                        else {
                            input_report_abs(input, ABS_DISTANCE, PROX_STATUS_FAR);
                            input_sync(input);
                            pCurrentButton->state = IDLE;
                        }
                    }
                    else {
                        dev_info(this->pdev, "Button %d still touched.\n",counter);
                    }
                    break;

                default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
                    break;
            };
        }

        dev_dbg(this->pdev, "Leaving Irq_Process_Close_Far()\n");
    }
}

#ifdef CONFIG_OF
static int sx9500_regulator_configure(struct sx86XX *this, bool bonoff)
{
    struct sx9500 *pDevice = NULL;
    struct sx9500_platform_data *pdata = NULL;

    int rc = 0;

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
        if (bonoff == true) {
            pdata->vdd_regulator = regulator_get(this->pdev, "Semtech,vdd");
            if (IS_ERR(pdata->vdd_regulator)) {
                rc = PTR_ERR(pdata->vdd_regulator);
                dev_err(this->pdev,
                        "Lookup and obtain a reference to a vdd_regulator. Fail![%d]\n", rc);
                return rc;
            }

            if (regulator_count_voltages(pdata->vdd_regulator) > 0) {
                rc = regulator_set_voltage(pdata->vdd_regulator, 
                                            pdata->vdd_supply_min, pdata->vdd_supply_max);
                if (rc < 0) {
                    dev_err(this->pdev,
                            "Set vdd_regulator output voltage. Fail![%d]\n", rc);
                    regulator_put(pdata->vdd_regulator);
                    return rc;
                }

                rc = regulator_set_optimum_mode(pdata->vdd_regulator, pdata->vdd_load_ua);
                if (rc < 0) {
                    dev_err(this->pdev,
                            "Set vdd_regulator optimum operating mode. Fail![%d]\n", rc);
                    regulator_put(pdata->vdd_regulator);
                    return rc;
                }

                rc = regulator_enable(pdata->vdd_regulator);
                if (rc) {
                    dev_err(this->pdev,
                            "Enable vdd_regulator output. Fail![%d]\n", rc);
                    regulator_set_optimum_mode(pdata->vdd_regulator, 0);
                    regulator_put(pdata->vdd_regulator);
                    return rc;
                }
            }

            pdata->svdd_regulator = regulator_get(this->pdev, "Semtech,svdd");
            if (IS_ERR(pdata->svdd_regulator)) {
                rc = PTR_ERR(pdata->svdd_regulator);
                dev_err(this->pdev,
                        "Lookup and obtain a reference to a svdd regulator, Fail![%d]\n", rc);
                return rc;
            }

            if (regulator_count_voltages(pdata->svdd_regulator) > 0) {
                rc = regulator_set_voltage(pdata->svdd_regulator,
                                            pdata->svdd_supply_min, pdata->svdd_supply_max);
                if (rc < 0) {
                    dev_err(this->pdev, "Set svdd regulator output voltage, Fail![%d]\n", rc);
                    regulator_put(pdata->svdd_regulator);
                    return rc;
                }

                rc = regulator_set_optimum_mode(pdata->svdd_regulator, pdata->vdd_load_ua);
                if (rc < 0) {
                    dev_err(this->pdev,
                            "Set svdd_regulator optimum operating mode. Fail![%d]\n", rc);
                    regulator_put(pdata->svdd_regulator);
                    return rc;
                }

                rc = regulator_enable(pdata->svdd_regulator);
                if (rc) {
                    dev_err(this->pdev, "Enable svdd_regulator output. Fail![%d]\n", rc);
                    regulator_set_optimum_mode(pdata->svdd_regulator, 0);
                    regulator_put(pdata->svdd_regulator);
                    return rc;
                }
            }
        }
        else {
            if (regulator_is_enabled(pdata->vdd_regulator) > 0) {
                regulator_set_voltage(pdata->vdd_regulator, 0, pdata->vdd_supply_max);
                regulator_set_optimum_mode(pdata->vdd_regulator, 0);
                regulator_put(pdata->vdd_regulator);
                regulator_disable(pdata->vdd_regulator);
            }

            if (regulator_is_enabled(pdata->svdd_regulator) > 0) {
                regulator_set_voltage(pdata->svdd_regulator, 0, pdata->svdd_supply_max);
                regulator_set_optimum_mode(pdata->svdd_regulator, 0);
                regulator_put(pdata->svdd_regulator);
                regulator_disable(pdata->svdd_regulator);
            }
        }
    }

    return rc;
}

static int sx9500_init_platform_hw(struct i2c_client *client)
{
    struct sx86XX *this = i2c_get_clientdata(client);
    struct sx9500 *pDevice = NULL;
    struct sx9500_platform_data *pdata = NULL;

    int rc;

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
        sx9500_regulator_configure(this, true);
        if (gpio_is_valid(pdata->irq_gpio)) {
            rc = gpio_request(pdata->irq_gpio, "sx9500_irq_gpio");
            if (rc) {
                dev_err(this->pdev, "Request gpio. Fail![%d]\n", rc);
                return rc;
            }

            rc = gpio_direction_input(pdata->irq_gpio);
            if (rc) {
                dev_err(this->pdev, "Set gpio direction. Fail![%d]\n", rc);
                return rc;
            }

            this->irq = client->irq = gpio_to_irq(pdata->irq_gpio);
        }
        else {
            dev_err(this->pdev, "Invalid irq gpio num.(init)\n");
        }
    }

    return rc;
}

static void sx9500_exit_platform_hw(struct i2c_client *client)
{
    struct sx86XX *this = i2c_get_clientdata(client);
    struct sx9500 *pDevice = NULL;
    struct sx9500_platform_data *pdata = NULL;

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
        sx9500_regulator_configure(this, false);          
        if (gpio_is_valid(pdata->irq_gpio)) {
            gpio_free(pdata->irq_gpio);
        }
        else {
            dev_err(this->pdev, "Invalid irq gpio num.(exit)\n");
        }
    }

    return;
}

static int parse_devicetree(struct device *dev, struct sx9500_platform_data *pdata)
{
    struct device_node *np = dev->of_node;

    int ret = 0;

    int i = 0;
    u32 temp_u32;
    u32 temp_array_u32[2];
    struct sensor_dt_to_platformdata *pdtentry = NULL;
    struct sensor_dt_to_platformdata ar_dt_entry_data[] = {
      {"Semtech,irq-gpio",         &pdata->irq_gpio,          DT_GPIO},
      {"Semtech,vdd_supply_min",   &pdata->vdd_supply_min,    DT_U32},
      {"Semtech,vdd_supply_max",   &pdata->vdd_supply_max,    DT_U32},
      {"Semtech,vdd_load_ua",      &pdata->vdd_load_ua,       DT_U32},
      {"Semtech,svdd_supply_min",  &pdata->svdd_supply_min,   DT_U32},
      {"Semtech,svdd_supply_max",  &pdata->svdd_supply_max,   DT_U32},
      {"Semtech,svdd_load_ua",     &pdata->svdd_load_ua,      DT_U32},
      {"Semtech,RegIrqMask",       &temp_array_u32,           DT_U8_ARRAY},
      {"Semtech,RegProxCtrl0",     &temp_array_u32,           DT_U8_ARRAY},
      {"Semtech,RegProxCtrl1",     &temp_array_u32,           DT_U8_ARRAY},
      {"Semtech,RegProxCtrl2",     &temp_array_u32,           DT_U8_ARRAY},
      {"Semtech,RegProxCtrl3",     &temp_array_u32,           DT_U8_ARRAY},
      {"Semtech,RegProxCtrl4",     &temp_array_u32,           DT_U8_ARRAY},
      {"Semtech,RegProxCtrl5",     &temp_array_u32,           DT_U8_ARRAY},
      {"Semtech,RegProxCtrl6",     &temp_array_u32,           DT_U8_ARRAY},
      {"Semtech,RegProxCtrl7",     &temp_array_u32,           DT_U8_ARRAY},
      {"Semtech,RegProxCtrl8",     &temp_array_u32,           DT_U8_ARRAY},
      {"Semtech,RegSensorSel",     &temp_array_u32,           DT_U8_ARRAY},
      {"Semtech,InputPinsNum",     &pdata->input_pins_num,    DT_U32},
      {"Semtech,InputMainSensor",  &pdata->input_mainsensor,  DT_U8},
      {"Semtech,InputMainSensor2", &pdata->input_main2sensor, DT_U8},
      {"Semtech,InputRefSensor",   &pdata->input_refsensor,   DT_U8},
      {"Semtech,DynamicThres_Offset",
                            &pdata->pStartupCheckParameters->dynamicthreshold_offset,     DT_U32},
      {"Semtech,DynamicThres_Temp_Slope",
                            &pdata->pStartupCheckParameters->dynamicthreshold_temp_slope, DT_U32},
      {"Semtech,DynamicThres_Hysteresis",
                            &pdata->pStartupCheckParameters->dynamicthreshold_hysteresis, DT_U32},
      {"Semtech,Calibration_Margin",
                            &pdata->pStartupCheckParameters->calibration_margin,          DT_U32},
      {"Semtech,Startup_Touch_RegAvgThres",
                            &pdata->pStartupCheckParameters->startup_touch_regavgthresh,   DT_U8},
      {"Semtech,Startup_Release_RegAvgThres",
                            &pdata->pStartupCheckParameters->startup_release_regavgthresh, DT_U8},
      {NULL,                       NULL,                      0},
    };

    for (pdtentry = ar_dt_entry_data; pdtentry->dt_name ; ++pdtentry) {
        switch (pdtentry->type) {
            case DT_GPIO:
                ret = of_get_named_gpio(np, pdtentry->dt_name, 0);
                if (ret >= 0) {
                    *((int *) pdtentry->ptr_data) = ret;
                    ret = 0;
                    dev_dbg(dev, "[%s] is [%d]. ret=%d\n",
                            pdtentry->dt_name, *((int *)pdtentry->ptr_data), ret);
                }
                break;

            case DT_U32:
                ret = of_property_read_u32(np, pdtentry->dt_name, (u32 *) pdtentry->ptr_data);
                if (ret == 0) {
                    dev_dbg(dev, "[%s] is [%d]. ret=%d\n",
                            pdtentry->dt_name, *((int *)pdtentry->ptr_data), ret);
                }
                break;

            case DT_U8_ARRAY:
                temp_array_u32[0] = 0; temp_array_u32[1] = 0;
                ret = of_property_read_u32_array(np,
                                                pdtentry->dt_name,
                                                (u32 *) pdtentry->ptr_data, 2);
                if (ret == 0) {
                    pdata->pi2c_reg[i].reg = (unsigned char) temp_array_u32[0];
                    pdata->pi2c_reg[i].val = (unsigned char) temp_array_u32[1];
                    pdata->i2c_reg_num = i;
                    dev_dbg(dev, "[%s] are [0x%02x], [0x%02x]. ret=%d\n", 
                            pdtentry->dt_name, pdata->pi2c_reg[i].reg, pdata->pi2c_reg[i].val, ret);
                    i++;
                }
                break;

            case DT_U8:
                temp_u32 = 0;
                ret = of_property_read_u32(np, pdtentry->dt_name, (u32 *) &temp_u32);
                if (ret == 0) {
                    *((u8 *) pdtentry->ptr_data) = (u8) temp_u32;
                    dev_dbg(dev, "[%s] is [0x%02x]. ret=%d\n",
                            pdtentry->dt_name, *((u8 *)pdtentry->ptr_data), ret);
                }
                break;

            default:
                dev_err(dev, "%d is an unknown DT entry type. Fail!\n", pdtentry->type);
                ret = -EBADE;
                break;
        }

        if (ret) {
            dev_err(dev, "Check '%s' DT entry. Fail!\n", pdtentry->dt_name);
            return ret;
        }
    }

    return 0;
}
#endif

/* get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
static int sx9500_get_nirq_state(void)
{
    return !gpio_get_value(GPIO_SX9500_NIRQ);
}

/*! \fn static int sx9500_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx9500_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i = 0;
    int err = 0;

    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

    struct sx86XX *this = NULL;
    struct sx9500 *pDevice = NULL;
    struct sx9500_platform_data *pplatData = NULL;
    struct _totalbuttoninformation *pButtonInformationData = NULL;
    struct _startupcheckparameters *pStartupCheckParameters = NULL;

    struct input_dev *input = NULL;

    dev_info(&client->dev, "sx9500_probe()\n");

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA)) {
        dev_err(&client->dev, "Check i2c functionality.Fail!\n");
        err = -EIO;
        goto exit;
    }

    pButtonInformationData = devm_kzalloc(&client->dev,
                                            sizeof(struct _totalbuttoninformation), GFP_KERNEL);
    if (!pButtonInformationData) {
        dev_err(&client->dev, "Failed to allocate memory(_totalButtonInformation)\n");
        err = -ENOMEM;
        goto exit;
    }

    pButtonInformationData->buttons = psmtcButtons;
    pButtonInformationData->buttonSize = ARRAY_SIZE(psmtcButtons),

    pStartupCheckParameters = devm_kzalloc(&client->dev,
                                            sizeof(struct _startupcheckparameters), GFP_KERNEL);
    if (!pStartupCheckParameters) {
        dev_err(&client->dev, "Failed to allocate memory(_startupCheckParameters)\n");
        err = -ENOMEM;
        goto exit;
    }

    //pStartupCheckParameters = &smtcTouchCheckParameters;

    pplatData = devm_kzalloc(&client->dev, sizeof(struct sx9500_platform_data), GFP_KERNEL);
    if (!pplatData) {
        dev_err(&client->dev, "Failed to allocate memory(sx9500_platform_data)\n");
        err = -ENOMEM;
        goto exit;
    }

    pplatData->pButtonInformation = pButtonInformationData;
    pplatData->pStartupCheckParameters = pStartupCheckParameters;
    pplatData->get_is_nirq_low = sx9500_get_nirq_state;
    pplatData->init_platform_hw = NULL; 
    pplatData->exit_platform_hw = NULL;

    client->dev.platform_data = pplatData;

#ifdef CONFIG_OF
    if (client->dev.of_node) {
        unsigned char val_regproxctrl1 = 0;
        unsigned char val_regproxctrl2 = 0;

        err = parse_devicetree(&client->dev, pplatData);
        if (err) {
            dev_err(&client->dev, "Failed to parse device tree.[%d]\n", err);
            goto exit;
        }

        while (i < pplatData->i2c_reg_num) {
            if (pplatData->pi2c_reg[i].reg == SX9500_CPS_CTRL1_REG)
                val_regproxctrl1 = pplatData->pi2c_reg[i].val;
            else if (pplatData->pi2c_reg[i].reg == SX9500_CPS_CTRL2_REG)
                val_regproxctrl2 = pplatData->pi2c_reg[i].val;
            i++;
        }
        pplatData->capacitance_range = conv_regval_to_range(val_regproxctrl1);
        pplatData->gain_factor = conv_regval_to_gain(val_regproxctrl2);

        /* set functions of platform data */
        pplatData->init_platform_hw = sx9500_init_platform_hw;
        pplatData->exit_platform_hw = sx9500_exit_platform_hw;
    }
#endif

    /* create memory for main struct */
    this = devm_kzalloc(&client->dev, sizeof(struct sx86XX), GFP_KERNEL); 
    if (!this) {
        dev_err(&client->dev, "Failed to allocate memory(sx86XX)\n");
        err = -ENOMEM;
        goto exit;
    }

    if (this) {
        /* In case we need to reinitialize data 
        * (e.q. if suspend reset device) */
        this->init = initialize_device;
        /* shortcut to read status of interrupt */
        this->refreshStatus = read_regIrqStat;
        /* pointer to function from platform data to get pendown 
         * (1->NIRQ=0, 0->NIRQ=1) */
        this->get_nirq_low = pplatData->get_is_nirq_low;
        /* save irq in case we need to reference it */
        this->irq = client->irq;
        /* do we need to create an irq timer after interrupt ? */
        this->useIrqTimer = 0;

        /* Setup function to call on corresponding reg irq source bit */
        if (MAX_NUM_STATUS_BITS >= 8) {
            this->statusFunc[0] = 0; /* TXEN_STAT */
            this->statusFunc[1] = 0; /* UNUSED */
            this->statusFunc[2] = 0; /* UNUSED */
            this->statusFunc[3] = 0; /* CONVERSION_IRQ(CONV_STAT) */
            this->statusFunc[4] = Irq_Process_Close_Far; /* COMPENSATION_IRQ(COMP_STAT) */
            this->statusFunc[5] = Irq_Process_Close_Far; /* FAR_IRQ(RELEASE_STAT) */
            this->statusFunc[6] = Irq_Process_Close_Far; /* CLOSE_IRQ(TOUCH_STAT) */
            this->statusFunc[7] = 0; /* RESET_STAT */
        }

        /* setup i2c communication */
        this->bus = client;
        i2c_set_clientdata(client, this);

        /* record device struct */
        this->pdev = &client->dev;

        /* create memory for device specific struct */
        pDevice = devm_kzalloc(&client->dev, sizeof(struct sx9500), GFP_KERNEL);
        if (!pDevice) {
            dev_err(&client->dev, "Failed to allocate memory(sx9500)\n");
            err = -ENOMEM;
            goto exit;
        }

        this->pDevice = pDevice;

        if (pDevice) {
            /* Add Pointer to main platform data struct */
            pDevice->hw = pplatData;
      
            /* Initialize the button information initialized with keycodes */
            pDevice->pButtonInformation = pplatData->pButtonInformation;

            /* Initialize the startup parameters */
            pDevice->pStartupCheckParameters = pplatData->pStartupCheckParameters;

            /* Check if we hava a platform initialization function to call*/
            if (pplatData->init_platform_hw)
                pplatData->init_platform_hw(client);

            /* Create the input device */
            input = input_allocate_device();
            if (!input) {
                return -ENOMEM;
            }

            /* Set all the keycodes */
            /*__set_bit(EV_KEY, input->evbit);*/
            __set_bit(EV_ABS, input->evbit);
            input_set_abs_params(input, ABS_DISTANCE, 0, 1, 0, 0);
            for (i = 0; i < pDevice->pButtonInformation->buttonSize; i++) {
                /*__set_bit(pDevice->pButtonInformation->buttons[i].keycode, input->keybit);*/
                pDevice->pButtonInformation->buttons[i].state = IDLE;
            }

            /* save the input pointer and finish initialization */
            pDevice->pButtonInformation->input = input;
            input->name = "SX9500 Cap Touch";
            input->id.bustype = BUS_I2C;
            //input->id.product = sx863x->product;
            //input->id.version = sx863x->version;
            if(input_register_device(input))
                return -ENOMEM;

            /* for accessing items in user data (e.g. calibrate) */
            err = sysfs_create_group(&client->dev.kobj, &sx9500_attr_group);
            if (err)
                printk(KERN_INFO"sysfs create fail!");

            wake_lock_init(&this->capsensor_wake_lock, WAKE_LOCK_SUSPEND, "capsensor_wakeup");
        }

        sx86XX_init(this);

        /* default sensor off */
        onoff_sensor(this, DISABLE_IRQ_MASK | DISABLE_SENSOR_PINS);

        return  0;
    }

    return -1;

exit:
    return err;
}

/*! \fn static int sx9500_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx86XX_remove()
 */
static int sx9500_remove(struct i2c_client *client)
{
    struct sx86XX *this = i2c_get_clientdata(client);
    struct sx9500 *pDevice = NULL;
    struct sx9500_platform_data *pplatData = NULL;

    if (this && (pDevice = this->pDevice))
    {
        disable_irq_wake(this->irq);

        input_unregister_device(pDevice->pButtonInformation->input);

        sysfs_remove_group(&client->dev.kobj, &sx9500_attr_group);

        pplatData = client->dev.platform_data;
        if (pplatData && pplatData->exit_platform_hw)
            pplatData->exit_platform_hw(client);

        kfree(this->pDevice);
    }

    return sx86XX_remove(this);
}

#ifdef CONFIG_PM
/*====================================================*/
/***** Kernel Suspend *****/
static int sx9500_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct sx86XX *this = i2c_get_clientdata(client);
    sx86XX_suspend(this);

    return 0;
}

/***** Kernel Resume *****/
static int sx9500_resume(struct i2c_client *client)
{
    struct sx86XX *this = i2c_get_clientdata(client);
    sx86XX_resume(this);

    return 0;
}
/*====================================================*/
#else
#define sx9500_suspend    NULL
#define sx9500_resume    NULL
#endif /* CONFIG_PM */

static struct i2c_device_id sx9500_id[] = {
    { "sx9500", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sx9500_id);

#ifdef CONFIG_OF
static struct of_device_id sx9500_match_table[] = {
    { .compatible = "Semtech,sx9500",},
    { },
};
#else
#define sx9500_match_table NULL
#endif

static struct i2c_driver sx9500_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = "sx9500",
        .of_match_table = sx9500_match_table,
    },
    .id_table = sx9500_id,
    .probe    = sx9500_probe,
    .remove   = __devexit_p(sx9500_remove),
    .suspend  = sx9500_suspend,
    .resume   = sx9500_resume,
};

static int __init sx9500_init(void)
{
    printk(KERN_INFO "sx9500 driver: initialize.");
    return i2c_add_driver(&sx9500_driver);
}

static void __exit sx9500_exit(void)
{
    printk(KERN_INFO "sx9500 driver: release.");
    i2c_del_driver(&sx9500_driver);
}

module_init(sx9500_init);
module_exit(sx9500_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX9500 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
