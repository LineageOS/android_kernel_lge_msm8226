#ifndef __REFCODE_H__
#define __REFCODE_H__

//#include "Validation.h"

#define F54_Porting


#ifdef F54_Porting
#include <linux/kernel.h>	//printk
#include <linux/string.h>	//memset
#include <linux/i2c.h>
#include <linux/input/lge_touch_core.h>
extern void device_I2C_read(unsigned char add, unsigned char *value, unsigned short len);
extern void device_I2C_write(unsigned char add, unsigned char *value, unsigned short len);
extern void InitPage(void);
extern void SetPage(unsigned char page);
extern void readRMI(unsigned short add, unsigned char *value, unsigned short len);
extern void writeRMI(unsigned short add, unsigned char *value, unsigned short len);
extern void delayMS(int val);
extern void cleanExit(int code);
extern int waitATTN(int code, int time);
extern void write_log(char *data);
extern int get_limit(unsigned char Tx, unsigned char Rx, struct i2c_client client, struct touch_platform_data *pdata);
extern int LimitFile[30][46*2];
#endif

#define _F54_TEST_
#define _FW_TESTING_
#define _DS4_3_2_	// TM2371, TM2370, PLG137, PLG122
#define NoiseMitigation 0x155	// PLG124 E008
#define F54_CBCPolarity 0x16E	// PLG124 E008
#define F54_CBCSettings_0D 0x16E
#define TOUCH_INFO_MSG(fmt, args...) \
		printk(KERN_INFO "[Touch] " fmt, ##args);

#ifdef _DS4_3_2_
#define F55_PhysicalRx_Addr 0x301	// TM2371, TM2370, PLG137, PLG122
#endif

#ifdef _F54_TEST_
unsigned char F54_FullRawCap(int);
unsigned char F54_RxToRxReport(void);
unsigned char F54_TxToGndReport(void);
unsigned char F54_TxToTxReport(void);
unsigned char F54_TxOpenReport(void);
unsigned char F54_RxOpenReport(void);
unsigned char F54_HighResistance(void);

int F54_GetFullRawCap(int, char *);
int F54_GetRxToRxReport(char *);
int F54_GetTxToGndReport(char *);
int F54_GetTxToTxReport(char *);
int F54_GetTxOpenReport(char *);
int F54_GetRxOpenReport(char *);
int F54_GetHighResistance(char *);
#endif

#ifdef _BUTTON_DELTA_IMAGE_TEST_
unsigned char F54_ButtonDeltaImage();
#endif

#ifdef _FW_TESTING_
void HostImplementationTesting( void );
#endif

#ifdef _F34_TEST_
void CompleteReflash_OmitLockdown();
void CompleteReflash();
void CompleteReflash_Lockdown();
void ConfigBlockReflash();
#endif

void FirmwareCheck( void );
void AttentionTest( void );
void FirmwareCheck_temp( void );

#endif

