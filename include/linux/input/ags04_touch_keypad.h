#ifndef __AGS04_KEYTOUCH_H
#define __AGS04_KEYTOUCH_H

#define AGS04_KEYTOUCH_NAME				"AGS04"
#define AGS04_KEYTOUCH_I2C_SLAVE_ADDR			0x6A
#define VENDOR_LGE					0x1004

#define KEY_TOUCH_GPIO_I2C_SCL				7
#define KEY_TOUCH_GPIO_I2C_SDA				6
#define KEY_TOUCH_GPIO_INT				13
#define MAX_NUM_OF_KEY					2

struct key_touch_platform_data {
	int (*tk_power)(int on, bool log_on);
	int irq;
	int scl;
	int sda;
    int ldo;
	//unsigned char *keycode;
	int keycode[MAX_NUM_OF_KEY];
	int keycodemax;
	unsigned int gpio_int;
    bool tk_power_flag;
};
#endif
