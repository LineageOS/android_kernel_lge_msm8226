#define DEBUG /* if DEBUG is activated dev_info will be printed */
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/input/aps_ts.h>
#include <linux/completion.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

/* Matrix maximum */
#define MAX_ROW			30
#define MAX_COL			20



/*  Show the touch log  */
#define KERNEL_LOG_MSG		1

#define MAX_FINGER_NUM		10
#define FINGER_EVENT_SZ		8
#define MAX_WIDTH		30
#define MAX_PRESSURE		255
#define MAX_LOG_LENGTH		128

/* Registers */

#define APS_REGH_CMD				0x10

#define APS_REGL_MODE_CONTROL			0x01
#define APS_REGL_ROW_NUM			0x0B
#define APS_REGL_COL_NUM			0x0C
#define APS_REGL_RAW_TRACK			0x0E
#define APS_REGL_EVENT_PKT_SZ			0x0F
#define APS_REGL_INPUT_EVENT			0x10
#define APS_REGL_UCMD				0xA0
#define APS_REGL_UCMD_RESULT_LENGTH		0xAE
#define APS_REGL_UCMD_RESULT			0xAF


/* Event types */
#define APS_ET_LOG		0xD
#define APS_ET_NOTIFY		0xE
#define APS_ET_ERROR		0xF

/* ISP mode */
#define ISP_ENTRY1		{ 0xAC, 0xCE, 0xFF, 0xFD, 0xAC, 0xCE }
#define ISP_ENTRY2		{ 0xAC, 0xCE, 0xFF, 0xFE, 0xAC, 0xCE }
#define ISP_ENTRY3		{ 0xAC, 0xCE, 0xFF, 0xFF, 0xAC, 0xCE }

#define ISP_UNLOCK1		{ 0x00, 0x04, 0xFE, 0xDC, 0xBA, 0x98 }
#define ISP_UNLOCK2		{ 0x00, 0x04, 0x76, 0x54, 0x32, 0x10 }
#define ISP_UNLOCK3			{ 0x00, 0x4C, 0x05, 0xFA, 0x00, 0x05 }
#define ISP_LOCK		{ 0x00, 0x04, 0x05, 0xFA, 0x00, 0x00 }

#define ISP_FULL_ERASE		{ 0x00, 0x10, 0x00, 0x00, 0x10, 0x44 }
#define ISP_CLEAR_DONE		{ 0x00, 0x0C, 0x00, 0x00, 0x00, 0x20 }
#define ISP_WIRTE_MODE		{ 0x00, 0x10, 0x00, 0x00, 0x10, 0x01 }
#define ISP_WIRTE		{ 0x00, 0x10, 0x00, 0x00, 0x10, 0x41 }
#define ISP_READ_MOED		{ 0x00, 0x10, 0x00, 0x00, 0x10, 0x08 }
#define ISP_READ		{ 0x00, 0x10, 0x00, 0x00, 0x10, 0x48 }

/* Firmware file name */
#define FW_NAME			"aps_ts.fw"


enum {
	GET_COL_NUM	= 1,
	GET_ROW_NUM,
	GET_EVENT_DATA,
};

enum {
	LOG_TYPE_U08	= 2,
	LOG_TYPE_S08,
	LOG_TYPE_U16,
	LOG_TYPE_S16,
	LOG_TYPE_U32	= 8,
	LOG_TYPE_S32,
};

enum{
	SYS_INTENSITY = 2,
	SYS_ROWNUM,
	SYS_COLNUM,
	SYS_CLEAR,
	SYS_ENABLE,
	SYS_DISABLE,
};

struct aps_ts_info {
	struct i2c_client 		*client;
	struct input_dev 		*input_dev;
	char 				phys[32];

	u8				row_num;
	u8				col_num;

	int 			irq;

	struct aps_ts_platform_data	*pdata;
	char 				*fw_name;
	struct completion 		init_done;
#ifdef CONFIG_FB
	struct notifier_block		fb_notifier;
#endif
	struct mutex 		lock;
	bool				enabled;

	struct cdev			cdev;
	dev_t				aps_dev;
	struct class		*class;
	
	char				*cm_intensity;
	u8					*raw_data;

	char				raw_cmd;
	int				data_cmd;
	struct aps_log_data {
		u8			*data;
		int			cmd;
	} log;
};


static u16 raw[MAX_ROW][MAX_COL];
static s16 cm[MAX_ROW][MAX_COL];
static void aps_report_input_data(struct aps_ts_info *info, u8 sz, u8 *buf);

static void aps_report_input_data(struct aps_ts_info *info, u8 sz, u8 *buf);
#ifdef CONFIG_FB
static int aps_ts_fb_notifier_call(struct notifier_block *self, unsigned long event, void *data);
#endif
static int aps_ts_config(struct aps_ts_info *info);

static void aps_ts_enable(struct aps_ts_info *info)
{
	dev_dbg(&info->client->dev, "%s\n", __func__);
	if (info->enabled)
		return;
	
	mutex_lock(&info->lock);

	info->enabled = true;
	enable_irq(info->irq);

	mutex_unlock(&info->lock);

}

static void aps_ts_disable(struct aps_ts_info *info)
{
	if (!info->enabled)
		return;
	dev_dbg(&info->client->dev, "%s\n", __func__);

	mutex_lock(&info->lock);

	disable_irq(info->irq);

	info->enabled = false;

	mutex_unlock(&info->lock);
}

#if 0
static void aps_reset()
{
	gpio_set_value(178, 1);
	msleep(500);
	gpio_set_value(178, 0);
	msleep(500);
	gpio_set_value(178, 1);
	msleep(500);
}
#endif
static void aps_clear_input_data(struct aps_ts_info *info)
{
	int i;
	dev_dbg(&info->client->dev, "%s\n", __func__);

	for (i = 0; i < MAX_FINGER_NUM; i++) {
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
	}
	input_sync(info->input_dev);

	return;
}

static int get_intensity(struct aps_ts_info *info)
{
	struct i2c_client *client = info->client;
	
	int col, row;

	int nIdx;

	u8 write_buf[8];
	u8 read_buf[MAX_ROW*2];


	u8 nLength = 0;

	s16 nIntensity;

	
	for(col = 0 ; col < info->col_num ; col++)
	{
		
		
#if KERNEL_LOG_MSG		
		printk("[%2d]  ",col);
#endif

		write_buf[0] = APS_REGH_CMD;
		write_buf[1] = APS_REGL_UCMD;
		write_buf[2] = 0x70;
		write_buf[3] = 0xFF;
		write_buf[4] = col;
		
		if(i2c_master_send(client,write_buf,5)!=5)
		{
			dev_err(&client->dev, "intensity i2c send failed\n");
			enable_irq(info->irq);
			return -1;
		}

		write_buf[0] = APS_REGH_CMD;
		write_buf[1] = APS_REGL_UCMD_RESULT_LENGTH;

		if(i2c_master_send(client,write_buf,2)!=2)
		{
			dev_err(&client->dev,"send : i2c failed\n");
			enable_irq(info->irq);
			return -1;
		}

		if(i2c_master_recv(client,read_buf,1)!=1)
		{
			dev_err(&client->dev,"recv : i2c failed\n");
			enable_irq(info->irq);
			return -1;
		}
	
		nLength = read_buf[0];
		write_buf[0] = APS_REGH_CMD;
		write_buf[1] = APS_REGL_UCMD_RESULT;

		if(i2c_master_send(client,write_buf,2)!=2)
		{
			dev_err(&client->dev,"send : i2c failed\n");
			enable_irq(info->irq);
			return -1;
		}
			
		if(i2c_master_recv(client,read_buf,nLength)!=nLength)
		{
			dev_err(&client->dev,"recv : i2c failed\n");
			enable_irq(info->irq);
			return -1;
		}

		nLength >>=1;
		for(row = 0 ; row <nLength ; row++)
		{			
			nIntensity = (s16)(read_buf[2*row] | (read_buf[2*row+1] << 8));
			//nIntensity = (nIntensity<<8) 
			
			cm[col][row]   = nIntensity;
			
#if KERNEL_LOG_MSG
			printk("%d, \t", nIntensity);
		}
		printk("\n");
#else
		}
#endif
	}

	nIdx = 0;
	for(col = 0 ; col < info->col_num ; col++){
		for(row = 0 ; row < info->row_num ; row++){
			info->cm_intensity[nIdx++]=(char)((cm[col][row] &0xFF00)>>8);
			info->cm_intensity[nIdx++]=(char)( cm[col][row] &0xFF);
		}
	}

	return 0;
}
static int get_rawdata(struct aps_ts_info *info)
{
	struct i2c_client *client = info->client;
	int col, row;
	int nIdx;
//	u8 RefLevel;
	u8 write_buf[8];
	u8 read_buf[MAX_ROW * 8];
	u8 nLength = 0;
	//u16* p;
	u16 nReference;
/*
	write_buf[0] = APS_REGH_CMD;
	write_buf[1] = APS_REGL_REF_TRACK;


	if(i2c_master_send(client, write_buf,2)!=2)
	{
			dev_err(&client->dev, "Ref track level i2c send failed\n");
			return -1;
	}
	if(i2c_master_recv(client, read_buf,1)!=1)
	{
			dev_err(&client->dev, "Ref track level i2c recv failed\n");
			return -1;
	}

	RefLevel = read_buf[0];*/

	disable_irq(info->irq);

	nIdx = 0;
	for(col = 0 ; col < info->col_num ; col++)
	{
		printk("[%2d]  ",col);


		write_buf[0] = APS_REGH_CMD;
		write_buf[1] = APS_REGL_UCMD;
		write_buf[2] = 0x75;
		write_buf[3] = 0xFF;
		write_buf[4] = col;

		if(i2c_master_send(client,write_buf,5)!=5)
		{
			dev_err(&client->dev, "intensity i2c send failed\n");
			enable_irq(info->irq);
			return -1;
		}

		write_buf[0] = APS_REGH_CMD;
		write_buf[1] = APS_REGL_UCMD_RESULT_LENGTH;

		if(i2c_master_send(client,write_buf,2)!=2)
		{
			dev_err(&client->dev,"send : i2c failed\n");
			enable_irq(info->irq);
			return -1;
		}

		if(i2c_master_recv(client,read_buf,1)!=1)
		{
			dev_err(&client->dev,"recv1: i2c failed\n");
			enable_irq(info->irq);
			return -1;
		}

		nLength = read_buf[0];
		write_buf[0] = APS_REGH_CMD;
		write_buf[1] = APS_REGL_UCMD_RESULT;

		if(i2c_master_send(client,write_buf,2)!=2)
		{
			dev_err(&client->dev,"send : i2c failed\n");
			enable_irq(info->irq);
			return -1;
		}

		if(i2c_master_recv(client,read_buf,nLength)!=nLength)
		{
			dev_err(&client->dev,"recv2: i2c failed\n");
			enable_irq(info->irq);
			return -1;
		}

		nLength >>=1;
		for(row = 0 ; row <nLength ; row++)
		{
			nReference = (u16)(read_buf[2*row] | (read_buf[2*row+1] << 8));
			raw[col][row]   = nReference;
			printk("%d, \t", nReference);
		}
		printk("\n");
	}

	nIdx = 0;
	for(col = 0 ; col < info->col_num ; col++){
		for(row = 0 ; row < info->row_num ; row++){
			info->raw_data[nIdx++]=(char)((raw[col][row] &0xFF00)>>8);
			info->raw_data[nIdx++]=(char)( raw[col][row] &0xFF);
		}
	}

	enable_irq(info->irq);
	return 0;
}
static int get_dtxdata(struct aps_ts_info *info)
{
	struct i2c_client *client = info->client;

	int col, row;

	int nIdx;
	//u16* p;
	u8 write_buf[8];
	u8 read_buf[MAX_ROW*2];


	u8 nLength = 0;

	s16 nIntensity;

	disable_irq(info->irq);
	for(col = 0 ; col < info->col_num ; col++)
	{


#if KERNEL_LOG_MSG
		printk("[%2d]  ",col);
#endif

		write_buf[0] = APS_REGH_CMD;
		write_buf[1] = APS_REGL_UCMD;
		write_buf[2] = 0x74;
		write_buf[3] = 0xFF;
		write_buf[4] = col;

		if(i2c_master_send(client,write_buf,5)!=5)
		{
			dev_err(&client->dev, "intensity i2c send failed\n");
			enable_irq(info->irq);
			return -1;
		}

		write_buf[0] = APS_REGH_CMD;
		write_buf[1] = APS_REGL_UCMD_RESULT_LENGTH;

		if(i2c_master_send(client,write_buf,2)!=2)
		{
			dev_err(&client->dev,"send : i2c failed\n");
			enable_irq(info->irq);
			return -1;
		}

		if(i2c_master_recv(client,read_buf,1)!=1)
		{
			dev_err(&client->dev,"recv : i2c failed\n");
			enable_irq(info->irq);
			return -1;
		}

		nLength = read_buf[0];
		write_buf[0] = APS_REGH_CMD;
		write_buf[1] = APS_REGL_UCMD_RESULT;

		if(i2c_master_send(client,write_buf,2)!=2)
		{
			dev_err(&client->dev,"send : i2c failed\n");
			enable_irq(info->irq);
			return -1;
		}

		if(i2c_master_recv(client,read_buf,nLength)!=nLength)
		{
			dev_err(&client->dev,"recv : i2c failed\n");
			enable_irq(info->irq);
			return -1;
		}


		nLength >>=1;
		for(row = 0 ; row <nLength ; row++)
		{
			nIntensity = (read_buf[2*row] | (read_buf[2*row+1] << 8));
			raw[col][row]   = nIntensity;
			printk("%5d, \t", nIntensity);
		}
		printk("\n");
	}
	enable_irq(info->irq);
	nIdx = 0;
	for(col = 0 ; col < info->col_num ; col++){
		for(row = 0 ; row < info->row_num ; row++){
			info->raw_data[nIdx++]=(char)((raw[col][row] &0xFF00)>>8);
			info->raw_data[nIdx++]=(char)( raw[col][row] &0xFF);
		}
	}

	return 0;
}
static int aps_fs_open(struct inode *node, struct file *fp)
{
	struct aps_ts_info *info;
	struct i2c_client *client;
	struct i2c_msg msg;
	u8 buf[4] = {
		APS_REGH_CMD,
		APS_REGL_UCMD,
		0x20,
		true,
	};

	info = container_of(node->i_cdev, struct aps_ts_info, cdev);
	client = info->client;
	dev_dbg(&info->client->dev, "%s\n", __func__);

	disable_irq(info->irq);
	fp->private_data = info;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = buf;
	msg.len = sizeof(buf);

	i2c_transfer(client->adapter, &msg, 1);

	info->log.data = kzalloc(MAX_LOG_LENGTH * 20 + 5, GFP_KERNEL);

	aps_clear_input_data(info);

	return 0;
}

static int aps_fs_release(struct inode *node, struct file *fp)
{
	struct aps_ts_info *info = fp->private_data;
	struct i2c_client *client = info->client;
	struct i2c_msg msg;
	u8 buf[4] = {
		APS_REGH_CMD,
		APS_REGL_UCMD,
		0x20,
		false,
	};	
	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = buf;
	msg.len = sizeof(buf);
	
	
	dev_dbg(&info->client->dev, "%s\n", __func__);
	aps_clear_input_data(info);
	

	i2c_transfer(client->adapter, &msg, 1);
	kfree(info->log.data);
	enable_irq(info->irq);

	return 0;
}

static void aps_event_handler(struct aps_ts_info *info)
{
	struct i2c_client *client = info->client;
	u8 sz;
	int ret;
	int row_num;
	u8 reg[2] ={APS_REGH_CMD, APS_REGL_EVENT_PKT_SZ};
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = reg,
			.len = 2,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = info->log.data,
		},

	};
	struct aps_log_pkt {
		u8	marker;
		u8	log_info;
		u8	code;
		u8	element_sz;
		u8	row_sz;
		u8	rigth_shift;
	} __attribute__ ((packed)) *pkt = (struct aps_log_pkt *)info->log.data;

	dev_dbg(&info->client->dev, "%s\n", __func__);
	memset(pkt, 0, sizeof(*pkt));

	if (gpio_get_value(info->pdata->gpio_irq))
		return;

	if(i2c_master_send(client,reg,2)!=2)
		dev_err(&client->dev,"send : i2c failed\n");
	if(i2c_master_recv(client,&sz,1)!=1)
		dev_err(&client->dev,"recv : i2c failed\n");
	msg[1].len = sz;

	reg[1] = APS_REGL_INPUT_EVENT;
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev,
			"failed to read %d bytes of data\n",
			sz);
		return;
	}

	if ((pkt->marker & 0xf) == APS_ET_LOG) {
		if ((pkt->log_info & 0x7) == 0x1) {
			pkt->element_sz = 0;
			pkt->row_sz = 0;

			return;
		}

		switch (pkt->log_info >> 4) {
		case LOG_TYPE_U08:
		case LOG_TYPE_S08:
			msg[1].len = pkt->element_sz;
			break;
		case LOG_TYPE_U16:
		case LOG_TYPE_S16:
			msg[1].len = pkt->element_sz * 2;
			break;
		case LOG_TYPE_U32:
		case LOG_TYPE_S32:
			msg[1].len = pkt->element_sz * 4;
			break;
		default:
			dev_err(&client->dev, "invalied log type\n");
			return;
		}

		msg[1].buf = info->log.data + sizeof(struct aps_log_pkt);
		reg[1] = APS_REGL_UCMD_RESULT;
		row_num = pkt->row_sz ? pkt->row_sz : 1;

		while (row_num--) {
			while (gpio_get_value(info->pdata->gpio_irq))
				;
			ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
			msg[1].buf += msg[1].len;
		};
	} else {
		aps_report_input_data(info, sz, info->log.data);
		memset(pkt, 0, sizeof(*pkt));
	}

	return;
}


static void aps_report_input_data(struct aps_ts_info *info, u8 sz, u8 *buf)
{
	int i;
	struct i2c_client *client = info->client;
	int id;
	int x;
	int y;
	int touch_major;
	int pressure;
	u8 *tmp;

	if (buf[0] == APS_ET_NOTIFY) {
		dev_dbg(&client->dev, "TSP mode changed (%d)\n", buf[1]);
		goto out;
	} else if (buf[0] == APS_ET_ERROR) {
		dev_info(&client->dev, "Error detected, restarting TSP\n");
		aps_clear_input_data(info);
		goto out;
	}

	for (i = 0; i < sz; i += FINGER_EVENT_SZ) {
		tmp = buf + i;

		id = (tmp[0] & 0xf) -1;
		x = tmp[2] | ((tmp[1] & 0xf) << 8);
		y = tmp[3] | (((tmp[1] >> 4 ) & 0xf) << 8);
		touch_major = tmp[4];
		pressure = tmp[5];
			if (pressure == 0) {
				dev_info(&client->dev, "id:%d, x:%d, y:%d, tourch_major:%d, pressure:%d\n", id, x,y,touch_major,pressure);
			}
		input_mt_slot(info->input_dev, id);
			
		if (!(tmp[0] & 0x80)) {
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
			continue;
		}
		//printk("[TOUCH]%s finger = %d, x = %d, y = %d, width = %d, pressure = %d \n", __func__, id, x, y, touch_major, pressure);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, true);
		input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, touch_major);
		input_report_abs(info->input_dev, ABS_MT_PRESSURE, pressure);
		
	}

	input_sync(info->input_dev);

out:
	return;

}



static ssize_t aps_fs_read(struct file *fp, char *rbuf, size_t cnt, loff_t *fpos)
{
	struct aps_ts_info *info = fp->private_data;
	struct i2c_client *client = info->client;
	int ret = 0;
	dev_dbg(&info->client->dev, "%s\n", __func__);

	switch (info->log.cmd) {
	case GET_COL_NUM:
		ret = copy_to_user(rbuf, &info->col_num, 1);
		break;
	case GET_ROW_NUM:
		ret = copy_to_user(rbuf, &info->row_num, 1);
		break;
	case GET_EVENT_DATA:
		aps_event_handler(info);
		/* copy data without log marker */
		ret = copy_to_user(rbuf, info->log.data + 1, cnt);
		break;
	default:
		dev_err(&client->dev, "unknown command\n");
		ret = -EFAULT;
		break;
	}

	return ret;
}

static ssize_t aps_fs_write(struct file *fp, const char *wbuf, size_t cnt, loff_t *fpos)
{
	struct aps_ts_info *info = fp->private_data;
	struct i2c_client *client = info->client;
	u8 *buf;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = cnt,
	};
	int ret = 0;
	dev_dbg(&info->client->dev, "%s\n", __func__);

	mutex_lock(&info->lock);

	if (!info->enabled)
		goto tsp_disabled;
	
	msg.buf = buf = kzalloc(cnt + 1, GFP_KERNEL);

	if ((buf == NULL) || copy_from_user(buf, wbuf, cnt)) {
		dev_err(&client->dev, "failed to read data from user\n");
		ret = -EIO;
		goto out;
	}

	if (cnt == 1) {
		info->log.cmd = *buf;
	} else {
		if (i2c_transfer(client->adapter, &msg, 1) != 1) {
			dev_err(&client->dev, "failed to transfer data\n");
			ret = -EIO;
			goto out;
		}
	}

	ret = 0;

out:
	kfree(buf);
tsp_disabled:
	mutex_unlock(&info->lock);

	return ret;
}

static int aps_isp_entry(struct aps_ts_info *info){
	struct i2c_client *client = info->client;
	int cnt=0, ret = 0;
	u8 entry_a[6] = ISP_ENTRY1;
	u8 entry_b[6] = ISP_ENTRY2;
	u8 entry_c[6] = ISP_ENTRY3;

	dev_dbg(&info->client->dev, "%s\n", __func__);

retry_entry_a:
	if(i2c_master_send(client, entry_a, 6) != 6){
		if (cnt <50){
			cnt++;
			printk("%s, retry_entry_a, cnt=%d\n",__func__, cnt);
			usleep(500);
			goto retry_entry_a;
		}
		cnt=0;
		ret = -1;
		return ret;
	}
	usleep(500);
	cnt=0;
retry_entry_b:

	if(i2c_master_send(client, entry_b, 6) != 6){
		if (cnt <50){
			cnt++;
			printk("%s, retry_entry_b, cnt=%d\n",__func__, cnt);
			usleep(500);
			goto retry_entry_b;
		}
		cnt=0;
		ret = -1;
		return ret;
	}
	usleep(500);
	cnt=0;
retry_entry_c:
	if(i2c_master_send(client, entry_c, 6) != 6){
		if (cnt <50){
			cnt++;
			printk("%s, retry_entry_c, cnt=%d\n",__func__, cnt);
			usleep(500);
			goto retry_entry_c;
		}
		cnt=0;
		ret = -1;
		return ret;
	}
	usleep(500);
	return ret;
}

static int aps_isp_unlock(struct aps_ts_info *info){
	struct i2c_client *client = info->client;
	int cnt=0, ret = 0;
	u8 unlock_a[6] = ISP_UNLOCK1;
	u8 unlock_b[6] = ISP_UNLOCK2;
	u8 unlock_c[6] = ISP_UNLOCK3;

	dev_dbg(&info->client->dev, "%s\n", __func__);

retry_unlock_c:
	if(i2c_master_send(client, unlock_c, 6) != 6){
		if (cnt <100){
			cnt++;
			printk("%s, retry_unlock_c, cnt=%d\n",__func__, cnt);
			usleep(500);
			goto retry_unlock_c;
		}
		cnt=0;
		ret = -1;
		return ret;
	}
	cnt=0;
	usleep(500);
retry_unlock_a:
	if(i2c_master_send(client, unlock_a, 6) != 6){
		if (cnt <3){
			cnt++;
			printk("%s, retry_unlock_a, cnt=%d\n",__func__, cnt);
			usleep(500);
			goto retry_unlock_a;
		}
		cnt=0;
		ret = -1;
		return ret;
	}
	usleep(500);
	cnt=0;

retry_unlock_b:
	if(i2c_master_send(client, unlock_b, 6) != 6){
		if (cnt <3){
			cnt++;
			printk("%s, retry_unlock_b, cnt=%d\n",__func__, cnt);
			usleep(500);
			goto retry_unlock_b;
		}
		cnt=0;
		ret = -1;
		return ret;
	}
	usleep(500);
	cnt=0;

	return ret;
}
static int aps_isp_write_done(struct aps_ts_info *info){
	struct i2c_client *client = info->client;
	u8 cmd[2] = {0x00, 0x0C};
	u8 rbuf[4];
	int ret = 0;
	int i = 0;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = cmd,
			.len = 2,
		},{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rbuf,
			.len = 4,
		},
	};	
	
	dev_dbg(&info->client->dev, "%s\n", __func__);
	while(true){
		if(i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
			dev_err(&client->dev,"isp write done i2c_transfer failed\n");
			ret = -1;
			return ret;
		}
		if(rbuf[3] & 0x20){
			break;
		}
		dev_err(&client->dev," write done while\n");

		if (i++ > 1000){
			dev_err(&client->dev,"isp erase wait-done failed\n");
			ret = -1;
			return ret;
		}
	}	
	return ret;
}
static int aps_isp_clean(struct aps_ts_info *info){
	struct i2c_client *client = info->client;
	int ret = 0;
	u8 clr_done[6] =  ISP_CLEAR_DONE;
	
	dev_dbg(&info->client->dev, "%s\n", __func__);
	if( i2c_master_send(client,clr_done, 6)!= 6){
		dev_err(&client->dev,"isp clr_done i2c_master_send failed\n");
		ret = -1;
		return ret;
	}
	return ret;
}
static int aps_isp_erase(struct aps_ts_info *info){
	struct i2c_client *client = info->client;
	u8 erase[6] = ISP_FULL_ERASE;
	int ret = 0;

	dev_dbg(&info->client->dev, "%s\n", __func__);
	if(i2c_master_send(client, erase,6)!=6){
		dev_err(&client->dev,"isp erase i2c_transfer failed\n");
		ret = -1;
		return ret;
	}

	ret = aps_isp_write_done(info);

	ret = aps_isp_clean(info);
	
	return ret;	
}

static int aps_isp_lock(struct aps_ts_info *info){
	struct i2c_client *client = info->client;
	u8 buf[6] = ISP_LOCK;
	int ret = 0;
	dev_dbg(&info->client->dev, "%s\n", __func__);
	if(i2c_master_send(client, buf, 6)!=6){
		ret = -1;
		dev_err(&client->dev,"isp fw-addr i2c_master_send failed\n");
		return ret;
	}
	return ret;	
}

static int aps_isp_addr(struct aps_ts_info *info, const size_t addr){
	struct i2c_client *client = info->client;
	int ret = 0;
	u8 wbuf_addr[6];
	wbuf_addr[0] = 0x00;
	wbuf_addr[1] = 0x14;
	wbuf_addr[2] = (addr>>24)&0xFF;
	wbuf_addr[3] = (addr>>16)&0xFF;
	wbuf_addr[4] = (addr>>8)&0xFF;
	wbuf_addr[5] = (addr>>0)&0xFF;
	
	dev_dbg(&info->client->dev, "%s\n", __func__);
	if(i2c_master_send(client, wbuf_addr, 6)!=6){
		ret = -1;
		dev_err(&client->dev,"isp fw-addr i2c_master_send failed\n");
		return ret;
	}
	
	return ret;	
}

static int aps_flash_data(struct aps_ts_info *info, const u8 *data, const size_t addr){
	struct i2c_client *client = info->client;
	int ret = 0;
	u8 write_mode[6] = ISP_WIRTE_MODE;
	u8 write_cmd[6] = ISP_WIRTE;
	u8 wbuf_a[6] = {0x00, 0x28, };
	u8 wbuf_b[6] = {0x00, 0x2C, };
	
	dev_dbg(&info->client->dev, "%s\n", __func__);
	if(i2c_master_send(client, write_mode, 6)!=6){
		dev_err(&client->dev,"isp write mode i2c_master_send failed\n");
		ret = -1;
		return ret;
	}
	
	wbuf_a[2] = data[3];
	wbuf_a[3] = data[2];
	wbuf_a[4] = data[1];
	wbuf_a[5] = data[0];
	
	if(i2c_master_send(client, wbuf_a, 6)!=6){
		ret = -1;
		dev_err(&client->dev,"isp fw-data(a) i2c_master_send failed\n");
		return ret;
	}
	
	wbuf_b[2] = data[7];
	wbuf_b[3] = data[6];
	wbuf_b[4] = data[5];
	wbuf_b[5] = data[4];
	
	if(i2c_master_send(client, wbuf_b, 6)!=6){
		ret = -1;
		dev_err(&client->dev,"isp fw-data(b) i2c_master_send failed\n");
		return ret;
	}
	
	ret = aps_isp_addr(info, addr);
	
	if(i2c_master_send(client, write_cmd, 6)!=6){
		dev_err(&client->dev,"isp write cmd i2c_master_send failed\n");
		ret = -1;
		return ret;
	}

	ret = aps_isp_write_done(info);
	ret = aps_isp_clean(info);
	return ret;
}
#if 0
static int aps_read_data(struct aps_ts_info *info){
	struct i2c_client *client = info->client;
	int ret = 0;
	u8 read_a[2] = {0x00, 0x0C};
	//u8 read_b[2] = {0x00, 0x34};
	u8 rbuf[4];
	u8 r_cmp[8];
	//u8 read_mode[6] = ISP_READ_MOED;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = read_a,
			.len =2,
		},{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rbuf,
			.len = 4,
		},
	};

	if(i2c_transfer(client->adapter,msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
		ret= -1;
		dev_err(&client->dev,"isp read data(a) i2c_master_send failed\n");
		return ret;
	}
	r_cmp[0] = rbuf[3];
	r_cmp[1] = rbuf[2];
	r_cmp[2] = rbuf[1];
	r_cmp[3] = rbuf[0];

	printk("%s, read data - 0x%x, 0x%x, 0x%x, 0x%x\n",__func__, rbuf[0], rbuf[1], rbuf[2], rbuf[3]);

	return 0;

}

#endif

static int aps_verify_data(struct aps_ts_info *info, const u8 *data, const size_t addr){
	struct i2c_client *client = info->client;
	int ret = 0;
	u8 read_a[2] = {0x00, 0x30};
	u8 read_b[2] = {0x00, 0x34};
	u8 rbuf[4];
	u8 r_cmp[8];
	u8 read_mode[6] = ISP_READ_MOED;
	u8 read_cmd[6] = ISP_READ;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = read_a,
			.len =2,
		},{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rbuf,
			.len = 4,
		},
	};
	dev_dbg(&info->client->dev, "%s\n", __func__);
	if(i2c_master_send(client, read_mode, 6)!=6){
		ret= -1;
		dev_err(&client->dev,"isp read mode i2c_master_send failed\n");
		return ret;
	}
	ret = aps_isp_addr(info, addr);
	if(i2c_master_send(client, read_cmd, 6)!=6){
		ret= -1;
		dev_err(&client->dev,"isp read cmd i2c_master_send failed\n");
		return ret;
	}
	if(i2c_transfer(client->adapter,msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
		ret= -1;
		dev_err(&client->dev,"isp read data(a) i2c_master_send failed\n");
		return ret;		
	}
	r_cmp[0] = rbuf[3];
	r_cmp[1] = rbuf[2];
	r_cmp[2] = rbuf[1];
	r_cmp[3] = rbuf[0];
	
	msg[0].buf = read_b;
	
	if(i2c_transfer(client->adapter,msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
		ret= -1;
		dev_err(&client->dev,"isp read data(b) i2c_master_send failed\n");
		return ret;		
	}	


	r_cmp[4] = rbuf[3];
	r_cmp[5] = rbuf[2];
	r_cmp[6] = rbuf[1];
	r_cmp[7] = rbuf[0];
	
	ret = aps_isp_write_done(info);
			
	if(memcmp(data,r_cmp, 8)){
		ret =-1;
		return ret;
	}
	
	ret = aps_isp_clean(info);
	return ret;
}

static int aps_fw_flash(const struct firmware *fw, struct aps_ts_info *info){
	u8 *fw_data;
	int addr, prgs;
	int ret = 0;
	int fw_size = (int)fw->size;
	fw_data=kmalloc(0x10000, GFP_KERNEL);
	disable_irq(info->irq);
	memset(fw_data,0xff, 0x10000);
	memcpy(fw_data,fw->data,fw->size);
	if((fw_size % 8 )!=0)
		fw_size = fw_size + ( 8 - ( fw_size % 8 ));
			
	if(aps_isp_entry(info)){
		dev_err(&info->client->dev,"isp entry failed\n");
		ret = -1;
		goto out;
	}
	
	if(aps_isp_unlock(info)){
		dev_err(&info->client->dev,"isp unlock failed\n");
		ret = -1;
		goto out;
	}
	
	if(aps_isp_erase(info)){
		dev_err(&info->client->dev,"isp erase failed\n");
		ret = -1;
		goto out;
	}

	for(addr = fw_size - 8 ; addr >= 0 ; addr -= 8){
		if(((fw_size - 8 - addr)%4096)==0){
			prgs=100 - ((100*addr)/(fw_size - 8));
			dev_info(&info->client->dev,"%s, Melfas Firmware flashing is on progress. %d%%. \n", __func__,prgs);
			}
		if(aps_flash_data(info,(u8 *)&fw_data[addr], addr )){
			dev_err(&info->client->dev,"isp flash failed\n");
			ret = -1;
			goto out;
		}
		if(aps_verify_data(info,(u8 *)&fw_data[addr], addr )){
			dev_err(&info->client->dev,"isp verify failed\n");
			ret = -1;
			goto out;
		}
	}
	
	ret = aps_isp_lock(info);
	
out:
	kfree(fw_data);
	release_firmware(fw);
	enable_irq(info->irq);
	return ret;
}

static void aps_fw_update_controller(const struct firmware *fw, void * context)
{
	struct aps_ts_info *info = context;
	int ret;

	dev_info(&info->client->dev, "%s\n", __func__);
	ret = aps_fw_flash(fw,info);
	if(ret){
		dev_err(&info->client->dev, "firmware flash failed");
	}else{
		dev_dbg(&info->client->dev, "firmware flash succeed");
	}
	gpio_direction_output(info->pdata->gpio_reset, 0);
	msleep(10);
	gpio_direction_output(info->pdata->gpio_reset, 1);
	msleep(10);
	dev_info(&info->client->dev, "%s, Melfas firmware flash DONE. 100%%.\n", __func__);
	return;
}
static ssize_t aps_start_isp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aps_ts_info *info = dev_get_drvdata(dev);

	const char *fw_name = FW_NAME;
	int ret = 0;
	
	dev_dbg(&info->client->dev, "buf data = %s\n",fw_name);
	info->fw_name = kstrdup(fw_name, GFP_KERNEL);

	ret = request_firmware_nowait(THIS_MODULE, true, fw_name, &info->client->dev,
			GFP_KERNEL, info,	aps_fw_update_controller);

	if (ret) {
		dev_err(&info->client->dev, "failed to schedule firmware update\n");
		return -EIO;
	}

	kfree(info->fw_name);
	return 1;
}

static ssize_t aps_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	//struct aps_ts_info *info = dev_get_drvdata(dev);

	return count;
}

static DEVICE_ATTR(aps_isp, 0666, aps_start_isp, aps_store);

static struct attribute *aps_attrs[] = {
	&dev_attr_aps_isp.attr,
	NULL,
};

static const struct attribute_group aps_attr_group = {
	.attrs = aps_attrs,
};



static irqreturn_t aps_ts_interrupt(int irq, void *dev_id)
{
	struct aps_ts_info *info = dev_id;
	struct i2c_client *client = info->client;
	u8 buf[MAX_FINGER_NUM * FINGER_EVENT_SZ] = { 0, };
	int ret;
	u8 sz=0;
	u8 reg[2] = {APS_REGH_CMD, APS_REGL_INPUT_EVENT};
	u8 cmd[2] = {APS_REGH_CMD, APS_REGL_EVENT_PKT_SZ};
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = cmd,
			.len = 2,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = &sz,
		},
	};
	
	msg[1].len = 1;
	i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		
	msg[0].buf = reg;
	msg[1].buf = buf;
	msg[1].len = sz;
	
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));

	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev,
			"failed to read %d bytes of touch data (%d)\n",
			sz, ret);
	} else {
		aps_report_input_data(info, sz, buf);
	}

	return IRQ_HANDLED;
}

static int aps_ts_input_open(struct input_dev *dev)
{
	struct aps_ts_info *info = input_get_drvdata(dev);
	int ret;

	dev_dbg(&info->client->dev, "%s\n", __func__);
	ret = wait_for_completion_interruptible_timeout(&info->init_done,
			msecs_to_jiffies(90 * MSEC_PER_SEC));

	if (ret > 0) {
		if (info->irq != -1) {
			aps_ts_enable(info);
			ret = 0;
		} else {
			ret = -ENXIO;
		}
	} else {
		dev_err(&dev->dev, "error while waiting for device to init\n");
		ret = -ENXIO;
	}

	return ret;
}

static void aps_ts_input_close(struct input_dev *dev)
{
	struct aps_ts_info *info = input_get_drvdata(dev);

	aps_ts_disable(info);
}

static int aps_ts_config(struct aps_ts_info *info)
{
	struct i2c_client *client = info->client;
	int ret;
	u8 cmd[2] = {APS_REGH_CMD, APS_REGL_ROW_NUM};
	u8 num[2];
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = cmd,
			.len = 2,
		},{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = num,
			.len = 2,
		},
	};

	dev_dbg(&info->client->dev, "%s\n", __func__);
	//info->irq=gpio_to_irq(info->pdata->gpio_irq);
	ret = request_threaded_irq(client->irq, NULL, aps_ts_interrupt,
				(int)info->pdata->irq_flags,
				LGD_TOUCH_NAME, info);

	if (ret) {
		dev_err(&client->dev, "failed to register irq\n");
		goto out;
	}

	disable_irq(client->irq);
	info->irq = client->irq;
	barrier();

	ret=i2c_transfer(client->adapter,msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev,
			"failed to read bytes of touch data (%d)\n",
			 ret);
	}
	info->row_num = 20;//num[0];
	info->col_num = 12;//num[1];
	printk("row  %d\n",info->row_num);
	printk("col  %d\n",info->col_num);
	dev_dbg(&client->dev, "APS touch controller initialized\n");

	complete_all(&info->init_done);

out:
	return ret;
}
static ssize_t bin_report_read(struct file *fp, struct kobject *kobj, struct bin_attribute *attr,
                                char *buf, loff_t off, size_t count)
{
        struct device *dev = container_of(kobj, struct device, kobj);
        struct i2c_client *client = to_i2c_client(dev);
        struct aps_ts_info *info = i2c_get_clientdata(client);

	u8 result;
	result = 1;
	count = 0;
	
	dev_dbg(&info->client->dev, "%s\n", __func__);
	switch(info->data_cmd){
	case SYS_INTENSITY:
		dev_info(&info->client->dev, "Intensity Test\n");
		if(get_intensity(info)!=0){
			dev_err(&info->client->dev, "Intensity Test failed\n");
			return -1;
		}
		count = (info->row_num * 2) * info->col_num;
		dev_info(&info->client->dev, "%d",count);
		memcpy(buf,info->cm_intensity,count);
		info->data_cmd=0;
		break;
	case SYS_ROWNUM:
		dev_info(&info->client->dev, "row send %d \n",info->row_num);
		buf[0]=info->row_num;
		count =1;
		info->data_cmd=0;
		break;
	case SYS_COLNUM:
		dev_info(&info->client->dev, "cloumn send%d\n", info->col_num);
		buf[0]=info->col_num;
		count =1;
		info->data_cmd=0;
		break;
	case SYS_CLEAR:
		dev_info(&info->client->dev, "Input clear\n");
		aps_clear_input_data(info);
		info->data_cmd=0;
		break;
	case SYS_ENABLE:
		dev_info(&info->client->dev, "enable_irq  \n");
		enable_irq(info->irq);
		info->data_cmd=0;
		break;
	case SYS_DISABLE:
		dev_info(&info->client->dev, "disable_irq  \n");
		disable_irq(info->irq);
		info->data_cmd=0;
		break;
	}
	return count;
}

static ssize_t bin_report_write(struct file *fp, struct kobject *kobj, struct bin_attribute *attr,
                                char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct i2c_client *client = to_i2c_client(dev);
	struct aps_ts_info *info = i2c_get_clientdata(client);
	dev_info(&info->client->dev, "%s\n", __func__);

	info->data_cmd=(int)buf[0];
	return count;
        
}

static struct bin_attribute bin_attr_data = {
        .attr = {
                .name = "report_data",
                .mode = S_IRWXUGO,
        },
        .size = PAGE_SIZE,
        .read = bin_report_read,
        .write = bin_report_write,
};

static ssize_t bin_ref_read(struct file *fp, struct kobject *kobj, struct bin_attribute *attr,
                                char *buf, loff_t off, size_t count)
{
        struct device *dev = container_of(kobj, struct device, kobj);
        struct i2c_client *client = to_i2c_client(dev);
        struct aps_ts_info *info = i2c_get_clientdata(client);

		switch(info->raw_cmd)
		{
		case '1':

			get_dtxdata(info);
			count = (info->row_num  * info->col_num) * sizeof(u16);
			dev_dbg(&info->client->dev, "%d",count);
			memcpy(buf,info->raw_data , count);
			info->raw_cmd = '0';
			break;
		case '2':
			get_rawdata(info);
			count = (info->row_num  * info->col_num) * sizeof(u16);
			dev_dbg(&info->client->dev, "%d",count);
			memcpy(buf,info->raw_data , count);
			info->raw_cmd = '0';
			break;
		default:
			break;
		}

	return count;
}

static ssize_t bin_ref_write(struct file *fp, struct kobject *kobj, struct bin_attribute *attr,
                                char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct i2c_client *client = to_i2c_client(dev);
	struct aps_ts_info *info = i2c_get_clientdata(client);

	info->raw_cmd=(char)buf[0];

	return count;
}
static struct bin_attribute bin_attr_rawdata = {
        .attr = {
                .name = "raw_data",
                .mode = S_IRWXUGO,
        },
        .size = PAGE_SIZE,
        .read = bin_ref_read,
        .write = bin_ref_write,
};

static struct file_operations aps_fops = {
	.owner		= THIS_MODULE,
	.open		= aps_fs_open,
	.release	= aps_fs_release,
	.read		= aps_fs_read,
	.write		= aps_fs_write,
};

static int lgd_incell_parse_dt(struct device *dev,
				struct aps_ts_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;//, num_buttons;
	int rc;

	printk("[TOUCH]*******%s\n",__func__);


	rc = of_property_read_u32(np, "lgd_melfas,max_x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read max_x\n");
		return rc;
	} else {
		pdata->max_x = temp_val;
	}

	rc = of_property_read_u32(np, "lgd_melfas,max_y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read max_y\n");
		return rc;
	} else {
		pdata->max_y = temp_val;
	}

/*	rc = of_property_read_u32(np, "lgd,id_min", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read id_min\n");
		return rc;
	} else {
		pdata->id_min = temp_val;
	}

	rc = of_property_read_u32(np, "lgd,max_fingers", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read max_fingers\n");
		return rc;
	} else {
		pdata->max_fingers = temp_val;
	}
*/
	/* irq gpio info */

	pdata->gpio_irq = of_get_named_gpio_flags(np,
			"lgd_melfas,irq_gpio", 0, &pdata->irq_flags);

	printk("[TOUCH]gpio_irq %d\n", pdata->gpio_irq);
	printk("[TOUCH]irq_flags %d\n", (int)pdata->irq_flags);

	pdata->gpio_reset = of_get_named_gpio_flags(np,
			"lgd_melfas,reset_gpio", 0, &pdata->reset_flags);

	printk("[TOUCH]gpio_reset %d\n", pdata->gpio_reset);
	printk("[TOUCH]reset_flags %d\n", (int)pdata->reset_flags);

	pdata->name = LGD_TOUCH_NAME;
	
#if 0
	prop = of_find_property(np, "synaptics,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);

		rmi4_pdata->capacitance_button_map = devm_kzalloc(dev,
			sizeof(*rmi4_pdata->capacitance_button_map),
			GFP_KERNEL);
		if (!rmi4_pdata->capacitance_button_map)
			return -ENOMEM;

		rmi4_pdata->capacitance_button_map->map = devm_kzalloc(dev,
			sizeof(*rmi4_pdata->capacitance_button_map->map) *
			MAX_NUMBER_OF_BUTTONS, GFP_KERNEL);
		if (!rmi4_pdata->capacitance_button_map->map)
			return -ENOMEM;

		if (num_buttons <= MAX_NUMBER_OF_BUTTONS) {
			rc = of_property_read_u32_array(np,
				"synaptics,button-map", button_map,
				num_buttons);
			if (rc) {
				dev_err(dev, "Unable to read key codes\n");
				return rc;
			}
			for (i = 0; i < num_buttons; i++)
				rmi4_pdata->capacitance_button_map->map[i] =
					button_map[i];
			rmi4_pdata->capacitance_button_map->nbuttons =
				num_buttons;
		} else {
			return -EINVAL;
		}
	}
#endif 
	return 0;
}

static int __devinit aps_ts_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct aps_ts_info *info;
	struct input_dev *input_dev;
	struct aps_ts_platform_data *platform_data = client->dev.platform_data;
	
	int ret = 0;
	int result;
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!info || !input_dev) {
		dev_err(&client->dev, "Failed to allocated memory\n");
		return -ENOMEM;
	}

	info->client = client;
	info->input_dev = input_dev;
	info->pdata = client->dev.platform_data;
	init_completion(&info->init_done);
	info->irq = -1;
	
	printk("[TOUCH]%s start\n",__func__);

	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
			sizeof(*platform_data),
			GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = lgd_incell_parse_dt(&client->dev, platform_data);
		if (ret)
			return ret;
	} else {
		info->pdata = client->dev.platform_data;
	}

	info->pdata = platform_data;



	mutex_init(&info->lock);

	input_mt_init_slots(input_dev, MAX_FINGER_NUM);

	snprintf(info->phys, sizeof(info->phys),
		"%s/input0", dev_name(&client->dev));

	input_dev->name = "aps_ts";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = aps_ts_input_open;
	input_dev->close = aps_ts_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_WIDTH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, MAX_PRESSURE, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, info->pdata->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, info->pdata->max_y, 0, 0);

	input_set_drvdata(input_dev, info);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register input dev\n");
		return -EIO;
	}

	//Added reset pin high.	
	ret = gpio_request(info->pdata->gpio_reset, "touch_reset");	
	if(ret < 0){		
		printk(KERN_ERR "[TOUCH]can't request qpio!\n");	
	}	
	gpio_direction_output(info->pdata->gpio_reset, 1);
	msleep(10);

	printk("[TOUCH]%s i2c_set_clientdata\n",__func__);
	i2c_set_clientdata(client, info);

	aps_ts_config(info);

#ifdef CONFIG_FB
	info->fb_notifier.notifier_call = aps_ts_fb_notifier_call;
	ret = fb_register_client(&info->fb_notifier);
	if (ret) {
		dev_err(&client->dev, "%s: failed to register fb_notifier: %d\n", __func__, ret);
	}
#endif

	if (alloc_chrdev_region(&info->aps_dev, 0, 1, "aps_ts")) {
		dev_err(&client->dev, "failed to allocate device region\n");
		return -ENOMEM;
	}

	cdev_init(&info->cdev, &aps_fops);
	info->cdev.owner = THIS_MODULE;

	if (cdev_add(&info->cdev, info->aps_dev, 1)) {
		dev_err(&client->dev, "failed to add ch dev\n");
		return -EIO;
	}

	info->class = class_create(THIS_MODULE, "aps_ts");
	device_create(info->class, NULL, info->aps_dev, NULL, "aps_ts");
	result = sysfs_create_bin_file(&client->dev.kobj ,&bin_attr_data);
	result = sysfs_create_bin_file(&client->dev.kobj ,&bin_attr_rawdata);
	if (sysfs_create_link(NULL, &client->dev.kobj, "aps_ts")) {
		dev_err(&client->dev, "failed to create sysfs symlink\n");
		return -EAGAIN;
	}

	if (sysfs_create_group(&client->dev.kobj, &aps_attr_group)) {
		dev_err(&client->dev, "failed to create sysfs group\n");
		return -EAGAIN;
	}
	dev_notice(&client->dev, "aps dev initialized\n");
	info->cm_intensity = kzalloc(2048 , GFP_KERNEL);
	info->raw_data = kzalloc(MAX_COL*MAX_ROW * sizeof(u16) , GFP_KERNEL);
	return 0;
}

static int __devexit aps_ts_remove(struct i2c_client *client)
{
	struct aps_ts_info *info = i2c_get_clientdata(client);

	if (info->irq >= 0)
		free_irq(info->irq, info);
	sysfs_remove_bin_file(&client->dev.kobj, &bin_attr_rawdata);
	sysfs_remove_bin_file(&client->dev.kobj, &bin_attr_data);		
	sysfs_remove_group(&info->client->dev.kobj, &aps_attr_group);
	sysfs_remove_link(NULL, "aps_ts");
	input_unregister_device(info->input_dev);
#ifdef CONFIG_FB
	fb_unregister_client(&info->fb_notifier);
#endif

	device_destroy(info->class, info->aps_dev);
	class_destroy(info->class);
	kfree(info->raw_data);
	kfree(info->cm_intensity);
	kfree(info->fw_name);
	kfree(info);

	return 0;
}

static int aps_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aps_ts_info *info = i2c_get_clientdata(client);

	dev_info(&info->client->dev, "%s\n", __func__);
	mutex_lock(&info->input_dev->mutex);

	if (info->input_dev->users) {
		aps_ts_disable(info);
		aps_clear_input_data(info);
	}

	mutex_unlock(&info->input_dev->mutex);
	gpio_direction_output(info->pdata->gpio_reset, 0);
	return 0;

}

static int aps_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aps_ts_info *info = i2c_get_clientdata(client);

	dev_info(&info->client->dev, "%s\n", __func__);
	gpio_direction_output(info->pdata->gpio_reset, 1);
	//msleep(10);
	mutex_lock(&info->input_dev->mutex);

	if (info->input_dev->users)
		aps_ts_enable(info);

	mutex_unlock(&info->input_dev->mutex);

	return 0;
}
#ifdef CONFIG_FB
static int aps_ts_fb_notifier_call(struct notifier_block *self,
				   unsigned long event,
				   void *data)
{
	struct fb_event *evdata = data;
	int *fb;
	struct aps_ts_info *info = container_of(self, struct aps_ts_info, fb_notifier);
	dev_info(&info->client->dev, "%s\n", __func__);
	if(evdata && evdata->data && event == FB_EVENT_BLANK && info && info->client) {
		fb = evdata->data;
		switch (*fb) {
			case FB_BLANK_UNBLANK:
				aps_ts_resume(&info->client->dev);
				break;
			case FB_BLANK_POWERDOWN:
				aps_ts_suspend(&info->client->dev);
				break;
			default:
				break;
		}
	}
	return 0;
}
#endif

static const struct i2c_device_id aps_ts_id[] = {
	{"aps_ts", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, aps_ts_id);

static struct of_device_id lgd_match_table[] = {
	{ .compatible = "lgd_melfas,incell",},
	{ },};

static struct i2c_driver aps_ts_driver = {
	.probe		= aps_ts_probe,
	.remove		= __devexit_p(aps_ts_remove),
	.driver		= {
				.name	= "aps_ts",
				.of_match_table = lgd_match_table,
	},
	.id_table	= aps_ts_id,
};

static int __init aps_ts_init(void)
{
	return i2c_add_driver(&aps_ts_driver);
}

static void __exit aps_ts_exit(void)
{
	return i2c_del_driver(&aps_ts_driver);
}

module_init(aps_ts_init);
module_exit(aps_ts_exit);

MODULE_VERSION("0.1");
MODULE_DESCRIPTION("APS Touchscreen driver");
MODULE_LICENSE("GPL");

