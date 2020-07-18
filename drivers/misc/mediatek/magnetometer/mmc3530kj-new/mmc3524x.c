
/* mmc3524x.c - mmc3524x compass driver
 *
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
 */


/*-------------------------MT6516&MT6573 define-------------------------------*/

#define POWER_NONE_MACRO MT65XX_POWER_NONE
#define MTK_I2C_FUNCTION
#include <cust_mag.h>
#include "mmc3524x.h"
#include <hwmsen_helper.h>
#include "mag.h"
/*----------------------------------------------------------------------------*/
#define DEBUG 0
#define MMC3524X_DEV_NAME         "mmc3524x"
#define DRIVER_VERSION          "1.0.0"
/*----------------------------------------------------------------------------*/
#define MMC3524X_DEBUG		1
#define MMC3524X_DEBUG_MSG	1
#define MMC3524X_DEBUG_FUNC	1
#define MMC3524X_DEBUG_DATA	1
#define MAX_FAILURE_COUNT	3
#define MMC3524X_RETRY_COUNT	3
#define MMC3524X_DEFAULT_DELAY	100
#define MMC3524X_BUFSIZE  0x20

#if MMC3524X_DEBUG_MSG
#define MMCDBG(format, ...)	printk(KERN_INFO "mmc3524x " format "\n", ## __VA_ARGS__)
#else
#define MMCDBG(format, ...)
#endif

#if MMC3524X_DEBUG_FUNC
#define MMCFUNC(func) printk(KERN_INFO "mmc3524x " func " is called\n")
#else
#define MMCFUNC(func)
#endif


//extern struct mag_hw* get_cust_mag_hw(void);

static struct i2c_client *this_client = NULL;
#if 0
extern struct platform_driver m_sensor_driver ;
extern bool m_sensor_deteced ;
#endif

// calibration msensor and orientation data
static int sensor_data[CALIBRATION_DATA_SIZE];
static struct mutex sensor_data_mutex;
static struct mutex read_i2c_xyz;
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static int mmcd_delay = MMC3524X_DEFAULT_DELAY;

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id mmc3524x_i2c_id[] = {{MMC3524X_DEV_NAME,0},{}};
struct i2c_board_info __initdata i2c_mmc3524x={ I2C_BOARD_INFO("mmc3524x", (0x60>>1))};

/*the adapter id will be available in customization*/
//static unsigned short mmc3524x_force[] = {0x00, MMC3524x_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const mmc3524x_forces[] = { mmc3524x_force, NULL };
//static struct i2c_client_address_data mmc3524x_addr_data = { .forces = mmc3524x_forces,};
/*----------------------------------------------------------------------------*/
static int mmc3524x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mmc3524x_i2c_remove(struct i2c_client *client);
//static int mmc3524x_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int platform_mmc3524x_probe(struct platform_device *pdev);
static int platform_mmc3524x_remove(struct platform_device *pdev);
static int mmc3524x_suspend(struct i2c_client *client, pm_message_t msg);
static int mmc3524x_resume(struct i2c_client *client);
/*----------------------------------------------------------------------------*/
typedef enum {
	MMC_FUN_DEBUG  = 0x01,
	MMC_DATA_DEBUG = 0X02,
	MMC_HWM_DEBUG  = 0X04,
	MMC_CTR_DEBUG  = 0X08,
	MMC_I2C_DEBUG  = 0x10,
} MMC_TRC;

/* Define Delay time */
#define MMC3524X_DELAY_TM		10	/* ms */
#define MMC3524X_DELAY_SET		50	/* ms */
#define MMC3524X_DELAY_RESET	50  /* ms */
#define MMC3524X_DELAY_STDN		1	/* ms */

#define MMC3524X_RESET_INTV		250
static u32 read_idx = 0;
#define READMD	0

/*----------------------------------------------------------------------------*/
struct mmc3524x_i2c_data {
    struct i2c_client *client;
    struct mag_hw *hw;
    atomic_t layout;
    atomic_t trace;
	struct hwmsen_convert   cvt;
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};

static int  mmc3524x_local_init(void);
static int mmc3524x_remove(void);

static int mmc3524x_init_flag = 0; // 0<==>OK -1 <==> fail

/* Maintain  cust info here */
struct mag_hw mag_cust;
static struct mag_hw *hw = &mag_cust;

/* For  driver get cust info */
struct mag_hw *get_cust_mag(void)
{
	return &mag_cust;
}


static struct mag_init_info mmc3524x_init_info = {
	 .name = "mmc3524x",
	 .init = mmc3524x_local_init,
	 .uninit = mmc3524x_remove,
};


#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,msensor"},
	{},
};
#endif


//add by wangxiqiang
#ifdef CONFIG_SLT_DRV_DEVINFO_SUPPORT
#define SLT_DEVINFO_MAGN_DEBUG
#include  <linux/dev_info.h>
//u8 ver_id;
//u8 ver_module;
//static int devinfo_first=0;
//static char* temp_ver;
static char* temp_comments;
struct devinfo_struct *s_DEVINFO_magn;   //suppose 10 max lcm device 
//The followd code is for GTP style
static void devinfo_ctp_regchar(char *module,char * vendor,char *version,char *used, char* comments)
{

	s_DEVINFO_magn =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);	
	s_DEVINFO_magn->device_type="magnetometer";
	s_DEVINFO_magn->device_module=module;
	s_DEVINFO_magn->device_vendor=vendor;
	s_DEVINFO_magn->device_ic="MMC3530KJ";
	s_DEVINFO_magn->device_info=DEVINFO_NULL;
	s_DEVINFO_magn->device_version=version;
	s_DEVINFO_magn->device_used=used;
	s_DEVINFO_magn->device_comments=comments;
#ifdef SLT_DEVINFO_MAGN_DEBUG
		printk("[DEVINFO accel sensor]registe CTP device! type:<%s> module:<%s> vendor<%s> ic<%s> version<%s> info<%s> used<%s> comment<%s>\n",
				s_DEVINFO_magn->device_type,s_DEVINFO_magn->device_module,s_DEVINFO_magn->device_vendor,
				s_DEVINFO_magn->device_ic,s_DEVINFO_magn->device_version,s_DEVINFO_magn->device_info,s_DEVINFO_magn->device_used,s_DEVINFO_magn->device_comments);
#endif
       DEVINFO_CHECK_DECLARE_COMMEN(s_DEVINFO_magn->device_type,s_DEVINFO_magn->device_module,s_DEVINFO_magn->device_vendor,s_DEVINFO_magn->device_ic,s_DEVINFO_magn->device_version,s_DEVINFO_magn->device_info,s_DEVINFO_magn->device_used,s_DEVINFO_magn->device_comments);
      //devinfo_check_add_device(s_DEVINFO_ctp);

}
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_driver mmc3524x_i2c_driver = {
    .driver = {
     //   .owner = THIS_MODULE,
        .name  = MMC3524X_DEV_NAME,
	#ifdef CONFIG_OF
	.of_match_table = mag_of_match,
	#endif
    },
	.probe      = mmc3524x_i2c_probe,
	.remove     = mmc3524x_i2c_remove,
	//.detect     = mmc3524x_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend    = mmc3524x_suspend,
	.resume     = mmc3524x_resume,
#endif
	.id_table = mmc3524x_i2c_id,
	//.address_data = &mmc3524x_addr_data,
};

/*----------------------------------------------------------------------------*/

 struct platform_driver mmc3524x_platform_driver = {
	.probe      = platform_mmc3524x_probe,
	.remove     = platform_mmc3524x_remove,
	.driver     = {
		.name  = "msensor",
		//.owner = THIS_MODULE,
	}
};

/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;
/*
static int mmc3524x_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];
	int res = 0;
	u8 addr = MMC3524X_REG_CTRL;
	//struct mmc3524x_i2c_data *obj = i2c_get_clientdata(client);

	if(hwmsen_read_byte(client, addr, databuf))
	{
		printk("mmc3524x: read power ctl register err and retry!\n");
		if(hwmsen_read_byte(client, addr, databuf))
	    {
		   printk("mmc3524x: read power ctl register retry err!\n");
		   return -1;
	    }
	}

	databuf[0] &= ~MMC3524X_CTRL_TM;

	if(enable == true)
	{
		databuf[0] |= MMC3524X_CTRL_TM;
	}
	else
	{
		// do nothing
	}
	databuf[1] = databuf[0];
	databuf[0] = MMC3524X_REG_CTRL;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		printk("mmc3524x: set power mode failed!\n");
		return -1;
	}
	else
	{
		printk("mmc3524x: set power mode ok %x!\n", databuf[1]);
	}

	return 0;
}
*/
/*----------------------------------------------------------------------------*/
static void mmc3524x_power(struct mag_hw *hw, unsigned int on)
{

}


static DEFINE_MUTEX(mmc3524_i2c_mutex);
static int mag_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };

	mutex_lock(&mmc3524_i2c_mutex);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&mmc3524_i2c_mutex);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&mmc3524_i2c_mutex);
		printk(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	if (err != 2) {
		printk("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	mutex_unlock(&mmc3524_i2c_mutex);
	return err;

}

static int mag_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{				/*because address also occupies one byte, the maximum length for write is 7 bytes */
	int err = 0, idx = 0, num = 0;
	char buf[C_I2C_FIFO_SIZE];

	err = 0;
	mutex_lock(&mmc3524_i2c_mutex);
	if (!client) {
		mutex_unlock(&mmc3524_i2c_mutex);
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		mutex_unlock(&mmc3524_i2c_mutex);
		printk(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		mutex_unlock(&mmc3524_i2c_mutex);
		printk("send command error!!\n");
		return -EFAULT;
	}
	mutex_unlock(&mmc3524_i2c_mutex);
	return err;
}
static int I2C_RxData(char *rxData, int length)
{
#ifdef MTK_I2C_FUNCTION
	struct i2c_client *client = this_client;
	int res = 0;
	char addr = rxData[0];

	if ((rxData == NULL) || (length < 1))
		return -EINVAL;
	res = mag_i2c_read_block(client, addr, rxData, length);
	if (res < 0)
		return -1;
	return 0;
#else
	uint8_t loop_i;

#if DEBUG
	int i;
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
	char addr = rxData[0];
#endif

	/* Caller should check parameter validity.*/
	if((rxData == NULL) || (length < 1))
	{
		return -EINVAL;
	}

	for(loop_i = 0; loop_i < MMC3524X_RETRY_COUNT; loop_i++)
	{
		this_client->addr = this_client->addr & I2C_MASK_FLAG | I2C_WR_FLAG;
		if(i2c_master_send(this_client, (const char*)rxData, ((length<<0X08) | 0X01)))
		{
			break;
		}
		printk("I2C_RxData delay!\n");
		mdelay(10);
	}

	if(loop_i >= MMC3524X_RETRY_COUNT)
	{
		printk(KERN_ERR "%s retry over %d\n", __func__, MMC3524X_RETRY_COUNT);
		return -EIO;
	}
#if DEBUG
	if(atomic_read(&data->trace) & MMC_I2C_DEBUG)
	{
		printk(KERN_INFO "RxData: len=%02x, addr=%02x\n  data=", length, addr);
		for(i = 0; i < length; i++)
		{
			printk(KERN_INFO " %02x", rxData[i]);
		}
	    printk(KERN_INFO "\n");
	}
#endif
	return 0;
#endif
}

static int I2C_TxData(char *txData, int length)
{

	#ifdef MTK_I2C_FUNCTION
	struct i2c_client *client = this_client;
	int res = 0;
	char addr = txData[0];
	u8 *buff = &txData[1];

	if ((txData == NULL) || (length < 2))
		return -EINVAL;
	res = mag_i2c_write_block(client, addr, buff, (length - 1));
	if (res < 0)
		return -1;
	return 0;
#else
	uint8_t loop_i;

#if DEBUG
	int i;
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2))
	{
		return -EINVAL;
	}

	this_client->addr = this_client->addr & I2C_MASK_FLAG;
	for(loop_i = 0; loop_i < MMC3524X_RETRY_COUNT; loop_i++)
	{
		if(i2c_master_send(this_client, (const char*)txData, length) > 0)
		{
			break;
		}
		printk("I2C_TxData delay!\n");
		mdelay(10);
	}

	if(loop_i >= MMC3524X_RETRY_COUNT)
	{
		printk(KERN_ERR "%s retry over %d\n", __func__, MMC3524X_RETRY_COUNT);
		return -EIO;
	}
#if DEBUG
	if(atomic_read(&data->trace) & MMC_I2C_DEBUG)
	{
		printk(KERN_INFO "TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
		for(i = 0; i < (length-1); i++)
		{
			printk(KERN_INFO " %02x", txData[i + 1]);
		}
		printk(KERN_INFO "\n");
	}
#endif
	return 0;
#endif
}

// Daemon application save the data
static int ECS_SaveData(int buf[12])
{
#if DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

	mutex_lock(&sensor_data_mutex);
	memcpy(sensor_data, buf, sizeof(sensor_data));
	mutex_unlock(&sensor_data_mutex);

#if DEBUG
	if(atomic_read(&data->trace) & MMC_HWM_DEBUG)
	{
		MMCDBG("Get daemon data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],
			sensor_data[4],sensor_data[5],sensor_data[6],sensor_data[7],
			sensor_data[8],sensor_data[9],sensor_data[10],sensor_data[11]);
	}
#endif

	return 0;
}

static int ECS_ReadXYZData(int *vec, int size)
{
	unsigned char data[6] = {0,0,0,0,0,0};
	static int last_data[3];
	struct timespec time1, time2, time3;
#if DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *clientdata = i2c_get_clientdata(client);
#endif
//set_current_state(TASK_INTERRUPTIBLE);
time1 = current_kernel_time();

	if(size < 3)
	{
		return -1;
	}
	mutex_lock(&read_i2c_xyz);
	time2 = current_kernel_time();

if (!(read_idx % MMC3524X_RESET_INTV))
	{
		/* Reset Sensor Periodly*/
		/* SET */
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_REFILL;
			/* not check return value here, assume it always OK */
			I2C_TxData(data, 2);
			/* wait external capacitor charging done for next RM */
			msleep(MMC3524X_DELAY_SET);
		    data[0] = MMC3524X_REG_CTRL;
		    data[1] = MMC3524X_CTRL_SET;
		    I2C_TxData(data, 2);
		    msleep(1);
		    data[0] = MMC3524X_REG_CTRL;
		    data[1] = 0;
		    I2C_TxData(data, 2);
		    msleep(1);

	            data[0] = MMC3524X_REG_CTRL;
	            data[1] = MMC3524X_CTRL_REFILL;
	            I2C_TxData(data, 2);
	            //msleep(MMC3524X_DELAY_RST);
	            data[0] = MMC3524X_REG_CTRL;
	            data[1] = MMC3524X_CTRL_RESET;
	            I2C_TxData(data, 2);
	            msleep(1);
	            data[0] = MMC3524X_REG_CTRL;
	            data[1] = 0;
	            I2C_TxData(data, 2);
	            msleep(1);	  
	}

	time3 = current_kernel_time();
	/* send TM cmd before read */
	data[0] = MMC3524X_REG_CTRL;
	data[1] = MMC3524X_CTRL_TM;
	/* not check return value here, assume it always OK */
	I2C_TxData(data, 2);
	msleep(MMC3524X_DELAY_TM);

#if READMD
	/* Read MD */
		data[0] = MMC3524X_REG_DS;
		I2C_RxData(data, 1);
		while (!(data[0] & 0x01)) {
			msleep(1);
			/* Read MD again*/
			data[0] = MMC3524X_REG_DS;
			I2C_RxData(data, 1);
			if (data[0] & 0x01) break;
			MD_times++;
			if (MD_times > 3) {
				printk("TM not work!!");
				mutex_unlock(&read_i2c_xyz);
				return -EFAULT;
			}
		}
#endif
	read_idx++;
	data[0] = MMC3524X_REG_DATA;
	if(I2C_RxData(data, 6) < 0)
	{
		mutex_unlock(&read_i2c_xyz);
		return -EFAULT;
	}
	vec[0] = data[1] << 8 | data[0];
	vec[1] = data[3] << 8 | data[2];
	vec[2] = data[5] << 8 | data[4];
	vec[2] = 65536 - vec[2] ;
#if DEBUG
	if(atomic_read(&clientdata->trace) & MMC_DATA_DEBUG)
	{
		printk("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
	}
#endif
	mutex_unlock(&read_i2c_xyz);
	last_data[0] = vec[0];
	last_data[1] = vec[1];
	last_data[2] = vec[2];
	return 0;
}

static int ECS_GetRawData(int data[3])
{
	int err = 0;
	err = ECS_ReadXYZData(data, 3);
	if(err !=0 )
	{
		printk(KERN_ERR "MMC3524X_IOC_TM failed\n");
		return -1;
	}

	// sensitivity 2048 count = 1 Guass = 100uT
	data[0] = (data[0] - MMC3524X_OFFSET_X) * 100 / MMC3524X_SENSITIVITY_X;
	data[1] = (data[1] - MMC3524X_OFFSET_X) * 100 / MMC3524X_SENSITIVITY_X;
	data[2] = (data[2] - MMC3524X_OFFSET_X) * 100 / MMC3524X_SENSITIVITY_X;

	return err;
}
static int ECS_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}


/*----------------------------------------------------------------------------*/
static int mmc3524x_ReadChipInfo(char *buf, int bufsize)
{
	if((!buf)||(bufsize <= MMC3524X_BUFSIZE -1))
	{
		return -1;
	}
	if(!this_client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "mmc3524x Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[MMC3524X_BUFSIZE];
	mmc3524x_ReadChipInfo(strbuf, MMC3524X_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	int sensordata[3];
	char strbuf[MMC3524X_BUFSIZE];

	ECS_GetRawData(sensordata);

	sprintf(strbuf, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);

	return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_posturedata_value(struct device_driver *ddri, char *buf)
{
	int tmp[3];
	char strbuf[MMC3524X_BUFSIZE];
	tmp[0] = sensor_data[0] * CONVERT_O / CONVERT_O_DIV;
	tmp[1] = sensor_data[1] * CONVERT_O / CONVERT_O_DIV;
	tmp[2] = sensor_data[2] * CONVERT_O / CONVERT_O_DIV;
	sprintf(strbuf, "%d, %d, %d\n", tmp[0],tmp[1], tmp[2]);

	return sprintf(buf, "%s\n", strbuf);;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri,const char *buf, size_t count)
{	
	int layout = 0;
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
	

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			printk(KERN_ERR "HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			printk(KERN_ERR "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			printk(KERN_ERR "invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
	ssize_t len = 0;

	if(data->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			data->hw->i2c_num, data->hw->direction, data->hw->power_id, data->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);
	if(NULL == obj)
	{
		printk(KERN_ERR "mmc3524x_i2c_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);
	int trace;
	if(NULL == obj)
	{
		printk(KERN_ERR "mmc3524x_i2c_data is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else
	{
		printk(KERN_ERR "invalid content: '%s', length = %ld\n", buf, count);
	}

	return count;
}
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[MMC3524X_BUFSIZE];
	sprintf(strbuf, "memsicd3524x");
	return sprintf(buf, "%s", strbuf);
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,      S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(status,      S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *mmc3524x_attr_list[] = {
    &driver_attr_daemon,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_posturedata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
};

/*----------------------------------------------------------------------------*/
static int mmc3524x_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(mmc3524x_attr_list)/sizeof(mmc3524x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, mmc3524x_attr_list[idx])))
		{
			printk(KERN_ERR "driver_create_file (%s) = %d\n", mmc3524x_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}

/*----------------------------------------------------------------------------*/
static int mmc3524x_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)(sizeof(mmc3524x_attr_list)/sizeof(mmc3524x_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, mmc3524x_attr_list[idx]);
	}

	return 0;
}


/*----------------------------------------------------------------------------*/
static int mmc3524x_open(struct inode *inode, struct file *file)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);
	int ret = -1;

	if(atomic_read(&obj->trace) & MMC_CTR_DEBUG)
	{
		MMCDBG("Open device node:mmc3524x\n");
	}
	ret = nonseekable_open(inode, file);

	return ret;
}
/*----------------------------------------------------------------------------*/
static int mmc3524x_release(struct inode *inode, struct file *file)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);
	atomic_dec(&dev_open_count);
	if(atomic_read(&obj->trace) & MMC_CTR_DEBUG)
	{
		MMCDBG("Release device node:mmc3524x\n");
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int mmc3524x_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
static long mmc3524x_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)

{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char buff[MMC3524X_BUFSIZE];				/* for chip information */

	int value[12];			/* for SET_YPR */
	int delay;				/* for GET_DELAY */
	int status; 				/* for OPEN/CLOSE_STATUS */
	short sensor_status;		/* for Orientation and Msensor status */
	unsigned char data[16] = {0};
	int vec[3] = {0};
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *clientdata = i2c_get_clientdata(client);
	struct hwm_sensor_data* osensor_data;
	uint32_t enable;
	unsigned char reg_addr;
	unsigned char reg_value;


	switch (cmd)
	{
		case MMC31XX_IOC_TM:
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_TM;
			if (I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_TM failed\n");
				return -EFAULT;
			}
			/* wait TM done for coming data read */
			//msleep(MMC3524X_DELAY_TM);
			break;

		case MMC31XX_IOC_SET:
		case MMC31XX_IOC_RM:
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_REFILL;
			if(I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(MMC3524X_DELAY_SET);
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_SET;
			if(I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			data[0] = MMC3524X_REG_CTRL;
			data[1] = 0;
			if(I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			break;

		case MMC31XX_IOC_RESET:
		case MMC31XX_IOC_RRM:
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_REFILL;
			if(I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(MMC3524X_DELAY_RESET);
			data[0] = MMC3524X_REG_CTRL;
			data[1] = MMC3524X_CTRL_RESET;
			if(I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			data[0] = MMC3524X_REG_CTRL;
			data[1] = 0;
			if(I2C_TxData(data, 2) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(1);
			break;

		case MMC31XX_IOC_READ:
			data[0] = MMC3524X_REG_DATA;
			if(I2C_RxData(data, 6) < 0)
			{
				printk(KERN_ERR "MMC3524x_IOC_READ failed\n");
				return -EFAULT;
			}
		//	vec[0] = data[0] << 8 | data[1];
		//	vec[1] = data[2] << 8 | data[3];
		//	vec[2] = data[4] << 8 | data[5];
			vec[0] = data[1] << 8 | data[0];
			vec[1] = data[3] << 8 | data[2];
			vec[2] = data[5] << 8 | data[4];
#if DEBUG
			if(atomic_read(&clientdata->trace) & MMC_DATA_DEBUG)
			{
				printk("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
			}
#endif
			if(copy_to_user(argp, vec, sizeof(vec)))
			{
				printk(KERN_ERR "MMC3524x_IOC_READ: copy to user failed\n");
				return -EFAULT;
			}
			break;

		case MMC31XX_IOC_READXYZ:
			ECS_ReadXYZData(vec, 3);
			if(copy_to_user(argp, vec, sizeof(vec)))
			{
				printk(KERN_ERR "MMC3524x_IOC_READXYZ: copy to user failed\n");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_DELAY:
			delay = mmcd_delay;
			if(copy_to_user(argp, &delay, sizeof(delay)))
			{
				printk(KERN_ERR "copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_SET_YPR:
			if(argp == NULL)
			{
				MMCDBG("invalid argument.");
				return -EINVAL;
			}
			if(copy_from_user(value, argp, sizeof(value)))
			{
				MMCDBG("copy_from_user failed.");
				return -EFAULT;
			}
			ECS_SaveData(value);
			break;

		case ECOMPASS_IOC_GET_OPEN_STATUS:
			status = ECS_GetOpenStatus();
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				MMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_MFLAG:
			sensor_status = atomic_read(&m_flag);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				MMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_OFLAG:
			sensor_status = atomic_read(&o_flag);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				MMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;


		case MSENSOR_IOCTL_READ_CHIPINFO:
			if(argp == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}

			mmc3524x_ReadChipInfo(buff, MMC3524X_BUFSIZE);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				return -EFAULT;
			}
			break;

		case MSENSOR_IOCTL_READ_SENSORDATA:
			if(argp == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}
			ECS_GetRawData(vec);
			sprintf(buff, "%x %x %x", vec[0], vec[1], vec[2]);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_LAYOUT:
			status = atomic_read(&clientdata->layout);
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				MMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case MSENSOR_IOCTL_SENSOR_ENABLE:

			if(argp == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}
			if(copy_from_user(&enable, argp, sizeof(enable)))
			{
				MMCDBG("copy_from_user failed.");
				return -EFAULT;
			}
			else
			{
			    printk( "MSENSOR_IOCTL_SENSOR_ENABLE enable=%d!\r\n",enable);
				if(1 == enable)
				{
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&o_flag, 0);
					if(atomic_read(&m_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}
				}
				wake_up(&open_wq);

			}

			break;

		case MMC3524X_IOC_READ_REG:
		     printk("MMC3524X_IOC_READ_REG \n");
			if (copy_from_user(&reg_addr, argp, sizeof(reg_addr)))
				return -EFAULT;
			data[0] = reg_addr;
			if (I2C_RxData(data, 1) < 0) {
				//mutex_unlock(&ecompass_lock);
				return -EFAULT;
			}
			printk("<7>planar Read register No. 0x%0x\n", data[0]);
			reg_value = data[0];
			if (copy_to_user(argp, &reg_value, sizeof(reg_value))) {
				//mutex_unlock(&ecompass_lock);
				return -EFAULT;
			}		
			break;

		case MMC3524X_IOC_WRITE_REG:
		     printk(" MMC3524X_IOC_WRITE_REG \n");
			if (copy_from_user(&data, argp, sizeof(data)))
			return -EFAULT;
			if (I2C_TxData(data, 2) < 0) {
			//mutex_unlock(&ecompass_lock);
			return -EFAULT;
			}
			printk("<7>planar Write '0x%0x' to  register No. 0x%0x\n", data[0], data[1]);
			
		    break; 

		case MMC3524X_IOC_READ_REGS:
			printk(" MMC3524X_IOC_READ_REGS \n");
			if (copy_from_user(&data, argp, sizeof(data)))
				return -EFAULT;
			printk("<7> planar Read %d registers from 0x%0x\n", data[1], data[0]);	
			if (I2C_RxData(data, data[1]) < 0) {
				//mutex_unlock(&ecompass_lock);
				return -EFAULT;
			}
			printk("<7> data: %x %x %x \n%x %x %x\n", data[0], data[1], data[2], data[3], data[4], data[5]);	
			if (copy_to_user(argp, data, sizeof(data))) {
				//mutex_unlock(&ecompass_lock);
				return -EFAULT;
			}		
			break; 

		case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
			if(argp == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}
			osensor_data = (struct hwm_sensor_data *)buff;
		    	mutex_lock(&sensor_data_mutex);

			osensor_data->values[0] = sensor_data[8] * CONVERT_O;
			osensor_data->values[1] = sensor_data[9] * CONVERT_O;
			osensor_data->values[2] = sensor_data[10] * CONVERT_O;
			osensor_data->status = sensor_data[11];
			osensor_data->value_divide = CONVERT_O_DIV;

			mutex_unlock(&sensor_data_mutex);

			sprintf(buff, "%x %x %x %x %x", osensor_data->values[0], osensor_data->values[1],
				osensor_data->values[2],osensor_data->status,osensor_data->value_divide);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				return -EFAULT;
			}

			break;

		default:
			printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			return -ENOIOCTLCMD;
			break;
		}

	return 0;
}
#ifdef CONFIG_COMPAT
static long mmc3524x_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	
		void __user *arg64 = compat_ptr(arg);
	
		if(!file->f_op || !file->f_op->unlocked_ioctl)
		{
			printk(KERN_ERR "file->f_op OR file->f_op->unlocked_ioctl is null!\n");
			return -ENOTTY;
		}
	
		switch(cmd)
		{
			case COMPAT_MMC31XX_IOC_TM:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_TM, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_TM is failed!\n");
				}
				break;
	
			case COMPAT_MMC31XX_IOC_SET:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_SET, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_SET is failed!\n");
				}
				break;
				
			case COMPAT_MMC31XX_IOC_RM:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_RM, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_RM is failed!\n");
				}
	
				break;
	
			case COMPAT_MMC31XX_IOC_RESET:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_RESET, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_RESET is failed!\n");
				}
	
			case COMPAT_MMC31XX_IOC_RRM:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_RRM, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_RRM is failed!\n");
				}
	
				break;
	
			case COMPAT_MMC31XX_IOC_READ:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_READ, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_READ is failed!\n");
				}
	
				break;
	
			case COMPAT_MMC31XX_IOC_READXYZ:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_READXYZ, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC31XX_IOC_READXYZ is failed!\n");
				}
	
				break;
	
			case COMPAT_MMC3524X_IOC_READ_REG:
				ret = file->f_op->unlocked_ioctl(file, MMC3524X_IOC_READ_REG, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC3524X_IOC_READ_REG is failed!\n");
				}		
				break;
	
			case COMPAT_MMC3524X_IOC_WRITE_REG:
				ret = file->f_op->unlocked_ioctl(file, MMC3524X_IOC_WRITE_REG, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC3524X_IOC_WRITE_REG is failed!\n");
				}
	
				break; 
	
			case COMPAT_MMC3524X_IOC_READ_REGS:
				ret = file->f_op->unlocked_ioctl(file, MMC3524X_IOC_READ_REGS, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MMC3524X_IOC_READ_REGS is failed!\n");
				}	
				break; 
	
			case COMPAT_ECOMPASS_IOC_GET_DELAY:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_DELAY, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ECOMPASS_IOC_GET_DELAY is failed!\n");
				}
	
				break;
	
			case COMPAT_ECOMPASS_IOC_SET_YPR:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_SET_YPR, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ECOMPASS_IOC_SET_YPR is failed!\n");
				}
	
				break;
	
			case COMPAT_ECOMPASS_IOC_GET_OPEN_STATUS:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_OPEN_STATUS, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ECOMPASS_IOC_GET_OPEN_STATUS is failed!\n");
				}
	
				break;
	
			case COMPAT_ECOMPASS_IOC_GET_MFLAG:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_MFLAG, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ECOMPASS_IOC_GET_MFLAG is failed!\n");
				}
	
				break;
	
			case COMPAT_ECOMPASS_IOC_GET_OFLAG:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_OFLAG, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ECOMPASS_IOC_GET_OFLAG is failed!\n");
				}
	
				break;
	
	
			case COMPAT_MSENSOR_IOCTL_READ_CHIPINFO:
				if(arg64 == NULL)
				{
					printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
					break;
				}
	
				ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_CHIPINFO, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MSENSOR_IOCTL_READ_CHIPINFO is failed!\n");
				}
	
				break;
	
			case COMPAT_MSENSOR_IOCTL_READ_SENSORDATA:
				if(arg64 == NULL)
				{
					printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
					break;
				}
	
				ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MSENSOR_IOCTL_READ_SENSORDATA is failed!\n");
				}
	
				break;
	
			case COMPAT_ECOMPASS_IOC_GET_LAYOUT:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_LAYOUT, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ECOMPASS_IOC_GET_LAYOUT is failed!\n");
				}
	
				break;
	
			case COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE:
				if(arg64 == NULL)
				{
					printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
					break;
				}
	
				ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SENSOR_ENABLE, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE is failed!\n");
				}
	
				break;
	
			case COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
				if(arg64 == NULL)
				{
					printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
					break;
				}
	
				ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_FACTORY_SENSORDATA, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA is failed!\n");
				}
	
				break;
	
			default:
				printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
				return -ENOIOCTLCMD;
				break;		
		}
		return ret;
}

#endif
/*----------------------------------------------------------------------------*/
static struct file_operations mmc3524x_fops = {
	//.owner = THIS_MODULE,
	.open = mmc3524x_open,
	.release = mmc3524x_release,
	.unlocked_ioctl = mmc3524x_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mmc3524x_compat_ioctl,
#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice mmc3524x_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "msensor",
    .fops = &mmc3524x_fops,
};
/*----------------------------------------------------------------------------*/
int mmc3524x_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* msensor_data;

#if DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if(atomic_read(&data->trace) & MMC_FUN_DEBUG)
	{
		MMCFUNC("mmc3524x_operate");
	}
#endif
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 20)
				{
					mmcd_delay = 20;
				}
				mmcd_delay = value;
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{

				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&m_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&m_flag, 0);
					if(atomic_read(&o_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}
				}
				wake_up(&open_wq);

				// TODO: turn device into standby or normal mode
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				msensor_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);

				msensor_data->values[0] = sensor_data[4] * CONVERT_M;
				msensor_data->values[1] = sensor_data[5] * CONVERT_M;
				msensor_data->values[2] = sensor_data[6] * CONVERT_M;
				msensor_data->status = sensor_data[7];
				msensor_data->value_divide = CONVERT_M_DIV;

				mutex_unlock(&sensor_data_mutex);
#if DEBUG
				if(atomic_read(&data->trace) & MMC_HWM_DEBUG)
				{
					MMCDBG("Hwm get m-sensor data: %d, %d, %d. divide %d, status %d!\n",
						msensor_data->values[0],msensor_data->values[1],msensor_data->values[2],
						msensor_data->value_divide,msensor_data->status);
				}
#endif
			}
			break;
		default:
			printk(KERN_ERR "msensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
int mmc3524x_orientation_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* osensor_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if(atomic_read(&data->trace) & MMC_FUN_DEBUG)
	{
		MMCFUNC("mmc3524x_orientation_operate");
	}
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 20)
				{
					mmcd_delay = 20;
				}
				mmcd_delay = value;
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{

				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&o_flag, 0);
					if(atomic_read(&m_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}
				}
				wake_up(&open_wq);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				osensor_data = (struct hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);

				osensor_data->values[0] = sensor_data[8] * CONVERT_O;
				osensor_data->values[1] = sensor_data[9] * CONVERT_O;
				osensor_data->values[2] = sensor_data[10] * CONVERT_O;
				osensor_data->status = sensor_data[11];
				osensor_data->value_divide = CONVERT_O_DIV;

				mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if(atomic_read(&data->trace) & MMC_HWM_DEBUG)
			{
				MMCDBG("Hwm get o-sensor data: %d, %d, %d. divide %d, status %d!\n",
					osensor_data->values[0],osensor_data->values[1],osensor_data->values[2],
					osensor_data->value_divide,osensor_data->status);
			}
#endif
			}
			break;
		default:
			printk(KERN_ERR "gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
#ifndef	CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int mmc3524x_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(client);


	if(msg.event == PM_EVENT_SUSPEND)
	{
		mmc3524x_power(obj->hw, 0);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int mmc3524x_resume(struct i2c_client *client)
{
	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(client);


	mmc3524x_power(obj->hw, 1);


	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void mmc3524x_early_suspend(struct early_suspend *h)
{
	struct mmc3524x_i2c_data *obj = container_of(h, struct mmc3524x_i2c_data, early_drv);

	if(NULL == obj)
	{
		printk(KERN_ERR "null pointer!!\n");
		return;
	}

}
/*----------------------------------------------------------------------------*/
static void mmc3524x_late_resume(struct early_suspend *h)
{
	struct mmc3524x_i2c_data *obj = container_of(h, struct mmc3524x_i2c_data, early_drv);


	if(NULL == obj)
	{
		printk(KERN_ERR "null pointer!!\n");
		return;
	}

	mmc3524x_power(obj->hw, 1);

}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
//static int mmc3524x_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
//{
//	strcpy(info->type, MMC3524X_DEV_NAME);
//	return 0;
//}

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static int mmc3524x_m_enable(int en)
{
    int value = en;
	printk("mmc3524x_m_enable++\n");
	
	if(value == 1)
	{
		atomic_set(&m_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		atomic_set(&m_flag, 0);
		if(atomic_read(&o_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}
	}
	wake_up(&open_wq);
	printk("mmc3524x_m_enable--\n");
	return 0;
}

static int mmc3524x_m_set_delay(u64 ns)
{
    int value=0;
	printk("mmc3524x_m_set_delay++\n");
	value = (int)ns/1000/1000;
	if(value <= 20)
	{
		mmcd_delay = 20;
	}
	mmcd_delay = value;
	printk("mmc3524x_m_set_delay--\n");
	return 0;
}
static int mmc3524x_m_open_report_data(int open)
{
	return 0;
}
 
static int mmc3524x_o_enable(int en)
{
  	int value =en;
	printk("mmc3524x_o_enable++\n");
	if(value == 1)
	{
		atomic_set(&o_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		atomic_set(&o_flag, 0);
		if(atomic_read(&m_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}									
	}	
	wake_up(&open_wq);
	printk("mmc3524x_o_enable--\n");
	return 0;
}

#if 1
int mmc3524x_o_enable_for_AF(int en)
{
	return mmc3524x_o_enable(en);
}
#endif

static int mmc3524x_o_set_delay(u64 ns)
{
	int value=0;
	printk("mmc3524x_o_set_delay++\n");
	value = (int)ns/1000/1000;
	if(value <= 20)
	{
		mmcd_delay = 20;
	}
	mmcd_delay = value;
	printk("mmc3524x_o_set_delay--\n");
	return 0;
				
}
static int mmc3524x_o_open_report_data(int open)
{
	return 0;
}

static int mmc3524x_o_get_data(int* x ,int* y,int* z, int* status)
{
	mutex_lock(&sensor_data_mutex);
	
	*x = sensor_data[8] * CONVERT_O;
	*y = sensor_data[9] * CONVERT_O;
	*z = sensor_data[10] * CONVERT_O;
	*status = sensor_data[11];
		
	mutex_unlock(&sensor_data_mutex);	
	
	return 0;
}

#if 1
int mmc3524x_o_get_data_for_AF(int* x ,int* y,int* z, int* status)
{
	return mmc3524x_o_get_data(x, y, z, status);
}
#endif

static int mmc3524x_m_get_data(int* x ,int* y,int* z, int* status)
{
	mutex_lock(&sensor_data_mutex);
	
	*x = sensor_data[4] * CONVERT_M;
	*y = sensor_data[5] * CONVERT_M;
	*z = sensor_data[6] * CONVERT_M;
	*status = sensor_data[7];	
	mutex_unlock(&sensor_data_mutex);
	return 0;
}

/*----------------------------------------------------------------------------*/



static int mmc3524x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	
	struct i2c_client *new_client;
	struct mmc3524x_i2c_data *data;
	char tmp[2];
	int err = 0;
	struct mag_control_path ctl={0};
	struct mag_data_path mag_data={0};
	char product_id[2] = {0,0};

	printk("[M_Sensor]%s!\n", __func__);
	printk("%s: ++++\n", __func__);
    
	if(!(data = kmalloc(sizeof(struct mmc3524x_i2c_data), GFP_KERNEL)))
	{
		printk("[M_Sensor]%s,line=%d!\n", __func__,__LINE__);
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct mmc3524x_i2c_data));
	printk("[M_Sensor]%s,line=%d!\n", __func__,__LINE__);
	//data->hw = get_cust_mag_hw();
	data->hw = hw;
	printk("[M_Sensor]%s,line=%d!\n", __func__,__LINE__);
	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);

	mutex_init(&sensor_data_mutex);
	mutex_init(&read_i2c_xyz);

	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);
	printk("[M_Sensor]%s,line=%d!\n", __func__,__LINE__);
	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);
	printk("[M_Sensor]%s,line=%d!\n", __func__,__LINE__);
	this_client = new_client;
	//this_client->timing=400;
        msleep(10);
	printk("[M_Sensor]%s,line=%d!\n", __func__,__LINE__);
	/* send TM cmd to mag sensor first of all */
	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_TM;
	if(I2C_TxData(tmp, 2) < 0)
	{
		printk(KERN_ERR "mmc3524x_device set TM cmd failed\n");
		goto exit_kfree;
	}

	printk("%s: ++++\n", __func__);
	
	//AGold Bruce compat 2015.05.12
	product_id[0] = MMC3524X_REG_PRODUCTID_1;
	if(I2C_RxData(product_id, 1) < 0)
	{
		printk("[AGold][Bruce][mmc3524x] read id fail\n");
		//read again
		I2C_RxData(product_id, 1);
	}
	
	printk("[AGold][Bruce][mmc3524x] product_id[0] = %d\n",product_id[0]);
	if(product_id[0] != 0x8)
	{
		printk("[M_Sensor]%s,line=%d!\n", __func__,__LINE__);
		//goto exit_kfree;
	}
	//AGold end

#if 1

      /*   if ((err = mmc3524x_dev_init(client)) < 0) 
    {
        MMCDBG("init device error!\n");
        goto exit_kfree;
    }*/
	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_REFILL;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(MMC3524X_DELAY_SET);

	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_SET;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(1);
	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = 0;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(MMC3524X_DELAY_SET);

/*	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_REFILL;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(MMC3524X_DELAY_RST);
	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_RESET;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(1);
	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = 0;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(1);
*/
	tmp[0] = MMC3524X_REG_BITS;
	tmp[1] = MMC3524X_BITS_SLOW_16;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(MMC3524X_DELAY_TM);

	tmp[0] = MMC3524X_REG_CTRL;
	tmp[1] = MMC3524X_CTRL_TM;
	if (I2C_TxData(tmp, 2) < 0) {
	}
	msleep(MMC3524X_DELAY_TM);


#endif
	if((err = mmc3524x_create_attr(&(mmc3524x_init_info.platform_diver_addr->driver))))
	{
		printk(KERN_ERR "create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}


	if((err = misc_register(&mmc3524x_device)))
	{
		printk(KERN_ERR "mmc3524x_device register failed\n");
		goto exit_misc_device_register_failed;	
	}
	ctl.is_use_common_factory = false;
	ctl.m_enable = mmc3524x_m_enable;
	ctl.m_set_delay  = mmc3524x_m_set_delay;
	ctl.m_open_report_data = mmc3524x_m_open_report_data;
	ctl.o_enable = mmc3524x_o_enable;
	ctl.o_set_delay  = mmc3524x_o_set_delay;
	ctl.o_open_report_data = mmc3524x_o_open_report_data;
	ctl.is_report_input_direct = false;
	
	err = mag_register_control_path(&ctl);
	if(err)
	{
		printk(KERN_ERR "attach fail = %d\n", err);
		goto exit_kfree;
	}

	mag_data.div_m = CONVERT_M_DIV;
	mag_data.div_o = CONVERT_O_DIV;
	mag_data.get_data_o = mmc3524x_o_get_data;
	mag_data.get_data_m = mmc3524x_m_get_data;
	
	err = mag_register_data_path(&mag_data);
	if(err)
	{
		printk(KERN_ERR "attach fail = %d\n", err);
		goto exit_kfree;
	}


//add by wangxiqiang
   #ifdef CONFIG_SLT_DRV_DEVINFO_SUPPORT
   
	temp_comments=(char*) kmalloc(25, GFP_KERNEL);
	sprintf(temp_comments,"MAGNETOMETER:MMC3530KJ"); 
	devinfo_ctp_regchar("unknown","MEMSIC","1.0",DEVINFO_USED,temp_comments);

   #endif
   //end of add


#if defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	data->early_drv.suspend  = mmc3524x_early_suspend,
	data->early_drv.resume   = mmc3524x_late_resume,
	register_early_suspend(&data->early_drv);
#endif

	MMCDBG("%s: OK\n", __func__);
//	m_sensor_deteced = true;
	printk("mmc3524X IIC probe successful !");
	
	mmc3524x_init_flag = 1;
	return 0;

	exit_sysfs_create_group_failed:
	exit_misc_device_register_failed:
	exit_kfree:
	kfree(data);
	exit:
	printk(KERN_ERR "%s: err = %d\n", __func__, err);
//	m_sensor_deteced = false;
	mmc3524x_init_flag = -1;
	return err;
}
/*----------------------------------------------------------------------------*/
static int mmc3524x_i2c_remove(struct i2c_client *client)
{
	int err;	
	
    if((err = mmc3524x_delete_attr(&(mmc3524x_init_info.platform_diver_addr->driver))))
	{
		printk(KERN_ERR "mmc3524x_delete_attr fail: %d\n", err);
	}

	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	misc_deregister(&mmc3524x_device);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int platform_mmc3524x_probe(struct platform_device *pdev)
{
	//struct mag_hw *hw = get_cust_mag_hw();
	printk("[M_Sensor]%s!\n", __func__);
	

	mmc3524x_power(hw, 1);
	atomic_set(&dev_open_count, 0);

	if(i2c_add_driver(&mmc3524x_i2c_driver))
	{
		printk(KERN_ERR "add driver error\n");
		return -1;
	}
	return 0;
}
static int platform_mmc3524x_remove(struct platform_device *pdev)
{
	//struct mag_hw *hw = get_cust_mag_hw();

	mmc3524x_power(hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&mmc3524x_i2c_driver);
	return 0;
}


static int mmc3524x_local_init(void)
{
	//struct mag_hw *hw = get_cust_mag_hw();
	printk("[M_Sensor] %s\n", __func__);

	

	mmc3524x_power(hw, 1);
	if(i2c_add_driver(&mmc3524x_i2c_driver))
	{
		MMCDBG("add driver error\n");
		return -1;
	}
	if(-1 == mmc3524x_init_flag)
	{
	   return -1;
	}
	return 0;
}

static int mmc3524x_remove(void)
{
    //struct mag_hw *hw = get_cust_mag_hw();
	mmc3524x_power(hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&mmc3524x_i2c_driver);
	return 0;
}



/*----------------------------------------------------------------------------*/
static int __init mmc3524x_init(void)
{
	const char *name = "mediatek,mmc3530kj";

	printk("[M_Sensor]%s!\n", __func__);
	hw = get_mag_dts_func(name, hw);
	if (!hw)
		printk("[M_Sensor]%s! get dts info fail\n", __func__);
	mag_driver_add(&mmc3524x_init_info);

	return 0;
}



/*----------------------------------------------------------------------------*/
static void __exit mmc3524x_exit(void)
{
	//platform_driver_unregister(&mmc3524x_platform_driver);
}
/*----------------------------------------------------------------------------*/
module_init(mmc3524x_init);
module_exit(mmc3524x_exit);

MODULE_AUTHOR("WD");
MODULE_DESCRIPTION("mmc3524x compass driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

