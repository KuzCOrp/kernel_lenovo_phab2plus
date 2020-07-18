
#ifndef __HALLSWITCH_H__
#define __HALLSWITCH_H__


#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <batch.h>
#include <sensors_io.h>
#include <hwmsensor.h>
#include <hwmsen_dev.h>

#define HALLSWITCH_TAG					"<HALLSWITCH> "
#define HALLSWITCH_FUN(f)			printk(HALLSWITCH_TAG"%s\n", __func__)
#define HALLSWITCH_ERR(fmt, args...)	printk(HALLSWITCH_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define HALLSWITCH_LOG(fmt, args...)	printk(HALLSWITCH_TAG fmt, ##args)
#define HALLSWITCH_VER(fmt, args...)   printk(HALLSWITCH_TAG"%s: "fmt, __func__, ##args) //((void)0)

#define HALLSWITCH_INVALID_VALUE -1

#define EVENT_TYPE_HALLSWITCH_VALUE         		REL_Z
#define EVENT_TYPE_HALLSWITCH_STATUS        		REL_Y

#define MAX_CHOOSE_HALLSWITCH_NUM 5

struct hallswitch_control_path
{
	int (*open_report_data)(int open);//open data rerport to HAL
	int (*enable_nodata)(int en);//only enable not report event to HAL
	int (*set_delay)(u64 delay);
	int (*access_data_fifo)(void);//version2.used for flush operate
	int (*hall_calibration)(int type, int value);
	int (*hall_threshold_setting)(int type, int value[2]);
	bool is_report_input_direct;
	bool is_support_batch;//version2.used for batch mode support flag
	bool is_polling_mode;
	bool is_use_common_factory;
};

struct hallswitch_init_info
{
    	char *name;
	int (*init)(void);
	int (*uninit)(void);
	struct platform_driver* platform_diver_addr;
};

struct hallswitch_context {
	struct input_dev   		*idev;
	struct miscdevice   	mdev;
	struct work_struct  	report_hall;
	
	atomic_t                	early_suspend;

	struct hallswitch_control_path hallswitch_ctl;
};

//for auto detect
extern int hallswitch_driver_add(struct hallswitch_init_info* obj) ;
extern int hallswitch_report_interrupt_data(int value);
extern int hallswitch_data_report(struct input_dev *dev, int value,int status);
extern int hallswitch_register_control_path(struct hallswitch_control_path *ctl);
extern struct platform_device *get_hallswitch_platformdev(void);
#if defined(CONFIG_GTP_GLOVE_MODE)
extern int gtp_glove_mode_status;
extern struct i2c_client * i2c_client_point;
extern s32 gtp_send_cfg(struct i2c_client *client);
#endif
#endif
