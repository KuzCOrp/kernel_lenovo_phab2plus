
#ifndef __FLIGHTSENSE_H__
#define __FLIGHTSENSE_H__


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

#define FLIGHTSENSE_TAG					"<FLIGHTSENSE> "
#define FLIGHTSENSE_FUN(f)			printk(FLIGHTSENSE_TAG"%s\n", __func__)
#define FLIGHTSENSE_ERR(fmt, args...)	printk(FLIGHTSENSE_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define FLIGHTSENSE_LOG(fmt, args...)	printk(FLIGHTSENSE_TAG fmt, ##args)
#define FLIGHTSENSE_VER(fmt, args...)   printk(FLIGHTSENSE_TAG"%s: "fmt, __func__, ##args) //((void)0)

#define FLIGHTSENSE_INVALID_VALUE -1

#define EVENT_TYPE_FLIGHTSENSE_VALUE         		REL_Z
#define EVENT_TYPE_FLIGHTSENSE_STATUS        		REL_Y

#define MAX_CHOOSE_FLIGHTSENSE_NUM 5

struct flightsense_control_path
{
	int (*open_report_data)(int open);//open data rerport to HAL
	int (*enable_nodata)(int en);//only enable not report event to HAL
	int (*set_delay)(u64 delay);
	int (*access_data_fifo)(void);//version2.used for flush operate
	int (*flight_calibration)(int type, int value);
	int (*flight_threshold_setting)(int type, int value[2]);
	bool is_report_input_direct;
	bool is_support_batch;//version2.used for batch mode support flag
	bool is_polling_mode;
	bool is_use_common_factory;
};

struct flightsense_init_info
{
    	char *name;
	int (*init)(void);
	int (*uninit)(void);
	struct platform_driver* platform_diver_addr;
};

struct flightsense_context {
	struct input_dev   		*idev;
	struct miscdevice   	mdev;
	struct work_struct  	report_flightsense;
	
	atomic_t                	early_suspend;

	struct flightsense_control_path flightsense_ctl;
};

//for auto detect
extern int flightsense_driver_add(struct flightsense_init_info* obj) ;
extern int flightsense_report_interrupt_data(int value);
extern int flightsense_data_report(struct input_dev *dev, int value,int status);
extern int flightsense_register_control_path(struct flightsense_control_path *ctl);
extern struct platform_device *get_flightsense_platformdev(void);

#endif
