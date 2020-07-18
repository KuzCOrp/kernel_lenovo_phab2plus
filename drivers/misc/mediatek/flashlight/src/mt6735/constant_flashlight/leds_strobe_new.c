#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include "kd_camera_feature.h"


/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static BOOL g_strobe_On = FALSE;

extern int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char *mode_name);

extern void M12MO_write_category_parameter_debug(u8 category, u8 byte, u8 uPara);
extern u8 M12MO_read_category_parameter_debug(u8 category, u8 byte);



int FL_Enable(void)
{
	kal_uint16 INT_status = 0;
	kal_uint8 retry = 50;
	return 0;
	if(g_strobe_On == FALSE)
	{
		kdCISModulePowerOn(0,"m12mo_yuv",true,"kd_camera_hw");
		mdelay(50);
		INT_status = M12MO_read_category_parameter_debug(0x0F, 0x10);
		PK_DBG("(0x0F, 0x10) is 0x%x\n", INT_status);
		while((INT_status !=0x01) && retry)
		{
			INT_status = M12MO_read_category_parameter_debug(0x0F,0x10);
			PK_DBG("(0x0F, 0x10) is 0x%x\n", INT_status);
			mdelay(20);
			retry--;
		}
			
		if((INT_status !=0x01) && (retry==0))
		{
			 PK_DBG("boot up fail!!!\n");
			//goto BOOTUP_FAIL;
		}

		M12MO_write_category_parameter_debug(0x0F, 0x12, 0x01);
		mdelay(100);
		INT_status = 0;
		retry = 50;
		while((INT_status !=0x01) && retry)
		{
			mdelay(10);
			INT_status = M12MO_read_category_parameter_debug(0x00,0x1C);
			PK_DBG("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
		M12MO_write_category_parameter_debug(0x03, 0x3C, 0x03);

		g_strobe_On = TRUE;
	}
	return 0;
}



int FL_Disable(void)
{
	return 0;
	if(g_strobe_On == TRUE)
	{
		M12MO_write_category_parameter_debug(0x03, 0x3C, 0x00);
		kdCISModulePowerOn(0,"m12mo_yuv",false,"kd_camera_hw");
		g_strobe_On = FALSE;
	}
	return 0;
}


/*****************************************************************************
User interface
*****************************************************************************/


static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));

	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		//g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		//FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {
			FL_Enable();
		} else {
			FL_Disable();
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	return 0;
}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");
	return 0;
}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
