/* drivers/hwmon/mt6516/amit/AH1887.c - AH1887/HALL driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>

#include <linux/io.h>
#include "ah1887.h"
#include "upmu_sw.h"
#include "upmu_common.h"
#include <linux/gpio.h>
#include <linux/of_irq.h>

#include <linux/wakelock.h>
#include <linux/sched.h>

#include <hallswitch.h>
/******************************************************************************
 * configuration
*******************************************************************************/
#define AH1887_DEV_NAME     "AH1887"

/******************************************************************************
 * extern functions
*******************************************************************************/

/*----------------------------------------------------------------------------*/
#define HALLSWITCH_EINT_TOUCH	(1)
#define HALLSWITCH_EINT_NO_TOUCH	(0)
/*----------------------------------------------------------------------------*/
static struct work_struct hallswitch_eint_work;
int hallswitch_eint_status=0;
int hallswitch_irq;
unsigned int hallswitch_gpiopin;
unsigned int hallswitch_debounce;
unsigned int hallswitch_eint_type;

static int hallswitch_local_init(void);
static int hallswitch_remove(void);

/*----------------------------------------------------------------------------*/
static struct hallswitch_init_info ah1887_init_info = {
		.name = AH1887_DEV_NAME,
		.init = hallswitch_local_init,
		.uninit = hallswitch_remove,
	
};
/*-----------------------------------------------------------------------------*/
static irqreturn_t ah1887_eint_func(int irq, void *desc)
{
	//HALLSWITCH_LOG(" debug eint function performed!\n");
	disable_irq_nosync(hallswitch_irq);
	schedule_work(&hallswitch_eint_work);

	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
int ah1887_setup_eint(void)
{	
	int ret;
	u32 ints[2] = { 0, 0 };
	u32 ints1[2] = { 0, 0 };	
	struct device_node *node = NULL;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_eint_as_int;
	struct platform_device *hallswitch_pdev = get_hallswitch_platformdev();

	HALLSWITCH_LOG("ah1887_setup_eint\n");

	/* gpio setting */
	pinctrl = devm_pinctrl_get(&hallswitch_pdev->dev);
	if (IS_ERR(pinctrl))
	{
		ret = PTR_ERR(pinctrl);
		HALLSWITCH_ERR("Cannot find hallswitch pinctrl!\n");
		return ret;
	}

	pins_default = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR(pins_default))
	{
		ret = PTR_ERR(pins_default);
		HALLSWITCH_ERR("Cannot find hallswitch pinctrl default!\n");
	}

	pins_eint_as_int = pinctrl_lookup_state(pinctrl, "state_eint_as_int");
	if (IS_ERR(pins_eint_as_int))
	{
		ret = PTR_ERR(pins_eint_as_int);
		HALLSWITCH_ERR("Cannot find hallswitch pinctrl state_eint_as_int!\n");
	}
	pinctrl_select_state(pinctrl, pins_eint_as_int);

	node = of_find_compatible_node(NULL, NULL, "mediatek, hallswit-eint");

	if (node)
	{
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
		hallswitch_gpiopin = ints[0];
		hallswitch_debounce = ints[1];
		hallswitch_eint_type = ints1[1];
		HALLSWITCH_LOG("ints[0] = %d, ints[1] = %d, ints1[1] = %d!!\n", ints[0], ints[1], ints1[1]);
		gpio_set_debounce(hallswitch_gpiopin, hallswitch_debounce);
		hallswitch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(hallswitch_irq, ah1887_eint_func, IRQF_TRIGGER_NONE, "hallswit-eint", NULL);
		if (ret != 0) 
		{
			HALLSWITCH_ERR("EINT IRQ LINE NOT AVAILABLE\n");
		}
		else
		{
			HALLSWITCH_LOG("hallswitch set EINT finished, hallswitch_irq=%d, hallswitch_debounce=%d\n", hallswitch_irq, hallswitch_debounce);
		}
	}
	else
	{
		HALLSWITCH_ERR("%s can't find compatible node\n", __func__);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static void ah1887_eint_work(struct work_struct *work)
{
	int err;

	if (hallswitch_eint_status == HALLSWITCH_EINT_NO_TOUCH)
	{
		hallswitch_eint_status = HALLSWITCH_EINT_TOUCH;
		if (hallswitch_eint_type == IRQ_TYPE_LEVEL_LOW)
		{
			irq_set_irq_type(hallswitch_irq, IRQ_TYPE_LEVEL_HIGH);
		}
		else
		{
			irq_set_irq_type(hallswitch_irq, IRQ_TYPE_LEVEL_LOW);
		}
	}
	else
	{
		hallswitch_eint_status = HALLSWITCH_EINT_NO_TOUCH;
		if (hallswitch_eint_type == IRQ_TYPE_LEVEL_LOW)
		{
			irq_set_irq_type(hallswitch_irq, IRQ_TYPE_LEVEL_LOW);
		}
		else
		{
			irq_set_irq_type(hallswitch_irq, IRQ_TYPE_LEVEL_HIGH);
		}
	}

	//let up layer to know
	if((err = hallswitch_report_interrupt_data(hallswitch_eint_status)))
	{
		HALLSWITCH_ERR("ah1887 call hallswitch_report_interrupt_data fail = %d\n", err);
	}	

	enable_irq(hallswitch_irq);
	
	return;
}

/*----------------------------------------------------------------------------*/
static ssize_t ah1887_show_enable(struct device_driver *ddri, char *buf)
{
	HALLSWITCH_LOG("ah1887_show_enable \n");

	enable_irq(hallswitch_irq);

	return sprintf(buf, "%u\n", 2);
}

static DRIVER_ATTR(ah1887_enable, S_IWUSR | S_IRUGO, ah1887_show_enable, NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *ah1887_attr_list[] =
{
	&driver_attr_ah1887_enable,
};
/*----------------------------------------------------------------------------*/
static int ah1887_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(ah1887_attr_list) / sizeof(ah1887_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, ah1887_attr_list[idx]);
		if (err) {
			HALLSWITCH_ERR("driver_create_file (%s) = %d\n", ah1887_attr_list[idx]->attr.name,
				err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int hallswitch_local_init(void) 
{
	struct hallswitch_control_path hall_ctl={0};
	int err = 0;

	HALLSWITCH_FUN();

	INIT_WORK(&hallswitch_eint_work, ah1887_eint_work);

	ah1887_setup_eint();

	if((err = ah1887_create_attr(&ah1887_init_info.platform_diver_addr->driver)))
	{
		HALLSWITCH_ERR("ah1887 create attribute err = %d\n", err);
		return err;
	}

	err = hallswitch_register_control_path(&hall_ctl);
	if(err)
	{
		HALLSWITCH_ERR("hallswitch register fail = %d\n", err);
		return err;
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int hallswitch_remove(void)
{
	HALLSWITCH_FUN();
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init ah1887_init(void)
{
	HALLSWITCH_FUN();
	
	hallswitch_driver_add(&ah1887_init_info);
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ah1887_exit(void)
{
	HALLSWITCH_FUN();	
}
/*----------------------------------------------------------------------------*/
module_init(ah1887_init);
module_exit(ah1887_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("AH1887 driver");
MODULE_LICENSE("GPL");
