
#include "inc/hallswitch.h"
struct hallswitch_context *hallswitch_context_obj = NULL;
static struct platform_device *pltfm_dev;

static struct hallswitch_init_info* hallswitch_init_list[MAX_CHOOSE_HALLSWITCH_NUM]= {0}; //modified

static bool hallswitch_misc_dev_init;

int hallswitch_data_report(struct input_dev *dev, int value,int status)
{
	HALLSWITCH_LOG("+hallswitch_data_report! %d, %d\n",value,status);
#if defined(CONFIG_GTP_GLOVE_MODE)
	gtp_glove_mode_status=value;
	gtp_send_cfg(i2c_client_point);
#endif
	input_report_rel(dev, EVENT_TYPE_HALLSWITCH_VALUE, (value+1));
	input_report_rel(dev, EVENT_TYPE_HALLSWITCH_STATUS, status);
	input_sync(dev); 
	return 0;
}

static struct hallswitch_context *hallswitch_context_alloc_object(void)
{
	
	struct hallswitch_context *obj = kzalloc(sizeof(*obj), GFP_KERNEL); 
    	HALLSWITCH_LOG("hallswitch_context_alloc_object++++\n");
	if(!obj)
	{
		HALLSWITCH_ERR("Alloc hallswitch object error!\n");
		return NULL;
	}

	HALLSWITCH_LOG("hallswitch_context_alloc_object----\n");
	return obj;
}

static ssize_t hall_store_state(struct device* dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t hall_show_state(struct device* dev, struct device_attribute *attr, char *buf) 
{
	ssize_t len;
	len = sprintf(buf, "%d", gtp_glove_mode_status);
	return len;
}

static ssize_t hall_store_active(struct device* dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t hall_show_active(struct device* dev, struct device_attribute *attr, char *buf) 
{
	return 1;
}

static ssize_t hall_store_delay(struct device* dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t hall_show_delay(struct device* dev, struct device_attribute *attr, char *buf) 
{
	return 0;
}


static ssize_t hall_store_batch(struct device* dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}

static ssize_t hall_show_batch(struct device* dev, struct device_attribute *attr, char *buf) 
{
	return 0; 
}

static ssize_t hall_store_flush(struct device* dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}

static ssize_t hall_show_flush(struct device* dev, struct device_attribute *attr, char *buf) 
{
	return 0;
}

static ssize_t hall_show_devnum(struct device* dev, struct device_attribute *attr, char *buf) 
{
	const char *devname =NULL;
	devname = dev_name(&hallswitch_context_obj->idev->dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", devname+5); 
}

static int hall_switch_remove(struct platform_device *pdev)
{
	HALLSWITCH_LOG("hall_switch_remove\n");
	return 0;
}

static int hall_switch_probe(struct platform_device *pdev) 
{
	HALLSWITCH_LOG("hall_switch_probe\n");
	pltfm_dev = pdev;
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hall_switch_of_match[] = {
	{ .compatible = "mediatek,hall_switch", },
	{},
};
#endif

static struct platform_driver hall_switch_driver = {
	.probe      = hall_switch_probe,
	.remove     = hall_switch_remove,    
	.driver     = 
	{
		.name  = "hall_switch",
        #ifdef CONFIG_OF
		.of_match_table = hall_switch_of_match,
		#endif
	}
};

static int hallswitch_real_driver_init(void) 
{
	int i = 0;
	int err = 0;

	HALLSWITCH_LOG(" hallswitch_real_driver_init +\n");
	for (i = 0; i < MAX_CHOOSE_HALLSWITCH_NUM; i++) {
		HALLSWITCH_LOG("hallswitch_real_driver_init i=%d\n", i);
		if (0 != hallswitch_init_list[i]) {
			HALLSWITCH_LOG(" hallswitch try to init driver %s\n", hallswitch_init_list[i]->name);
			err = hallswitch_init_list[i]->init();
			if (0 == err) {
				HALLSWITCH_LOG(" hallswitch real driver %s probe ok\n", hallswitch_init_list[i]->name);
				break;
			}
		}
	}

	if (i == MAX_CHOOSE_HALLSWITCH_NUM) {
		HALLSWITCH_LOG(" hallswitch_real_driver_init fail\n");
		err =  -1;
	}

	return err;
}

int hallswitch_driver_add(struct hallswitch_init_info* obj) 
{
	int err = 0;
	int i = 0;

	HALLSWITCH_FUN();
	if (!obj) {
		HALLSWITCH_ERR("HALLSWITCH driver add fail, hallswitch_init_info is NULL\n");
		return -1;
	}

	for (i = 0; i < MAX_CHOOSE_HALLSWITCH_NUM; i++) {
		if ((i == 0) && (NULL == hallswitch_init_list[0])) {
			HALLSWITCH_LOG("register hallswitch driver for the first time\n");
			if (platform_driver_register(&hall_switch_driver))
				HALLSWITCH_ERR("failed to register hallswitch driver already exist\n");
		}

		if (NULL == hallswitch_init_list[i]) {
			obj->platform_diver_addr = &hall_switch_driver;
			hallswitch_init_list[i] = obj;
			break;
		}
	}
	if (i >= MAX_CHOOSE_HALLSWITCH_NUM) {
		HALLSWITCH_ERR("HALLSWITCH driver add err\n");
		err =  -1;
	}

	return err;
}
EXPORT_SYMBOL_GPL(hallswitch_driver_add);
struct platform_device *get_hallswitch_platformdev(void)
{
	return pltfm_dev;
}

int hallswitch_report_interrupt_data(int value) 
{
	struct hallswitch_context *cxt = NULL;
	//int err =0;
	cxt = hallswitch_context_obj;	

	hallswitch_data_report(cxt->idev,value,3);
	
	return 0;
}
/*----------------------------------------------------------------------------*/
EXPORT_SYMBOL_GPL(hallswitch_report_interrupt_data);

static int hallswitch_misc_init(struct hallswitch_context *cxt)
{

    int err=0;
    cxt->mdev.minor = MISC_DYNAMIC_MINOR;
	cxt->mdev.name  = HALLSWITCH_MISC_DEV_NAME;
	if((err = misc_register(&cxt->mdev)))
	{
		HALLSWITCH_ERR("unable to register hallswitch misc device!!\n");
	}
	return err;
}

static int hallswitch_input_init(struct hallswitch_context *cxt)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = HALLSWITCH_INPUTDEV_NAME;

	set_bit(EV_REL, dev->evbit);
	set_bit(EV_SYN, dev->evbit);
	input_set_capability(dev, EV_REL, EVENT_TYPE_HALLSWITCH_VALUE);
	input_set_capability(dev, EV_REL, EVENT_TYPE_HALLSWITCH_STATUS);
	
	input_set_drvdata(dev, cxt);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	cxt->idev= dev;

	return 0;
}

DEVICE_ATTR(hallstate,     		S_IWUSR | S_IRUGO, hall_show_state, hall_store_state);
DEVICE_ATTR(hallactive,     		S_IWUSR | S_IRUGO, hall_show_active, hall_store_active);
DEVICE_ATTR(halldelay,      		S_IWUSR | S_IRUGO, hall_show_delay,  hall_store_delay);
DEVICE_ATTR(hallbatch,      		S_IWUSR | S_IRUGO, hall_show_batch,  hall_store_batch);
DEVICE_ATTR(hallflush,      		S_IWUSR | S_IRUGO, hall_show_flush,  hall_store_flush);
DEVICE_ATTR(halldevnum,      		S_IWUSR | S_IRUGO, hall_show_devnum,  NULL);

static struct attribute *hallswitch_attributes[] = {
	&dev_attr_hallstate.attr,
	&dev_attr_hallactive.attr,
	&dev_attr_halldelay.attr,
	&dev_attr_hallbatch.attr,
	&dev_attr_hallflush.attr,
	&dev_attr_halldevnum.attr,
	NULL
};

static struct attribute_group hallswitch_attribute_group = {
	.attrs = hallswitch_attributes
};

int hallswitch_register_control_path(struct hallswitch_control_path *ctl)
{
	struct hallswitch_context *cxt = NULL;
	int err = 0;

	cxt = hallswitch_context_obj;
	cxt->hallswitch_ctl.set_delay = ctl->set_delay;
	cxt->hallswitch_ctl.open_report_data = ctl->open_report_data;
	cxt->hallswitch_ctl.enable_nodata = ctl->enable_nodata;
	cxt->hallswitch_ctl.is_support_batch = ctl->is_support_batch;
	cxt->hallswitch_ctl.is_report_input_direct = ctl->is_report_input_direct;
	cxt->hallswitch_ctl.hall_calibration = ctl->hall_calibration;
	cxt->hallswitch_ctl.hall_threshold_setting = ctl->hall_threshold_setting;
	cxt->hallswitch_ctl.is_use_common_factory = ctl->is_use_common_factory;
	cxt->hallswitch_ctl.is_polling_mode = ctl->is_polling_mode;

	if (!hallswitch_misc_dev_init) {
		/* add misc dev for sensor hal control cmd */
		err = hallswitch_misc_init(hallswitch_context_obj);
		if (err) {
			HALLSWITCH_ERR("unable to register hallswitch misc device!!\n");
			return -2;
		}
		err = sysfs_create_group(&hallswitch_context_obj->mdev.this_device->kobj,
				&hallswitch_attribute_group);
		if (err < 0) {
			HALLSWITCH_ERR("unable to create hallswitch attribute file\n");
			return -3;
		}
		kobject_uevent(&hallswitch_context_obj->mdev.this_device->kobj, KOBJ_ADD);
		hallswitch_misc_dev_init = true;
	}
	return 0;
}

static int hallswitch_probe(void) 
{
	int err;

	HALLSWITCH_LOG("+++++++++++++hallswitch_probe!!\n");

	hallswitch_context_obj = hallswitch_context_alloc_object();
	if (!hallswitch_context_obj)
	{
		err = -ENOMEM;
		HALLSWITCH_ERR("unable to allocate devobj!\n");
		goto exit_alloc_data_failed;
	}

	//init input dev
	err = hallswitch_input_init(hallswitch_context_obj);
	if(err)
	{
		HALLSWITCH_ERR("unable to register hallswitch input device!\n");
		goto exit_alloc_input_dev_failed;
	}

	//init real alspseleration driver
	err = hallswitch_real_driver_init();
	if(err)
	{
		HALLSWITCH_ERR("hallswitch real driver init fail\n");
		goto real_driver_init_fail;
	}
  
	HALLSWITCH_LOG("----hallswitch_probe OK !!\n");
	return 0;

real_driver_init_fail:
exit_alloc_input_dev_failed:
	kfree(hallswitch_context_obj);
	hallswitch_context_obj = NULL;
exit_alloc_data_failed:
	HALLSWITCH_ERR("----hallswitch_probe fail !!!\n");
	return err;
}

static int hallswitch_remove(void)
{
	int err=0;
	HALLSWITCH_FUN(f);
	input_unregister_device(hallswitch_context_obj->idev);        
	
	if((err = misc_deregister(&hallswitch_context_obj->mdev)))
	{
		HALLSWITCH_ERR("misc_deregister fail: %d\n", err);
	}
	kfree(hallswitch_context_obj);

	return 0;
}

static int __init hallswitch_init(void) 
{
	HALLSWITCH_FUN();

	if (hallswitch_probe()) {
		HALLSWITCH_ERR("failed to register hallswitch driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit hallswitch_exit(void)
{
	hallswitch_remove();
	platform_driver_unregister(&hall_switch_driver);

}
late_initcall_sync(hallswitch_init);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("HALLSWITCH device driver");
MODULE_AUTHOR("Mediatek");

