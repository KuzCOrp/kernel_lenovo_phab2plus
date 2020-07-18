
#include "inc/flightsense.h"
struct flightsense_context *flightsense_context_obj = NULL;
static struct platform_device *pltfm_dev;

static struct flightsense_init_info* flightsense_init_list[MAX_CHOOSE_FLIGHTSENSE_NUM]= {0}; //modified

static bool flightsense_misc_dev_init;

int flightsense_data_report(struct input_dev *dev, int value,int status)
{
	FLIGHTSENSE_LOG("+flightsense_data_report! %d, %d\n",value,status);
	input_report_rel(dev, EVENT_TYPE_FLIGHTSENSE_VALUE, (value+1));
	input_report_rel(dev, EVENT_TYPE_FLIGHTSENSE_STATUS, status);
	input_sync(dev); 
	return 0;
}

static struct flightsense_context *flightsense_context_alloc_object(void)
{
	
	struct flightsense_context *obj = kzalloc(sizeof(*obj), GFP_KERNEL); 
    	FLIGHTSENSE_LOG("flightsense_context_alloc_object++++\n");
	if(!obj)
	{
		FLIGHTSENSE_ERR("Alloc flightsense object error!\n");
		return NULL;
	}	
	
	FLIGHTSENSE_LOG("flightsense_context_alloc_object----\n");
	return obj;
}

static ssize_t flight_store_active(struct device* dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t flight_show_active(struct device* dev, struct device_attribute *attr, char *buf) 
{
	return 1;
}

static ssize_t flight_store_delay(struct device* dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t flight_show_delay(struct device* dev, struct device_attribute *attr, char *buf) 
{
	return 0;
}


static ssize_t flight_store_batch(struct device* dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}

static ssize_t flight_show_batch(struct device* dev, struct device_attribute *attr, char *buf) 
{
	return 0; 
}

static ssize_t flight_store_flush(struct device* dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}

static ssize_t flight_show_flush(struct device* dev, struct device_attribute *attr, char *buf) 
{
	return 0;
}

static ssize_t flight_show_devnum(struct device* dev, struct device_attribute *attr, char *buf) 
{
	const char *devname =NULL;
	devname = dev_name(&flightsense_context_obj->idev->dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", devname+5); 
}

static int flight_sense_remove(struct platform_device *pdev)
{
	FLIGHTSENSE_LOG("flight_sense_remove\n");
	return 0;
}

static int flight_sense_probe(struct platform_device *pdev) 
{
	FLIGHTSENSE_LOG("flight_sense_probe\n");
	pltfm_dev = pdev;
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id flight_sense_of_match[] = {
	{ .compatible = "mediatek,flight_sense", },
	{},
};
#endif

static struct platform_driver flight_sense_driver = {
	.probe      = flight_sense_probe,
	.remove     = flight_sense_remove,    
	.driver     = 
	{
		.name  = "flight_sense",
        #ifdef CONFIG_OF
		.of_match_table = flight_sense_of_match,
		#endif
	}
};

static int flightsense_real_driver_init(void) 
{
	int i = 0;
	int err = 0;

	FLIGHTSENSE_LOG(" flightsense_real_driver_init +\n");
	for (i = 0; i < MAX_CHOOSE_FLIGHTSENSE_NUM; i++) {
		FLIGHTSENSE_LOG("flightsense_real_driver_init i=%d\n", i);
		if (0 != flightsense_init_list[i]) {
			FLIGHTSENSE_LOG(" flightsense try to init driver %s\n", flightsense_init_list[i]->name);
			err = flightsense_init_list[i]->init();
			if (0 == err) {
				FLIGHTSENSE_LOG(" flightsense real driver %s probe ok\n", flightsense_init_list[i]->name);
				break;
			}
		}
	}

	if (i == MAX_CHOOSE_FLIGHTSENSE_NUM) {
		FLIGHTSENSE_LOG(" flightsense_real_driver_init fail\n");
		err =  -1;
	}

	return err;
}

  int flightsense_driver_add(struct flightsense_init_info* obj) 
{
	int err = 0;
	int i = 0;

	FLIGHTSENSE_FUN();
	if (!obj) {
		FLIGHTSENSE_ERR("FLIGHTSENSE driver add fail, flightsense_init_info is NULL\n");
		return -1;
	}

	for (i = 0; i < MAX_CHOOSE_FLIGHTSENSE_NUM; i++) {
		if ((i == 0) && (NULL == flightsense_init_list[0])) {
			FLIGHTSENSE_LOG("register flightsense driver for the first time\n");
			if (platform_driver_register(&flight_sense_driver))
				FLIGHTSENSE_ERR("failed to register flightsense driver already exist\n");
		}

		if (NULL == flightsense_init_list[i]) {
			obj->platform_diver_addr = &flight_sense_driver;
			flightsense_init_list[i] = obj;
			break;
		}
	}
	if (i >= MAX_CHOOSE_FLIGHTSENSE_NUM) {
		FLIGHTSENSE_ERR("FLIGHTSENSE driver add err\n");
		err =  -1;
	}

	return err;
}
EXPORT_SYMBOL_GPL(flightsense_driver_add);
struct platform_device *get_flightsense_platformdev(void)
{
	return pltfm_dev;
}

int flightsense_report_interrupt_data(int value) 
{
	struct flightsense_context *cxt = NULL;
	//int err =0;
	cxt = flightsense_context_obj;	

	flightsense_data_report(cxt->idev,value,3);
	
	return 0;
}
/*----------------------------------------------------------------------------*/
EXPORT_SYMBOL_GPL(flightsense_report_interrupt_data);

static int flightsense_misc_init(struct flightsense_context *cxt)
{

    int err=0;
    cxt->mdev.minor = MISC_DYNAMIC_MINOR;
	//cxt->mdev.name  = FLIGHTSENSE_MISC_DEV_NAME;
	if((err = misc_register(&cxt->mdev)))
	{
		FLIGHTSENSE_ERR("unable to register flightsense misc device!!\n");
	}
	return err;
}

static int flightsense_input_init(struct flightsense_context *cxt)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	//dev->name = FLIGHTSENSE_INPUTDEV_NAME;

	set_bit(EV_REL, dev->evbit);
	set_bit(EV_SYN, dev->evbit);
	input_set_capability(dev, EV_REL, EVENT_TYPE_FLIGHTSENSE_VALUE);
	input_set_capability(dev, EV_REL, EVENT_TYPE_FLIGHTSENSE_STATUS);
	
	input_set_drvdata(dev, cxt);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	cxt->idev= dev;

	return 0;
}

DEVICE_ATTR(flightactive,     		S_IWUSR | S_IRUGO, flight_show_active, flight_store_active);
DEVICE_ATTR(flightdelay,      		S_IWUSR | S_IRUGO, flight_show_delay,  flight_store_delay);
DEVICE_ATTR(flightbatch,      		S_IWUSR | S_IRUGO, flight_show_batch,  flight_store_batch);
DEVICE_ATTR(flightflush,      		S_IWUSR | S_IRUGO, flight_show_flush,  flight_store_flush);
DEVICE_ATTR(flightdevnum,      		S_IWUSR | S_IRUGO, flight_show_devnum,  NULL);

static struct attribute *flightsense_attributes[] = {
	&dev_attr_flightactive.attr,
	&dev_attr_flightdelay.attr,
	&dev_attr_flightbatch.attr,
	&dev_attr_flightflush.attr,
	&dev_attr_flightdevnum.attr,
	NULL
};

static struct attribute_group flightsense_attribute_group = {
	.attrs = flightsense_attributes
};

int flightsense_register_control_path(struct flightsense_control_path *ctl)
{
	struct flightsense_context *cxt = NULL;
	int err = 0;

	cxt = flightsense_context_obj;
	cxt->flightsense_ctl.set_delay = ctl->set_delay;
	cxt->flightsense_ctl.open_report_data = ctl->open_report_data;
	cxt->flightsense_ctl.enable_nodata = ctl->enable_nodata;
	cxt->flightsense_ctl.is_support_batch = ctl->is_support_batch;
	cxt->flightsense_ctl.is_report_input_direct = ctl->is_report_input_direct;
	cxt->flightsense_ctl.flight_calibration = ctl->flight_calibration;
	cxt->flightsense_ctl.flight_threshold_setting = ctl->flight_threshold_setting;
	cxt->flightsense_ctl.is_use_common_factory = ctl->is_use_common_factory;
	cxt->flightsense_ctl.is_polling_mode = ctl->is_polling_mode;

	if (!flightsense_misc_dev_init) {
		/* add misc dev for sensor hal control cmd */
		err = flightsense_misc_init(flightsense_context_obj);
		if (err) {
			FLIGHTSENSE_ERR("unable to register flightsense misc device!!\n");
			return -2;
		}
		err = sysfs_create_group(&flightsense_context_obj->mdev.this_device->kobj,
				&flightsense_attribute_group);
		if (err < 0) {
			FLIGHTSENSE_ERR("unable to create flightsense attribute file\n");
			return -3;
		}
		kobject_uevent(&flightsense_context_obj->mdev.this_device->kobj, KOBJ_ADD);
		flightsense_misc_dev_init = true;
	}
	return 0;
}

static int flightsense_probe(void) 
{
	int err;

	FLIGHTSENSE_LOG("+++++++++++++flightsense_probe!!\n");

	flightsense_context_obj = flightsense_context_alloc_object();
	if (!flightsense_context_obj)
	{
		err = -ENOMEM;
		FLIGHTSENSE_ERR("unable to allocate devobj!\n");
		goto exit_alloc_data_failed;
	}

	//init input dev
	err = flightsense_input_init(flightsense_context_obj);
	if(err)
	{
		FLIGHTSENSE_ERR("unable to register flightsense input device!\n");
		goto exit_alloc_input_dev_failed;
	}

	//init real alspseleration driver
	err = flightsense_real_driver_init();
	if(err)
	{
		FLIGHTSENSE_ERR("flightsense real driver init fail\n");
		goto real_driver_init_fail;
	}
  
	FLIGHTSENSE_LOG("----flightsense_probe OK !!\n");
	return 0;

real_driver_init_fail:
exit_alloc_input_dev_failed:
	kfree(flightsense_context_obj);
	flightsense_context_obj = NULL;
exit_alloc_data_failed:
	FLIGHTSENSE_ERR("----flightsense_probe fail !!!\n");
	return err;
}

static int flightsense_remove(void)
{
	int err=0;
	FLIGHTSENSE_FUN(f);
	input_unregister_device(flightsense_context_obj->idev);        
	
	if((err = misc_deregister(&flightsense_context_obj->mdev)))
	{
		FLIGHTSENSE_ERR("misc_deregister fail: %d\n", err);
	}
	kfree(flightsense_context_obj);

	return 0;
}

static int __init flightsense_init(void) 
{
	FLIGHTSENSE_FUN();

	if (flightsense_probe()) {
		FLIGHTSENSE_ERR("failed to register flightsense driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit flightsense_exit(void)
{
	flightsense_remove();
	platform_driver_unregister(&flight_sense_driver);

}
late_initcall(flightsense_init);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FLIGHTSENSE device driver");
MODULE_AUTHOR("Mediatek");

