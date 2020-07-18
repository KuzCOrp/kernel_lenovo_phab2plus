#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include "mach/mt_clkmgr.h"
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/semaphore.h>
#include <linux/proc_fs.h>	/*proc */
#include <linux/uaccess.h>

#include "m12mo_yuv_Sensor.h"

#define M12MOYUV_SPI_DEBUG
#ifdef M12MOYUV_SPI_DEBUG
#define PFX "[M12MO_SPI]"
#define SENSORDB(fmt, args...) pr_info(PFX "[%s]" fmt, __FUNCTION__,##args)
#else
#define SENSORDB(fmt, args...) 
#endif

static struct spi_device *g_pstSPIDevice = NULL;
int m12mo_laser_flag_value = 0;

void __iomem *gpio_base;
extern int fw_version_id;
extern int fw_real_id;
extern int check_sum_value;
extern struct M12MO_MIPI_sensor_struct M12MO_Driver;

unsigned int gCaptureIndex = 1; //raw data 0 or yuv data 1

static int camera_spi_sync(struct spi_message *pSpiMessage)
{
	int status = -1;
	//SENSORDB("camera_spi_sync enter \r\n");

	if (g_pstSPIDevice == NULL){
		SENSORDB("spi ==NULL");
		status = -ESHUTDOWN;
	}else{	
	status = spi_sync(g_pstSPIDevice, pSpiMessage);
	}	
	printk("camera spi sync status = %d\n",status);
	if (0 != status)
	{
		SENSORDB("camera spi sync fail \r\n");
	}

	return status;	
}

int camera_spi_write(unsigned char *pBuf, unsigned int uSize)
{
	int ret;
	struct spi_message stSPIMessage;
	struct spi_transfer transfer;    

	//SENSORDB("camera_spi_write enter dma mode...\n");
	spi_message_init(&stSPIMessage);

	transfer.tx_buf =pBuf;
	transfer.len = uSize;
	transfer.bits_per_word = 8;
	transfer.tx_nbits = 1;
	transfer.rx_buf = 0;
	printk("camera spi write transfer len = %d\n",transfer.len);	
	spi_message_add_tail(&transfer, &stSPIMessage);
	
	ret = camera_spi_sync(&stSPIMessage);

	return ret;
}
//char *filepath = "data/M12MO_FW.bin\0";
static ssize_t spi_write_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/*unsigned char 	tx[] = {
		0x12, 0x34, 0x56, 0x78, 0x90, 0xFF,
		0x40, 0x00, 0x00, 0x00, 0x00, 0x95,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xAD,
		0xF0, 0x0D,
	};
	unsigned int size = 38;*/
	int ret ;

	static char fw_data[] = 
			{
				#include "M12MO_SFW_26MHz.i"
			};
	static char *rs_m12mo_update = fw_data;
	static int fwsize = sizeof(fw_data);
	if(fwsize < 0)
	{return sprintf(buf,"read fwsize error size=%d\n",fwsize);}

	ret = camera_spi_write(rs_m12mo_update,fwsize);
	if(ret < 0)
	{
		SENSORDB("camera spi write fail!!!\n");
		return sprintf(buf,"spi write fail !!!!");
	}
	return sprintf(buf, "spi write ok size = %d\n", fwsize);
	
}

static DEVICE_ATTR(spi_write, 0444, spi_write_show, NULL);


static int camera_spi_probe(struct spi_device *spi)
{	

	SENSORDB("enter  camera_spi_probe  ++\n");		
	g_pstSPIDevice = spi;
	spi->bits_per_word = 8;
	device_create_file(&spi->dev, &dev_attr_spi_write);
	gpio_base = ioremap(0x10211000, 0x1000);
	printk("===========function(%s), line(%d),gpio_base(%p)\n", __FUNCTION__, __LINE__, gpio_base);
	printk("=====0x50(0x%x)\n", *(volatile u32 *)(gpio_base+0x50));	
	return 0;
}
static int camera_spi_remove(struct spi_device *spi)
{	
	device_remove_file(&spi->dev, &dev_attr_spi_write);
	return 0;
}

static const struct of_device_id m12mo_yuv_spidev_of_match[] = {
	{.compatible = "mediatek,m12mo_yuv_spidev",},
	{},
};
MODULE_DEVICE_TABLE(of, m12mo_yuv_spidev_of_match);

static struct spi_driver camera_spi_driver = {
	.driver = {
		.name =	"m12mo_yuv_spidev",
		.owner =	THIS_MODULE,
		.of_match_table = m12mo_yuv_spidev_of_match,
	},
	.probe = camera_spi_probe,
	.remove = camera_spi_remove,
};

#define M12MO_DEBUG_REG
#ifdef M12MO_DEBUG_REG
static unsigned int r_catagory = 0, r_byte = 0;
static int r_para_size = 0;
extern u8 M12MO_read_category_parameter_debug(u8 category, u8 byte);
extern void M12MO_write_category_parameter_debug(u8 category, u8 byte, u8 uPara);
extern u32 M12MO_read_category_parameter_4Byte_debug(u8 category, u8 byte);
extern void M12MO_write_category_parameter_4Byte_debug(u8 category, u8 byte, u32 uPara);

static ssize_t m12mo_read_reg_debug(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	char *page = NULL;
	char *ptr = NULL;
	int len,err = -1;
	unsigned int read_para = 0;
	page = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!page)
	{
		kfree(page);
		return -ENOMEM;
	}
	ptr = page;
	ptr += sprintf(ptr, "/*******M12MO read reg just write debug********/\n");

	if(r_para_size)
	{	
		read_para=M12MO_read_category_parameter_4Byte_debug(r_catagory,r_byte);
	}
	else
	{
		read_para=M12MO_read_category_parameter_debug(r_catagory,r_byte);
	}

	ptr += sprintf(ptr, "read reg para:0x%x 0x%x:0x%08x\n", r_catagory,r_byte,read_para);
	
	len = ptr - page;
	if(*ppos >= len)
	{
		kfree(page);
		return 0;
	}
	err = copy_to_user(buffer,(char *)page,len);
	*ppos += len;
	if(err)
	{
		kfree(page);
		return err;
	}
	kfree(page);
	return len;
}

static ssize_t m12mo_write_reg_debug(struct file *file, const char *buffer, size_t count,
					loff_t *ppos)
{
	char regBuf[128]="{\0}";
	unsigned int temp_category=0, temp_byte =0;
	unsigned int temp_upara=0;//,temp_rbdata=0;
	unsigned int uCopyBufSize = (count <(sizeof(regBuf)-1))?(count):(sizeof(regBuf)-1);
	  if(copy_from_user(regBuf,buffer,uCopyBufSize))	
	  	return -EFAULT;

	  if(sscanf(regBuf, "%x %x %08x", &temp_category,&temp_byte,&temp_upara) == 3)
	  {
	  		r_catagory = temp_category;
			r_byte = temp_byte;
	  	if((temp_upara & 0xffffff00) !=0)
	  		{
				M12MO_write_category_parameter_4Byte_debug(temp_category,temp_byte,temp_upara);
				//temp_rbdata=M12MO_read_category_parameter_4Byte_debug(temp_category,temp_byte);
				SENSORDB("enter M12MO_write_category_parameter_4Byte_debug \r\n");
				r_para_size = 1;
			}
		else
			{
				M12MO_write_category_parameter_debug(temp_category,temp_byte,(temp_upara&0x00ff));
				//temp_rbdata=M12MO_read_category_parameter_debug(temp_category,temp_byte);
				SENSORDB("entern M12MO_write_category_parameter_debug \r\n");
			}
				SENSORDB("temp_category = 0x%x ,temp_byte = 0x%x , temp_upara = 0x%x \r\n",temp_category,temp_byte,temp_upara);
				//SENSORDB("temp_rbdata = 0x%x \r\n",temp_rbdata);
  	}
	return count;
}

static const struct file_operations m12mo_proc_fops = {
	.write = m12mo_write_reg_debug,
	.read = m12mo_read_reg_debug
};

static ssize_t m12mo_main_led1_test_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	int temp = 0;

	sscanf(buffer, "%d", &temp);
	printk("led1_temp=%d\n",temp);
	if(temp)
	{
		M12MO_write_category_parameter_debug(0x0d, 0xf0, 0xff);
		msleep(5);
		M12MO_write_category_parameter_debug(0x0d, 0xf0, 0x01);
	}
	else
	{
		M12MO_write_category_parameter_debug(0x0d, 0xf0, 0xff);
	}
	return count;
}

static const struct file_operations m12mo_main_led1_test_proc_fops = {
	.write = m12mo_main_led1_test_write
};

static ssize_t m12mo_main_led2_test_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	int temp = 0;

	sscanf(buffer, "%d", &temp);
	printk("led2_temp=%d\n",temp);
	if(temp)
	{
		M12MO_write_category_parameter_debug(0x0d, 0xf5, 0x7f);
		msleep(1);
		M12MO_write_category_parameter_debug(0x0d, 0xf0, 0xff);
		msleep(5);
		M12MO_write_category_parameter_debug(0x0d, 0xf0, 0x02);
	}
	else
	{
		M12MO_write_category_parameter_debug(0x0d, 0xf0, 0xff);
	}
	return count;
}

static const struct file_operations m12mo_main_led2_test_proc_fops = {
	.write = m12mo_main_led2_test_write
};

static ssize_t m12mo_sub_led_test_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	int temp = 0;

	sscanf(buffer, "%d", &temp);
	printk("sub_led_temp=%d\n",temp);
	if(temp)
	{
		M12MO_write_category_parameter_debug(0x0d, 0x6e, 0x01);
	}
	else
	{
		M12MO_write_category_parameter_debug(0x0d, 0x6e, 0x00);
	}
	return count;
}

static const struct file_operations m12mo_sub_led_test_proc_fops = {
	.write = m12mo_sub_led_test_write
};

static ssize_t m12mo_laser_test_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	char *page = NULL;
	char *ptr = NULL;
	int len, err = -1;
	unsigned int read_para = 0;

	page = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!page)
	{
		kfree(page);
		return -ENOMEM;
	}
	ptr = page;

	read_para = M12MO_read_category_parameter_debug(0x0D, 0x36);

	ptr += sprintf(ptr, "0x%x\n", read_para);

	len = ptr - page;
	if (*ppos >= len)
	{
		kfree(page);
		return 0;
	}
	err = copy_to_user(buffer,(char *)page,len);
	*ppos += len;
	if (err)
	{
		kfree(page);
		return err;
	}
	kfree(page);
	return len;
}

static ssize_t m12mo_laser_test_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	char regBuf[128]="{\0}";
	unsigned int temp_category=0, temp_byte =0;
	unsigned int temp_upara=0;
	unsigned int uCopyBufSize = (count <(sizeof(regBuf)-1))?(count):(sizeof(regBuf)-1);

	if(copy_from_user(regBuf,buffer,uCopyBufSize))
		return -EFAULT;

	if(sscanf(regBuf, "%x %x %x", &temp_category,&temp_byte,&temp_upara) == 3)
	{
		printk("temp_category = 0x%x ,temp_byte = 0x%x , temp_upara = 0x%x\n",temp_category,temp_byte,temp_upara);
		M12MO_write_category_parameter_debug(temp_category, temp_byte, temp_upara);
	}

	return count;
}

static const struct file_operations m12mo_laser_test_proc_fops = {
	.write = m12mo_laser_test_write,
	.read = m12mo_laser_test_read
};

static ssize_t m12mo_laser_flag_test_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	sscanf(buffer, "%d", &m12mo_laser_flag_value);
	printk("m12mo_laser_flag_value = %d\n", m12mo_laser_flag_value);
	return count;
}

static const struct file_operations m12mo_laser_flag_test_proc_fops = {
	.write = m12mo_laser_flag_test_write
};

static ssize_t m12mo_fw_version_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	char *page = NULL;
	char *ptr = NULL;
	int len, err = -1;

	page = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!page)
	{
		kfree(page);
		return -ENOMEM;
	}
	ptr = page;

	ptr += sprintf(ptr, "first fw = 0x%x,real fw = 0x%x,checksum = 0x%02x\n", fw_version_id,fw_real_id,check_sum_value);

	len = ptr - page;
	if (*ppos >= len)
	{
		kfree(page);
		return 0;
	}
	err = copy_to_user(buffer,(char *)page,len);
	*ppos += len;
	if (err)
	{
		kfree(page);
		return err;
	}
	kfree(page);
	return len;
}

static const struct file_operations m12mo_fw_version_proc_fops = {
	.write = NULL,
	.read = m12mo_fw_version_read
};
static ssize_t m12mo_get_raw_data_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	int temp = 0;

	sscanf(buffer, "%d", &temp);
	SENSORDB("raw data temp=%d\n",temp);
	gCaptureIndex = temp;
	return count;
}

static const struct file_operations m12mo_get_raw_data_proc_fops = {
	.write = m12mo_get_raw_data_write
};

static ssize_t m12mo_module_info_read_proc(struct file *file, char *buffer, size_t count,  loff_t *ppos)
{
	char *page = NULL;
	char *ptr = NULL;
	char *vender_string=NULL;
	char *rear_vender_string=NULL;
	char *front_vender_string=NULL;
	char *stage_string=NULL;
	char *lens_string = NULL;
	char *front_lens_string = NULL;
	char *vcm_string = NULL;
	char *driver_ic_string = NULL;
	char *ir_bg_string = NULL;
	char *color_temper_string = NULL;
	char *af_ff_string = NULL;
	char *light_source_string = NULL;
	char pdaf_pixel_flag = 0;
	char pdaf_calibretion_flag = 0;
	char depth_flag =0;
	char basic_info_flag = 0;
	unsigned int af_value = 0;
	int len,err = -1;
	  page = kmalloc(PAGE_SIZE, GFP_KERNEL);	
	  if (!page) 
	  {		
		kfree(page);		
		return -ENOMEM;	
	  }
	if (M12MO_Driver.s5k2m8.supplier_id == 0x01)
	{
		vender_string = "sunny";
	}
	else
	{
		vender_string = "Unknow";
	}
	
	if (M12MO_Driver.ov13850.module_id == 0x01)
	{
		rear_vender_string = "sunny";
	}
	else
	{
		rear_vender_string = "Unknow";
	}

	if (M12MO_Driver.ov8865.module_id == 0x07)
	{
		front_vender_string = "o-film";
	}
	else
	{
		front_vender_string = "Unknow";
		
	}
	
	if(M12MO_Driver.s5k2m8.build_stage == 0x11)
	{
		stage_string = "EVT";
	}
	else if(M12MO_Driver.s5k2m8.build_stage == 0x21)
	{
		stage_string = "DVT1";
	}
	else if(M12MO_Driver.s5k2m8.build_stage == 0x22)
	{
		stage_string = "DVT2";
	}
	else if(M12MO_Driver.s5k2m8.build_stage == 0x31)
	{
		stage_string = "PVT1";
	}
	else if(M12MO_Driver.s5k2m8.build_stage == 0x32)
	{
		stage_string = "PVT2";
	}
	else if(M12MO_Driver.s5k2m8.build_stage == 0xF1)
	{
		stage_string = "MP";
	}
	else
	{
		stage_string = "Unknow";
	}

	if(M12MO_Driver.ov13850.lens_id == 0x1C)
	{
		lens_string = "sunny3902";
	}
	else
	{
		lens_string = "Unknow";
	}

	 if(M12MO_Driver.ov8865.lens_id == 0x23)
	{
		front_lens_string = "Largan 50093B13";
	}
	else
	{
		front_lens_string = "Unknow";
	}
	
	if(M12MO_Driver.ov13850.vcm_id== 0x0C)
	{
		vcm_string = "TDK668ABC";
	}
	else
	{
		vcm_string = "Unknow";
	}
	
	if(M12MO_Driver.ov13850.driver_ic_id== 0x03)
	{
		driver_ic_string = "DW9761B";
	}
	else if(M12MO_Driver.ov13850.driver_ic_id== 0x08)
	{
		driver_ic_string = "DW9763";
	}
	else
	{
		driver_ic_string = "Unknow";
	}

	if(M12MO_Driver.ov13850.ir_bg_id == 0x01)
	{
		ir_bg_string = "IR";
	}
	else if(M12MO_Driver.ov13850.ir_bg_id == 0x02)
	{
		ir_bg_string = "BG";
	}
	else
	{
		ir_bg_string = "Unknow";
	}

	if(M12MO_Driver.ov13850.color_temperature_id == 0x01)
	{
		color_temper_string = "5100K";
	}
	else
	{
		color_temper_string = "Unknow";
	}

	if(M12MO_Driver.ov13850.af_ff_flag == 0x01)
	{
		af_ff_string = "AF";
	}
	else if(M12MO_Driver.ov13850.af_ff_flag== 0x00)
	{
		af_ff_string = "FF";
	}
	else
	{
		af_ff_string = "Unknow";
	}

	if(M12MO_Driver.ov13850.light_source_flag == 0x01)
	{
		light_source_string = "DNP";
	}
	else if(M12MO_Driver.ov13850.light_source_flag == 0x02)
	{
		light_source_string = "ACL-G4C";
	}
	else
	{
		light_source_string = "Unknow";
	}
	af_value = (M12MO_Driver.ov13850.af_50cm_value_h << 8) |M12MO_Driver.ov13850.af_50cm_value_l ;
	pdaf_pixel_flag =( M12MO_Driver.s5k2m8.pdaf_pixel_flag & 0xC0) | ( M12MO_Driver.s5k2m8.pdaf_pixel_flag & 0x30) | ( M12MO_Driver.s5k2m8.pdaf_pixel_flag & 0x0C); 
	pdaf_calibretion_flag =( M12MO_Driver.s5k2m8.pdaf_calibration_flag& 0xC0) | ( M12MO_Driver.s5k2m8.pdaf_calibration_flag & 0x30) | ( M12MO_Driver.s5k2m8.pdaf_calibration_flag & 0x0C); 
	depth_flag =( M12MO_Driver.s5k2m8.depthmap_flag& 0xC0) | ( M12MO_Driver.s5k2m8.depthmap_flag & 0x30) | ( M12MO_Driver.s5k2m8.depthmap_flag & 0x0C); 
	basic_info_flag = ( M12MO_Driver.ov8865.basic_info_flag & 0xC0) |( M12MO_Driver.ov8865.basic_info_flag & 0x30) |( M12MO_Driver.ov8865.basic_info_flag & 0x0C);
	ptr = page;
	ptr += sprintf(ptr, "[S5K2M8] [Vendor] %s, [Build Stage] %s, [PDAF Pixel Flag] 0x%x, [PDAF CALIBRATION FLAG] 0x%x, [Depth Flag] 0x%x\n", 
		vender_string,stage_string,pdaf_pixel_flag,pdaf_calibretion_flag,depth_flag);

	ptr += sprintf(ptr, "[OV13850] [Vendor] %s, [LSC_AWB_FLAG] %d, [Product Time] 20%d.%d.%d, [Lens] %s, [VCM] %s, [DRIVER IC] %s, [IR BG] %s, [COLOR TEMPER] %s, [AF FF] %s, [LIGHT SOURCE] %s,[AF_50CM_VALUE]%d\n", 
		rear_vender_string,M12MO_Driver.ov13850.lsc_awb_flag,M12MO_Driver.ov13850.year,M12MO_Driver.ov13850.month,M12MO_Driver.ov13850.day,lens_string,vcm_string,driver_ic_string,ir_bg_string,color_temper_string,af_ff_string,light_source_string,af_value);

	ptr += sprintf(ptr, "[OV8865] [Vendor] %s, [Basic Information] 0x%x, [Product Time] 20%d.%d.%d, [Lens] %s\n", 
		front_vender_string,basic_info_flag,M12MO_Driver.ov8865.year,M12MO_Driver.ov8865.month,M12MO_Driver.ov8865.day,front_lens_string);
	len = ptr - page; 			 	
	  if(*ppos >= len)
	  {		
		  kfree(page); 		
		  return 0; 	
	  }	
	  err = copy_to_user(buffer,(char *)page,len); 			
	  *ppos += len; 	
	  if(err) 
	  {		
	    kfree(page); 		
		  return err; 	
	  }	
	  kfree(page); 	
	  return len;

}

static const struct file_operations m12mo_module_info_proc_fops = {
	.write = NULL,
	.read = m12mo_module_info_read_proc
};

#endif
static int __init spi_add_driver(void)
{
	int status;
	
	status = spi_register_driver(&camera_spi_driver);
	#ifdef M12MO_DEBUG_REG
	proc_create("driver/m12mo_debug_reg", 0666, NULL,&m12mo_proc_fops);
	proc_create("driver/m12mo_main_led1_test", 0220, NULL, &m12mo_main_led1_test_proc_fops);
	proc_create("driver/m12mo_main_led2_test", 0220, NULL, &m12mo_main_led2_test_proc_fops);
	proc_create("driver/m12mo_sub_led_test", 0220, NULL, &m12mo_sub_led_test_proc_fops);
	proc_create("driver/m12mo_laser_test", 0660, NULL, &m12mo_laser_test_proc_fops);
	proc_create("driver/m12mo_fw_version", 0444, NULL, &m12mo_fw_version_proc_fops);
	proc_create("driver/m12mo_get_raw_data", 0222, NULL, &m12mo_get_raw_data_proc_fops);
	proc_create("driver/m12mo_laser_flag", 0220, NULL, &m12mo_laser_flag_test_proc_fops);
	proc_create("driver/m12mo_module_info", 0444, NULL,&m12mo_module_info_proc_fops);
	#endif
	return status;
}
static void __exit spi_remove_driver(void)
{	
	spi_unregister_driver(&camera_spi_driver);
}

module_init(spi_add_driver);
module_exit(spi_remove_driver);

MODULE_DESCRIPTION ( "isp SPI test device driver" );
MODULE_AUTHOR ( "lct" );
MODULE_LICENSE("GPL");
