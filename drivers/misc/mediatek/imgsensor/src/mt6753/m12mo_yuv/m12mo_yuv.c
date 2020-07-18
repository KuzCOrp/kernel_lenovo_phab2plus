#include <linux/vmalloc.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/stat.h>
#include <linux/fs.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>

#include "m12mo_yuv_Sensor.h"

#define Sleep(ms) mdelay(ms)
#define M12MOYUV_DEBUG

#ifdef M12MOYUV_DEBUG
#define PFX "[M12MO]"
#define SENSORDB(fmt, args...) pr_info(PFX "[%s]" fmt, __FUNCTION__,##args)
#else
#define SENSORDB(fmt, args...) 
#endif

#ifdef SLT_DEVINFO_CMM
#include  <linux/dev_info.h>
static struct devinfo_struct *s_DEVINFO_ccm,*s_DEVINFO_ccm1, *s_DEVINFO_ccm2 = NULL;
#endif
static char filepath[] = "system/etc/RS_M12MO.bin\0";
static int m12mo_fwsize = 0;
char * rs_ms2_update = NULL;
static volatile M12MO_SYS_MODE g_eSysMode = SYS_MODE_POWEROFF;
static MSDK_SENSOR_CONFIG_STRUCT M12MOSensorConfigData;
MSDK_SCENARIO_ID_ENUM M12MOCurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
static int g_bIsStartlongexposertime = 0;
static M12MO_CAMERA_MODE g_eCameramode = M12MO_MAIN_CAMERA;
static DEFINE_SPINLOCK(m12mo_drv_lock);
//static kal_bool g_main_status = KAL_FALSE;
struct M12MO_MIPI_sensor_struct M12MO_Driver;
int fw_version_id,fw_real_id = 0;
int check_sum_value = -1;
extern int m12mo_laser_flag_value;


extern int camera_spi_write(unsigned char *pBuf, unsigned int uSize);
extern int mtkcam_gpio_set(int PinIdx, int PwrType, int Val);
extern int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char *mode_name);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId);
#define m12mo_multi_dma_write_cmos_sensor(pData, lens)	 iBurstWriteReg((u8*) pData, (u16) lens, M12MO_WRITE_ID)

extern unsigned int gCameraIndex; //rear camera 1 sansung or 0 ov

extern unsigned int gCaptureIndex; //raw data 0 or yuv data 1


static M12MO_AAA_STATUS_ENUM M12MO_Cancel_AF(void);

extern void stmvl53l0_read_calibration_to_isp(void);

void M12MO_Set_CameraMode(M12MO_CAMERA_MODE mode)
{
	g_eCameramode = mode;
}
u8 M12MO_Get_CameraMode(void)
{
	return g_eCameramode;
}

/*--------------------- read / write memory ---------------------*/
/*void fusjutong_write(char * write_data_head, kal_uint8 data_size)
{
	kal_uint8 i = 0;
	for (;i< data_size; i++)
	{
		SENSORDB(" write_data_head[%d] is 0x%x\r\n", i, write_data_head[i]);
	}
	 iWriteRegI2C(write_data_head , data_size,M12MO_WRITE_ID);
}
*/

/*static void M12MO_read_memory(u32 addr, u8 *pReadData, u32 uDataSize)
{
	u8 uSendCmd[8] = {0};

	u8 * pTempBuff = (u8 *)kmalloc((MAX_MEMORY_DATA_SIZE+3), GFP_KERNEL);
	if (NULL == pTempBuff)
	{
		SENSORDB("[M12MO_read_memory]----kmalloc  fail \r\n");
		return;
	}

	if ((0 == addr) || (0 == uDataSize))
	{
		SENSORDB("[M12MO_read_memory] Parameter Invalid \r\n");
		return;
	}

	while (uDataSize > 0)
	{
		*uSendCmd = 0x00;
		*(uSendCmd+1) = COMMAND_ID_READ_MEMORY_8BIT_ACCESS;
		*(uSendCmd+2) = (addr >> 24) & 0xFF;
		*(uSendCmd+3) = (addr >> 16) & 0xFF;
		*(uSendCmd+4) = (addr >> 8) & 0xFF;
		*(uSendCmd+5) = addr & 0xFF;

		if (uDataSize >= MAX_MEMORY_DATA_SIZE)
		{
			*(uSendCmd+6) = MAX_MEMORY_DATA_SIZE  >> 8;
			*(uSendCmd+7) = MAX_MEMORY_DATA_SIZE & 0xFF;

			//fusjutong_write(uSendCmd, 8);
			//ms2r_multi_dma_read_cmos_sensor(pTempBuff, MAX_MEMORY_DATA_SIZE+3);
			iReadRegI2C(uSendCmd, 8, pTempBuff, MAX_MEMORY_DATA_SIZE+3, M12MO_WRITE_ID);
			memcpy(pReadData, pTempBuff+3, MAX_MEMORY_DATA_SIZE);

			addr += MAX_MEMORY_DATA_SIZE;
			pReadData += MAX_MEMORY_DATA_SIZE;
			uDataSize -= MAX_MEMORY_DATA_SIZE;
		}
		else
		{
			*(uSendCmd+6) = (u8)(uDataSize >> 8);
			*(uSendCmd+7) = (u8)(uDataSize & 0xFF);
			//fusjutong_write(uSendCmd, 8);
			//ms2r_multi_dma_read_cmos_sensor(pTempBuff, uDataSize+3);
			iReadRegI2C(uSendCmd, 8, pTempBuff, uDataSize+3, M12MO_WRITE_ID);
			memcpy(pReadData, pTempBuff+3, uDataSize);
			
			uDataSize = 0;
		}
		
	}

	if (NULL != pTempBuff)
	{
		kfree(pTempBuff);
	}
	
}
*/
static void M12MO_write_memory(u32 addr, u8 *pSendData, u32 uDataSize)
{	
	int nCount = 1;
	char *pSendCmd = NULL;
	if ((0 == addr) || (NULL == pSendData) || (0 == uDataSize))
	{
		SENSORDB("[M12MO_write_memory] Parameter Invalid \r\n");
		return;
	}
	
	pSendCmd = (char *)kmalloc(MAX_MEMORY_DATA_SIZE+8, GFP_KERNEL);
	if (NULL == pSendCmd)
	{
		SENSORDB("[M12MO_write_memory]----kmalloc  fail \r\n");
		return;
	}

	while (uDataSize > 0)
	{
		*pSendCmd = 0x00;
		*(pSendCmd+1) = COMMAND_ID_WRITE_MEMORY_8BIT_ACCESS;
		*(pSendCmd+2) = (addr >> 24) & 0xFF;
		*(pSendCmd+3) = (addr >> 16) & 0xFF;
		*(pSendCmd+4) = (addr >> 8) & 0xFF;
		*(pSendCmd+5) = addr & 0xFF;

		if (uDataSize >= MAX_MEMORY_DATA_SIZE)
		{
			SENSORDB("write to memory ----[%d]---addr is 0x%x----uDataSize is %x\r\n", nCount, addr,uDataSize);
			nCount++;
			
			*(pSendCmd+6) = MAX_MEMORY_DATA_SIZE >> 8;
			*(pSendCmd+7) = MAX_MEMORY_DATA_SIZE & 0xFF;
			
			memcpy(pSendCmd+8, pSendData, MAX_MEMORY_DATA_SIZE);
			m12mo_multi_dma_write_cmos_sensor(pSendCmd, MAX_MEMORY_DATA_SIZE+8);


			pSendData += MAX_MEMORY_DATA_SIZE;
			addr += MAX_MEMORY_DATA_SIZE;
			uDataSize = uDataSize - MAX_MEMORY_DATA_SIZE;
		}
		else
		{
			SENSORDB("the last bytes transfer start  addr is 0x%x----uDataSize is %x \r\n", addr,uDataSize);
			*(pSendCmd+6) = (u8)(uDataSize >> 8);
			*(pSendCmd+7) = (u8)(uDataSize & 0xFF);

			memcpy(pSendCmd+8, pSendData, uDataSize);
			m12mo_multi_dma_write_cmos_sensor(pSendCmd, uDataSize+8);


			SENSORDB("the last bytes transfer finish \r\n");

			uDataSize = 0;
		}
	}

	if (NULL != pSendCmd)
	{
		kfree(pSendCmd);
	}
	
}

/*-------------  read / write category parameter  ----------*/
static u8 M12MO_read_category_parameter_ram(u8 category, u8 byte)
{
	u8 uSendCmd[5] = 
	{
		0x05, 
		COMMAND_ID_READ_CATEGORY_PARAMETER, 
		category, 
		byte, 
		1
	};
	int ret;
	u8 uGetData[2] = {0};

	uGetData[1]  = 0xff;
	ret = iReadRegI2C(uSendCmd, 5, uGetData, 2, M12MO_WRITE_ID);

	SENSORDB("uGetData[0] is 0x%x, uGetData[1] is 0x%x,\r\n", uGetData[0], uGetData[1]);
	return uGetData[1];
	
}

static void M12MO_write_category_parameter_ram(u8 category, u8 byte, u8 uPara)
{
	u8 uSendCmd[5] =
	{
		0x05, 
		COMMAND_ID_WRITE_CATEGORY_PARAMETER, 
		category, 
		byte, 
		uPara
	};
	Sleep(1);
	SENSORDB("uSendCmd [0x%x, 0x%x,0x%x,0x%x,0x%x]\r\n", uSendCmd[0], uSendCmd[1],uSendCmd[2],uSendCmd[3],uSendCmd[4]);
	iWriteRegI2C(uSendCmd, 5, M12MO_WRITE_ID);
	
}

static u16 M12MO_read_category_parameter_2byte(u8 category, u8 byte)
{
	u8 uSendCmd[5] = 
	{
		0x05, 
		COMMAND_ID_READ_CATEGORY_PARAMETER, 
		category, 
		byte, 
		2
	};
	int ret;
	u8 uGetData[3] = {0};
	u16 byte2return = 0;
	uGetData[0] = 0xff;
	uGetData[1] = 0xff;
	uGetData[2] = 0xff;

	ret = iReadRegI2C(uSendCmd, 5, uGetData, 3, M12MO_WRITE_ID);
	
	SENSORDB("uSendCmd [0x%x, 0x%x,0x%x,0x%x,0x%x], read resulte [0x%x, 0x%x,0x%x]\r\n", uSendCmd[0], uSendCmd[1],uSendCmd[2],uSendCmd[3],uSendCmd[4],uGetData[0],uGetData[1],uGetData[2]);
	return byte2return = (uGetData[1]<<8 | uGetData[2]);
	
}



static u32 M12MO_read_category_parameter_4Byte(u8 category, u8 byte)
{
	u32 uReturnValue = 0;
	u8 uSendCmd[5] = 
	{
		0x05, 
		COMMAND_ID_READ_CATEGORY_PARAMETER, 
		category, 
		byte, 
		4
	};
	u8 uGetData[5] = {0};

	iReadRegI2C(uSendCmd, 5, uGetData, 5, M12MO_WRITE_ID);
	
	SENSORDB("uGetData[0] is 0x%x, uGetData[1] is 0x%x,uGetData[2] is 0x%x, uGetData[3] is 0x%x,uGetData[5] is 0x%x\n", uGetData[0], uGetData[1],uGetData[2], uGetData[3],uGetData[4]);
	
	uReturnValue = ((u32)uGetData[1] << 24)
				    | ((u32)uGetData[2] << 16)
				    | ((u32)uGetData[3] << 8)
				    | (u32)uGetData[4] ;
	
	return uReturnValue;
	
}

static void M12MO_write_category_parameter_4Byte(u8 category, u8 byte, u32 uPara)
{
	u8 uSendCmd[8] = 
	{
		8, 
		COMMAND_ID_WRITE_CATEGORY_PARAMETER, 
		category, 
		byte,
		(u8)((uPara>>24)&0xFF),
		(u8)((uPara>>16)&0xFF),
		(u8)((uPara>>8)&0xFF),
		(u8)(uPara&0xFF)
	};
	SENSORDB("uSendCmd [0x%x, 0x%x,0x%x,0x%x,0x%x]\r\n", uSendCmd[0], uSendCmd[1],uSendCmd[2],uSendCmd[3],uPara);
	iWriteRegI2C(uSendCmd, 8, M12MO_WRITE_ID);
	
}


#ifdef M12MO_DEBUG_REG
		 u8 M12MO_read_category_parameter_debug(u8 category, u8 byte)
			{
				return M12MO_read_category_parameter_ram( category,  byte); 
			}
		
		void M12MO_write_category_parameter_debug( u8 category,  u8 byte,  u8 uPara)
			{
				M12MO_write_category_parameter_ram(category, byte, uPara);
			}
		
		u32 M12MO_read_category_parameter_4Byte_debug(u8 category, u8 byte)
			{
				return M12MO_read_category_parameter_4Byte( category,  byte);
			}
		
		void M12MO_write_category_parameter_4Byte_debug(u8 category, u8 byte, u32 uPara)
			{
				M12MO_write_category_parameter_4Byte( category,  byte,  uPara);
			}
#endif

 int fusjutong_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s", firmware_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		SENSORDB("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

 int fusjutong_ReadFirmware(char *firmware_name,
			       unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		SENSORDB("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}


int M12MO_get_sum(void)
{
	int sum = 0;
	sum = M12MO_read_category_parameter_2byte(0x0F, 0x0A);
	SENSORDB("[m12mo_get_sum]    sum is 0x%x \r\n", sum);
	check_sum_value = sum;
	return sum;
}
kal_uint16 M12MO_DownLoadFW_ToRAM(void)
{
	int ret = 0,count = 0;
	char *pSendSpiData = NULL;

	SENSORDB(" camera isp firmware size = %d \n",m12mo_fwsize);
	pSendSpiData = (char *)kmalloc(MAX_SPI_TRANSFER_SIZE, GFP_KERNEL);
	//send camera firmware
	SENSORDB("----send firmware by SPI start --->>>>  \n");
	mtkcam_gpio_set(0,7,0);
	Sleep(1);
	while(m12mo_fwsize>0)
	{
		if (m12mo_fwsize >= MAX_SPI_TRANSFER_SIZE)
		{
			count++;
			
			memcpy(pSendSpiData, rs_ms2_update, MAX_SPI_TRANSFER_SIZE);
			SENSORDB("write spi ----[%d]---fwsize is %x,SendData is %x\n", count, m12mo_fwsize,pSendSpiData[0]);
			ret = camera_spi_write(pSendSpiData,MAX_SPI_TRANSFER_SIZE);
			if(ret < 0)
			{
				SENSORDB("camera spi write fail!!!\n");
				return -1;
			}

			rs_ms2_update += MAX_SPI_TRANSFER_SIZE;
			m12mo_fwsize -= MAX_SPI_TRANSFER_SIZE;
		}
		else
		{
			SENSORDB("the last bytes transfer start  fwsize is %x  SendData is %x\n", m12mo_fwsize,rs_ms2_update[0]);

			memcpy(pSendSpiData, rs_ms2_update, m12mo_fwsize);
			ret = camera_spi_write(pSendSpiData,m12mo_fwsize);
			if(ret < 0)
			{
				SENSORDB("camera spi write fail!!!\n");
				return -1;
			}

			SENSORDB("the last bytes transfer finish \r\n");

			m12mo_fwsize = 0;
		}
	}

	mtkcam_gpio_set(0,7,1);
	 if (NULL != pSendSpiData)
	{
		kfree(pSendSpiData);
	}
	 if (rs_ms2_update != NULL)
	{
		vfree(rs_ms2_update);
		rs_ms2_update = NULL;
	}
	SENSORDB("----send firmware by SPI finish ---<<<<  \r\n");
	return 0;
	}


static kal_bool M12MO_send_cameraFW_by_SIO_mode(void)
{
	kal_uint8 retry = 50;
	int INT_status = 0;
	static char fw_sio_data[] = 
		{
			#include "M12MO_SFW_26MHz.i"
		};
	static char *m12mo_sio_fw = fw_sio_data;
	static int sio_fwsize = sizeof(fw_sio_data);

	INT_status = M12MO_read_category_parameter_ram(0x0F, 0x10);
	while((INT_status !=0x01) && retry)
		{
			INT_status = M12MO_read_category_parameter_ram(0x0F,0x10);
			SENSORDB("(0x0F, 0x10) is 0x%x\n", INT_status);
			Sleep(20);
			retry--;
		}
		
	if((INT_status !=0x01) && (retry==0))
		{
			 SENSORDB("boot up fail!!!\n");
			//goto BOOTUP_FAIL;
		}

	//send SIO Loader Program by I2C
	M12MO_write_memory(SIO_LOADER_PROGRAM_START_ADDR, m12mo_sio_fw, sio_fwsize);
	//
	//set the PC jump addr
	M12MO_write_category_parameter_4Byte(0x0F, 0x0C, SIO_LOADER_PROGRAM_START_ADDR);
	//
	//jump the update addr
	M12MO_write_category_parameter_ram(0x0F, 0x12, 0x02);
	//read interrupt
	retry = 50;
	Sleep(5);
	INT_status = M12MO_read_category_parameter_ram(0x0F,0x10);
		
        SENSORDB("0x0F,0x10 INT status is 0x%x\n", INT_status);
	while ((INT_status != 0x04) && retry != 0)
	{
		Sleep(20);
		INT_status = M12MO_read_category_parameter_ram(0x0F,0x10);
		SENSORDB(" INT status is 0x%x, retry is %d\n", INT_status,retry);
		retry--;
	}
	if((INT_status != 0x04) && (retry==0))
		{
			 SENSORDB("SDRAM Active fail !!!\n");
			//goto BOOTUP_FAIL;
		}
	//spi download FW to ROM
	M12MO_DownLoadFW_ToRAM();
	//Sleep(2000);
	retry = 100;
	INT_status = M12MO_read_category_parameter_ram(0x0F,0x10);
	SENSORDB("0x0F ,0x10 INT_status is 0x%x\n", INT_status);
	while (INT_status != 0x05 && retry != 0)
	{
		Sleep(50);
		INT_status = M12MO_read_category_parameter_ram(0x0F,0x10);
		SENSORDB("INT_status is 0x%x, retry is %d\n", INT_status,retry);
		retry--;
	}
	if((INT_status != 0x05) && (retry==0))
	{
		 SENSORDB("flashrom updated fail !!!\n");
		return 0;
	}
	return 1;
}

static kal_bool M12MO_Check_FWversion(void)
{
	u16  version =0;
	u16  FW_Version =0;
	kal_uint8 retry = 20;
	kal_uint16 INT_status = 0;

		INT_status = M12MO_read_category_parameter_ram(0x0F, 0x10);
		SENSORDB("(0x0F, 0x10) is 0x%x\n", INT_status);
		while((INT_status !=0x01) && retry)
		{
			INT_status = M12MO_read_category_parameter_ram(0x0F,0x10);
			SENSORDB("(0x0F, 0x10) is 0x%x\n", INT_status);
			Sleep(20);
			retry--;
		}
		
		if((INT_status !=0x01) && (retry==0))
		{
			 SENSORDB("boot up fail!!!\n");
			//goto BOOTUP_FAIL;
		}

		M12MO_write_category_parameter_ram(0x0F, 0x12, 0x01);
		Sleep(200);
		INT_status = 0;
		retry = 20;
		INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
		SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
		while((INT_status !=0x01) && retry)
		{
			Sleep(20);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
		if(INT_status == 0x01)
		{
		M12MO_Driver.s5k2m8.supplier_id = M12MO_read_category_parameter_ram(0x0E,0x40);
		M12MO_Driver.s5k2m8.build_stage = M12MO_read_category_parameter_ram(0x0E,0x41);
		M12MO_Driver.s5k2m8.pdaf_pixel_flag = M12MO_read_category_parameter_ram(0x0E,0x42);
		M12MO_Driver.s5k2m8.pdaf_calibration_flag = M12MO_read_category_parameter_ram(0x0E,0x43);
		M12MO_Driver.s5k2m8.depthmap_flag = M12MO_read_category_parameter_ram(0x0E,0x44);

		M12MO_Driver.ov13850.lsc_awb_flag = M12MO_read_category_parameter_ram(0x0E,0x45);
		M12MO_Driver.ov13850.module_id = M12MO_read_category_parameter_ram(0x0E,0x46);
		M12MO_Driver.ov13850.year = M12MO_read_category_parameter_ram(0x0E,0x47);
		M12MO_Driver.ov13850.month = M12MO_read_category_parameter_ram(0x0E,0x48);
		M12MO_Driver.ov13850.day = M12MO_read_category_parameter_ram(0x0E,0x49);
		M12MO_Driver.ov13850.lens_id = M12MO_read_category_parameter_ram(0x0E,0x4B);
		M12MO_Driver.ov13850.vcm_id = M12MO_read_category_parameter_ram(0x0E,0x4C);
		M12MO_Driver.ov13850.driver_ic_id = M12MO_read_category_parameter_ram(0x0E,0x4D);
		M12MO_Driver.ov13850.ir_bg_id = M12MO_read_category_parameter_ram(0x0E,0x4E);
		M12MO_Driver.ov13850.color_temperature_id = M12MO_read_category_parameter_ram(0x0E,0x4F);
		M12MO_Driver.ov13850.af_ff_flag = M12MO_read_category_parameter_ram(0x0E,0x50);
		M12MO_Driver.ov13850.light_source_flag = M12MO_read_category_parameter_ram(0x0E,0x51);
		M12MO_Driver.ov13850.af_50cm_value_h = M12MO_read_category_parameter_ram(0x0E,0x58);
		M12MO_Driver.ov13850.af_50cm_value_l = M12MO_read_category_parameter_ram(0x0E,0x59);
		
		SENSORDB("supplier_id = %x,build_stage = %x,pdaf_pixel_flag=%x,pdaf_calibration_flag=%x,depthmap_flag=%x\n",
				M12MO_Driver.s5k2m8.supplier_id,
				M12MO_Driver.s5k2m8.build_stage,
				M12MO_Driver.s5k2m8.pdaf_pixel_flag,
				M12MO_Driver.s5k2m8.pdaf_calibration_flag,
				M12MO_Driver.s5k2m8.depthmap_flag);
		
		SENSORDB("lsc_awb_flag=%x,module_id=%x,year=%d,month=%d,day=%d,lens_id=%x,vcm_id = %x,driver_ic_id = %x,ir_bg_id=%x,color_temperature_id=%x,af_ff_flag=%x,light_source_flag=%x,af_50cm_value=%d|%d\n",
				M12MO_Driver.ov13850.lsc_awb_flag,
				M12MO_Driver.ov13850.module_id ,
				M12MO_Driver.ov13850.year ,
				M12MO_Driver.ov13850.month ,
				M12MO_Driver.ov13850.day,
				M12MO_Driver.ov13850.lens_id,
				M12MO_Driver.ov13850.vcm_id,
				M12MO_Driver.ov13850.driver_ic_id ,
				M12MO_Driver.ov13850.ir_bg_id ,
				M12MO_Driver.ov13850.color_temperature_id ,
				M12MO_Driver.ov13850.af_ff_flag,
				M12MO_Driver.ov13850.light_source_flag,
				M12MO_Driver.ov13850.af_50cm_value_h ,
				M12MO_Driver.ov13850.af_50cm_value_l);
			}

	
	 version = M12MO_read_category_parameter_2byte(0x00,0x02);

	 SENSORDB("boot up success,  current FW Version is 0x%x\n", version);
	//get  FW version  of the bin file 
	m12mo_fwsize = fusjutong_GetFirmwareSize(filepath);
	 if(m12mo_fwsize <= 0)
	 	{
			SENSORDB("get FW version of bin file fail!!!! ignore FW update\r\n");
			return 1;	 		
	 	}
	rs_ms2_update = (kal_uint8 *)vmalloc(m12mo_fwsize);
	if(rs_ms2_update != NULL)
		{
			memset(rs_ms2_update,0,m12mo_fwsize);
			 if (!fusjutong_ReadFirmware(filepath, rs_ms2_update))
			{
				FW_Version=rs_ms2_update[24]<<8 |rs_ms2_update[25];
				SENSORDB("get FW version of bin file success!  FW_Version =0x%x\r\n",FW_Version);
			}
		}

	if(FW_Version != version)
	{
		spin_lock(&m12mo_drv_lock);
    		fw_version_id= FW_Version;
   	 	spin_unlock(&m12mo_drv_lock);
		SENSORDB("Need update FW, new fw is 0x%x\n",fw_version_id);
		return 0;
	}
	else
	{
		spin_lock(&m12mo_drv_lock);
    		fw_version_id= version;
   		spin_unlock(&m12mo_drv_lock);
		 SENSORDB("Not need update FW, current fw is 0x%x\n",fw_version_id);
		 if (rs_ms2_update != NULL)
		{
			vfree(rs_ms2_update);
			rs_ms2_update = NULL;
		}
		return 1;
	}

}

static BOOL M12MOCheckIsAlive(void) 
{
	u16 sum = 0;
	char ret = 0;
	SENSORDB("Check M12MO is alive! \r\n");
	//check if I2C communication is OK
	//before send camera firmware, only category 0x0F is available
		
	/*------------   boot up progress  -----------*/
#ifdef SLT_DEVINFO_CMM
if(s_DEVINFO_ccm1 == NULL)
{
	s_DEVINFO_ccm1 =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
	s_DEVINFO_ccm1->device_type = "CCM";
	s_DEVINFO_ccm1->device_module = "Y13S03A";
	s_DEVINFO_ccm1->device_vendor = "Sunny";
	s_DEVINFO_ccm1->device_ic = "F13S08A(S5K2M8SX03/F13V01L-201)";
	s_DEVINFO_ccm1->device_version = "null";
	s_DEVINFO_ccm1->device_info = "1300W";
}
if(s_DEVINFO_ccm2 == NULL)
{
	s_DEVINFO_ccm2 =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
	s_DEVINFO_ccm2->device_type = "CCM";
	s_DEVINFO_ccm2->device_module = "Y13S03A";
	s_DEVINFO_ccm2->device_vendor = "Sunny";
	s_DEVINFO_ccm2->device_ic = "F13S08A(OV13850)";
	s_DEVINFO_ccm2->device_version = "null";
	s_DEVINFO_ccm2->device_info = "1300W";
}
if(s_DEVINFO_ccm == NULL)
{
	s_DEVINFO_ccm =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
	s_DEVINFO_ccm->device_type = "CCM";
	s_DEVINFO_ccm->device_module = "L8865F60";
	s_DEVINFO_ccm->device_vendor = "Ofilm";
	s_DEVINFO_ccm->device_ic = "OV8865";
	s_DEVINFO_ccm->device_version = "null";
	s_DEVINFO_ccm->device_info = "800W";
}
#endif
	if(M12MO_Check_FWversion() != 1)
	{
		kdCISModulePowerOn(0,"m12mo_yuv",false,"kd_camera_hw");
		Sleep(500);
		kdCISModulePowerOn(0,"m12mo_yuv",true,"kd_camera_hw");
		
		ret = M12MO_send_cameraFW_by_SIO_mode();
		sum = M12MO_get_sum();
		if((sum  == 0) && (ret != 0))
		 return KAL_TRUE;
		else
		return KAL_FALSE;
	}
	
	return KAL_TRUE;	
}

static void M12MO_Sensor_Init(void)
{	
	kal_uint16 INT_status = 0;
	kal_uint8 retry = 50;
	SENSORDB("[M12MO_Sensor_Init] start \r\n");

	if(g_eSysMode == SYS_MODE_POWEROFF)
	{
	      	{			
			/*------------   boot up progress  -----------*/
			INT_status = M12MO_read_category_parameter_ram(0x0F, 0x10);
			SENSORDB("(0x0F, 0x10) is 0x%x\n", INT_status);
			while((INT_status !=0x01) && retry)
			{
				INT_status = M12MO_read_category_parameter_ram(0x0F,0x10);
				SENSORDB("(0x0F, 0x10) is 0x%x\n", INT_status);
				Sleep(20);
				retry--;
			}
			
			if((INT_status !=0x01) && (retry==0))
			{
				 SENSORDB("boot up fail!!!\n");
				//goto BOOTUP_FAIL;
			}

			M12MO_write_category_parameter_ram(0x0F, 0x12, 0x01);
			Sleep(100);
			INT_status = 0;
			retry = 50;
			while((INT_status !=0x01) && retry)
			{
				Sleep(10);
				INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				retry--;
			}
			
			 fw_real_id = M12MO_read_category_parameter_2byte(0x00,0x02);
			 SENSORDB("boot up finished real fw is 0x%x\n",fw_real_id);
			stmvl53l0_read_calibration_to_isp();
			M12MO_write_category_parameter_ram(0x00, 0x10, 0xFF);	
		}

	spin_lock(&m12mo_drv_lock);
	 g_eSysMode = SYS_MODE_PRE_INITIALIZATION;
	M12MO_Driver.afMode = M12MO_AF_MODE_RSVD;
    	M12MO_Driver.afState = M12MO_AF_STATE_UNINIT;
	spin_unlock(&m12mo_drv_lock);
	}	
    SENSORDB("[M12MO_Sensor_Init] end \r\n");	
}

UINT32 M12MOOpen(void)
{		
    	M12MO_Sensor_Init();
   	 return ERROR_NONE;	
}

UINT32 M12MOClose(void)
{
	if(g_eSysMode == SYS_MODE_POWEROFF)
	return ERROR_NONE;	
	
	spin_lock(&m12mo_drv_lock);
	g_eSysMode = SYS_MODE_POWEROFF;
	spin_unlock(&m12mo_drv_lock);
	
    	return ERROR_NONE;
}

UINT32 M12MOIQPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	unsigned char INT_status = 0;
	unsigned int retry = 5;
	static unsigned int j = 0;
	SENSORDB("IN ,sensor_config_data->ImageTargetWidth is %d,  sensor_config_data->ImageTargetHeight is %d\r\n", sensor_config_data->ImageTargetWidth, sensor_config_data->ImageTargetHeight);

	image_window->GrabStartX= M12MO_PREVIEW_START_X;
	image_window->GrabStartY= M12MO_PREVIEW_START_Y;
	image_window->ExposureWindowWidth= M12MO_REAL_PV_WIDTH;
	image_window->ExposureWindowHeight= M12MO_REAL_PV_HEIGHT;

	if(g_eSysMode != SYS_MODE_MONITOR)
	{
		if((j == 0)  || (g_eSysMode == SYS_MODE_PRE_INITIALIZATION))
		{
			j = 0;
			if(g_eSysMode != SYS_MODE_SINGLE_CAPTURE)
			{
				M12MO_write_category_parameter_ram(0x00, 0x0b, 0x01);
				INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				while(((INT_status & 0x01) !=0x01) && retry)
				{
					INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
					SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
					Sleep(20);
					retry--;
				}
				M12MO_write_category_parameter_ram(0x01, 0x01, 0x37);//1440*1080

				if (M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
				{
					M12MO_write_category_parameter_ram(0x00, 0x17, gCameraIndex);//rear
				}
				else if (M12MO_Get_CameraMode()==M12MO_SUB_CAMERA)
				{
					M12MO_write_category_parameter_ram(0x00, 0x17, 0x02);//front
				}
				M12MO_write_category_parameter_ram(0x01, 0x10, 0x00);
			}
			M12MO_write_category_parameter_ram(0x00, 0x0b, 0x02);	
			Sleep(100);
			INT_status = 0;
			retry = 50;
			INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			while(((INT_status & 0x01) !=0x01) && retry)
			{
				INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				Sleep(20);
				retry--;
			}
			j++;
		}
		else if(j == 1)
		{
			if(g_eSysMode == SYS_MODE_SINGLE_CAPTURE)
			{
				M12MO_write_category_parameter_ram(0x0C, 0x09, 0x06);	
				Sleep(100);
				M12MO_write_category_parameter_ram(0x0C, 0x09, 0x07);	
				INT_status = 0;
				retry = 50;
				INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				while(((INT_status & 0x01) !=0x01) && retry)
				{
					INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
					SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
					Sleep(20);
					retry--;
				}
				j++;
			}
		}
		else if(j == 2)
		{
			if(g_eSysMode == SYS_MODE_SINGLE_CAPTURE)
			{
				M12MO_write_category_parameter_ram(0x0C, 0x09, 0x06);	
				Sleep(100);
				M12MO_write_category_parameter_ram(0x0C, 0x09, 0x07);	
				INT_status = 0;
				retry = 50;
				INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				while(((INT_status & 0x01) !=0x01) && retry)
				{
					INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
					SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
					Sleep(20);
					retry--;
				}
				j = 0;
			}
		}
	}
	spin_lock(&m12mo_drv_lock);
	g_eSysMode = SYS_MODE_MONITOR; 
	M12MO_Driver.Preview_Width = M12MO_REAL_PV_WIDTH;
	M12MO_Driver.Preview_Height =  M12MO_REAL_PV_HEIGHT;
	spin_unlock(&m12mo_drv_lock); 
 
	memcpy(&M12MOSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    	return ERROR_NONE;	
}

UINT32 M12MOPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	unsigned char INT_status = 0;
	unsigned int retry = 0;

	image_window->GrabStartX= M12MO_PREVIEW_START_X;
	image_window->GrabStartY= M12MO_PREVIEW_START_Y;
	image_window->ExposureWindowWidth= M12MO_REAL_PV_WIDTH;
	image_window->ExposureWindowHeight= M12MO_REAL_PV_HEIGHT;


	if(g_eSysMode != SYS_MODE_MONITOR)
	{
		if(g_eSysMode != SYS_MODE_SINGLE_CAPTURE)
		{
			if(g_eSysMode != SYS_MODE_PRE_INITIALIZATION)
			{
				retry =20;
			}
			M12MO_write_category_parameter_ram(0x00, 0x0b, 0x01);
			while(((INT_status & 0x01) !=0x01) && retry)
			{
				Sleep(10);
				INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				retry--;
			}
			M12MO_write_category_parameter_ram(0x02, 0x69, 0x01);
			M12MO_write_category_parameter_ram(0x0C, 0x0C, 0x00);
			M12MO_write_category_parameter_ram(0x0C, 0x00, 0x00);
			M12MO_write_category_parameter_ram(0x01, 0x01, 0x37);//1440*1080

			if (M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
			{
				M12MO_write_category_parameter_ram(0x00, 0x17, gCameraIndex);//rear
			}
			else if (M12MO_Get_CameraMode()==M12MO_SUB_CAMERA)
			{
				M12MO_write_category_parameter_ram(0x00, 0x17, 0x02);//front
			}
			M12MO_write_category_parameter_ram(0x01, 0x10, 0x00);
		}
		
		M12MO_write_category_parameter_ram(0x00, 0x0b, 0x02);	

		INT_status = 0;
		retry = 100;
		while(((INT_status & 0x01) !=0x01) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
	}
	if ((M12MO_Get_CameraMode()==M12MO_SUB_CAMERA) && (M12MO_Driver.ov8865.read_flag != 1))
	{
		M12MO_Driver.ov8865.basic_info_flag= M12MO_read_category_parameter_ram(0x0E,0x52);
		M12MO_Driver.ov8865.module_id = M12MO_read_category_parameter_ram(0x0E,0x53);
		M12MO_Driver.ov8865.year = M12MO_read_category_parameter_ram(0x0E,0x55);
		M12MO_Driver.ov8865.month = M12MO_read_category_parameter_ram(0x0E,0x56);
		M12MO_Driver.ov8865.day = M12MO_read_category_parameter_ram(0x0E,0x57);
		M12MO_Driver.ov8865.lens_id = M12MO_read_category_parameter_ram(0x0E,0x54);
		M12MO_Driver.ov8865.read_flag = 1;
	SENSORDB("basic_info_flag=%x,module_id=%x,year=%d,month=%d,day=%d,lens_id=%x\n",
				M12MO_Driver.ov8865.basic_info_flag,
				M12MO_Driver.ov8865.module_id ,
				M12MO_Driver.ov8865.year ,
				M12MO_Driver.ov8865.month ,
				M12MO_Driver.ov8865.day,
				M12MO_Driver.ov8865.lens_id);	
	}
	spin_lock(&m12mo_drv_lock);
	g_eSysMode = SYS_MODE_MONITOR; 
	M12MO_Driver.Preview_Width = M12MO_REAL_PV_WIDTH;
	M12MO_Driver.Preview_Height =  M12MO_REAL_PV_HEIGHT;
	spin_unlock(&m12mo_drv_lock); 
 
	memcpy(&M12MOSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    	return ERROR_NONE;	
}

UINT32 M12MODualPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	unsigned char INT_status = 0;
	unsigned int retry = 0;

	image_window->GrabStartX= M12MO_PREVIEW_START_X;
	image_window->GrabStartY= M12MO_PREVIEW_START_Y;
	image_window->ExposureWindowWidth= M12MO_REAL_PV_WIDTH;
	image_window->ExposureWindowHeight= M12MO_REAL_PV_HEIGHT;

	if(g_eSysMode != SYS_MODE_DUALMONITOR)
	{
		if(g_eSysMode != SYS_MODE_MULTI_CAPTURE)
		{
			if(g_eSysMode != SYS_MODE_PRE_INITIALIZATION)
			{
				retry =20;
			}
			M12MO_write_category_parameter_ram(0x00, 0x0b, 0x01);
			while(((INT_status & 0x01) !=0x01) && retry)
			{
				Sleep(10);
				INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				retry--;
			}
			M12MO_write_category_parameter_ram(0x0C, 0x00, 0x02);
			M12MO_write_category_parameter_ram(0x0C, 0x0C, 0x00);
			M12MO_write_category_parameter_ram(0x01, 0x01, 0x37);//1440*1080
			M12MO_write_category_parameter_ram(0x00, 0x17, 0x01);//rear
			M12MO_write_category_parameter_ram(0x01, 0x10, 0x00);
		}
		
		M12MO_write_category_parameter_ram(0x00, 0x0b, 0x02);	

		INT_status = 0;
		retry = 100;
		while(((INT_status & 0x01) !=0x01) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
	}
	spin_lock(&m12mo_drv_lock);
	g_eSysMode = SYS_MODE_DUALMONITOR; 
	M12MO_Driver.Preview_Width = M12MO_REAL_PV_WIDTH;
	M12MO_Driver.Preview_Height =  M12MO_REAL_PV_HEIGHT;
	spin_unlock(&m12mo_drv_lock); 
 
	memcpy(&M12MOSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    	return ERROR_NONE;	
}

UINT32 M12MOZSLPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	unsigned char INT_status = 0;
	unsigned int retry = 0;

	image_window->GrabStartX= M12MO_PREVIEW_START_X;
	image_window->GrabStartY= M12MO_PREVIEW_START_Y;
	image_window->ExposureWindowWidth= M12MO_REAL_PV_WIDTH;
	image_window->ExposureWindowHeight= M12MO_REAL_PV_HEIGHT;

	if(g_eSysMode != SYS_MODE_ZSLMONITOR)
	{
		if(g_eSysMode != SYS_MODE_SINGLE_CAPTURE)
		{
			if(g_eSysMode != SYS_MODE_PRE_INITIALIZATION)
			{
				retry =20;
			}
			M12MO_write_category_parameter_ram(0x00, 0x0b, 0x01);
			while(((INT_status & 0x01) !=0x01) && retry)
			{
				Sleep(10);
				INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				retry--;
			}
			M12MO_write_category_parameter_ram(0x0C, 0x00, 0x0E);
			M12MO_write_category_parameter_ram(0x0C, 0x0C, 0x00);
			M12MO_write_category_parameter_ram(0x01, 0x01, 0x37);//1440*1080

			if (M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
			{
				M12MO_write_category_parameter_ram(0x00, 0x17, gCameraIndex);//rear
			}
			else if (M12MO_Get_CameraMode()==M12MO_SUB_CAMERA)
			{
				M12MO_write_category_parameter_ram(0x00, 0x17, 0x02);//front
			}
			M12MO_write_category_parameter_ram(0x01, 0x10, 0x00);
		}
		
		M12MO_write_category_parameter_ram(0x00, 0x0b, 0x02);	

		INT_status = 0;
		retry = 100;
		while(((INT_status & 0x01) !=0x01) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
	}
	spin_lock(&m12mo_drv_lock);
	g_eSysMode = SYS_MODE_ZSLMONITOR; 
	M12MO_Driver.Preview_Width = M12MO_REAL_PV_WIDTH;
	M12MO_Driver.Preview_Height =  M12MO_REAL_PV_HEIGHT;
	spin_unlock(&m12mo_drv_lock); 
 
	memcpy(&M12MOSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    	return ERROR_NONE;	
}

UINT32 M12MOMultiPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	unsigned char INT_status = 0;
	unsigned int retry = 0;

	image_window->GrabStartX= M12MO_PREVIEW_START_X;
	image_window->GrabStartY= M12MO_PREVIEW_START_Y;
	image_window->ExposureWindowWidth= M12MO_PV_DEPTH_WIDTH;
	image_window->ExposureWindowHeight= M12MO_PV_DEPTH_HEIGHT;
	
	if(g_eSysMode != SYS_MODE_MULTIMONITOR)
	{
		if(g_eSysMode != SYS_MODE_MULTI_CAPTURE)
		{
			if(g_eSysMode != SYS_MODE_PRE_INITIALIZATION)
			{
				retry =20;
			}
			M12MO_write_category_parameter_ram(0x00, 0x0b, 0x01);
			while(((INT_status & 0x01) !=0x01) && retry)
			{
				Sleep(10);
				INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				retry--;
			}
			M12MO_write_category_parameter_ram(0x01, 0x01, 0x28);//1920*1080
			M12MO_write_category_parameter_ram(0x0C, 0x0C, 0x00);
			M12MO_write_category_parameter_ram(0x00, 0x17, 0x01);//rear
			M12MO_write_category_parameter_ram(0x01, 0x10, 0x04);
		}
		M12MO_write_category_parameter_ram(0x00, 0x0b, 0x02);	

		INT_status = 0;
		retry = 100;
		while(((INT_status & 0x01) !=0x01) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
	}
	spin_lock(&m12mo_drv_lock);
	g_eSysMode = SYS_MODE_MULTIMONITOR; 
	M12MO_Driver.Preview_Width = M12MO_PV_DEPTH_WIDTH;
	M12MO_Driver.Preview_Height =  M12MO_PV_DEPTH_HEIGHT-60;
	spin_unlock(&m12mo_drv_lock); 
 
	memcpy(&M12MOSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    	return ERROR_NONE;	
}

UINT32 M12MOIQCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	unsigned char INT_status = 0;
	unsigned int retry = 50;
	static unsigned int i = 0;
	SENSORDB("IN ,sensor_config_data->ImageTargetWidth is %d,  sensor_config_data->ImageTargetHeight is %d\r\n", sensor_config_data->ImageTargetWidth, sensor_config_data->ImageTargetHeight);
 
	image_window->GrabStartX= M12MO_CAPTURE_START_X;
	image_window->GrabStartY= M12MO_CAPTURE_START_Y;
	if (M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
		{
			image_window->ExposureWindowWidth = M12MO_REAL_CAP_WIDTH;
			image_window->ExposureWindowHeight = M12MO_REAL_CAP_HEIGHT;
		}
	else if (M12MO_Get_CameraMode()==M12MO_SUB_CAMERA)
		{
			image_window->ExposureWindowWidth = M12MO_SUBSESNOR_REAL_CAP_WIDTH;
			image_window->ExposureWindowHeight = M12MO_SUBSENSOR_REAL_CAP_HEIGHT;
		}

	if(i == 0)
	{
		M12MO_Cancel_AF();
		M12MO_write_category_parameter_ram(0x0C, 0x00, 0x00);	
		M12MO_write_category_parameter_ram(0x0B, 0x00, 0x00);	
		M12MO_write_category_parameter_ram(0x0B, 0x01, 0x2c);	
		M12MO_write_category_parameter_ram(0x00, 0x0b, 0x03);	
		Sleep(100);
		INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
		SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
		while(((INT_status & 0x08) !=0x08) && retry)
			{
				INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				Sleep(20);
				retry--;
			}	
		M12MO_write_category_parameter_ram(0x0C, 0x09, 0x04);	
		INT_status = 0;
		retry = 50;
		//Sleep(100);
		INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
		SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
		while(((INT_status & 0x08) !=0x08) && retry)
			{
				INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				Sleep(20);
				retry--;
			}
		i++;
	}
	else if(i == 1)
	{
		M12MO_write_category_parameter_ram(0x0C, 0x09, 0x08);	
		//Sleep(100);
		INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
		SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
		while(((INT_status & 0x01) !=0x01) && retry)
			{
				INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				Sleep(20);
				retry--;
			}	
		M12MO_write_category_parameter_ram(0x0C, 0x09, 0x05);	
		INT_status = 0;
		retry = 50;
		//Sleep(100);
		INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
		SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
		while(((INT_status & 0x08) !=0x08) && retry)
			{
				INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				Sleep(20);
				retry--;
			}
		i++;
	}
	else if(i == 2)
	{
		M12MO_write_category_parameter_ram(0x0C, 0x09, 0x08);	
		//Sleep(100);
		INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
		SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
		while(((INT_status & 0x01) !=0x01) && retry)
			{
				INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				Sleep(20);
				retry--;
			}	
		M12MO_write_category_parameter_ram(0x0C, 0x09, 0x01);	
		INT_status = 0;
		retry = 50;
		//Sleep(100);
		INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
		SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
		while(((INT_status & 0x08) !=0x08) && retry)
			{
				INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
				SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
				Sleep(20);
				retry--;
			}
		i = 0;
	}
	spin_lock(&m12mo_drv_lock);
	//g_bIsStartPreview = KAL_FALSE;
	g_eSysMode = SYS_MODE_SINGLE_CAPTURE;
	spin_unlock(&m12mo_drv_lock);
	memcpy(&M12MOSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
	
}

unsigned int M12MO_GetExposureTime(void)
{
    unsigned int interval=0, interval0=0, interval1= 0;
    unsigned int exposure7 ,exposure6,exposure5,exposure4,exposure3,exposure2,exposure1,exposure0 = 0;
	
    exposure7 = M12MO_read_category_parameter_ram(0x07, 0x00);
    exposure6 = M12MO_read_category_parameter_ram(0x07, 0x01);
    exposure5 = M12MO_read_category_parameter_ram(0x07, 0x02);
    exposure4 = M12MO_read_category_parameter_ram(0x07, 0x03);
    interval0 = (exposure7 << 24) |(exposure6 << 16)|(exposure5 << 8)|exposure4;

    exposure3 = M12MO_read_category_parameter_ram(0x07, 0x04);
    exposure2 = M12MO_read_category_parameter_ram(0x07, 0x05);
    exposure1 = M12MO_read_category_parameter_ram(0x07, 0x06);
    exposure0 = M12MO_read_category_parameter_ram(0x07, 0x07);
    interval1 = (exposure3 << 24) |(exposure2 << 16)|(exposure1 << 8)|exposure0;
    if(interval1 != 0)
    	interval = interval0 *1000*1000/interval1;
	
    SENSORDB("[M12MO] Exposuer Time = %d us\n", interval);
    SENSORDB("[M12MO] Exposuer Time = %d %d %d %d ,%d %d %d %d\n", exposure7 ,exposure6,exposure5,exposure4,exposure3,exposure2,exposure1,exposure0);
    return interval;
}

UINT32 M12MOCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	unsigned char INT_status = 0;
	unsigned int retry = 50;
 
	image_window->GrabStartX= M12MO_CAPTURE_START_X;
	image_window->GrabStartY= M12MO_CAPTURE_START_Y;
	if (M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
	{
		image_window->ExposureWindowWidth = M12MO_REAL_CAP_WIDTH;
		image_window->ExposureWindowHeight = M12MO_REAL_CAP_HEIGHT;
	}
	else if (M12MO_Get_CameraMode()==M12MO_SUB_CAMERA)
	{
		image_window->ExposureWindowWidth = M12MO_SUBSESNOR_REAL_CAP_WIDTH;
		image_window->ExposureWindowHeight = M12MO_SUBSENSOR_REAL_CAP_HEIGHT;
	}
	
	M12MO_Cancel_AF();

	//M12MO_write_category_parameter_ram(0x0C, 0x00, 0x00);	
	M12MO_write_category_parameter_ram(0x0B, 0x00, 0x00);	
	M12MO_write_category_parameter_ram(0x0B, 0x01, 0x2c);	   
	M12MO_write_category_parameter_ram(0x00, 0x0b, 0x03);	

	if(g_bIsStartlongexposertime)
	{
	retry = g_bIsStartlongexposertime *150;
	INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
	SENSORDB("(0x00, 0x1C) is 0x%x,%d\n", INT_status,g_bIsStartlongexposertime);
	while(((INT_status & 0x80) !=0x80) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
	}
	INT_status = 0;
	retry = 500;
	while(((INT_status & 0x08) !=0x08) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}	
	M12MO_write_category_parameter_ram(0x0C, 0x09, 0x01);	
	INT_status = 0;
	retry = 100;
	while(((INT_status & 0x08) !=0x08) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
	M12MO_Driver.capExposureTime = M12MO_GetExposureTime();
	spin_lock(&m12mo_drv_lock);
	g_eSysMode = SYS_MODE_SINGLE_CAPTURE;
	spin_unlock(&m12mo_drv_lock);
	memcpy(&M12MOSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
	
}

UINT32 M12MOHdrCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	unsigned char INT_status = 0;
	unsigned int retry = 50;
 	static unsigned int i = 0;
	image_window->GrabStartX= M12MO_CAPTURE_START_X;
	image_window->GrabStartY= M12MO_CAPTURE_START_Y;
	image_window->ExposureWindowWidth = M12MO_REAL_CAP_WIDTH;
	image_window->ExposureWindowHeight = M12MO_REAL_CAP_HEIGHT;

	if(i == 0)
	{
		M12MO_Cancel_AF();

		M12MO_write_category_parameter_ram(0x0C, 0x00, 0x00);	
		M12MO_write_category_parameter_ram(0x0B, 0x00, 0x00);	
		M12MO_write_category_parameter_ram(0x0C, 0x0C, 0x02);	
		M12MO_write_category_parameter_ram(0x03, 0x98, M12MO_Driver.Hdr_EV1);	 
		M12MO_write_category_parameter_ram(0x03, 0x99, M12MO_Driver.Hdr_EV2);	 
		M12MO_write_category_parameter_ram(0x00, 0x0b, 0x03);	
	
		INT_status = 0;
		retry = 200;
		while(((INT_status & 0x08) !=0x08) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}	
		M12MO_write_category_parameter_ram(0x0C, 0x09, 0x01);	
		INT_status = 0;
		retry = 100;
		while(((INT_status & 0x08) !=0x08) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
		i++;
	}
	else if(i == 1)
	{
		INT_status = 0;
		retry = 100;
		M12MO_write_category_parameter_ram(0x0C, 0x09, 0x06);	
		while(((INT_status & 0x01) !=0x01) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}	
		M12MO_write_category_parameter_ram(0x0C, 0x09, 0x09);	
		INT_status = 0;
		retry = 100;
		while(((INT_status & 0x08) !=0x08) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
		i++;
	}
	else if(i == 2)
	{
		INT_status = 0;
		retry = 100;
		M12MO_write_category_parameter_ram(0x0C, 0x09, 0x06);	
		while(((INT_status & 0x01) !=0x01) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}	
		M12MO_write_category_parameter_ram(0x0C, 0x09, 0x0A);	
		INT_status = 0;
		retry = 100;
		while(((INT_status & 0x08) !=0x08) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
		i = 0;
	}
	spin_lock(&m12mo_drv_lock);
	g_eSysMode = SYS_MODE_SINGLE_CAPTURE;
	spin_unlock(&m12mo_drv_lock);
	memcpy(&M12MOSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
	
}

UINT32 M12MOBurstCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	unsigned char INT_status = 0;
	unsigned int retry = 10;
 
	image_window->GrabStartX= M12MO_CAPTURE_START_X;
	image_window->GrabStartY= M12MO_CAPTURE_START_Y;
	if (M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
	{
		image_window->ExposureWindowWidth = M12MO_REAL_CAP_WIDTH;
		image_window->ExposureWindowHeight = M12MO_REAL_CAP_HEIGHT;
	}
	else if (M12MO_Get_CameraMode()==M12MO_SUB_CAMERA)
	{
		image_window->ExposureWindowWidth = M12MO_SUBSESNOR_REAL_CAP_WIDTH;
		image_window->ExposureWindowHeight = M12MO_SUBSENSOR_REAL_CAP_HEIGHT;
	}
	
	M12MO_Cancel_AF();
	
	M12MO_write_category_parameter_ram(0x00, 0x0b, 0x01);

	while(((INT_status & 0x01) !=0x01) && retry)
	{
		Sleep(10);
		INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
		SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
		retry--;
	}
		
	M12MO_write_category_parameter_ram(0x0C, 0x00, 0x01);	
	M12MO_write_category_parameter_ram(0x0B, 0x00, 0x00);	
	M12MO_write_category_parameter_ram(0x0B, 0x01, 0x2c);	   
	M12MO_write_category_parameter_ram(0x00, 0x0b, 0x02);	
	
	INT_status = 0;
	retry = 50;
	while(((INT_status & 0x01) !=0x01) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
	
	INT_status = 0;
	retry = 100;
	while(((INT_status & 0x08) !=0x08) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}
	spin_lock(&m12mo_drv_lock);
	g_eSysMode = SYS_MODE_BURST_CAPTURE;
	spin_unlock(&m12mo_drv_lock);
	memcpy(&M12MOSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
	
}

UINT32 M12MOMultiCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	unsigned char INT_status = 0;
	unsigned int retry = 200;
 
	image_window->GrabStartX= M12MO_CAPTURE_START_X;
	image_window->GrabStartY= M12MO_CAPTURE_START_Y;

	image_window->ExposureWindowWidth = M12MO_FULL_DEPTH_WIDTH;
	image_window->ExposureWindowHeight = M12MO_FULL_DEPTH_HEIGHT;

	M12MO_Cancel_AF();

	//M12MO_write_category_parameter_ram(0x0C, 0x00, 0x02);
	M12MO_write_category_parameter_ram(0x0B, 0x01, 0x2c);	
	M12MO_write_category_parameter_ram(0x0B, 0x00, 0x0D);	
	M12MO_write_category_parameter_ram(0x00, 0x0b, 0x03);	

	while(((INT_status & 0x08) !=0x08) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}	

	M12MO_write_category_parameter_ram(0x0D, 0xDE, 0x01);	

	INT_status = 0;
	retry = 100;
	while(((INT_status & 0x08) !=0x08) && retry)
		{
			Sleep(10);
			INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
			SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
			retry--;
		}

	spin_lock(&m12mo_drv_lock);
	g_eSysMode = SYS_MODE_MULTI_CAPTURE;
	spin_unlock(&m12mo_drv_lock);
	memcpy(&M12MOSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
	
}

UINT32 M12MOGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	SENSORDB("Enter M12MOGetResolution.\n");
	switch(M12MOCurrentScenarioId)
		{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorResolution->SensorPreviewWidth	= M12MO_REAL_CAP_WIDTH;
			pSensorResolution->SensorPreviewHeight	= M12MO_REAL_CAP_HEIGHT;
			break;
		default:
			pSensorResolution->SensorPreviewWidth	= M12MO_REAL_PV_WIDTH;
    			pSensorResolution->SensorPreviewHeight	= M12MO_REAL_PV_HEIGHT;
			break;
		}

			pSensorResolution->SensorVideoWidth	= M12MO_VIDEO_PV_WIDTH;
			pSensorResolution->SensorVideoHeight 	= M12MO_VIDEO_PV_HEIGHT;

			pSensorResolution->SensorMultiPreviewWidth	= M12MO_PV_DEPTH_WIDTH;
    			pSensorResolution->SensorMultiPreviewHeight	= M12MO_PV_DEPTH_HEIGHT;
				
			pSensorResolution->SensorMultiCameraWidth = M12MO_FULL_DEPTH_WIDTH;
			pSensorResolution->SensorMultiCameraHeight = M12MO_FULL_DEPTH_HEIGHT;
			if (M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
			{
				pSensorResolution->SensorFullWidth = M12MO_REAL_CAP_WIDTH;
				pSensorResolution->SensorFullHeight = M12MO_REAL_CAP_HEIGHT;
			SENSORDB("Enter MAIN GetResolution  SensorFullWidth= %d,SensorFullHeight = %d\n",pSensorResolution->SensorFullWidth,pSensorResolution->SensorFullHeight);
			}
			else if (M12MO_Get_CameraMode()==M12MO_SUB_CAMERA)
			{
				pSensorResolution->SensorFullWidth             = M12MO_SUBSESNOR_REAL_CAP_WIDTH;
				pSensorResolution->SensorFullHeight		= M12MO_SUBSENSOR_REAL_CAP_HEIGHT;
			SENSORDB("Enter SUB GetResolution  SensorFullWidth= %d,SensorFullHeight = %d\n",pSensorResolution->SensorFullWidth,pSensorResolution->SensorFullHeight);
			}
	 
	return ERROR_NONE;
	
}

UINT32 M12MOGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
						MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
						MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch(ScenarioId){
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorPreviewResolutionX = M12MO_REAL_CAP_WIDTH;
			pSensorInfo->SensorPreviewResolutionY = M12MO_REAL_CAP_HEIGHT;
			pSensorInfo->SensorFullResolutionX = M12MO_REAL_CAP_WIDTH;
			pSensorInfo->SensorFullResolutionY = M12MO_REAL_CAP_HEIGHT;
			pSensorInfo->SensorCameraPreviewFrameRate=15;
			break;
		case MSDK_SCENARIO_ID_MULTI_CAMERA_PREVIEW:
			pSensorInfo->SensorPreviewResolutionX = M12MO_PV_DEPTH_WIDTH;
			pSensorInfo->SensorPreviewResolutionY = M12MO_PV_DEPTH_HEIGHT;
			pSensorInfo->SensorFullResolutionX = M12MO_FULL_DEPTH_WIDTH;
			pSensorInfo->SensorFullResolutionY = M12MO_FULL_DEPTH_HEIGHT;
			pSensorInfo->SensorCameraPreviewFrameRate=24;
			SENSORDB("Enter MAIN GetInfo  SensorMultiFullResolutionX= %d,SensorFullResolutionY = %d\n",pSensorInfo->SensorFullResolutionX,pSensorInfo->SensorFullResolutionY);
			break;
		default:
			pSensorInfo->SensorPreviewResolutionX = M12MO_REAL_PV_WIDTH;
			pSensorInfo->SensorPreviewResolutionY = M12MO_REAL_PV_HEIGHT;

			if (M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
			{
				pSensorInfo->SensorFullResolutionX  = M12MO_REAL_CAP_WIDTH;
				pSensorInfo->SensorFullResolutionY = M12MO_REAL_CAP_HEIGHT;
				SENSORDB("Enter MAIN GetInfo  SensorFullResolutionX= %d,SensorFullResolutionY = %d\n",pSensorInfo->SensorFullResolutionX,pSensorInfo->SensorFullResolutionY);
			}
			else if (M12MO_Get_CameraMode()==M12MO_SUB_CAMERA)
			{
				pSensorInfo->SensorFullResolutionX = M12MO_SUBSESNOR_REAL_CAP_WIDTH;
				pSensorInfo->SensorFullResolutionY = M12MO_SUBSENSOR_REAL_CAP_HEIGHT;
				SENSORDB("Enter Sub GetInfo  SensorFullResolutionX= %d,SensorFullResolutionY = %d\n",pSensorInfo->SensorFullResolutionX,pSensorInfo->SensorFullResolutionY);
			}
			pSensorInfo->SensorCameraPreviewFrameRate = 30;
			break;
	}
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=4;//13;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
    pSensorInfo->CaptureDelayFrame = 0;//3
    pSensorInfo->PreviewDelayFrame = 0; 
    pSensorInfo->VideoDelayFrame = 5; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;      
    pSensorInfo->AEShutDelayFrame = 1;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 1;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;	
	   
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
	case MSDK_SCENARIO_ID_MULTI_CAMERA_PREVIEW:
	case MSDK_SCENARIO_ID_ZSL_PREVIEW:
	case MSDK_SCENARIO_ID_MULTI_CAMERA_CAPTURE_PREVIEW:
            pSensorInfo->SensorClockFreq=26;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = M12MO_PREVIEW_START_X; 
            pSensorInfo->SensorGrabStartY = M12MO_PREVIEW_START_Y;           		
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	   pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	   pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
        case MSDK_SCENARIO_ID_MULTI_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CONTINUOUS_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_HDR_CAPTURE_JPEG:
            pSensorInfo->SensorClockFreq=26;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = M12MO_CAPTURE_START_X;
            pSensorInfo->SensorGrabStartY = M12MO_CAPTURE_START_Y;    			
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
			
        default:
            pSensorInfo->SensorClockFreq=26;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = 1; 
            pSensorInfo->SensorGrabStartY = 1;             
            break;
    }
	
    memcpy(pSensorConfigData, &M12MOSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}  

UINT32 M12MOControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	SENSORDB("Enter M12MOControl ScenarioId = %d\n",ScenarioId);
	M12MOCurrentScenarioId = ScenarioId;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (gCaptureIndex == 0)
		  M12MOIQPreview(pImageWindow, pSensorConfigData);
		else
		  M12MOPreview(pImageWindow, pSensorConfigData);
          	  break;
	case MSDK_SCENARIO_ID_MULTI_CAMERA_PREVIEW:
		  M12MOMultiPreview(pImageWindow, pSensorConfigData);
          	  break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
		if (gCaptureIndex == 0)
		M12MOIQCapture(pImageWindow, pSensorConfigData);
		else
		M12MOCapture(pImageWindow, pSensorConfigData);
		break;

	case MSDK_SCENARIO_ID_MULTI_CAMERA_CAPTURE_JPEG:
		M12MOMultiCapture(pImageWindow, pSensorConfigData);
		break;

	case MSDK_SCENARIO_ID_ZSL_PREVIEW:
		M12MOZSLPreview(pImageWindow, pSensorConfigData);
		break;

	case MSDK_SCENARIO_ID_CONTINUOUS_CAPTURE_JPEG:
		M12MOBurstCapture(pImageWindow, pSensorConfigData);
		break;

	case MSDK_SCENARIO_ID_MULTI_CAMERA_CAPTURE_PREVIEW:
		M12MODualPreview(pImageWindow, pSensorConfigData);
		break;

	case MSDK_SCENARIO_ID_HDR_CAPTURE_JPEG:
		M12MOHdrCapture(pImageWindow, pSensorConfigData);
		break;

        default:
            	break;
    }
    return ERROR_NONE;
} /* MS2RControl() */


M12MO_AAA_STATUS_ENUM M12MO_AF_Start(M12MO_AF_MODE_ENUM mode)
{
    unsigned char INT_status = 0;

    spin_lock(&m12mo_drv_lock);
    if (mode == M12MO_AF_MODE_SINGLE)
    {
        SENSORDB("[M12MO] SAF_Start+\n\n");
        M12MO_Driver.afMode = M12MO_AF_MODE_SINGLE;
        M12MO_Driver.afState = M12MO_AF_STATE_ENTERING;
    }
    else
    {
        SENSORDB("[M12MO] CAF_Start+\n\n");
        if (M12MO_AF_MODE_CONTINUOUS == M12MO_Driver.afMode)
        {
            spin_unlock(&m12mo_drv_lock);
            SENSORDB("[M12MO] CAF_Start: Been at this Mode...\n");
            return M12MO_AAA_AF_STATUS_OK;
        }
        M12MO_Driver.afMode = M12MO_AF_MODE_CONTINUOUS;
        M12MO_Driver.afState = M12MO_AF_STATE_ENTERING;
    }
    spin_unlock(&m12mo_drv_lock);


    if (mode == M12MO_AF_MODE_SINGLE)
    {
    	INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
	SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
	if(gCameraIndex == 1)
	M12MO_write_category_parameter_ram(0x0A, 0x00, 0x0A);
	else
	M12MO_write_category_parameter_ram(0x0A, 0x00, 0x01);
	//M12MO_write_category_parameter_ram(0x0A, 0x30, 0x00);
	M12MO_write_category_parameter_ram(0x0A, 0x02, 0x01);
    }
    else
    {
        	INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
	SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
	if(gCameraIndex == 1)
	M12MO_write_category_parameter_ram(0x0A, 0x00, 0x0B);
	else
	M12MO_write_category_parameter_ram(0x0A, 0x00, 0x06);
	M12MO_write_category_parameter_ram(0x0A, 0x02, 0x01);
    }
    spin_lock(&m12mo_drv_lock);
    M12MO_Driver.afState = M12MO_AF_STATE_ENTERED;
    spin_unlock(&m12mo_drv_lock);

    return M12MO_AAA_AF_STATUS_OK;
}

static M12MO_AAA_STATUS_ENUM M12MO_AE_Set_Window(uintptr_t zone_addr,unsigned int prevW,unsigned int prevH)
{
    unsigned int x0, y0, x1, y1, width, height;
    static unsigned int pre_x0 =0, pre_y0 = 0;
    static unsigned int update_count = 0;
    unsigned int* ptr = (unsigned int*)zone_addr;
    unsigned int srcW_maxW; //source window's max width
    unsigned int srcW_maxH; //source window's max height

    x0 = *ptr       ;
    y0 = *(ptr + 1) ;
    x1 = *(ptr + 2) ;
    y1 = *(ptr + 3) ;
    width = *(ptr + 4);
    height = *(ptr + 5);
    srcW_maxW = width;
    srcW_maxH = height;

    SENSORDB("[M12MO] AE_Set_Window 3AWin: (%d,%d)~(%d,%d)~~(%d,%d)\n",x0, y0, x1, y1,width,height);

    spin_lock(&m12mo_drv_lock);
    M12MO_Driver.afStateOnOriginalSet = 0;
    if ((x0 == x1) && (y0 == y1))
    {
        M12MO_Driver.afStateOnOriginalSet = 1;
	update_count ++;
    }
   else
    {
    	update_count = 0;
    }
    spin_unlock(&m12mo_drv_lock);

    if (M12MO_Driver.afStateOnOriginalSet == 0)
    {
	if((abs(x0 - pre_x0) > 10) ||( abs(y0 - pre_y0) >10))
	{
	        SENSORDB("[M12MO] AE Window do not Update New Window!!!\n");
		pre_x0 = x0;
		pre_y0 = y0;
		//update_count ++;
		return M12MO_AAA_AF_STATUS_OK;
	}
	pre_x0 = x0;
	pre_y0 = y0;
    //Map 320x240 coordinate to preview size window
       x0 = x0 * prevW / srcW_maxW;
       y0 = y0 * prevH / srcW_maxH;
       x1 = x1* prevW / srcW_maxW;
       y1 = y1 * prevH / srcW_maxH;
        SENSORDB("[M12MO] AE_Set_Window AP's setting: (0x%x,0x%x)~(0x%x,0x%x)~(%d,%d).size:(%d,%d)(%d,%d)\n",x0, y0, x1, y1, pre_x0,pre_y0,srcW_maxW, srcW_maxH,prevW,prevH);	
        SENSORDB("[M12MO] AE Window Update New Window.......\n");
	
	M12MO_write_category_parameter_ram(0x03, 0xA1, ((x0 & 0xff00) >> 8));
	M12MO_write_category_parameter_ram(0x03, 0xA2, (x0 & 0x00ff));
	M12MO_write_category_parameter_ram(0x03, 0xA3, ((y0 & 0xff00) >> 8));
	M12MO_write_category_parameter_ram(0x03, 0xA4, (y0 & 0x00ff));
	M12MO_write_category_parameter_ram(0x03, 0xA5, ((x1 & 0xff00) >> 8));
	M12MO_write_category_parameter_ram(0x03, 0xA6, (x1 & 0x00ff));
	M12MO_write_category_parameter_ram(0x03, 0xA7, ((y1 & 0xff00) >> 8));
	M12MO_write_category_parameter_ram(0x03, 0xA8, (y1 & 0x00ff));
	M12MO_write_category_parameter_ram(0x03, 0xA0, 0x02);
	M12MO_write_category_parameter_ram(0x03, 0x01, 0x10);
	M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
    }
   else
   {
   	if(update_count < 2)
   	{
   	         SENSORDB("[M12MO] AE Window do not Update center Window!!!\n");
		return M12MO_AAA_AF_STATUS_OK;
   	}
	else if(update_count > 65535)
	{
		update_count = 0;
	}
           SENSORDB("[M12MO] AE Window Update center Window.......count=%d\n",update_count);
	M12MO_write_category_parameter_ram(0x03, 0x01, 0x02);
	M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
   }
    return M12MO_AAA_AF_STATUS_OK;
}

static M12MO_AAA_STATUS_ENUM M12MO_AF_Set_Window(uintptr_t zone_addr, unsigned int prevW, unsigned int prevH)
{
    unsigned int x0, y0, x1, y1, FD_XS, FD_YS;
    unsigned int* ptr = (unsigned int*)zone_addr;
    unsigned int srcW_maxW = M12MO_MIPI_AF_CALLER_WINDOW_WIDTH;
    unsigned int srcW_maxH = M12MO_MIPI_AF_CALLER_WINDOW_HEIGHT;

    x0 = *ptr;
    y0 = *(ptr + 1) ;
    x1 = *(ptr + 2) ;
    y1 = *(ptr + 3) ;
    FD_XS = *(ptr + 4);
    FD_YS = *(ptr + 5);
    if (FD_XS == 0)
    {
        FD_XS = 320;
    }

    if (FD_YS == 0)
    {
        FD_YS = 240;
    }

    if (FD_XS > prevW)
    {
        FD_XS = prevW;
    }

    if (FD_YS > prevH)
    {
        FD_YS = prevH;
    }

    SENSORDB("[M12MO] AF_Set_Window AP's setting: (%d,%d)~(%d,%d).size:(%d,%d)\n",x0, y0, x1, y1, FD_XS, FD_YS);

    spin_lock(&m12mo_drv_lock);
    M12MO_Driver.afStateOnOriginalSet = 0;
    if ((x0 == x1) && (y0 == y1))
    {
        M12MO_Driver.afStateOnOriginalSet = 1;
    }
    spin_unlock(&m12mo_drv_lock);


    srcW_maxW = FD_XS;
    srcW_maxH = FD_YS;

    if (x0 >= srcW_maxW)
    {
        x0 = srcW_maxW - 1;
    }

    if (x1 >= srcW_maxW)
    {
        x1 = srcW_maxW - 1;
    }

    if (y0 >= srcW_maxH)
    {
        y0 = srcW_maxH - 1;
    }

    if (y1 >= srcW_maxH)
    {
        y1 = srcW_maxH - 1;
    }

    //Map 320x240 coordinate to preview size window
    x0 = x0 * prevW / srcW_maxW;
    y0 = y0 * prevH / srcW_maxH;
    x1 = x1* prevW / srcW_maxW;
    y1 = y1 * prevH / srcW_maxH;

    SENSORDB("[M12MO] AF_Set_Window AP's setting: (0x%x,0x%x)~(0x%x,0x%x).size:(%d,%d)(%d,%d)\n",x0, y0, x1, y1, FD_XS, FD_YS,prevW,prevH);
    if (M12MO_Driver.afStateOnOriginalSet)
    {
        //Rollback AE/AF setting for CAF mode starting
        SENSORDB("[M12MO] AF window afStateOnOriginalSet = 1.......\n");
	M12MO_write_category_parameter_ram(0x03, 0x01, 0x02);
	M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
	M12MO_write_category_parameter_ram(0x0A, 0x30, 0x00);

    }
    else
    {
        SENSORDB("[M12MO] AF window Update New Window.......\n");
	M12MO_write_category_parameter_ram(0x03, 0x01, 0x10);
	M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
	M12MO_write_category_parameter_ram(0x0A, 0x30, 0x01);
	
	M12MO_write_category_parameter_ram(0x02, 0xA1, ((x0 & 0xff00) >> 8));
	M12MO_write_category_parameter_ram(0x02, 0xA2, (x0 & 0x00ff));
	M12MO_write_category_parameter_ram(0x02, 0xA3, ((y0 & 0xff00) >> 8));
	M12MO_write_category_parameter_ram(0x02, 0xA4, (y0 & 0x00ff));
	M12MO_write_category_parameter_ram(0x02, 0xA5, ((x1 & 0xff00) >> 8));
	M12MO_write_category_parameter_ram(0x02, 0xA6, (x1 & 0x00ff));
	M12MO_write_category_parameter_ram(0x02, 0xA7, ((y1 & 0xff00) >> 8));
	M12MO_write_category_parameter_ram(0x02, 0xA8, (y1 & 0x00ff));
	M12MO_write_category_parameter_ram(0x02, 0xA0, 0x01);

    }

    return M12MO_AAA_AF_STATUS_OK;
}	

#if 1
extern int bmi160_acc_get_data_for_AF(int* x, int* y, int* z, int* status);
extern int bmi160_gyro_get_data_for_AF(int* x, int* y, int* z, int* status);
extern int mmc3524x_o_get_data_for_AF(int* x, int* y, int* z, int* status);
extern int mmc3524x_o_enable_for_AF(int en);

static int orientationsensor_value_transform(int value)
{
	if (value >= 0)
		return ((value * 10 / 8192 + 5) / 10);
	else
		return (-((-value) * 10 / 8192 + 5) / 10);
}

static int gsensor_value_transform(int value)
{
	if (value >= 0)
		return ((value * 40/1000 + 5) / 10);
	else
		return (-((-value) * 40/ 1000 + 5) / 10);
}

static int gyrosensor_value_transform(int value)
{
	if (value >= 0)
		return ((value * 80/ 57 + 5) / 10);
	else
		return (-((-value) * 80 / 57 + 5) / 10);
}

static void update_orientation_data_for_AF(void)
{
	int x = 0, y = 0, z = 0, status = 0;
	int pitch_h = 0, pitch_l = 0, roll_h = 0, roll_l = 0;
	int value_patch = 0, value_roll = 0;

	mmc3524x_o_enable_for_AF(1);
	mmc3524x_o_get_data_for_AF(&x, &y, &z, &status);
	SENSORDB(" m12mo get orientation x = %d, y = %d, z = %d\n",  x, y, z);
	//remap data for Camera AF
	y = -y;
	z = -z;

	value_patch = orientationsensor_value_transform(y);
	value_roll = orientationsensor_value_transform(z);
	pitch_h = (value_patch >> 8) & 0xff;
	pitch_l = (value_patch & 0xff);
	roll_h = (value_roll >> 8) & 0xff;
	roll_l = (value_roll & 0xff);
	SENSORDB(" m12mo get orientation value_patch = %d,value_roll = %d,pitch_h = %d, pitch_l= %d, roll_h= %d,roll_l=%d\n",
		value_patch,value_roll,pitch_h, pitch_l, roll_h,roll_l);
	SENSORDB(" m12mo get orientation value_patch = %d,value_roll = %d,pitch_h = 0x%x, pitch_l= 0x%x, roll_h= 0x%x,roll_l=0x%x\n",
		value_patch,value_roll,pitch_h, pitch_l, roll_h,roll_l);
	//write pitch & roll data to ISP
	M12MO_write_category_parameter_debug(0x0A, 0x12, pitch_h);
	M12MO_write_category_parameter_debug(0x0A, 0x13, pitch_l);
	M12MO_write_category_parameter_debug(0x0A, 0x14, roll_h);
	M12MO_write_category_parameter_debug(0x0A, 0x15, roll_l);
}

static void update_gsensor_data_for_AF(void)
{
	int x = 0, y = 0, z = 0, status = 0;
	int value1 = 0, value2 = 0, value3 = 0;

	bmi160_acc_get_data_for_AF(&x, &y, &z, &status);
	SENSORDB(" m12mo get gsensor x = %d, y = %d, z = %d\n",  x, y, z);

	//remap data for Camera AF
	x = -x;
	y = -y;

	value1 = gsensor_value_transform(x);
	value2 = gsensor_value_transform(y);
	value3 = gsensor_value_transform(z);
	SENSORDB(" m12mo get gsensor value1 = %d, value2 = %d, value3 = %d\n",  value1,value2,value3);
	SENSORDB(" m12mo get gsensor value1 = 0x%x, value2 = 0x%x, value3 = 0x%x\n",  value1,value2,value3);
	//write gsensor data to ISP
	M12MO_write_category_parameter_debug(0x0A, 0x59, value1);
	M12MO_write_category_parameter_debug(0x0A, 0x5A, value2);
	M12MO_write_category_parameter_debug(0x0A, 0x5B, value3);
}

static void update_gyrosensor_data_for_AF(void)
{
	int x = 0, y = 0, z = 0, status = 0;
	int value1 = 0, value2 = 0, value3 = 0;

	bmi160_gyro_get_data_for_AF(&x, &y, &z, &status);
	SENSORDB(" m12mo get gyrosensor x = %d, y = %d, z = %d\n",  x, y, z);

	//remap data for Camera AF
	x = -x;
	y = -y;

	value1 = gyrosensor_value_transform(x);
	value2 = gyrosensor_value_transform(y);
	value3 = gyrosensor_value_transform(z);
	SENSORDB(" m12mo get gyrosensor value1 = %d, value2 = %d, value3 = %d\n",  value1,value2,value3);
	SENSORDB(" m12mo get gyrosensor value1 = 0x%x, value2 = 0x%x, value3 = 0x%x\n",  value1,value2,value3);
	//write gyro sensor data to ISP
	M12MO_write_category_parameter_debug(0x0A, 0x5C, value1);
	M12MO_write_category_parameter_debug(0x0A, 0x5D, value2);
	M12MO_write_category_parameter_debug(0x0A, 0x5E, value3);
}
#endif

static M12MO_AAA_STATUS_ENUM M12MO_get_AF_status(UINT32 *pAFResult)
{
	static u8 u8AFResult = 0;
	int INT_status = 0;
	static int update_count = 4;
	
	 if (M12MO_AF_STATE_ENTERING == M12MO_Driver.afState)
    	{
        		SENSORDB("\n[M12Mo]AFGet_Status: ENTERING State\n");
        		*pAFResult = SENSOR_AF_IDLE;
        		return M12MO_AAA_AF_STATUS_OK;
    	}
	//if (u8AFResult != 0x10)
	//	u8AFResult = 0;
	if (M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
	{
		if( update_count)
		{
			SENSORDB("M12MO Ingnor update AF information!\n");
			update_count--;
		}
		else
		{
			SENSORDB("M12MO update AF information!\n");
			update_orientation_data_for_AF();
			update_gsensor_data_for_AF();
			update_gyrosensor_data_for_AF();
			M12MO_write_category_parameter_debug(0x0A, 0x58, 0x01);
			update_count = 4;
		}
	}
	 INT_status = M12MO_read_category_parameter_ram(0x00, 0x1C);
		SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
	//if(((INT_status & 0x10) == 0x10) || ((INT_status & 0x02) == 0x02))
	{
		u8AFResult = M12MO_read_category_parameter_ram(0x0A, 0x03);
		SENSORDB("(0x0A, 0x03) is 0x%x\n", u8AFResult);
	}
		//SENSORDB("(0x0A, 0x03) is 0x%x!!!!\n", u8AFResult);
	switch (u8AFResult)
	{
		case 0x00:
		{
			*pAFResult = SENSOR_AF_IDLE;
			SENSORDB("M12MO_get_AF_status  SENSOR_AF_IDLE\n");
			break;
		}

		case 0x01:
		{
		         *pAFResult = SENSOR_AF_FOCUSED;
			SENSORDB("M12MO_get_AF_status  SENSOR_AF_FOCUSED\n");
			break;
		}
		case 0x03:
		case 0x02:
		case 0x0b:
		{
			*pAFResult = SENSOR_AF_ERROR;
			SENSORDB("M12MO_get_AF_status  SENSOR_AF_ERROR\n");
			break;
		}
		case 0x10:
		case 0x0c:
		{
			*pAFResult = SENSOR_AF_FOCUSING;
			SENSORDB("M12MO_get_AF_status  SENSOR_AF_FOCUSING\n");
			break;
		}

		default:
		{
			*pAFResult = SENSOR_AF_SCENE_DETECTING;
			SENSORDB("M12MO_get_AF_status  defult status!!!\n");
			break;
		}	
	}

    	  return M12MO_AAA_AF_STATUS_OK;
}

BOOL M12MO_set_param_exposure(UINT16 para)
{  
   if(M12MO_Driver.sceneMode != 0)
	return TRUE;
   switch (para)
   {
      case AE_EV_COMP_30:  //+3 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x3C);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
      case AE_EV_COMP_20:  //+2 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x32);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
      case AE_EV_COMP_17:  // +1.7 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x2F);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
     case AE_EV_COMP_15:  // +1.5 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x2D);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
     case AE_EV_COMP_13:  // +1.3 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x2B);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
      case AE_EV_COMP_10:  // +1 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x28);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
     case AE_EV_COMP_07:  // +0.7 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x25);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
      case AE_EV_COMP_05:  // +0.5 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x23);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
      case AE_EV_COMP_03:  // +0.3 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x21);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
      case AE_EV_COMP_00:  // +0 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x1E);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
     case AE_EV_COMP_n03:  // -0.3 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x1B);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
     case AE_EV_COMP_n05:  // -0.5 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x19);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
    case AE_EV_COMP_n07:  // -0.7 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x17);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
      case AE_EV_COMP_n10:  // -1 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x14);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
     case AE_EV_COMP_n13:  // -1.3 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x11);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
     case AE_EV_COMP_n15:  // -1.5 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x0F);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
      case AE_EV_COMP_n17:  // -1.7 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x0D);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
      case AE_EV_COMP_n20:  // -2 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x0A);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
      case AE_EV_COMP_n30:   //-3 EV
		M12MO_write_category_parameter_ram(0x03, 0x09, 0x00);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
           break;
      default:
           break;
   }

   return TRUE;

} 

BOOL M12MO_Set_Exposure_Hdr(UINT16 para)
{ 
	unsigned char parameter = 0;
	if((para <= HDR_EV_COMP_30) && (para > HDR_EV_COMP_00))
	{
	  switch (para)
   	  {
      	 	case HDR_EV_COMP_30:  //+3 EV
			parameter = 0x3c;
         	  	break;
          	case HDR_EV_COMP_20:  //+2 EV
			parameter = 0x32;
           	 	break;
     	 	case HDR_EV_COMP_17:  // +1.7 EV
			parameter = 0x2F;
           		break;
     		case HDR_EV_COMP_15:  // +1.5 EV
			parameter = 0x2D;
          		 break;
     		case HDR_EV_COMP_13:  // +1.3 EV
			parameter = 0x2B;
           		break;
      		case HDR_EV_COMP_10:  // +1 EV
			parameter = 0x28;
          		 break;
     		case HDR_EV_COMP_07:  // +0.7 EV
			parameter = 0x25;
           		break;
      		case HDR_EV_COMP_05:  // +0.5 EV
			parameter = 0x23;
           		break;
      		case HDR_EV_COMP_03:  // +0.3 EV
			parameter = 0x21;
           		break;
		default:
			break;
		}
		M12MO_Driver.Hdr_EV1 = parameter;
	}
	else if(para < HDR_EV_COMP_TOTAL_NUM)
	{
	  switch (para)
   	  {
      	 	case HDR_EV_COMP_n30:  //-3 EV
			parameter = 0x00;
         	  	break;
          	case HDR_EV_COMP_n20:  //-2 EV
			parameter = 0x0A;
           	 	break;
     	 	case HDR_EV_COMP_n17:  // -1.7 EV
			parameter = 0x0D;
           		break;
     		case HDR_EV_COMP_n15:  // -1.5 EV
			parameter = 0x0F;
          		 break;
     		case HDR_EV_COMP_n13:  // -1.3 EV
			parameter = 0x11;
           		break;
      		case HDR_EV_COMP_n10:  // -1 EV
			parameter = 0x14;
          		 break;
     		case HDR_EV_COMP_n07:  // -0.7 EV
			parameter = 0x17;
           		break;
      		case HDR_EV_COMP_n05:  // -0.5 EV
			parameter = 0x19;
           		break;
      		case HDR_EV_COMP_n03:  // -0.3 EV
			parameter = 0x1B;
           		break;
		default:
			break;
		}
	M12MO_Driver.Hdr_EV2 = parameter;
	}
	   return TRUE;
}

BOOL M12MO_set_param_banding(UINT16 para)
{

   if(M12MO_Driver.Banding == para)
       return TRUE;

   spin_lock(&m12mo_drv_lock);
   M12MO_Driver.Banding = para;
   spin_unlock(&m12mo_drv_lock);

   switch (para)
   {
       case AE_FLICKER_MODE_60HZ:
       {
		M12MO_write_category_parameter_ram(0x03, 0x06, 0x02);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);

       }
       break;

       case AE_FLICKER_MODE_OFF:
       {
		M12MO_write_category_parameter_ram(0x03, 0x06, 0x04);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01); 
       }
       break;

       case AE_FLICKER_MODE_50HZ:
       case AE_FLICKER_MODE_AUTO:
       default:
       {
		M12MO_write_category_parameter_ram(0x03, 0x06, 0x01);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
          break;
       }
   }

   return KAL_TRUE;
}

void M12MO_set_iso(UINT16 para)
{
   spin_lock(&m12mo_drv_lock);
   M12MO_Driver.isoSpeed= para;
   spin_unlock(&m12mo_drv_lock);

   switch (para)
   {
       case AE_ISO_100:
       {
		M12MO_write_category_parameter_ram(0x03, 0x05, 0x02);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
       }
       break;

       case AE_ISO_200:
       {
		M12MO_write_category_parameter_ram(0x03, 0x05, 0x03);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);

       }
       break;

       case AE_ISO_400:
       {
		M12MO_write_category_parameter_ram(0x03, 0x05, 0x04);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01); 
       }
       break;
	   
       case AE_ISO_800:
       {
		M12MO_write_category_parameter_ram(0x03, 0x05, 0x05);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01); 
       }
       break;
	   
       case AE_ISO_1600:
       {
		M12MO_write_category_parameter_ram(0x03, 0x05, 0x06);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01); 
       }
       break;

       case AE_ISO_AUTO:
       default:
       {
		M12MO_write_category_parameter_ram(0x03, 0x05, 0x00);
		M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
          break;
       }
   }
}

BOOL M12MO_set_param_wb(UINT16 para)
{
	if(M12MO_Driver.sceneMode != 0)
		return TRUE;
	
      	spin_lock(&m12mo_drv_lock);
   	M12MO_Driver.awbMode= para;
  	spin_unlock(&m12mo_drv_lock);
	
	switch (para)
	{
		
		case AWB_MODE_AUTO:
		M12MO_write_category_parameter_ram(0x06, 0x02, 0x01);
		M12MO_write_category_parameter_ram(0x06, 0x0C, 0x01);
		break;
		
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
		M12MO_write_category_parameter_ram(0x06, 0x02, 0x02);
		M12MO_write_category_parameter_ram(0x06, 0x03, 0x05);
		M12MO_write_category_parameter_ram(0x06, 0x0C, 0x01);
		break;
		
		case AWB_MODE_DAYLIGHT: //sunny
		M12MO_write_category_parameter_ram(0x06, 0x02, 0x02);
		M12MO_write_category_parameter_ram(0x06, 0x03, 0x04);
		M12MO_write_category_parameter_ram(0x06, 0x0C, 0x01);		
		break;
		
		case AWB_MODE_INCANDESCENT: //office
		M12MO_write_category_parameter_ram(0x06, 0x02, 0x02);
		M12MO_write_category_parameter_ram(0x06, 0x03, 0x01);
		M12MO_write_category_parameter_ram(0x06, 0x0C, 0x01);	
		break;
		
		case AWB_MODE_FLUORESCENT:
		M12MO_write_category_parameter_ram(0x06, 0x02, 0x02);
		M12MO_write_category_parameter_ram(0x06, 0x03, 0x03);
		M12MO_write_category_parameter_ram(0x06, 0x0C, 0x01);	
		break;

		case AWB_MODE_WARM_FLUORESCENT:
		M12MO_write_category_parameter_ram(0x06, 0x02, 0x02);
		M12MO_write_category_parameter_ram(0x06, 0x03, 0x02);
		M12MO_write_category_parameter_ram(0x06, 0x0C, 0x01);	
		break;
		case AWB_MODE_SHADE: 	
		M12MO_write_category_parameter_ram(0x06, 0x02, 0x02);
		M12MO_write_category_parameter_ram(0x06, 0x03, 0x06);
		M12MO_write_category_parameter_ram(0x06, 0x0C, 0x01);	
		break;
		
		default:
		break;
	}
	    return TRUE;
}

void M12MO_set_flashlight_mode(UINT16 para)
{
	if(M12MO_Get_CameraMode()==M12MO_SUB_CAMERA)
	{
	   switch (para)
	   {		 
	       case 1://force on
		M12MO_write_category_parameter_ram(0x03, 0x3C, 0x30);
	         break;

	       case 2://force off
		M12MO_write_category_parameter_ram(0x03, 0x3C, 0x00);
	         break;

	       default:
		break;
	    }
	}
	else if(M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
	{
	   switch (para)
	   {
	        case 0://auto
		M12MO_write_category_parameter_ram(0x03, 0x3C, 0x01);
	         break;
			 
	       case 1://force on
		M12MO_write_category_parameter_ram(0x03, 0x3C, 0x02);
	         break;

	       case 2://force off
		M12MO_write_category_parameter_ram(0x03, 0x3C, 0x00);
	         break;

	       default:
		M12MO_write_category_parameter_ram(0x03, 0x3C, 0x03);
		break;
	    }
	}
}

void M12MO_set_exposure_time(void)
{
	//M12MO_write_category_parameter_ram(0x03, 0x00, 0x00);
	//M12MO_write_category_parameter_ram(0x03, 0x01, 0x01);
	M12MO_write_category_parameter_ram(0x03, 0x94, M12MO_Driver.manual_shut.cap_uint4);
	M12MO_write_category_parameter_ram(0x03, 0x95, M12MO_Driver.manual_shut.cap_uint3);
	M12MO_write_category_parameter_ram(0x03, 0x96, M12MO_Driver.manual_shut.cap_uint2);
	M12MO_write_category_parameter_ram(0x03, 0x97, M12MO_Driver.manual_shut.cap_uint1);
	M12MO_write_category_parameter_ram(0x03, 0x0D, 0x01);
}

void M12MO_set_manual_shutter(UINT16 para)
{


   int DiffEv = 0;
   long DiffEv_value = 0;
   unsigned int diff_ev4,diff_ev3,diff_ev2,diff_ev1 = 0;

   switch (para)
   {
       case EXPOSURE_TIME_1_3200:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x00;
	M12MO_Driver.manual_shut.cap_uint1 = 0x06;
	g_bIsStartlongexposertime = 1;
       }
       break;

       case EXPOSURE_TIME_1_1600:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x00;
	M12MO_Driver.manual_shut.cap_uint1 = 0x0C;
	g_bIsStartlongexposertime = 1;
       }
       break;

       case EXPOSURE_TIME_1_800:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x00;
	M12MO_Driver.manual_shut.cap_uint1 = 0x19;
	g_bIsStartlongexposertime = 1;
       }
       break;
	   
       case EXPOSURE_TIME_1_400:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x00;
	M12MO_Driver.manual_shut.cap_uint1 = 0x32;
	g_bIsStartlongexposertime = 1;
       }
       break;
	   
       case EXPOSURE_TIME_1_200:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x00;
	M12MO_Driver.manual_shut.cap_uint1 = 0x64;
	g_bIsStartlongexposertime = 1;
       }
       break;
	   
       case EXPOSURE_TIME_1_100:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x00;
	M12MO_Driver.manual_shut.cap_uint1 = 0xC8;
	g_bIsStartlongexposertime = 1;
       }
       break;

       case EXPOSURE_TIME_1_50:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x01;
	M12MO_Driver.manual_shut.cap_uint1 = 0x90;
	g_bIsStartlongexposertime = 1;
       }
       break;

       case EXPOSURE_TIME_1_25:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x03;
	M12MO_Driver.manual_shut.cap_uint1 = 0x20;
	g_bIsStartlongexposertime = 1;
       }
       break;

       case EXPOSURE_TIME_1_10:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x07;
	M12MO_Driver.manual_shut.cap_uint1 = 0xD0;
	g_bIsStartlongexposertime = 1;
       }
       break;
	   
       case EXPOSURE_TIME_1_5:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x0F;
	M12MO_Driver.manual_shut.cap_uint1 = 0xA0;
	g_bIsStartlongexposertime = 1;
       }
       break;

       case EXPOSURE_TIME_1_2:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x27;
	M12MO_Driver.manual_shut.cap_uint1 = 0x10;
	g_bIsStartlongexposertime = 1;
       }
       break;

       case EXPOSURE_TIME_1:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x4E;
	M12MO_Driver.manual_shut.cap_uint1 = 0x20;
	g_bIsStartlongexposertime = 1;
       }
       break;

       case EXPOSURE_TIME_2:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0x9C;
	M12MO_Driver.manual_shut.cap_uint1 = 0x40;
	g_bIsStartlongexposertime = 2;
       }
       break;

       case EXPOSURE_TIME_3:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x00;
	M12MO_Driver.manual_shut.cap_uint2 = 0xEA;
	M12MO_Driver.manual_shut.cap_uint1 = 0x60;
	g_bIsStartlongexposertime = 3;
       }
       break;

       case EXPOSURE_TIME_4:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x01;
	M12MO_Driver.manual_shut.cap_uint2 = 0x38;
	M12MO_Driver.manual_shut.cap_uint1 = 0x80;
	g_bIsStartlongexposertime = 4;
       }
       break;

       case EXPOSURE_TIME_6:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x01;
	M12MO_Driver.manual_shut.cap_uint2 = 0xD4;
	M12MO_Driver.manual_shut.cap_uint1 = 0xC0;
	g_bIsStartlongexposertime = 6;
       }
       break;

       case EXPOSURE_TIME_8:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x02;
	M12MO_Driver.manual_shut.cap_uint2 = 0x71;
	M12MO_Driver.manual_shut.cap_uint1 = 0x00;
	g_bIsStartlongexposertime = 8;
       }
       break;

       case EXPOSURE_TIME_10:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x03;
	M12MO_Driver.manual_shut.cap_uint2 = 0x0D;
	M12MO_Driver.manual_shut.cap_uint1 = 0x40;
	g_bIsStartlongexposertime = 10;
       }
       break;

       case EXPOSURE_TIME_12:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x03;
	M12MO_Driver.manual_shut.cap_uint2 = 0xA9;
	M12MO_Driver.manual_shut.cap_uint1 = 0x80;
	g_bIsStartlongexposertime = 12;
       }
       break;

       case EXPOSURE_TIME_14:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x04;
	M12MO_Driver.manual_shut.cap_uint2 = 0x45;
	M12MO_Driver.manual_shut.cap_uint1 = 0xC0;
	g_bIsStartlongexposertime = 14;
       }
       break;

       case EXPOSURE_TIME_16:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x04;
	M12MO_Driver.manual_shut.cap_uint2 = 0xE2;
	M12MO_Driver.manual_shut.cap_uint1 = 0x00;
	g_bIsStartlongexposertime = 16;
       }
       break;

       case EXPOSURE_TIME_20:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x06;
	M12MO_Driver.manual_shut.cap_uint2 = 0x1A;
	M12MO_Driver.manual_shut.cap_uint1 = 0x80;
	g_bIsStartlongexposertime = 20;
       }
       break;

       case EXPOSURE_TIME_24:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x07;
	M12MO_Driver.manual_shut.cap_uint2 = 0x53;
	M12MO_Driver.manual_shut.cap_uint1 = 0x00;
	g_bIsStartlongexposertime = 24;
       }
       break;

       case EXPOSURE_TIME_28:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x08;
	M12MO_Driver.manual_shut.cap_uint2 = 0x8B;
	M12MO_Driver.manual_shut.cap_uint1 = 0x80;
	g_bIsStartlongexposertime = 28;
       }
       break;

       case EXPOSURE_TIME_32:
       {
	M12MO_Driver.manual_shut.cap_uint4 = 0x00;
	M12MO_Driver.manual_shut.cap_uint3 = 0x09;
	M12MO_Driver.manual_shut.cap_uint2 = 0xC4;
	M12MO_Driver.manual_shut.cap_uint1 = 0x00;
	g_bIsStartlongexposertime = 32;
       }
       break;
	   
       case EXPOSURE_TIME_AUTO:
       default:
       {
	   M12MO_Driver.manual_shut.cap_uint4 = 0;
	   M12MO_Driver.manual_shut.cap_uint3 = 0;
	   M12MO_Driver.manual_shut.cap_uint2 = 0;
	   M12MO_Driver.manual_shut.cap_uint1 = 0;
	   g_bIsStartlongexposertime = 0;
       }
        break;

   }
   
   M12MO_set_exposure_time();

   SENSORDB("[Enter]M12MO_set_manual_shutter  para = %2x %2x %2x %2x ,time = %d\n",M12MO_Driver.manual_shut.cap_uint4,M12MO_Driver.manual_shut.cap_uint3,M12MO_Driver.manual_shut.cap_uint2,M12MO_Driver.manual_shut.cap_uint1,g_bIsStartlongexposertime);

  if (M12MO_Driver.isoSpeed != 0)
	{
	Sleep(100);
	diff_ev4 = M12MO_read_category_parameter_ram(0x03, 0x90);
	diff_ev3 = M12MO_read_category_parameter_ram(0x03, 0x91);
	diff_ev2 = M12MO_read_category_parameter_ram(0x03, 0x92);
	diff_ev1 = M12MO_read_category_parameter_ram(0x03, 0x93);

	DiffEv = (diff_ev4 << 24 ) |(diff_ev3 << 16) | (diff_ev2 << 8) | diff_ev1; 
	
	DiffEv_value = DiffEv *100 / 65535 ;
	
	SENSORDB("[Enter]M12MO_get diff ev  value = %2x %2x %2x %2x %8x %ld\n",diff_ev4,diff_ev3,diff_ev2,diff_ev1,DiffEv,DiffEv_value);
	}
}
 
void M12MO_set_manual_af(UINT16 para)
{
	unsigned char AF_status = 0;
	unsigned int retry = 20;
	 
	 M12MO_Cancel_AF();
	 
	 AF_status = M12MO_read_category_parameter_ram(0x0A,0x02);
	SENSORDB("(0x0A, 0x02) is 0x%x\n", AF_status);
	while((AF_status !=0x00) && retry)
	{
		Sleep(20);
		AF_status = M12MO_read_category_parameter_ram(0x0A,0x02);
		SENSORDB("(0x0A, 0x02) is 0x%x\n", AF_status);
		retry--;
	}
	 switch (para)
	{
		
		case FOCUS_DISTANCE_AUTO:
		M12MO_AF_Start(M12MO_AF_MODE_CONTINUOUS);
		break;
		
		case FOCUS_DISTANCE_10:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x00);
		break;
		
		case FOCUS_DISTANCE_11:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x01);
		break;
		
		case FOCUS_DISTANCE_13:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x03);		
		break;

		case FOCUS_DISTANCE_15:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x05);		
		break;
		
		case FOCUS_DISTANCE_20:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x06);
		break;

		case FOCUS_DISTANCE_25:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x07);
		break;
		
		case FOCUS_DISTANCE_30:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x08);
		break;
		
		case FOCUS_DISTANCE_40:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x0A);
		break;

		case FOCUS_DISTANCE_60:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x0D);
		break;
		
		case FOCUS_DISTANCE_80:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x0F);
		break;
		
		case FOCUS_DISTANCE_100:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x11);
		break;
		
		case FOCUS_DISTANCE_150:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x16);
		break;
		
		case FOCUS_DISTANCE_200:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x1B);
		break;
		
		case FOCUS_DISTANCE_300:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x1C);
		break;

		case FOCUS_DISTANCE_400:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x1D);
		break;
		
		case FOCUS_DISTANCE_500:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x1E);
		break;
		
		case FOCUS_DISTANCE_INF:
		M12MO_write_category_parameter_ram(0x0A, 0x17, 0x20);
		break;
		
		default:
		break;
	 }
	
}

void M12MO_set_scene_mode(UINT16 para)
{
	 spin_lock(&m12mo_drv_lock);
   	M12MO_Driver.sceneMode = para;
  	spin_unlock(&m12mo_drv_lock);
	
	 switch (para)
	 {			
        	   case SCENE_MODE_LANDSCAPE:
            SENSORDB("[M12MO]SCENE_MODE_LANDSCAPE\n");
	   M12MO_write_category_parameter_ram(0x02, 0x69, 0x02);
            break;
			
            case SCENE_MODE_SPORTS:
            SENSORDB("[M12MO]SCENE_MODE_SPORTS\n");
	   M12MO_write_category_parameter_ram(0x02, 0x69, 0x03);
	   break;	
	   
	   case SCENE_MODE_CANDLELIGHT:
	   SENSORDB("[M12MO]SCENE_MODE_CANDLELIGHT\n");
	   M12MO_write_category_parameter_ram(0x02, 0x69, 0x06);
	   break;   
	   			
            case SCENE_MODE_SUNSET:
            SENSORDB("[M12MO]SCENE_MODE_SUNSET\n");
            M12MO_write_category_parameter_ram(0x02, 0x69, 0x07);
            break;
			
            case SCENE_MODE_NIGHTSCENE:
            SENSORDB("[M12MO]SCENE_MODE_NIGHTSCENE\n");
	   M12MO_write_category_parameter_ram(0x02, 0x69, 0x08);
            break;
	   
	   case SCENE_MODE_BEACH:
	   case SCENE_MODE_SNOW:
            SENSORDB("[M12MO]SCENE_MODE_SNOW_BEACH\n");
	   M12MO_write_category_parameter_ram(0x02, 0x69, 0x09);
	   break;
	   
        	   case SCENE_MODE_PORTRAIT:
            SENSORDB("[M12MO]SCENE_MODE_PORTRAIT\n");
	   M12MO_write_category_parameter_ram(0x02, 0x69, 0x0A);
            break;	   

            case SCENE_MODE_OFF://auto mode
            SENSORDB("[M12MO]SCENE_MODE_OFF\n");
	   M12MO_write_category_parameter_ram(0x02, 0x69, 0x01);
	   break;
	   
            default:
            SENSORDB("[M12MO]SCENE_MODE default: %d\n", para);
            break;
    }
}

void M12MO_Set_Zoom(UINT16 para)
{
	 M12MO_write_category_parameter_ram(0x02, 0x01, para);
}

void M12MO_Set_WDR(UINT16 para)
{
	 M12MO_write_category_parameter_ram(0x0B, 0x2C, para);
	 M12MO_write_category_parameter_ram(0x0B, 0x2D, 0x02);
}

void M12MO_Set_MFC(UINT16 para)
{
	 M12MO_write_category_parameter_ram(0x0C, 0x0C, para);
}

static int M12MO_ReadGain(void)
{
	unsigned int gain_h,gain_l = 0;
	unsigned int IsoValue = 0;
	gain_h = M12MO_read_category_parameter_ram(0x07,0x28);
	gain_l = M12MO_read_category_parameter_ram(0x07,0x29);
	IsoValue = (gain_h << 8) | gain_l;

 	SENSORDB("[M12MO] M12MO_ReadGain: %d,%d,%d\n",gain_h,gain_l,IsoValue);
	
	return IsoValue;
}

static int M12MO_ReadPreviewGain(void)
{
	unsigned int gain_h,gain_l,gain = 0;
	unsigned int i;
	unsigned int IsoValue = 0;
	static unsigned int ValueBase = 2;
	gain_h = M12MO_read_category_parameter_ram(0x03,0x0E);
	gain_l = M12MO_read_category_parameter_ram(0x03,0x0F);
	gain = (gain_h << 8) | gain_l;
	gain = (gain+5)/10;
	if (M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
	{
		if(gain <= 6)
		{
			IsoValue = gain*100/6 +100;
		}
		else
		{
			for(i =2; i < 6; i++)
			{
				ValueBase = ValueBase *2;
				if(gain <= 6*i)
				{
					IsoValue = gain*100*ValueBase/(6 * i);
					break;
				}
			}
		}
	}
	else
	{
		if(gain <= 6)
		{
			IsoValue = gain*50/6 +50;
		}
		else
		{
			for(i =2; i < 7; i++)
			{
				ValueBase = ValueBase *2;
				if(gain <= 6*i)
				{
					IsoValue = gain*50*ValueBase/(6 * i);
					break;
				}
			}
		}
	}
	ValueBase = 2;
 	SENSORDB("[M12MO] M12MO_ReadPreviewGain: %d,%d,%d,%d\n",gain_h,gain_l,gain,IsoValue);
	return IsoValue;
}

void M12MO_GetExifInfo(uintptr_t exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = M12MO_Driver.isoSpeed;
    pExifInfo->AWBMode = M12MO_Driver.awbMode;
    pExifInfo->CapExposureTime = M12MO_Driver.capExposureTime;
    //pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue =M12MO_ReadGain();
    pExifInfo->PreISOValue =M12MO_ReadPreviewGain();
}

static void M12MO_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{
    unsigned int NormBr;

    NormBr = M12MO_read_category_parameter_ram(0x03,0x3D);
	
    SENSORDB("[M12MO] M12MO_FlashTriggerCheck: %x\n",NormBr);

     *pFeatureReturnPara32 = NormBr;

}

UINT32 M12MO_SensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{

    switch (iCmd)
    {
        case FID_SCENE_MODE:
            SENSORDB("[M12MO] FID_SCENE_MODE: para=%d\n",iPara);
            M12MO_set_scene_mode(iPara);
            break;
        case FID_AWB_MODE:
            SENSORDB("[M12MO]FID_AWB_MODE: para=%d\n", iPara);
            M12MO_set_param_wb(iPara);
            break;
        case FID_COLOR_EFFECT:
            SENSORDB("[M12MO]FID_COLOR_EFFECT para=%d\n", iPara);
           // M12MO_set_param_effect(iPara);
            break;
        case FID_FOCUS_DISTANCE:
            SENSORDB("[M12MO]FID_FOCUS_DISTANCE para=%d\n", iPara);
            M12MO_set_manual_af(iPara);
            break;
	case FID_FLASHLIGHT:
            SENSORDB("[M12MO]FID_FLASHLIGHT para=%d\n", iPara);
            M12MO_set_flashlight_mode(iPara);
            break;
        case FID_AE_EV:
            SENSORDB("[M12MO]FID_AE_EV para=%d\n", iPara);
            M12MO_set_param_exposure(iPara);
            break;
        case FID_AE_FLICKER:
            SENSORDB("[M12MO]FID_AE_FLICKER para=%d\n", iPara);
            M12MO_set_param_banding(iPara);
            break;
        case FID_AE_SCENE_MODE:
            SENSORDB("[M12MO]FID_AE_SCENE_MODE para=%d\n", iPara);
            break;
        case FID_ZOOM_FACTOR:
            SENSORDB("[M12MO]FID_ZOOM_FACTOR para=%d\n", iPara);
            break;
        case FID_ISP_CONTRAST:
            SENSORDB("[M12MO]FID_ISP_CONTRAST:%d\n",iPara);
           // M12Mo_set_contrast(iPara);
            break;
        case FID_ISP_BRIGHT:
            SENSORDB("[M12MO]FID_ISP_BRIGHT:%d\n",iPara);
            //M12MO_set_brightness(iPara);
            break;
        case FID_ISP_SAT:
            SENSORDB("[M12MO]FID_ISP_SAT:%d\n",iPara);
            //M12MO_set_saturation(iPara);
            break;
        case FID_AE_ISO:
            SENSORDB("[M12MO]FID_AE_ISO:%d\n",iPara);
            M12MO_set_iso(iPara);
            break;

        case FID_EXPOSURE_TIME:
        	SENSORDB("[M12MO]FID_EXPOSURE_TIME:%d\n",iPara);
           M12MO_set_manual_shutter(iPara);
           break;

        case FID_DIGITAL_ZOOM:
           SENSORDB("[M12MO]FID_DIGITAL_ZOOM:%d\n",iPara);
           M12MO_Set_Zoom(iPara);
           break;

        case FID_HDR_MODE:
           SENSORDB("[M12MO]FID_HDR_MODE:%d\n",iPara);
           M12MO_Set_WDR(iPara);
           break;

        case FID_MFC_MODE:
		  SENSORDB("[M12MO]FID_MFC_MODE:%d\n",iPara);
		  M12MO_Set_MFC(iPara);
		  break;

		case FID_HDR_EV:
		  SENSORDB("[M12MO]FID_HDR_EV:%d\n",iPara);
		  M12MO_Set_Exposure_Hdr(iPara);
		  break;

        default:
            SENSORDB("[M12MO]SensorSetting Default, FID=%d\n", iCmd);
            break;
    }
    return KAL_TRUE;
}   

static M12MO_AAA_STATUS_ENUM M12MO_Cancel_AF(void)
{
	unsigned char INT_status = 0;
	unsigned int retry = 50;
	unsigned int AFResult = 0;
	if ((M12MO_AF_MODE_RSVD == M12MO_Driver.afMode) ||  (M12MO_AF_MODE_SINGLE == M12MO_Driver.afMode))
    	{
        		//have been aborted
        		return M12MO_AAA_AF_STATUS_OK;
    	}

	AFResult = M12MO_read_category_parameter_ram(0x0A, 0x03);
	//if (AFResult == 0x10)
	//{
		while((AFResult == 0x10) && retry)
		{
			Sleep(10);
			AFResult = M12MO_read_category_parameter_ram(0x0A, 0x03);
			SENSORDB("(0x0A, 0x03) is 0x%x\n", AFResult);
			retry--;
		}
		INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
		SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
	//}
	
	SENSORDB("(0x0A, 0x03) is 0x%x!!!\n", AFResult);	
		
	M12MO_write_category_parameter_ram(0x0A, 0x02, 0x00);

	INT_status = 0;
	retry = 20;
	while(((INT_status & 0x10) != 0x10)  &&  ((INT_status & 0x02) !=0x02) && retry)
	{
		Sleep(10);
		INT_status = M12MO_read_category_parameter_ram(0x00,0x1C);
		SENSORDB("(0x00, 0x1C) is 0x%x\n", INT_status);
		retry--;
	}
	 spin_lock(&m12mo_drv_lock);
    	M12MO_Driver.afMode = M12MO_AF_MODE_RSVD;
    	M12MO_Driver.afState = M12MO_AF_STATE_DONE;
    	spin_unlock(&m12mo_drv_lock);

   	 return M12MO_AAA_AF_STATUS_OK;
}

UINT32 M12MO_GetDefaultFramerateByScenario(
  MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate)
{
    switch (scenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
	case MSDK_SCENARIO_ID_ZSL_PREVIEW:
	case MSDK_SCENARIO_ID_MULTI_CAMERA_CAPTURE_PREVIEW:
             *pframeRate = 300;
             break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_MULTI_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
        case MSDK_SCENARIO_ID_HDR_CAPTURE_JPEG:
             *pframeRate = 300;
             break;
        case MSDK_SCENARIO_ID_MULTI_CAMERA_PREVIEW: 
             *pframeRate = 240;
             break;
        default:
          break;
    }

  return ERROR_NONE;
}

UINT32 M12MOFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{

   	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
   	unsigned long long *pFeatureData=(unsigned long long *) pFeaturePara;
	static kal_bool SensorId = KAL_FALSE;
	static unsigned int check_retry = 5;
    	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	
	SENSORDB("Enter M12MOFeatureControl FeatureId = %d\n",FeatureId);
	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			SENSORDB("SENSOR_FEATURE_GET_RESOLUTION \n");	
           	 	*pFeatureReturnPara16++=M12MO_REAL_CAP_WIDTH;
           		 *pFeatureReturnPara16=M12MO_REAL_CAP_HEIGHT;
           		 *pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &M12MOSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:

		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
		break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
                        *pFeatureReturnPara32++=0;
                        *pFeatureParaLen=4;	    
		    break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;

		case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
             		M12MO_FlashTriggerCheck(pFeatureReturnPara32);
            	 	SENSORDB("[M12MO] F_GET_TRIGGER_FLASHLIGHT_INFO: %d\n", *pFeatureReturnPara32);
             		break;
		        //below is AF control
        		case SENSOR_FEATURE_INITIALIZE_AF:
             		SENSORDB("[M12MOFeatureControl] F_INITIALIZE_AF\n");
             		//M12MO_MIPI_AF_Init();
             		break;
		case SENSOR_FEATURE_GET_AF_STATUS:
		{
             		SENSORDB("[M12MOFeatureControl] F_GET_AF_STATUS\n");
			if (!m12mo_laser_flag_value)
				M12MO_get_AF_status(pFeatureReturnPara32);
			*pFeatureParaLen=4;
			break;
		}
		case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
		{
             		SENSORDB("[M12MOFeatureControl] F_SINGLE_FOCUS_MODE\n");			
			if (!m12mo_laser_flag_value)
				M12MO_AF_Start(M12MO_AF_MODE_SINGLE);
			break;
		}
		case SENSOR_FEATURE_CONSTANT_AF:
        		{
             		SENSORDB("[M12MOFeatureControl] F_CONSTANT_AF\n");
			if (!m12mo_laser_flag_value)
				M12MO_AF_Start(M12MO_AF_MODE_CONTINUOUS);
			break;
		}
		case SENSOR_FEATURE_CANCEL_AF:
		{
			SENSORDB("[M12MOFeatureControl] F_CANCEL_AF\r\n");	
			if (!m12mo_laser_flag_value)
				M12MO_Cancel_AF();
			break;
		}
		case SENSOR_FEATURE_INFINITY_AF:
            		SENSORDB("[M12MOFeatureControl] F_INFINITY_AF\n");
            		//M12MO_MIPI_AF_Infinity();
            		break;
		        case SENSOR_FEATURE_GET_AE_STATUS:
	           	 SENSORDB("[M12MOFeatureControl] F_GET_AE_STATUS\n");
	            	// M12MO_MIPI_AE_Get_Status(pFeatureReturnPara32);
	            	  *pFeatureParaLen=4;
	            	break;

	        case SENSOR_FEATURE_GET_AWB_STATUS:
	            	SENSORDB("[M12MOFeatureControl] F_GET_AWB_STATUS\n");
	            	//M12MO_MIPI_AWB_Get_Status(pFeatureReturnPara32);
	            	*pFeatureParaLen=4;
	           	 break;

	        case SENSOR_FEATURE_GET_AF_INF:
	             	SENSORDB("[M12MOFeatureControl] F_GET_AF_INF\n");
	             	*pFeatureReturnPara32=0;
	             	*pFeatureParaLen=4;
	            	 break;

	        case SENSOR_FEATURE_GET_AF_MACRO:
			SENSORDB("[M12MOFeatureControl] F_GET_AF_MACRO\n");
	             	*pFeatureReturnPara32=0;
	             	*pFeatureParaLen=4;
	             	break;
		case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
		//case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
		{
			SENSORDB("[M12MOFeatureControl] F_GET_AF_MAX_NUM_FOCUS_AREAS\n");
			if (M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
			{
				*pFeatureReturnPara32 = 1;
				*pFeatureParaLen=4;
				break;
			}
			else if (M12MO_Get_CameraMode()==M12MO_SUB_CAMERA)
				break;
		}
		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			SENSORDB("[M12MOFeatureControl] F_GET_AE_MAX_NUM_FOCUS_AREAS\n");
				*pFeatureReturnPara32 = 1;
				*pFeatureParaLen=4;
				break;
				
		case SENSOR_FEATURE_SET_AF_WINDOW:
             		 SENSORDB("[M12MOFeatureControl] F_SET_AF_WINDOW\n");
            		 M12MO_AF_Set_Window((uintptr_t)*pFeatureData, M12MO_Driver.Preview_Width, M12MO_Driver.Preview_Height);
             		 break;

        	         case SENSOR_FEATURE_SET_AE_WINDOW:
             		 SENSORDB("[M12MOFeatureControl] F_SET_AE_WINDOW\n");
			if (M12MO_Get_CameraMode()==M12MO_SUB_CAMERA)
            		 M12MO_AE_Set_Window((uintptr_t)*pFeatureData, M12MO_Driver.Preview_Width, M12MO_Driver.Preview_Height);
            		 break;

       		 case SENSOR_FEATURE_SET_YUV_CMD:
            		 M12MO_SensorSetting((FEATURE_ID)*pFeatureData, *(pFeatureData+1));
            		 break;
		
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			while(SensorId != KAL_TRUE && check_retry)
			{
				SensorId = M12MOCheckIsAlive();
				check_retry --;
			}
	            if (KAL_TRUE == SensorId)
	            {
			if (M12MO_Get_CameraMode()==M12MO_MAIN_CAMERA)
			{
				*pFeatureData32 =0xf1f1;
				s_DEVINFO_ccm1->device_used = DEVINFO_USED;
				devinfo_check_add_device(s_DEVINFO_ccm1);
				s_DEVINFO_ccm2->device_used = DEVINFO_USED;
				devinfo_check_add_device(s_DEVINFO_ccm2);

			}
			else if ((M12MO_Get_CameraMode()==M12MO_SUB_CAMERA) )
			{
				*pFeatureData32 =0xf1f2;
				s_DEVINFO_ccm->device_used = DEVINFO_USED;
				devinfo_check_add_device(s_DEVINFO_ccm);
			}
			else
			{
				*pFeatureData32 = 0xFFFFFFFF;
			}
			SENSORDB("SENSOR_FEATURE_CHECK_SENSOR_ID-1111111111 *pFeatureData32 =%x \n",*pFeatureData32);
				*pFeatureParaLen=4;
		    }
		  else
		    {
		    	s_DEVINFO_ccm1->device_used = DEVINFO_UNUSED;
			devinfo_check_add_device(s_DEVINFO_ccm1);
			s_DEVINFO_ccm2->device_used = DEVINFO_UNUSED;
			devinfo_check_add_device(s_DEVINFO_ccm2);
			s_DEVINFO_ccm->device_used = DEVINFO_UNUSED;
			devinfo_check_add_device(s_DEVINFO_ccm);
			*pFeatureData32 = 0xFFFFFFFF;
			SENSORDB("SENSOR_FEATURE_CHECK_SENSOR_ID-222222222 *pFeatureData32 =%x \n",*pFeatureData32);
			*pFeatureParaLen=4;
		     }
	            	break;    
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		       break;
			   
	        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
             		SENSORDB("[M12MO] F_GET_DEFAULT_FRAME_RATE_BY_SCENARIO\n");
             		M12MO_GetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData, (MUINT32 *)(uintptr_t)(*(pFeatureData+1)));
             	break;
			 
        		case SENSOR_FEATURE_GET_EXIF_INFO:
			SENSORDB("[M12MOFeatureControl] F_GET_EXIF_INFO\n");
             		M12MO_GetExifInfo((uintptr_t)*pFeatureData);
            		 break;
        		case SENSOR_FEATURE_SET_TEST_PATTERN:			 
            		//M12MOSetTestPatternMode((BOOL)*pFeatureData16);    			
			break;

       		 case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			break;
        
		default:
			break;			
	}
	return ERROR_NONE;
}

SENSOR_FUNCTION_STRUCT	SensorFuncM12MO=
{
    M12MOOpen,
    M12MOGetInfo,
    M12MOGetResolution,
    M12MOFeatureControl,
    M12MOControl,
    M12MOClose
};


UINT32 M12MO_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncM12MO;

    return ERROR_NONE;
}   /* SensorInit() */
