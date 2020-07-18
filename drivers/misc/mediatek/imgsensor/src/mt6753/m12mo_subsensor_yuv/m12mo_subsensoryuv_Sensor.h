#ifndef __m12mo_subsensorYUVSENSOR_H
#define __m12mo_subsensorYUVSENSOR_H

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <asm/atomic.h>

#include "tb_kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"
#include "kd_camera_typedef.h"

//#include "m12mo_bin.h"

typedef enum
{
	INT_STATUS_MODE,
	INT_STATUS_AF,
	INT_STATUS_ZOOM,
	INT_STATUS_CAPTURE,
	INT_STATUS_FRAMESYNC,
	INT_STATUS_FD,
	INT_STATUS_INIT,
	INT_STATUS_SOUND,
	INT_STATUS_MAX
} m12mo_subsensor_INTERRUPT_TYPE;

typedef enum
{
	SYS_MODE_POWEROFF,
	SYS_MODE_PRE_INITIALIZATION,
	SYS_MODE_INITIALIZATION,
	SYS_MODE_PARAMETER_SETTING,
	SYS_MODE_MONITOR,
	SYS_MODE_PRE_CAPTURE,
	SYS_MODE_SINGLE_CAPTURE,
	SYS_MODE_AUTO_FOCUS,
	SYS_MODE_FACE_DETECTION,
	SYS_MODE_MAX
}m12mo_subsensor_SYS_MODE;

typedef enum
{
	FOCUS_MODE_NORMAL,
	FOCUS_MODE_MACRO,
	FOCUS_MODE_FULL,
	FOCUS_MODE_MAX
}m12mo_subsensor_FOCUS_MODE;

typedef enum 
{
	FOCUS_AF_NODO,
	FOCUS_AF_NODO_TOUCH_FINISH,
	FOCUS_AF_DOING,
	FOCUS_AF_DONE_NEED_NEXT_DO,
}m12mo_subsensor_AF_DO_STATUS;

typedef enum
{
	m12mo_subsensor_AE_MODE_AUTO,
	m12mo_subsensor_AE_MODE_TOUCH,
	m12mo_subsensor_AE_MODE_FACE,
	m12mo_subsensor_AE_MODE_MAX
}m12mo_subsensor_AE_MODE;

#define m12mo_subsensor_READ_RAW_DATA				(0)

#define m12mo_subsensor_PV_WIDTH					(1920)//(1280)//(1280)//(1600)
#define m12mo_subsensor_PV_HEIGHT					(1080)//(720)//(960)//(1200)

#define m12mo_subsensor_FULL_WIDTH					(4128)//(3264)
#define m12mo_subsensor_FULL_HEIGHT				(3096)//(2448)

#define m12mo_subsensor_REAL_PV_WIDTH				(m12mo_subsensor_PV_WIDTH-16)//(m12mo_subsensor_PV_WIDTH-16)
#define m12mo_subsensor_REAL_PV_HEIGHT				(m12mo_subsensor_PV_HEIGHT-9)//(m12mo_subsensor_PV_HEIGHT-12)

#define m12mo_subsensor_VIDEO_PV_WIDTH				(1280)
#define m12mo_subsensor_VIDEO_PV_HEIGHT				(960)

#if m12mo_subsensor_READ_RAW_DATA
#define m12mo_subsensor_REAL_CAP_WIDTH				(4160/2-10)
#define m12mo_subsensor_REAL_CAP_HEIGHT			(2460)
#else
#define m12mo_subsensor_REAL_CAP_WIDTH				(m12mo_subsensor_FULL_WIDTH - 16)//(m12mo_subsensor_FULL_WIDTH - 64)//m12mo_subsensor_VIDEO_PV_WIDTH//m12mo_subsensor_REAL_PV_WIDTH//
#define m12mo_subsensor_REAL_CAP_HEIGHT			(m12mo_subsensor_FULL_HEIGHT - 12)//m12mo_subsensor_VIDEO_PV_HEIGHT//m12mo_subsensor_REAL_PV_HEIGHT//
#endif

#define m12mo_subsensor_PREVIEW_START_X			(16)//(16)
#define m12mo_subsensor_PREVIEW_START_Y			(9)//(12)  // The value must bigger or equal than 1.

#define m12mo_subsensor_CAPTURE_START_X			(16)//(4)//(4)//
#define m12mo_subsensor_CAPTURE_START_Y			(12)//(9)//(2)//(2)//

#define m12mo_subsensor_WRITE_ID 					(0x3E)
#define m12mo_subsensor_READ_ID					(0x3F)

#define m12mo_subsensor_SENSOR_ID					(0x0F16)

#define CAM_SIZE_QVGA_WIDTH         	(320)
#define CAM_SIZE_QVGA_HEIGHT        	(240)
#define CAM_SIZE_VGA_WIDTH            	(640)
#define CAM_SIZE_VGA_HEIGHT           	(480)
#define CAM_SIZE_05M_WIDTH            	(800)
#define CAM_SIZE_05M_HEIGHT           	(600)
#define CAM_SIZE_1M_WIDTH              	(1280)
#define CAM_SIZE_1M_HEIGHT             	(960)
#define CAM_SIZE_2M_WIDTH              	(1600)
#define CAM_SIZE_2M_HEIGHT             	(1200)
#define CAM_SIZE_3M_WIDTH              	(2048)
#define CAM_SIZE_3M_HEIGHT             	(1536)
#define CAM_SIZE_5M_WIDTH              	(2592)
#define CAM_SIZE_5M_HEIGHT             	(1944)
#define CAM_SIZE_8M_WIDTH              	(3264)
#define CAM_SIZE_8M_HEIGHT             	(2448)

/*Command ID*/
#define COMMAND_ID_READ_CATEGORY_PARAMETER		(0x01)
#define COMMAND_ID_WRITE_CATEGORY_PARAMETER		(0x02)
#define COMMAND_ID_READ_MEMORY_8BIT_ACCESS		(0x03)
#define COMMAND_ID_WRITE_MEMORY_8BIT_ACCESS		(0x04)

/*MAX data size send by I2C*/
#define MAX_MEMORY_DATA_SIZE					(0xFF-8) //256-8

/*SIO loader program start address*/
#define SIO_LOADER_PROGRAM_START_ADDR			(0x01000100)

/*Camera FirmWare top address*/
#define CAMERA_FIRMWARE_TOP_ADDR				(0x20000000)

/*Max data size send by SPI*/
#define MAX_SPI_TRANSFER_SIZE					(24 * 1024)//1024//(1024*256)//32//

/*m12mo_subsensor pin address*/
#define m12mo_subsensor_ADDRESS_OF_PIN1					(0x90001200)
#define m12mo_subsensor_ADDRESS_OF_PIN2					(0x90001000)
#define m12mo_subsensor_ADDRESS_OF_PIN3					(0x90001100)

/* set m12mo_subsensor pin by using "m12mo_subsensor memory write" command */
#define m12mo_subsensor_PIN_DATA_SIZE						(64)
char data_pin11[m12mo_subsensor_PIN_DATA_SIZE] = 
{
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
	0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF,
};

char data_pin21[m12mo_subsensor_PIN_DATA_SIZE] = 
{
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

char data_pin31[m12mo_subsensor_PIN_DATA_SIZE] = 
{
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,
	0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x10,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


//export functions
UINT32 m12mo_subsensorOpen(void);
UINT32 m12mo_subsensorGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 m12mo_subsensorGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 m12mo_subsensorControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 m12mo_subsensorFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 m12mo_subsensorClose(void);

//e_add for porting
//e_add for porting
//e_add for porting

#endif /* __m12mo_subsensorYUVSENSOR_H */

