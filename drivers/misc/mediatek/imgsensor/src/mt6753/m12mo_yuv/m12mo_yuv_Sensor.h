#ifndef __M12MOYUVSENSOR_H
#define __M12MOYUVSENSOR_H

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

#define M12MO_DEBUG_REG

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
} M12MO_INTERRUPT_TYPE;

typedef enum
{
	SYS_MODE_POWEROFF,
	SYS_MODE_PRE_INITIALIZATION,
	SYS_MODE_INITIALIZATION,
	SYS_MODE_PARAMETER_SETTING,
	SYS_MODE_MONITOR,
	SYS_MODE_MULTIMONITOR,
	SYS_MODE_DUALMONITOR,
	SYS_MODE_ZSLMONITOR,
	SYS_MODE_PRE_CAPTURE,
	SYS_MODE_SINGLE_CAPTURE,
	SYS_MODE_MULTI_CAPTURE,
	SYS_MODE_BURST_CAPTURE,
	SYS_MODE_AUTO_FOCUS,
	SYS_MODE_FACE_DETECTION,
	SYS_MODE_MAX
}M12MO_SYS_MODE;

typedef enum
{
	FOCUS_MODE_NORMAL,
	FOCUS_MODE_MACRO,
	FOCUS_MODE_FULL,
	FOCUS_MODE_MAX
}M12MO_FOCUS_MODE;

typedef enum 
{
	FOCUS_AF_NODO,
	FOCUS_AF_NODO_TOUCH_FINISH,
	FOCUS_AF_DOING,
	FOCUS_AF_DONE_NEED_NEXT_DO,
}M12MO_AF_DO_STATUS;

typedef enum
{
	M12MO_AE_MODE_AUTO,
	M12MO_AE_MODE_TOUCH,
	M12MO_AE_MODE_FACE,
	M12MO_AE_MODE_MAX
}M12MO_AE_MODE;
/*
typedef enum
{
	M12MO_NONE_CAMERA,
	M12MO_MAIN_CAMERA,
	M12MO_SUB_CAMERA,
	M12MO_ERROR_CAMERA
}M12MO_CAMERA_MODE;*/
typedef enum
{
	M12MO_FILL_FORCEOFF,
	M12MO_FILL_AUTO,
	M12MO_FILL_FORCEON
}M12MO_FILLLIGHT_MODE;
typedef enum
{
	POWEROFF,
	POWERON
}M12MO_FILLLIGHT_POWERMODE;

//AF related
typedef enum
{
    M12MO_AF_MODE_SINGLE,
    M12MO_AF_MODE_CONTINUOUS,
    M12MO_AF_MODE_RSVD,
}M12MO_AF_MODE_ENUM;

typedef enum
{
    M12MO_AAA_AF_STATUS_OK,
    M12MO_AAA_AF_STATUS_1ST_SEARCH_FAIL,
    M12MO_AAA_AF_STATUS_2ND_SEARCH_FAIL,
    M12MO_AAA_AF_STATUS_BAD_PARA,
    M12MO_AAA_AF_STATUS_BAD_RSVD,
}M12MO_AAA_STATUS_ENUM;


typedef enum
{
    M12MO_AF_STATE_UNINIT, //0
    //M12MO_AF_STATE_INIT_DONE,
    M12MO_AF_STATE_IDLE,  //1

    M12MO_AF_STATE_1ST_SEARCHING, //2, focusing
    //M12MO_AF_STATE_1ST_SEARCH_DONE, //focusing

    M12MO_AF_STATE_2ND_SEARCHING, //3, focused
    //M12MO_AF_STATE_2ND_SEARCH_DONE, //focused

    M12MO_AF_STATE_ERROR, //4
    M12MO_AF_STATE_CANCEL, //5

    M12MO_AF_STATE_DONE, //6, focused


    M12MO_AF_STATE_ENTERING, //
    M12MO_AF_STATE_ENTERED, //

    M12MO_AF_STATE_BAD_RSVD,
} M12MO_AF_STATE_ENUM;


typedef enum
{
    M12MO_AE_STATE_LOCK,
    M12MO_AE_STATE_UNLOCK,
    M12MO_AE_STATE_BAD_RSVD,
} M12MO_AE_STATE_ENUM;


typedef struct
{
   unsigned char cap_uint4; // 0x94
   unsigned char cap_uint3; // 0x95
   unsigned char cap_uint2; // 0x96
   unsigned char cap_uint1; // 0x97
}M12MO_MANUAL_SHUTTER_T;

typedef struct
{
  unsigned char supplier_id;
  unsigned char build_stage;
  unsigned char pdaf_pixel_flag;
  unsigned char pdaf_calibration_flag;
  unsigned char depthmap_flag;
}S5K2M8_INFORMATION_T;

typedef struct
{
  unsigned char lsc_awb_flag;
  unsigned char module_id;
  unsigned char lens_id;
  unsigned char year;
  unsigned char month;
  unsigned char day;
  unsigned char vcm_id;
  unsigned char driver_ic_id;
  unsigned char ir_bg_id;
  unsigned char color_temperature_id;
  unsigned char af_ff_flag;
  unsigned char light_source_flag;
  unsigned char af_50cm_value_h;
  unsigned char af_50cm_value_l;
}OV13850_INFORMATION_T;

typedef struct
{
  unsigned char basic_info_flag;
  unsigned char module_id;
  unsigned char lens_id;
  unsigned char year;
  unsigned char month;
  unsigned char day;
  bool read_flag;
}OV8865_INFORMATION_T;

typedef struct
{
   unsigned int inWx; // inner window width
   unsigned int inWy; // inner window height
   unsigned int inWw; // inner window width
   unsigned int inWh; // inner window height

   unsigned int outWx; // outer window width
   unsigned int outWy; // outer window height
   unsigned int outWw; // outer window width
   unsigned int outWh; // outer window height
}M12MO_MIPI_AF_WIN_T;


#define M12MO_MIPI_AF_CALLER_WINDOW_WIDTH  320
#define M12MO_MIPI_AF_CALLER_WINDOW_HEIGHT 240

//////////////////////////////////////////////////////////




typedef enum {
    SENSOR_MODE_INIT = 0,
    SENSOR_MODE_PREVIEW,
    SENSOR_MODE_CAPTURE,
    SENSOR_MODE_VIDEO
} M12MO_MIPI_SENSOR_MODE;



struct M12MO_MIPI_sensor_struct
{
    kal_uint16 sensor_id;

    kal_uint32 Preview_PClk;

    //kal_uint32 Preview_Lines_In_Frame;
    //kal_uint32 Capture_Lines_In_Frame;
    //kal_uint32 Preview_Pixels_In_Line;
    //kal_uint32 Capture_Pixels_In_Line;
    kal_uint32 Preview_Width;
    kal_uint32 Preview_Height;

    kal_uint16 Preview_Shutter;
    kal_uint16 Capture_Shutter;

    kal_uint16 StartX;
    kal_uint16 StartY;
    kal_uint16 iGrabWidth;
    kal_uint16 iGrabheight;

    kal_uint16 Capture_Size_Width;
    kal_uint16 Capture_Size_Height;
    kal_uint32 Digital_Zoom_Factor;

    kal_uint16 Max_Zoom_Factor;

    kal_uint32 Min_Frame_Rate;
    kal_uint32 Max_Frame_Rate;
    kal_uint32 Fixed_Frame_Rate;
    //M12MO_Camco_MODE Camco_mode;
    AE_FLICKER_MODE_T Banding;

    kal_bool Night_Mode;

    M12MO_MIPI_SENSOR_MODE sensorMode;
    kal_uint32 Period_PixelNum;
    kal_uint32 Period_LineNum;

    kal_uint16 Dummy_Pixels;
    kal_uint16 Dummy_Lines;

    kal_uint32 shutter;
    kal_uint32 sensorGain;

    kal_bool   aeEnable;
    kal_bool   awbEnable;


    //AF related
    M12MO_MIPI_AF_WIN_T  afWindows;
    M12MO_MIPI_AF_WIN_T  orignalAfWindows;
    unsigned int            prevAfWinWidth;
    unsigned int            prevAfWinHeight;

    kal_bool                afStateOnOriginalSet; //keep the AF on original setting
    kal_bool                aeStateOnOriginalSet; //keep the AF on original setting

    M12MO_AF_MODE_ENUM   afMode;
    M12MO_AF_STATE_ENUM  afState;
    M12MO_AF_MODE_ENUM   afPrevMode;


    M12MO_AE_STATE_ENUM  aeState;
	
    M12MO_MANUAL_SHUTTER_T manual_shut;
	
    S5K2M8_INFORMATION_T s5k2m8;
    OV13850_INFORMATION_T ov13850;
    OV8865_INFORMATION_T ov8865;
	
    unsigned int            aeWindows[4];
    unsigned int            apAEWindows[6];

    kal_uint32              videoFrameRate;
    unsigned char           ae_table[64]; //weighting table
    unsigned int            mapAEWindows[4]; //window

    ///To control 3A Lock
    kal_bool                userAskAeLock;
    kal_bool                userAskAwbLock;

    kal_uint32              sceneMode;

    ///For HDR mode to optimize EV stable time
    kal_uint32              currentExposureTime;
    kal_uint32              currentShutter;
    kal_uint32              currentAxDGain;
    kal_bool                manualAEStart;

    kal_uint16 	Hdr_EV1;
    kal_uint16 	Hdr_EV2;
    //For EXIF info
    unsigned char           isoSpeed;
    unsigned char           awbMode;
    unsigned int            capExposureTime;

    ACDK_SENSOR_JPEG_OUTPUT_PARA jpegSensorPara;
    ACDK_SENSOR_JPEG_INFO        jpegSensorInfo;
    unsigned int                 jpegSensorHDRDelayFrmCnt;


};

#define M12MO_READ_RAW_DATA				(0)
#define M12MO_READ_RAW_300W				(0)
#define BINNING_MODE 						(0)
#define M12MO_PV_WIDTH					(1440)//(1280)//(1280)//(1600)//
#define M12MO_PV_HEIGHT					(1080)//(720)//(960)//(1200)//

#define M12MO_PV_DEPTH_WIDTH					(1920)//(1280)//(1280)//(1600)//
#define M12MO_PV_DEPTH_HEIGHT					(1140)//(720)//(960)//(1200)//

#define M12MO_FULL_DEPTH_WIDTH				(4160)
#define M12MO_FULL_DEPTH_HEIGHT				(3454)

#define M12MO_FULL_WIDTH					(4160)
#define M12MO_FULL_HEIGHT				(3120)

#define M12MO_REAL_PV_WIDTH				(M12MO_PV_WIDTH)//(M12MO_PV_WIDTH-16)
#define M12MO_REAL_PV_HEIGHT				(M12MO_PV_HEIGHT)//(M12MO_PV_HEIGHT-9)//

#define M12MO_VIDEO_PV_WIDTH				(1920)
#define M12MO_VIDEO_PV_HEIGHT				(1080)

#define M12MO_REAL_CAP_WIDTH				(M12MO_FULL_WIDTH )//(M12MO_FULL_WIDTH - 64)//M12MO_VIDEO_PV_WIDTH//M12MO_REAL_PV_WIDTH//
#define M12MO_REAL_CAP_HEIGHT				(M12MO_FULL_HEIGHT)//M12MO_VIDEO_PV_HEIGHT//M12MO_REAL_PV_HEIGHT//


#define M12MO_SUBSESNOR_REAL_CAP_WIDTH				(3264)
#define M12MO_SUBSENSOR_REAL_CAP_HEIGHT				(2448)

#define M12MO_PREVIEW_START_X			(0)//(16)
#define M12MO_PREVIEW_START_Y			(0)//(9)//  // The value must bigger or equal than 1.

#define M12MO_CAPTURE_START_X			(0)//(4)//(4)//
#define M12MO_CAPTURE_START_Y			(0)//(9)//(2)//(2)//

#define M12MO_WRITE_ID 					(0x3E)
#define M12MO_READ_ID					(0x3F)

//#define M12MO_SENSOR_ID					(0x0F16)

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
#define MAX_SPI_TRANSFER_SIZE					(60 * 1024)//1024//(1024*256)//32//

/*m12mo pin address*/
#define M12MO_ADDRESS_OF_PIN1					(0x90001200)
#define M12MO_ADDRESS_OF_PIN2					(0x90001000)
#define M12MO_ADDRESS_OF_PIN3					(0x90001100)

/* set M12MO pin by using "M12MO memory write" command */
#define M12MO_PIN_DATA_SIZE						(64)
/*char data_pin1[M12MO_PIN_DATA_SIZE] = 
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

char data_pin2[M12MO_PIN_DATA_SIZE] = 
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

char data_pin3[M12MO_PIN_DATA_SIZE] = 
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
*/

//export functions
UINT32 M12MOOpen(void);
UINT32 M12MOGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 M12MOGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 M12MOControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 M12MOFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 M12MOClose(void);

//e_add for porting
//e_add for porting
//e_add for porting

#endif /* __M12MOYUVSENSOR_H */

