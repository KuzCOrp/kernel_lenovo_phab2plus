#include "m12mo_subsensoryuv_Sensor.h"
//#include <cust_eint.h>




   extern UINT32 M12MOOpen(void);
   extern UINT32  M12MOGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT * pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT * pSensorConfigData);
   extern UINT32  M12MOGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT * pSensorResolution);
   extern UINT32  M12MOFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 * pFeaturePara, UINT32 * pFeatureParaLen);
   extern UINT32  M12MOControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT * pImageWindow, MSDK_SENSOR_CONFIG_STRUCT * pSensorConfigData);
   extern UINT32  M12MOClose(void);

   UINT32 m12mo_subsensorOpen(void)
   	{
		return M12MOOpen();
   	}
   UINT32 m12mo_subsensorGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT * pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT * pSensorConfigData)
   	{
   	 	return M12MOGetInfo( ScenarioId, (MSDK_SENSOR_INFO_STRUCT *) pSensorInfo, (MSDK_SENSOR_CONFIG_STRUCT *) pSensorConfigData);
   	}
UINT32 m12mo_subsensorGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
	{
		return M12MOGetResolution((MSDK_SENSOR_RESOLUTION_INFO_STRUCT *)pSensorResolution);
	}
#if 1
UINT32 m12mo_subsensorFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 * pFeaturePara, UINT32 * pFeatureParaLen)
	{
		return  M12MOFeatureControl((MSDK_SENSOR_FEATURE_ENUM) FeatureId, (UINT8 * )pFeaturePara, (UINT32 * )pFeatureParaLen);
	}
#endif

UINT32 m12mo_subsensorControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
	{
		return M12MOControl((MSDK_SCENARIO_ID_ENUM) ScenarioId, (MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *)pImageWindow,(MSDK_SENSOR_CONFIG_STRUCT *)pSensorConfigData);
	}
UINT32 m12mo_subsensorClose(void)
	{
		return M12MOClose();
	}

#if 1
SENSOR_FUNCTION_STRUCT	SensorFuncm12mo_subsensor=
{

    m12mo_subsensorOpen,
    m12mo_subsensorGetInfo,
    m12mo_subsensorGetResolution,
    m12mo_subsensorFeatureControl,
    m12mo_subsensorControl,
    m12mo_subsensorClose
/*
    M12MOOpen,
    M12MOGetInfo,
    M12MOGetResolution,
    M12MOFeatureControl,
    M12MOControl,
    M12MOClose
    */

};


UINT32 M12MO_SUBSENSOR_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncm12mo_subsensor;

    return ERROR_NONE;
}   /* SensorInit() */
#endif

