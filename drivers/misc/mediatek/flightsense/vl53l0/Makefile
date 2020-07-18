ccflags-y += -I$(srctree)/drivers/misc/mediatek/flightsense/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach/

obj-y	:=  stmvl53l0.o
stmvl53l0-objs :=  stmvl53l0_module.o stmvl53l0_module-i2c.o stmvl53l0_module-cci.o vl53l0_api_calibration.o vl53l0_api_core.o vl53l0_api_histogram.o vl53l0_api_ranging.o vl53l0_api_strings.o vl53l0_api.o vl53l0_platform.o vl53l0_i2c_platform.o vl53l0_port_i2c.o vl53l010_api.o vl53l010_tuning.o
