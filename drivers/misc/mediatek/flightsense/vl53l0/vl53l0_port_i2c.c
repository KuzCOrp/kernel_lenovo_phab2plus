/*
 * vl53l0_port_i2c.c
 *
 *  Created on: July, 2015
 *      Author:  Teresa Tao
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include "stmvl53l0-i2c.h"
#include "stmvl53l0-cci.h"
#include "vl53l0_platform.h"
#include "vl53l0_i2c_platform.h"
#include "stmvl53l0.h"

#define I2C_M_WR			0x00
#define STATUS_OK			0x00
#define STATUS_FAIL			(-1)
/** int  VL53L0_I2CWrite(VL53L0_Dev_t dev, void *buff, uint8_t len);
 * @brief       Write data buffer to VL53L0 device via i2c
 * @param dev   The device to write to
 * @param buff  The data buffer
 * @param len   The length of the transaction in byte
 * @return      0 on success
 */
int VL53L0_I2CWrite(VL53L0_DEV dev, uint8_t *buff, uint8_t len)
{


	int err = 0;
	if (dev->bus_type == CCI_BUS) {
#ifdef CAMERA_CCI
		uint16_t index;
		struct cci_data *cci_client_obj =
			(struct cci_data *)dev->client_object;
		struct msm_camera_i2c_client *client = cci_client_obj->client;

		index = buff[0];
		/*pr_err("%s: index: %d len: %d\n", __func__, index, len); */

		if (len == 2) {
			uint8_t data;

			data = buff[1];
			/* for byte access */
			err = client->i2c_func_tbl->i2c_write(client, index,
					data, MSM_CAMERA_I2C_BYTE_DATA);
			if (err < 0) {
				pr_err("%s:%d failed status=%d\n",
					__func__, __LINE__, err);
				return err;
			}
		} else if (len == 3) {
			uint16_t data;

			data = ((uint16_t)buff[1] << 8) | (uint16_t)buff[2];
			err = client->i2c_func_tbl->i2c_write(client, index,
					data, MSM_CAMERA_I2C_WORD_DATA);
			if (err < 0) {
				pr_err("%s:%d failed status=%d\n",
					__func__, __LINE__, err);
				return err;
			}
		} else if (len >= 5) {
			err = client->i2c_func_tbl->i2c_write_seq(client,
					index, &buff[1], (len-1));
			if (err < 0) {
				pr_err("%s:%d failed status=%d\n",
					__func__, __LINE__, err);
				return err;
			}

		}
#endif
#ifndef CAMERA_CCI
	} else {
		struct i2c_msg msg[1];
		struct i2c_data *i2c_client_obj =
					(struct i2c_data *)dev->client_object;
		struct i2c_client *client = (struct i2c_client*)i2c_client_obj->client;

	#if 1
		uint8_t buffer[5] = {0};

		buffer[0] = 0x05;
		buffer[1] = 0x02;
		buffer[2] = 0x0d;
		buffer[3] = 0x32;
		buffer[4] = 0x01;

		memset(msg, 0, sizeof(struct i2c_msg));
		msg[0].addr = 0x1f;
		msg[0].flags = I2C_M_WR;
		msg[0].buf = buffer;
		msg[0].len = 5;

		err = i2c_transfer(client->adapter, msg, 1);
		/* return the actual messages transfer */
		if (err != 1) {
			pr_err("%s: set access device err:%d, addr:0x%x, reg:0x%x\n",
				__func__, err, client->addr,
				(buff[0] << 8 | buff[1]));
			return STATUS_FAIL;
		}

		buffer[0] = 0x05;
		buffer[1] = 0x02;
		buffer[2] = 0x0d;
		buffer[3] = 0x57;
		buffer[4] = buff[0];

		memset(msg, 0, sizeof(struct i2c_msg));
		msg[0].addr = 0x1f;
		msg[0].flags = I2C_M_WR;
		msg[0].buf = buffer;
		msg[0].len = 5;

		err = i2c_transfer(client->adapter, msg, 1);
		/* return the actual messages transfer */
		if (err != 1) {
			pr_err("%s: set write address err:%d, addr:0x%x, reg:0x%x\n",
				__func__, err, client->addr,
				(buff[0] << 8 | buff[1]));
			return STATUS_FAIL;
		}

		if (len == 2)
		{
			buffer[0] = 0x05;
			buffer[1] = 0x02;
			buffer[2] = 0x0d;
			buffer[3] = 0x53;
			buffer[4] = buff[1];

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: set 8bit write data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x05;
			buffer[1] = 0x02;
			buffer[2] = 0x0d;
			buffer[3] = 0x33;
			buffer[4] = 0x12;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: 8bit data write execution err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}
		}
		else if (len == 3)
		{
			buffer[0] = 0x05;
			buffer[1] = 0x02;
			buffer[2] = 0x0d;
			buffer[3] = 0x52;
			buffer[4] = buff[1];

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: set 16bit(15-8) write data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x05;
			buffer[1] = 0x02;
			buffer[2] = 0x0d;
			buffer[3] = 0x53;
			buffer[4] = buff[2];

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: set 16bit(7-0) write data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}


			buffer[0] = 0x05;
			buffer[1] = 0x02;
			buffer[2] = 0x0d;
			buffer[3] = 0x33;
			buffer[4] = 0x14;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: 16bit data write execution err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}
		}
		else if (len == 5)
		{
			buffer[0] = 0x05;
			buffer[1] = 0x02;
			buffer[2] = 0x0d;
			buffer[3] = 0x50;
			buffer[4] = buff[1];

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: set 32bit(31-24) write data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x05;
			buffer[1] = 0x02;
			buffer[2] = 0x0d;
			buffer[3] = 0x51;
			buffer[4] = buff[2];

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: set 32bit(23-16)  write data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x05;
			buffer[1] = 0x02;
			buffer[2] = 0x0d;
			buffer[3] = 0x52;
			buffer[4] = buff[3];

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: set 32bit(15-8) write data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x05;
			buffer[1] = 0x02;
			buffer[2] = 0x0d;
			buffer[3] = 0x53;
			buffer[4] = buff[4];

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: set 32bit(7-0) write data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x05;
			buffer[1] = 0x02;
			buffer[2] = 0x0d;
			buffer[3] = 0x33;
			buffer[4] = 0x16;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: 32bit data write execution err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}
		}
	#else
		msg[0].addr = client->addr;
		msg[0].flags = I2C_M_WR;
		msg[0].buf = buff;
		msg[0].len = len;

		err = i2c_transfer(client->adapter, msg, 1);
		/* return the actual messages transfer */
		if (err != 1) {
			pr_err("%s: i2c_transfer err:%d, addr:0x%x, reg:0x%x\n",
				__func__, err, client->addr,
				(buff[0] << 8 | buff[1]));
			return STATUS_FAIL;
		}
	#endif
#endif
	}

	return 0;
}


/** int VL53L0_I2CRead(VL53L0_Dev_t dev, void *buff, uint8_t len);
 * @brief       Read data buffer from VL53L0 device via i2c
 * @param dev   The device to read from
 * @param buff  The data buffer to fill
 * @param len   The length of the transaction in byte
 * @return      transaction status
 */
int VL53L0_I2CRead(VL53L0_DEV dev, uint8_t *buff, uint8_t len)
{

	int err = 0;

	if (dev->bus_type == CCI_BUS) {
#ifdef CAMERA_CCI
		uint16_t index;
		struct cci_data *cci_client_obj =
				(struct cci_data *)dev->client_object;
		struct msm_camera_i2c_client *client = cci_client_obj->client;

		index = buff[0];
		/* pr_err("%s: index: %d\n", __func__, index); */
		err = client->i2c_func_tbl->i2c_read_seq(client,
							index, buff, len);
		if (err < 0) {
			pr_err("%s:%d failed status=%d\n",
					__func__, __LINE__, err);
			return err;
		}
#endif
	} else {
#ifndef CAMERA_CCI
		struct i2c_msg msg[1];
		struct i2c_data *i2c_client_obj =
					(struct i2c_data *)dev->client_object;
		struct i2c_client *client = (struct i2c_client*) i2c_client_obj->client;

	#if 1
		uint8_t buffer[5] = {0};
	
		buffer[0] = 0x05;
		buffer[1] = 0x02;
		buffer[2] = 0x0d;
		buffer[3] = 0x33;
		if (len == 1)
		{
			buffer[4] = 0x11;
		}
		else if (len == 2)
		{
			buffer[4] = 0x13;
		}
		else if (len == 4)
		{
			buffer[4] = 0x15;
		}

		memset(msg, 0, sizeof(struct i2c_msg));
		msg[0].addr = 0x1f;
		msg[0].flags = I2C_M_WR;
		msg[0].buf = buffer;
		msg[0].len = 5;

		err = i2c_transfer(client->adapter, msg, 1);
		/* return the actual messages transfer */
		if (err != 1) {
			pr_err("%s: data read execution err:%d, addr:0x%x, reg:0x%x\n",
				__func__, err, client->addr,
				(buff[0] << 8 | buff[1]));
			return STATUS_FAIL;
		}

		if (len == 1)
		{
			buffer[0] = 0x05;
			buffer[1] = 0x01;
			buffer[2] = 0x0d;
			buffer[3] = 0x53;
			buffer[4] = 0x01;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: get read data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x00;
			buffer[1] = 0x00;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_RD|client->flags;
			msg[0].buf = buffer;
			msg[0].len = 2;

			err = i2c_transfer(client->adapter, &msg[0], 1);
			/* return the actual mesage transfer */
			if (err != 1) {
				pr_err("%s: Read i2c_transfer err:%d, addr:0x%x\n",
					__func__, err, client->addr);
				return STATUS_FAIL;
			}

			buff[0] = buffer[1];
		}

		if (len == 2)
		{
			buffer[0] = 0x05;
			buffer[1] = 0x01;
			buffer[2] = 0x0d;
			buffer[3] = 0x52;
			buffer[4] = 0x01;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: get read data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x00;
			buffer[1] = 0x00;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_RD|client->flags;
			msg[0].buf = buffer;
			msg[0].len = 2;

			err = i2c_transfer(client->adapter, &msg[0], 1);
			/* return the actual mesage transfer */
			if (err != 1) {
				pr_err("%s: Read i2c_transfer err:%d, addr:0x%x\n",
					__func__, err, client->addr);
				return STATUS_FAIL;
			}

			buff[0] = buffer[1];

			buffer[0] = 0x05;
			buffer[1] = 0x01;
			buffer[2] = 0x0d;
			buffer[3] = 0x53;
			buffer[4] = 0x01;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: get read data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x00;
			buffer[1] = 0x00;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_RD|client->flags;
			msg[0].buf = buffer;
			msg[0].len = 2;

			err = i2c_transfer(client->adapter, &msg[0], 1);
			/* return the actual mesage transfer */
			if (err != 1) {
				pr_err("%s: Read i2c_transfer err:%d, addr:0x%x\n",
					__func__, err, client->addr);
				return STATUS_FAIL;
			}

			buff[1] = buffer[1];
		}

		if (len == 4)
		{
			buffer[0] = 0x05;
			buffer[1] = 0x01;
			buffer[2] = 0x0d;
			buffer[3] = 0x50;
			buffer[4] = 0x01;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: get read data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x00;
			buffer[1] = 0x00;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_RD|client->flags;
			msg[0].buf = buffer;
			msg[0].len = 2;

			err = i2c_transfer(client->adapter, &msg[0], 1);
			/* return the actual mesage transfer */
			if (err != 1) {
				pr_err("%s: Read i2c_transfer err:%d, addr:0x%x\n",
					__func__, err, client->addr);
				return STATUS_FAIL;
			}

			buff[0] = buffer[1];

			buffer[0] = 0x05;
			buffer[1] = 0x01;
			buffer[2] = 0x0d;
			buffer[3] = 0x51;
			buffer[4] = 0x01;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: get read data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x00;
			buffer[1] = 0x00;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_RD|client->flags;
			msg[0].buf = buffer;
			msg[0].len = 2;

			err = i2c_transfer(client->adapter, &msg[0], 1);
			/* return the actual mesage transfer */
			if (err != 1) {
				pr_err("%s: Read i2c_transfer err:%d, addr:0x%x\n",
					__func__, err, client->addr);
				return STATUS_FAIL;
			}

			buff[1] = buffer[1];

			buffer[0] = 0x05;
			buffer[1] = 0x01;
			buffer[2] = 0x0d;
			buffer[3] = 0x52;
			buffer[4] = 0x01;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: get read data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x00;
			buffer[1] = 0x00;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_RD|client->flags;
			msg[0].buf = buffer;
			msg[0].len = 2;

			err = i2c_transfer(client->adapter, &msg[0], 1);
			/* return the actual mesage transfer */
			if (err != 1) {
				pr_err("%s: Read i2c_transfer err:%d, addr:0x%x\n",
					__func__, err, client->addr);
				return STATUS_FAIL;
			}

			buff[2] = buffer[1];

			buffer[0] = 0x05;
			buffer[1] = 0x01;
			buffer[2] = 0x0d;
			buffer[3] = 0x53;
			buffer[4] = 0x01;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_WR;
			msg[0].buf = buffer;
			msg[0].len = 5;

			err = i2c_transfer(client->adapter, msg, 1);
			/* return the actual messages transfer */
			if (err != 1) {
				pr_err("%s: get read data err:%d, addr:0x%x, reg:0x%x\n",
					__func__, err, client->addr,
					(buff[0] << 8 | buff[1]));
				return STATUS_FAIL;
			}

			buffer[0] = 0x00;
			buffer[1] = 0x00;

			memset(msg, 0, sizeof(struct i2c_msg));
			msg[0].addr = 0x1f;
			msg[0].flags = I2C_M_RD|client->flags;
			msg[0].buf = buffer;
			msg[0].len = 2;

			err = i2c_transfer(client->adapter, &msg[0], 1);
			/* return the actual mesage transfer */
			if (err != 1) {
				pr_err("%s: Read i2c_transfer err:%d, addr:0x%x\n",
					__func__, err, client->addr);
				return STATUS_FAIL;
			}

			buff[3] = buffer[1];
		}
	#else
		msg[0].addr = client->addr;
		msg[0].flags = I2C_M_RD|client->flags;
		msg[0].buf = buff;
		msg[0].len = len;

		err = i2c_transfer(client->adapter, &msg[0], 1);
		/* return the actual mesage transfer */
		if (err != 1) {
			pr_err("%s: Read i2c_transfer err:%d, addr:0x%x\n",
				__func__, err, client->addr);
			return STATUS_FAIL;
		}
	#endif
#endif
	}

	return 0;
}
