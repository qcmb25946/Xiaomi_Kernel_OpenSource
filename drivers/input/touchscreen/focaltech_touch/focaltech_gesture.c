<<<<<<< HEAD
=======
// SPDX-License-Identifier: GPL-2.0-only
>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d
/*
 *
 * FocalTech TouchScreen driver.
 *
<<<<<<< HEAD
 * Copyright (c) 2010-2017, Focaltech Ltd. All rights reserved.
 * Copyright (C) 2020 XiaoMi, Inc.
=======
 * Copyright (c) 2012-2019, Focaltech Ltd. All rights reserved.
>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d
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

/*****************************************************************************
*
* File Name: focaltech_gestrue.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"
<<<<<<< HEAD
#include <linux/input/touch_common_info.h>
#if FTS_GESTURE_EN
/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define KEY_GESTURE_U							KEY_WAKEUP
#define KEY_GESTURE_UP							KEY_UP
#define KEY_GESTURE_DOWN						KEY_DOWN
#define KEY_GESTURE_LEFT						KEY_LEFT
#define KEY_GESTURE_RIGHT						KEY_RIGHT
#define KEY_GESTURE_O							KEY_O
#define KEY_GESTURE_E							KEY_E
#define KEY_GESTURE_M							KEY_M
#define KEY_GESTURE_L							KEY_L
#define KEY_GESTURE_W							KEY_W
#define KEY_GESTURE_S							KEY_S
#define KEY_GESTURE_V							KEY_V
#define KEY_GESTURE_C							KEY_C
#define KEY_GESTURE_Z							KEY_Z

#define GESTURE_LEFT							0x20
#define GESTURE_RIGHT							0x21
#define GESTURE_UP								0x22
#define GESTURE_DOWN							0x23
#define GESTURE_DOUBLECLICK						0x24
#define GESTURE_O								0x30
#define GESTURE_W								0x31
#define GESTURE_M								0x32
#define GESTURE_E								0x33
#define GESTURE_L								0x44
#define GESTURE_S								0x46
#define GESTURE_V								0x54
#define GESTURE_Z								0x41
#define GESTURE_C								0x34
#define FTS_GESTRUE_POINTS						255
#define FTS_GESTRUE_POINTS_HEADER				8
=======

/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define KEY_GESTURE_U                           KEY_U
#define KEY_GESTURE_UP                          KEY_UP
#define KEY_GESTURE_DOWN                        KEY_DOWN
#define KEY_GESTURE_LEFT                        KEY_LEFT
#define KEY_GESTURE_RIGHT                       KEY_RIGHT
#define KEY_GESTURE_O                           KEY_O
#define KEY_GESTURE_E                           KEY_E
#define KEY_GESTURE_M                           KEY_M
#define KEY_GESTURE_L                           KEY_L
#define KEY_GESTURE_W                           KEY_W
#define KEY_GESTURE_S                           KEY_S
#define KEY_GESTURE_V                           KEY_V
#define KEY_GESTURE_C                           KEY_C
#define KEY_GESTURE_Z                           KEY_Z

#define GESTURE_LEFT                            0x20
#define GESTURE_RIGHT                           0x21
#define GESTURE_UP                              0x22
#define GESTURE_DOWN                            0x23
#define GESTURE_DOUBLECLICK                     0x24
#define GESTURE_O                               0x30
#define GESTURE_W                               0x31
#define GESTURE_M                               0x32
#define GESTURE_E                               0x33
#define GESTURE_L                               0x44
#define GESTURE_S                               0x46
#define GESTURE_V                               0x54
#define GESTURE_Z                               0x41
#define GESTURE_C                               0x34
>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
/*
<<<<<<< HEAD
* header        -   byte0:gesture id
*                   byte1:pointnum
*                   byte2~7:reserved
* coordinate_x  -   All gesture point x coordinate
* coordinate_y  -   All gesture point y coordinate
* mode          -   1:enable gesture function(default)
*               -   0:disable
* active        -   1:enter into gesture(suspend)
*                   0:gesture disable or resume
*/
struct fts_gesture_st {
	u8 header[FTS_GESTRUE_POINTS_HEADER];
	u16 coordinate_x[FTS_GESTRUE_POINTS];
	u16 coordinate_y[FTS_GESTRUE_POINTS];
	u8 mode;		/*host driver enable gesture flag */
	u8 active;		/*gesture actutally work */
=======
* gesture_id    - mean which gesture is recognised
* point_num     - points number of this gesture
* coordinate_x  - All gesture point x coordinate
* coordinate_y  - All gesture point y coordinate
* mode          - gesture enable/disable, need enable by host
*               - 1:enable gesture function(default)  0:disable
* active        - gesture work flag,
*                 always set 1 when suspend, set 0 when resume
*/
struct fts_gesture_st {
	u8 gesture_id;
	u8 point_num;
	u16 coordinate_x[FTS_GESTURE_POINTS_MAX];
	u16 coordinate_y[FTS_GESTURE_POINTS_MAX];
>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_gesture_st fts_gesture_data;
<<<<<<< HEAD
extern struct fts_ts_data *fts_data;
=======
>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
<<<<<<< HEAD
static ssize_t fts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t fts_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_gesture_buf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

/* sysfs gesture node
 *   read example: cat  fts_gesture_mode        ---read gesture mode
 *   write example:echo 01 > fts_gesture_mode   ---write gesture mode to 01
 *
 */
static DEVICE_ATTR(fts_gesture_mode, S_IRUGO | S_IWUSR, fts_gesture_show, fts_gesture_store);
/*
 *   read example: cat fts_gesture_buf        ---read gesture buf
 */
static DEVICE_ATTR(fts_gesture_buf, S_IRUGO | S_IWUSR, fts_gesture_buf_show, fts_gesture_buf_store);
static struct attribute *fts_gesture_mode_attrs[] = {
	&dev_attr_fts_gesture_mode.attr,
	&dev_attr_fts_gesture_buf.attr,
	NULL,
};

static struct attribute_group fts_gesture_group = {
	.attrs = fts_gesture_mode_attrs,
};

/************************************************************************
* Name: fts_gesture_show
*  Brief:
*  Input: device, device attribute, char buf
* Output:
* Return:
***********************************************************************/
static ssize_t fts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	u8 val;
	struct input_dev *input_dev = fts_data->input_dev;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	mutex_lock(&input_dev->mutex);
	fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &val);
	count = snprintf(buf, PAGE_SIZE, "Gesture Mode: %s\n", fts_gesture_data.mode ? "On" : "Off");
	count += snprintf(buf + count, PAGE_SIZE, "Reg(0xD0) = %d\n", val);
	mutex_unlock(&input_dev->mutex);
=======
static ssize_t fts_gesture_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;
	u8 val = 0;
	struct fts_ts_data *ts_data = fts_data;

	mutex_lock(&ts_data->input_dev->mutex);
	fts_read_reg(FTS_REG_GESTURE_EN, &val);
	count = snprintf(buf, PAGE_SIZE, "Gesture Mode:%s\n",
			ts_data->gesture_mode ? "On" : "Off");
	count += snprintf(buf + count, PAGE_SIZE, "Reg(0xD0)=%d\n", val);
	mutex_unlock(&ts_data->input_dev->mutex);
>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d

	return count;
}

<<<<<<< HEAD
/************************************************************************
* Name: fts_gesture_store
*  Brief:
*  Input: device, device attribute, char buf, char count
* Output:
* Return:
***********************************************************************/
static ssize_t fts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_dev = fts_data->input_dev;
	mutex_lock(&input_dev->mutex);
	if (FTS_SYSFS_ECHO_ON(buf)) {
		FTS_INFO("[GESTURE]enable gesture");
		fts_gesture_data.mode = ENABLE;
	} else if (FTS_SYSFS_ECHO_OFF(buf)) {
		FTS_INFO("[GESTURE]disable gesture");
		fts_gesture_data.mode = DISABLE;
	}
	mutex_unlock(&input_dev->mutex);
=======
static ssize_t fts_gesture_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = fts_data;

	mutex_lock(&ts_data->input_dev->mutex);
	if (FTS_SYSFS_ECHO_ON(buf)) {
		FTS_DEBUG("enable gesture");
		ts_data->gesture_mode = true;
	} else if (FTS_SYSFS_ECHO_OFF(buf)) {
		FTS_DEBUG("disable gesture");
		ts_data->gesture_mode = false;
	}
	mutex_unlock(&ts_data->input_dev->mutex);
>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d

	return count;
}

<<<<<<< HEAD
void fts_gesture_enable(bool enable)
{
	struct input_dev *input_dev = fts_data->input_dev;

	mutex_lock(&input_dev->mutex);

	if (enable) {
		FTS_INFO("[GESTURE]enable gesture");
		fts_gesture_data.mode = ENABLE;
	} else {
		FTS_INFO("[GESTURE]disable gesture");
		fts_gesture_data.mode = DISABLE;
	}

	mutex_unlock(&input_dev->mutex);
}

/************************************************************************
* Name: fts_gesture_buf_show
*  Brief:
*  Input: device, device attribute, char buf
* Output:
* Return:
***********************************************************************/
static ssize_t fts_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int i = 0;
	struct input_dev *input_dev = fts_data->input_dev;

	mutex_lock(&input_dev->mutex);
	count = snprintf(buf, PAGE_SIZE, "Gesture ID: 0x%x\n", fts_gesture_data.header[0]);
	count += snprintf(buf + count, PAGE_SIZE, "Gesture PointNum: %d\n", fts_gesture_data.header[1]);
	count += snprintf(buf + count, PAGE_SIZE, "Gesture Point Buf:\n");
	for (i = 0; i < fts_gesture_data.header[1]; i++) {
		count +=
			snprintf(buf + count, PAGE_SIZE, "%3d(%4d,%4d) ", i, fts_gesture_data.coordinate_x[i],
				fts_gesture_data.coordinate_y[i]);
=======
static ssize_t fts_gesture_buf_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;
	int i = 0;
	struct input_dev *input_dev = fts_data->input_dev;
	struct fts_gesture_st *gesture = &fts_gesture_data;

	mutex_lock(&input_dev->mutex);
	count = snprintf(buf, PAGE_SIZE, "Gesture ID:%d\n", gesture->gesture_id);
	count += snprintf(buf + count, PAGE_SIZE, "Gesture PointNum:%d\n",
			gesture->point_num);
	count += snprintf(buf + count, PAGE_SIZE, "Gesture Points Buffer:\n");

	/* save point data,max:6 */
	for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
		count += snprintf(buf + count, PAGE_SIZE, "%3d(%4d,%4d) ", i,
			gesture->coordinate_x[i], gesture->coordinate_y[i]);

>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d
		if ((i + 1) % 4 == 0)
			count += snprintf(buf + count, PAGE_SIZE, "\n");
	}
	count += snprintf(buf + count, PAGE_SIZE, "\n");
	mutex_unlock(&input_dev->mutex);

	return count;
}

<<<<<<< HEAD
/************************************************************************
* Name: fts_gesture_buf_store
*  Brief:
*  Input: device, device attribute, char buf, char count
* Output:
* Return:
***********************************************************************/
static ssize_t fts_gesture_buf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

/*****************************************************************************
*   Name: fts_create_gesture_sysfs
*  Brief:
*  Input:
* Output:
* Return: 0-success or others-error
*****************************************************************************/
int fts_create_gesture_sysfs(struct i2c_client *client)
{
	int ret = 0;

	ret = sysfs_create_group(&client->dev.kobj, &fts_gesture_group);
	if (ret != 0) {
		FTS_ERROR("[GESTURE]fts_gesture_mode_group(sysfs) create failed!");
		sysfs_remove_group(&client->dev.kobj, &fts_gesture_group);
		return ret;
	}
	return 0;
}

/*****************************************************************************
*   Name: fts_gesture_report
*  Brief:
*  Input:
* Output:
* Return:
*****************************************************************************/
=======
static ssize_t fts_gesture_buf_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return -EPERM;
}


/* sysfs gesture node
 *   read example: cat  fts_gesture_mode       ---read gesture mode
 *   write example:echo 1 > fts_gesture_mode   --- write gesture mode to 1
 *
 */
static DEVICE_ATTR(fts_gesture_mode, S_IRUGO | S_IWUSR, fts_gesture_show,
			fts_gesture_store);
/*
 *   read example: cat fts_gesture_buf        --- read gesture buf
 */
static DEVICE_ATTR(fts_gesture_buf, S_IRUGO | S_IWUSR,
			fts_gesture_buf_show, fts_gesture_buf_store);

static struct attribute *fts_gesture_mode_attrs[] = {
	&dev_attr_fts_gesture_mode.attr,
	&dev_attr_fts_gesture_buf.attr,
	NULL,
};

static struct attribute_group fts_gesture_group = {
	.attrs = fts_gesture_mode_attrs,
};

static int fts_create_gesture_sysfs(struct device *dev)
{
	int ret = 0;

	ret = sysfs_create_group(&dev->kobj, &fts_gesture_group);
	if (ret) {
		FTS_ERROR("gesture sys node create fail");
		sysfs_remove_group(&dev->kobj, &fts_gesture_group);
		return ret;
	}

	return 0;
}

>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d
static void fts_gesture_report(struct input_dev *input_dev, int gesture_id)
{
	int gesture;

<<<<<<< HEAD
	FTS_FUNC_ENTER();
	FTS_INFO("fts gesture_id==0x%x ", gesture_id);
=======
	FTS_DEBUG("gesture_id:0x%x", gesture_id);
>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d
	switch (gesture_id) {
	case GESTURE_LEFT:
		gesture = KEY_GESTURE_LEFT;
		break;
<<<<<<< HEAD
	case GESTURE_RIGHT:
		gesture = KEY_GESTURE_RIGHT;
		break;
	case GESTURE_UP:
		gesture = KEY_GESTURE_UP;
		break;
	case GESTURE_DOWN:
		gesture = KEY_GESTURE_DOWN;
		break;
	case GESTURE_DOUBLECLICK:
		gesture = KEY_GESTURE_U;
		break;
	case GESTURE_O:
		gesture = KEY_GESTURE_O;
		break;
	case GESTURE_W:
		gesture = KEY_GESTURE_W;
		break;
	case GESTURE_M:
		gesture = KEY_GESTURE_M;
		break;
	case GESTURE_E:
		gesture = KEY_GESTURE_E;
		break;
	case GESTURE_L:
		gesture = KEY_GESTURE_L;
		break;
	case GESTURE_S:
		gesture = KEY_GESTURE_S;
		break;
	case GESTURE_V:
		gesture = KEY_GESTURE_V;
		break;
	case GESTURE_Z:
		gesture = KEY_GESTURE_Z;
		break;
	case GESTURE_C:
		gesture = KEY_GESTURE_C;
		break;
=======

	case GESTURE_RIGHT:
		gesture = KEY_GESTURE_RIGHT;
		break;

	case GESTURE_UP:
		gesture = KEY_GESTURE_UP;
		break;

	case GESTURE_DOWN:
		gesture = KEY_GESTURE_DOWN;
		break;

	case GESTURE_DOUBLECLICK:
		gesture = KEY_POWER;

		break;

	case GESTURE_O:
		gesture = KEY_GESTURE_O;
		break;

	case GESTURE_W:
		gesture = KEY_GESTURE_W;
		break;

	case GESTURE_M:
		gesture = KEY_GESTURE_M;
		break;

	case GESTURE_E:
		gesture = KEY_GESTURE_E;
		break;

	case GESTURE_L:
		gesture = KEY_GESTURE_L;
		break;

	case GESTURE_S:
		gesture = KEY_GESTURE_S;
		break;

	case GESTURE_V:
		gesture = KEY_GESTURE_V;
		break;

	case GESTURE_Z:
		gesture = KEY_GESTURE_Z;
		break;

	case GESTURE_C:
		gesture = KEY_GESTURE_C;
		break;

>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d
	default:
		gesture = -1;
		break;
	}
<<<<<<< HEAD
=======

>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d
	/* report event key */
	if (gesture != -1) {
		FTS_DEBUG("Gesture Code=%d", gesture);
		input_report_key(input_dev, gesture, 1);
		input_sync(input_dev);
		input_report_key(input_dev, gesture, 0);
		input_sync(input_dev);
	}
<<<<<<< HEAD

	FTS_FUNC_EXIT();
}

/************************************************************************
*   Name: fts_gesture_read_buffer
*  Brief: read data from TP register
*  Input:
* Output:
* Return: fail <0
***********************************************************************/
static int fts_gesture_read_buffer(struct i2c_client *client, u8 *buf, int read_bytes)
{
	int remain_bytes;
	int ret;
	int i;

	if (read_bytes <= I2C_BUFFER_LENGTH_MAXINUM) {
		ret = fts_i2c_read(client, buf, 1, buf, read_bytes);
	} else {
		ret = fts_i2c_read(client, buf, 1, buf, I2C_BUFFER_LENGTH_MAXINUM);
		remain_bytes = read_bytes - I2C_BUFFER_LENGTH_MAXINUM;
		for (i = 1; remain_bytes > 0; i++) {
			if (remain_bytes <= I2C_BUFFER_LENGTH_MAXINUM)
				ret = fts_i2c_read(client, buf, 0, buf + I2C_BUFFER_LENGTH_MAXINUM * i, remain_bytes);
			else
				ret =
					fts_i2c_read(client, buf, 0, buf + I2C_BUFFER_LENGTH_MAXINUM * i,
						 I2C_BUFFER_LENGTH_MAXINUM);
			remain_bytes -= I2C_BUFFER_LENGTH_MAXINUM;
		}
	}

	return ret;
}

/************************************************************************
*   Name: fts_gesture_readdata
*  Brief: read data from TP register
*  Input:
* Output:
* Return: return 0 if succuss, otherwise reture error code
***********************************************************************/
int fts_gesture_readdata(struct fts_ts_data *ts_data)
{
	u8 buf[FTS_GESTRUE_POINTS * 4] = { 0 };
	int ret = 0;
	int i = 0;
	int gestrue_id = 0;
	int read_bytes = 0;
	u8 pointnum;
	u8 state;
	struct i2c_client *client = ts_data->client;
	struct input_dev *input_dev = ts_data->input_dev;

	if (!ts_data->suspended) {
		return -EINVAL;
	}

	ret = fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &state);
	if ((ret < 0) || (state != ENABLE)) {
		FTS_DEBUG("gesture not enable, don't process gesture");
		return -EIO;
	}

	/* init variable before read gesture point */
	memset(fts_gesture_data.header, 0, FTS_GESTRUE_POINTS_HEADER);
	memset(fts_gesture_data.coordinate_x, 0, FTS_GESTRUE_POINTS * sizeof(u16));
	memset(fts_gesture_data.coordinate_y, 0, FTS_GESTRUE_POINTS * sizeof(u16));

	buf[0] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
	ret = fts_i2c_read(client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
	if (ret < 0) {
		FTS_ERROR("[GESTURE]Read gesture header data failed!!");
		FTS_FUNC_EXIT();
		return ret;
	}

	memcpy(fts_gesture_data.header, buf, FTS_GESTRUE_POINTS_HEADER);
	gestrue_id = buf[0];
	pointnum = buf[1];
	read_bytes = ((int)pointnum) * 4 + 2;
	buf[0] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
	FTS_DEBUG("[GESTURE]PointNum=%d", pointnum);
	ret = fts_gesture_read_buffer(client, buf, read_bytes);
	if (ret < 0) {
		FTS_ERROR("[GESTURE]Read gesture touch data failed!!");
		FTS_FUNC_EXIT();
		return ret;
	}

	fts_gesture_report(input_dev, gestrue_id);

	for (i = 0; i < pointnum; i++) {
		fts_gesture_data.coordinate_x[i] =
			(((s16) buf[0 + (4 * i + 2)]) & 0x0F) << 8 | (((s16) buf[1 + (4 * i + 2)]) & 0xFF);
		fts_gesture_data.coordinate_y[i] =
			(((s16) buf[2 + (4 * i + 2)]) & 0x0F) << 8 | (((s16) buf[3 + (4 * i + 2)]) & 0xFF);
	}

	return 0;
}

/*****************************************************************************
*   Name: fts_gesture_recovery
*  Brief: recovery gesture state when reset or power on
*  Input:
* Output:
* Return:
*****************************************************************************/
void fts_gesture_recovery(struct i2c_client *client)
{
	if ((ENABLE == fts_gesture_data.mode) && (ENABLE == fts_gesture_data.active)) {
		FTS_INFO("enter fts_gesture_recovery");
		fts_i2c_write_reg(client, 0xD1, 0xff);
		fts_i2c_write_reg(client, 0xD2, 0xff);
		fts_i2c_write_reg(client, 0xD5, 0xff);
		fts_i2c_write_reg(client, 0xD6, 0xff);
		fts_i2c_write_reg(client, 0xD7, 0xff);
		fts_i2c_write_reg(client, 0xD8, 0xff);
		fts_gesture_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, true);
#ifdef CONFIG_TOUCHSCREEN_FTS_FOD
		fts_fod_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, true);
#endif
	}
}

#ifdef CONFIG_TOUCHSCREEN_FTS_FOD
void fts_fod_recovery(struct i2c_client *client)
{
	FTS_FUNC_ENTER();
	if (fts_data->suspended) {
		FTS_INFO("%s, tp is in suspend mode, write 0xd0 to 1", __func__);
		fts_gesture_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, true);
	}
	fts_fod_reg_write(client, FTS_REG_GESTURE_FOD_ON, true);
}

int fts_fod_reg_write(struct i2c_client *client, u8 mask, bool enable)
{
	int i;
	u8 state;
	u8 reg_value;

	for (i = 0; i < 5; i++) {
		fts_i2c_read_reg(client, FTS_REG_GESTURE_SUPPORT, &reg_value);
		if (enable)
			reg_value |= mask;
		else
			reg_value &= ~mask;
		fts_i2c_write_reg(client, FTS_REG_GESTURE_SUPPORT, reg_value);
		msleep(1);
		fts_i2c_read_reg(client, FTS_REG_GESTURE_SUPPORT, &state);
		if (state == reg_value)
			break;
	}

	if (i >= 5) {
		FTS_ERROR("[GESTURE]Write fod reg failed!\n");
		return -EIO;
	} else {
		FTS_ERROR("[GESTURE]Write fod reg success!\n");
		return 0;
	}
}
#endif

int fts_gesture_reg_write(struct i2c_client *client, u8 mask, bool enable)
{
	int i;
	u8 state;
	u8 reg_value;

	for (i = 0; i < 5; i++) {
		fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &reg_value);
		if (enable)
			reg_value |= mask;
		else
			reg_value &= ~mask;
		fts_i2c_write_reg(client, FTS_REG_GESTURE_EN, reg_value);
		msleep(1);
		fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &state);
		if (state == reg_value)
			break;
	}

	if (i >= 5) {
		FTS_ERROR("[GESTURE]Write gesture reg failed!\n");
		return -EIO;
	} else {
		FTS_ERROR("[GESTURE]Write gesture reg success!\n");
		return 0;
	}
}

/*****************************************************************************
*   Name: fts_gesture_suspend
*  Brief:
*  Input:
* Output:
* Return: return 0 if succuss, otherwise return error code
*****************************************************************************/
int fts_gesture_suspend(struct i2c_client *client)
{
	int ret;

	FTS_INFO("gesture suspend...");
	/* gesture not enable, return immediately */
	if (fts_gesture_data.mode == DISABLE) {
		FTS_INFO("gesture is disabled");
		return -EINVAL;
	}
	ret = fts_gesture_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, true);
	if (ret) {
		FTS_ERROR("[GESTURE]Enter into gesture(suspend) failed!\n");
		fts_gesture_data.active = DISABLE;
		return -EIO;
	}
#ifdef CONFIG_TOUCHSCREEN_FTS_FOD
	ret = fts_fod_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, true);
	if (ret) {
		FTS_ERROR("[GESTURE]Enter into gesture(suspend) failed!\n");
		fts_gesture_data.active = DISABLE;
		return -EIO;
	}
#endif
	fts_gesture_data.active = ENABLE;
	FTS_INFO("[GESTURE]Enter into gesture(suspend) successfully!");
=======
}

/*****************************************************************************
* Name: fts_gesture_readdata
* Brief: Read information about gesture: enable flag/gesture points..., if ges-
*        ture enable, save gesture points' information, and report to OS.
*        It will be called this function every intrrupt when FTS_GESTURE_EN = 1
*
*        gesture data length: 1(enable) + 1(reserve) + 2(header) + 6 * 4
* Input: ts_data - global struct data
*        data    - gesture data buffer if non-flash, else NULL
* Output:
* Return: 0 - read gesture data successfully, the report data is gesture data
*         1 - tp not in suspend/gesture not enable in TP FW
*         -Exx - error
*****************************************************************************/
int fts_gesture_readdata(struct fts_ts_data *ts_data, u8 *data)
{
	int ret = 0;
	int i = 0;
	int index = 0;
	u8 buf[FTS_GESTURE_DATA_LEN] = { 0 };
	struct input_dev *input_dev = ts_data->input_dev;
	struct fts_gesture_st *gesture = &fts_gesture_data;

	if (!ts_data->suspended || !ts_data->gesture_mode) {
		return 1;
	}


	ret = fts_read_reg(FTS_REG_GESTURE_EN, &buf[0]);
	if ((ret < 0) || (buf[0] != ENABLE)) {
		FTS_DEBUG("gesture not enable in fw, don't process gesture");
		return 1;
	}

	buf[2] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
	ret = fts_read(&buf[2], 1, &buf[2], FTS_GESTURE_DATA_LEN - 2);
	if (ret < 0) {
		FTS_ERROR("read gesture header data fail");
		return ret;
	}

	/* init variable before read gesture point */
	memset(gesture->coordinate_x, 0, FTS_GESTURE_POINTS_MAX * sizeof(u16));
	memset(gesture->coordinate_y, 0, FTS_GESTURE_POINTS_MAX * sizeof(u16));
	gesture->gesture_id = buf[2];
	gesture->point_num = buf[3];
	FTS_DEBUG("gesture_id=%d, point_num=%d",
		gesture->gesture_id, gesture->point_num);

	/* save point data,max:6 */
	for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
		index = 4 * i + 4;
		gesture->coordinate_x[i] = (u16)(((buf[0 + index] & 0x0F) << 8)
						+ buf[1 + index]);
		gesture->coordinate_y[i] = (u16)(((buf[2 + index] & 0x0F) << 8)
						+ buf[3 + index]);
	}

	/* report gesture to OS */
	fts_gesture_report(input_dev, gesture->gesture_id);
	return 0;
}

void fts_gesture_recovery(struct fts_ts_data *ts_data)
{
	if (ts_data->gesture_mode && ts_data->suspended) {
		FTS_DEBUG("gesture recovery...");
		fts_write_reg(0xD1, 0xFF);
		fts_write_reg(0xD2, 0xFF);
		fts_write_reg(0xD5, 0xFF);
		fts_write_reg(0xD6, 0xFF);
		fts_write_reg(0xD7, 0xFF);
		fts_write_reg(0xD8, 0xFF);
		fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
	}
}

int fts_gesture_suspend(struct fts_ts_data *ts_data)
{
	int i = 0;
	u8 state = 0xFF;

	FTS_FUNC_ENTER();
	if (enable_irq_wake(ts_data->irq)) {
		FTS_DEBUG("enable_irq_wake(irq:%d) fail", ts_data->irq);
	}

	for (i = 0; i < 5; i++) {
		fts_write_reg(0xD1, 0xFF);
		fts_write_reg(0xD2, 0xFF);
		fts_write_reg(0xD5, 0xFF);
		fts_write_reg(0xD6, 0xFF);
		fts_write_reg(0xD7, 0xFF);
		fts_write_reg(0xD8, 0xFF);
		fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
		msleep(1);
		fts_read_reg(FTS_REG_GESTURE_EN, &state);
		if (state == ENABLE)
			break;
	}

	if (i >= 5)
		FTS_ERROR("make IC enter into gesture(suspend) fail,state:%x", state);
	else
		FTS_INFO("Enter into gesture(suspend) successfully");

>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d
	FTS_FUNC_EXIT();
	return 0;
}

<<<<<<< HEAD
/*****************************************************************************
*   Name: fts_gesture_resume
*  Brief:
*  Input:
* Output:
* Return: return 0 if succuss, otherwise return error code
*****************************************************************************/
int fts_gesture_resume(struct i2c_client *client)
{
	int ret;

	FTS_INFO("gesture resume...");
	/* gesture not enable, return immediately */
	if (fts_gesture_data.mode == DISABLE) {
		FTS_DEBUG("gesture is disabled");
		return -EINVAL;
	}

	if (fts_gesture_data.active == DISABLE) {
		FTS_DEBUG("gesture in suspend is failed, no running fts_gesture_resume");
		return -EINVAL;
	}
	ret = fts_gesture_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, false);
	if (ret) {
		FTS_ERROR("[GESTURE]Resume from gesture failed!\n");
		return -EIO;
	}
#ifdef CONFIG_TOUCHSCREEN_FTS_FOD
	ret = fts_fod_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, false);
	if (ret) {
		FTS_ERROR("[GESTURE]resume from gesture(suspend) failed!\n");
		return -EIO;
	}
#endif

	fts_gesture_data.active = DISABLE;
	msleep(10);
	FTS_INFO("[GESTURE]resume from gesture successfully!");
=======
int fts_gesture_resume(struct fts_ts_data *ts_data)
{
	int i = 0;
	u8 state = 0xFF;

	FTS_FUNC_ENTER();
	if (disable_irq_wake(ts_data->irq)) {
		FTS_DEBUG("disable_irq_wake(irq:%d) fail", ts_data->irq);
	}

	for (i = 0; i < 5; i++) {
		fts_write_reg(FTS_REG_GESTURE_EN, DISABLE);
		msleep(1);
		fts_read_reg(FTS_REG_GESTURE_EN, &state);
		if (state == DISABLE)
			break;
	}

	if (i >= 5)
		FTS_ERROR("make IC exit gesture(resume) fail,state:%x", state);
	else
		FTS_INFO("resume from gesture successfully");

>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d
	FTS_FUNC_EXIT();
	return 0;
}

<<<<<<< HEAD
/*****************************************************************************
*   Name: fts_gesture_init
*  Brief:
*  Input:
* Output:
* Return:
*****************************************************************************/
int fts_gesture_init(struct fts_ts_data *ts_data)
{
	struct i2c_client *client = ts_data->client;
=======
int fts_gesture_init(struct fts_ts_data *ts_data)
{
>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d
	struct input_dev *input_dev = ts_data->input_dev;

	FTS_FUNC_ENTER();
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_U);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);

	__set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
	__set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
	__set_bit(KEY_GESTURE_UP, input_dev->keybit);
	__set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
	__set_bit(KEY_GESTURE_U, input_dev->keybit);
	__set_bit(KEY_GESTURE_O, input_dev->keybit);
	__set_bit(KEY_GESTURE_E, input_dev->keybit);
	__set_bit(KEY_GESTURE_M, input_dev->keybit);
	__set_bit(KEY_GESTURE_W, input_dev->keybit);
	__set_bit(KEY_GESTURE_L, input_dev->keybit);
	__set_bit(KEY_GESTURE_S, input_dev->keybit);
	__set_bit(KEY_GESTURE_V, input_dev->keybit);
	__set_bit(KEY_GESTURE_C, input_dev->keybit);
	__set_bit(KEY_GESTURE_Z, input_dev->keybit);

<<<<<<< HEAD
	fts_create_gesture_sysfs(client);
	fts_gesture_data.mode = DISABLE;
	fts_gesture_data.active = DISABLE;
=======
	fts_create_gesture_sysfs(ts_data->dev);

	memset(&fts_gesture_data, 0, sizeof(struct fts_gesture_st));

#if FTS_GESTURE_EN
	ts_data->gesture_mode = true;
#else
	ts_data->gesture_mode = false;
#endif
>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d

	FTS_FUNC_EXIT();
	return 0;
}

<<<<<<< HEAD
/************************************************************************
*   Name: fts_gesture_exit
*  Brief: call when driver removed
*  Input:
* Output:
* Return:
***********************************************************************/
int fts_gesture_exit(struct i2c_client *client)
{
	FTS_FUNC_ENTER();
	sysfs_remove_group(&client->dev.kobj, &fts_gesture_group);
	FTS_FUNC_EXIT();
	return 0;
}
#endif
=======
int fts_gesture_exit(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();
	sysfs_remove_group(&ts_data->dev->kobj, &fts_gesture_group);
	FTS_FUNC_EXIT();
	return 0;
}
>>>>>>> 42446a01b99d3dc7629a504d144b9e6bc438280d
