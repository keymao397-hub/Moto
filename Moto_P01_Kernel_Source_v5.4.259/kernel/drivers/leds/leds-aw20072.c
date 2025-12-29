// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Amlogic, Inc. All rights reserved.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>

/************* reg page0 ***************/
#define AW20072_REG_IDR           0x00
#define AW20072_REG_SLPCR         0x01
#define AW20072_REG_RSTR          0x02
#define AW20072_REG_GCCR          0x03
#define AW20072_REG_FCD           0x04
#define AW20072_REG_CLKSYS        0x05
#define AW20072_REG_FLTCFG1       0x09
#define AW20072_REG_FLTCFG2       0x0A
#define AW20072_REG_ISRFLT        0x0B
#define AW20072_REG_LEDON0        0x31
#define AW20072_REG_LEDON1        0x32
#define AW20072_REG_LEDON2        0x33
#define AW20072_REG_LEDON3        0x34
#define AW20072_REG_LEDON4        0x35
#define AW20072_REG_LEDON5        0x36
#define AW20072_REG_LEDON6        0x37
#define AW20072_REG_LEDON7        0x38
#define AW20072_REG_LEDON8        0x39
#define AW20072_REG_LEDON9        0x3A
#define AW20072_REG_LEDON10       0x3B
#define AW20072_REG_LEDON11       0x3C
#define AW20072_REG_PATCR         0x43
#define AW20072_REG_FADEH0        0x44
#define AW20072_REG_FADEH1        0x45
#define AW20072_REG_FADEH2        0x46
#define AW20072_REG_FADEL0        0x47
#define AW20072_REG_FADEL1        0x48
#define AW20072_REG_FADEL2        0x49
#define AW20072_REG_PAT0T0        0x4A
#define AW20072_REG_PAT0T1        0x4B
#define AW20072_REG_PAT0T2        0x4C
#define AW20072_REG_PAT0T3        0x4D
#define AW20072_REG_PAT1T0        0x4E
#define AW20072_REG_PAT1T1        0x4F
#define AW20072_REG_PAT1T2        0x50
#define AW20072_REG_PAT1T3        0x51
#define AW20072_REG_PAT2T0        0x52
#define AW20072_REG_PAT2T1        0x53
#define AW20072_REG_PAT2T2        0x54
#define AW20072_REG_PAT2T3        0x55
#define AW20072_REG_PAT0CFG       0x56
#define AW20072_REG_PAT1CFG       0x57
#define AW20072_REG_PAT2CFG       0x58
#define AW20072_REG_PATGO         0x59
#define AW20072_REG_DIS_SIZE      0x80
#define AW20072_REG_PAGE          0xF0
/************* reg pag1(write only) ***************/
#define AW20072_REG_DIM_START     0x00
#define AW20072_REG_DIM_END       0x47
/************* reg pag2(write only) ***************/
#define AW20072_REG_FADE_START     0x00
#define AW20072_REG_FADE_END       0x47
/************* reg pag3(write only) ***************/
#define AW20072_REG_PAT_START     0x00
#define AW20072_REG_PAT_END       0x47
/************* reg pag4(write only) ***************/
#define AW20072_REG_DIM_FADE_START     0x00
#define AW20072_REG_DIM_FADE_END       0x8F
/************* reg pag5(write only) ***************/
#define AW20072_REG_DIM_FADE_PAT_START     0x00
#define AW20072_REG_DIM_FADE_PAT_END       0x8F
/************* reg end **************************/
#define AW20072_REG_MAX                 (0xFF)

//reg bits
#define AW20072_REG_IMAX                AW20072_REG_GCCR
#define AW20072_REG_IMAX_MASK           0xF0
#define AW20072_REG_IMAX_SHIFT          4
//
#define REG_RD_ACCESS                   BIT(0)
#define REG_WR_ACCESS                   BIT(1)
#define AW20072_CHIP_ID                 0x18
#define AW20072_C_IO_MIN                1
#define AW20072_C_IO_MAX                6
#define AW20072_R_IO_MIN                1
#define AW20072_R_IO_MAX                12
//default config
#define DEFAULT_LED_CURRENT_LEVEL       0
#define LED_CURRENT_LEVEL_TOTAL         16

enum reg_page {
	PAGE0 = 0xC0,
	PAGE1 = 0xC1,
	PAGE2 = 0xC2,
	PAGE3 = 0xC3,
	PAGE4 = 0xC4,
	PAGE5 = 0xC5,
};

enum led_drive_current {
	IMAX_10MA = 0,
	IMAX_20MA,
	IMAX_30MA,
	IMAX_40MA,
	IMAX_60MA,
	IMAX_80MA,
	IMAX_120MA,
	IMAX_160MA,
	IMAX_3_3MA,
	IMAX_6_7MA,
	IMAX_10_1MA,
	IMAX_13_3MA,
	IMAX_20_1MA,
	IMAX_26_7MA,
	IMAX_40_1MA,
	IMAX_53_3MA
};

struct aw20072_led_io {
	u32 c_io;
	u32 r_io_red;
	u32 r_io_green;
	u32 r_io_blue;
};

struct aw20072_rgb_colors {
	u32 red;
	u32 green;
	u32 blue;
};

struct aw20072_drv_data {
	u8 led_nums;
	int irq;
	struct i2c_client *i2c;
	struct device *dev;
	struct led_classdev cdev;
	int interrupt_gpio;
	int enable_gpio;
	struct aw20072_led_io *leds;
	struct aw20072_rgb_colors *colors;
	int led_current_level;
	int soft_onoff;
	int hw_onoff;
	struct mutex lock;
};

static struct aw20072_drv_data *aw20072_leds_data = NULL;
static int aw20072_driver_curr[LED_CURRENT_LEVEL_TOTAL] = {
	IMAX_3_3MA,
	IMAX_6_7MA,
	IMAX_10MA,
	IMAX_10_1MA,
	IMAX_13_3MA,
	IMAX_20MA,
	IMAX_20_1MA,
	IMAX_26_7MA,
	IMAX_30MA,
	IMAX_40MA,
	IMAX_40_1MA,
	IMAX_53_3MA,
	IMAX_60MA,
	IMAX_80MA,
	IMAX_120MA,
	IMAX_160MA
};
static u8 access_reg_table[AW20072_REG_MAX] = {
[AW20072_REG_IDR] = REG_RD_ACCESS,
[AW20072_REG_SLPCR] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_RSTR] = REG_WR_ACCESS,
[AW20072_REG_GCCR] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_FCD] = REG_WR_ACCESS,
[AW20072_REG_CLKSYS] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_FLTCFG1] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_FLTCFG2] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_ISRFLT] = REG_RD_ACCESS,
[AW20072_REG_LEDON0] = REG_WR_ACCESS,
[AW20072_REG_LEDON1] = REG_WR_ACCESS,
[AW20072_REG_LEDON2] = REG_WR_ACCESS,
[AW20072_REG_LEDON3] = REG_WR_ACCESS,
[AW20072_REG_LEDON4] = REG_WR_ACCESS,
[AW20072_REG_LEDON5] = REG_WR_ACCESS,
[AW20072_REG_LEDON6] = REG_WR_ACCESS,
[AW20072_REG_LEDON7] = REG_WR_ACCESS,
[AW20072_REG_LEDON8] = REG_WR_ACCESS,
[AW20072_REG_LEDON9] = REG_WR_ACCESS,
[AW20072_REG_LEDON10] = REG_WR_ACCESS,
[AW20072_REG_LEDON11] = REG_WR_ACCESS,
[AW20072_REG_PATCR] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_FADEH0] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_FADEH1] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_FADEH2] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_FADEL0] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_FADEL1] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_FADEL2] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT0T0] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT0T1] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT0T2] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT0T3] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT1T0] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT1T1] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT1T2] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT1T3] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT2T0] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT2T1] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT2T2] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT2T3] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT0CFG] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT1CFG] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAT2CFG] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PATGO] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_DIS_SIZE] = REG_RD_ACCESS | REG_WR_ACCESS,
[AW20072_REG_PAGE] = REG_RD_ACCESS | REG_WR_ACCESS,
};

static int aw20072_reg_init(struct aw20072_drv_data *ddata);

static int aw20072_i2c_read(struct aw20072_drv_data *ddata, u8 reg_addr, u8 *reg_data)
{
    uint8_t data;
	int count = 0;
	int ret = -1;
    struct i2c_msg msgs[] = {
        [0] = {
            .addr = ddata->i2c->addr,
            .flags = 0,
            .len = sizeof(reg_addr),
            .buf = &reg_addr,
        },
 
        [1] = {
            .addr = ddata->i2c->addr,
            .flags = 1,
            .len = sizeof(data),
            .buf = &data,
        },
    };

	while ((ret < 0) && (count < 5))
	{
		ret =  i2c_transfer(ddata->i2c->adapter, msgs, 2);
		count++;
		if (ret < 0) {
			msleep(10);
			dev_err(ddata->dev, "%s: i2c_read retry %d\n", __func__, count);
		}
	}

	if(ret < 0){
        dev_err(ddata->dev, "%s: i2c_read error=%d\n", __func__, ret);
        return ret;
    }
	*reg_data = data;

    return 0;
}

static int aw20072_i2c_write(struct aw20072_drv_data *ddata, u8 reg_addr, u8 reg_data)
{
    uint8_t buff[64];
	int ret = -1;
	int count = 0;
	struct i2c_msg msgs = {
		.addr = ddata->i2c->addr,
		.flags = 0,			   //write
		.len = 2,        //addr+data
		.buf = buff,
	};
	
    buff[0] = reg_addr;
    memcpy(&buff[1], &reg_data, 1);

	while ((ret < 0) && (count < 5))
	{
		ret = i2c_transfer(ddata->i2c->adapter, &msgs, 1);
		count++;
		if (ret < 0) {
			msleep(10);
			dev_err(ddata->dev, "%s: i2c_write retry %d\n", __func__, count);
		}
	}
	if (ret < 0) {
		dev_err(ddata->dev, "%s: i2c_write error=%d\n", __func__, ret);
		return ret;
	}

    return 0;
}

static int aw20072_i2c_write_bits(struct aw20072_drv_data *ddata, u8 reg_addr,
		u8 mask, u8 shift, u8 reg_data)
{
	u8 reg_value;
	int ret;

	ret = aw20072_i2c_read(ddata, reg_addr, &reg_value);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: read reg(0x%x) mask(0x%x) shift(0x%x) bits fail\n",
				__func__, reg_addr, mask, shift);
		return -1;
	}
	reg_data = (reg_value & ~mask) | ((reg_data & (mask >> shift)) << shift);
	ret = aw20072_i2c_write(ddata, reg_addr, reg_data);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: write reg(0x%x) mask(0x%x) shift(0x%x) bits fail\n",
				__func__, reg_addr, mask, shift);
		return -1;
	}

	return ret;
}

static int aw20072_set_imax(struct aw20072_drv_data *ddata, u8 level)
{
	int ret;

	if (level > LED_CURRENT_LEVEL_TOTAL - 1)
		level = LED_CURRENT_LEVEL_TOTAL - 1;

	ddata->led_current_level = level;

	ret = aw20072_i2c_write_bits(ddata,
								AW20072_REG_IMAX,
								AW20072_REG_IMAX_MASK,
								AW20072_REG_IMAX_SHIFT,
								aw20072_driver_curr[level]);
	if (ret < 0) {
		dev_err(ddata->dev, "set imax fail\n");
		return -1;
	}

	return 0;
}

static int aw20072_get_chip_id(struct aw20072_drv_data *ddata, u8 *id)
{
	u8 reg_data = 0;
	int ret;

	ret = aw20072_i2c_read(ddata, AW20072_REG_IDR, &reg_data);
	if (ret < 0) {
		dev_err(ddata->dev, "get chip id fail\n");
		return -1;
	}

	*id = reg_data;
	return 0;
}

static int aw20072_get_int_status(struct aw20072_drv_data *ddata, u8 *status)
{
	u8 reg_data = 0;
	int ret;

	ret = aw20072_i2c_read(ddata, AW20072_REG_ISRFLT, &reg_data);
	if (ret < 0) {
		dev_err(ddata->dev, "get interrupt status fail\n");
		return -1;
	}

	*status = reg_data;
	return 0;
}

static int aw20072_switch_active_mode(struct aw20072_drv_data *ddata)
{
	u8 reg_data = 0x0;
	int ret;

	ret = aw20072_i2c_write(ddata, AW20072_REG_SLPCR, reg_data);
	if (ret < 0) {
		dev_err(ddata->dev, "switch ative mode fail\n");
		return -1;
	}

	return 0;
}

static int aw20072_switch_page(struct aw20072_drv_data *ddata, u8 page)
{
	u8 reg_data = 0x0;
	int ret;

	ret = aw20072_i2c_read(ddata, AW20072_REG_PAGE, &reg_data);
	if (ret < 0) {
		dev_err(ddata->dev, "read reg page fail\n");
		return -1;
	}

	if ((page & 0x03) == reg_data)
		return 0;
	
	ret = aw20072_i2c_write(ddata, AW20072_REG_PAGE, page);
	if (ret < 0) {
		dev_err(ddata->dev, "write reg page fail\n");
		return -1;
	}

	return 0;
}

static int aw20072_set_display_size(struct aw20072_drv_data *ddata)
{
	u8 c_io = 1;
	u8 i = 0;
	int ret = 0;

	for (i = 0; i < ddata->led_nums; i++) {
		if (ddata->leds[i].c_io > c_io)
			c_io = ddata->leds[i].c_io;
	}
	ret = aw20072_i2c_write(ddata, AW20072_REG_DIS_SIZE, c_io - 1);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set display size fail\n", __func__);
		return -1;
	}

	return 0;
}

static int aw20072_set_fade(struct aw20072_drv_data *ddata, \
		u8 *id_buff, u8 id_buff_len, u8 *data, u8 data_len)
{
	int ret = 0;
	u8 i = 0;

	if (id_buff_len != data_len) {
		dev_err(ddata->dev, "%s: param is error\n", __func__);
		return -1;
	}

	for (i = 0; i < id_buff_len; i++) {
		if (id_buff[i] + AW20072_REG_FADE_START > AW20072_REG_FADE_END) {
			dev_err(ddata->dev, "%s: id (%u) is error\n", __func__, id_buff[i]);
			return -1;
		}
		id_buff[i] = id_buff[i] + AW20072_REG_FADE_START;
	}

	//switch page2
	ret = aw20072_switch_page(ddata, PAGE2);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: switch page2 fail\n", __func__);
		return -1;
	}

	for (i = 0; i < id_buff_len; i++) {
		ret = aw20072_i2c_write(ddata, id_buff[i], data[i]);
		if (ret < 0) {
			dev_err(ddata->dev, "%s: set fade (0x%x) fail\n", __func__, id_buff[i]);
			//switch page0
			ret = aw20072_switch_page(ddata, PAGE0);
			if (ret < 0) {
				dev_err(ddata->dev, "%s: switch page0 fail\n", __func__);
				return -1;
			}
			return -1;
		}
	}

	//switch page0
	ret = aw20072_switch_page(ddata, PAGE0);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: switch page0 fail\n", __func__);
		return -1;
	}

	return 0;
}

static int aw20072_set_dim(struct aw20072_drv_data *ddata, \
		u8 *id_buff, u8 id_buff_len, u8 *data, u8 data_len)
{
	int ret = 0;
	u8 i = 0;

	if (id_buff_len != data_len) {
		dev_err(ddata->dev, "%s: param is error\n", __func__);
		return -1;
	}

	for (i = 0; i < id_buff_len; i++) {
		if (id_buff[i] + AW20072_REG_DIM_START > AW20072_REG_DIM_END) {
			dev_err(ddata->dev, "%s: id (%u) is error\n", __func__, id_buff[i]);
			return -1;
		}
		id_buff[i] = id_buff[i] + AW20072_REG_DIM_START;
	}

	//switch page1
	ret = aw20072_switch_page(ddata, PAGE1);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: switch page2 fail\n", __func__);
		return -1;
	}

	for (i = 0; i < id_buff_len; i++) {
		ret = aw20072_i2c_write(ddata, id_buff[i], data[i]);
		if (ret < 0) {
			dev_err(ddata->dev, "%s: set dim (0x%x) fail\n", __func__, id_buff[i]);
			//switch page0
			ret = aw20072_switch_page(ddata, PAGE0);
			if (ret < 0) {
				dev_err(ddata->dev, "%s: switch page0 fail\n", __func__);
				return -1;
			}
			return -1;
		}
	}

	//switch page0
	ret = aw20072_switch_page(ddata, PAGE0);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: switch page0 fail\n", __func__);
		return -1;
	}

	return 0;
}

static int aw20072_hw_onoff(struct aw20072_drv_data *ddata, u8 onoff)
{
	if (onoff) {
		gpio_direction_output(ddata->enable_gpio, 1);
		ddata->hw_onoff = 1;
		aw20072_reg_init(ddata);
	} else {
		gpio_direction_output(ddata->enable_gpio, 0);
		ddata->hw_onoff = 0;
	}

	return 0;
}

static u8 aw20072_get_led_id(u8 c_io, u8 r_io)
{
	return (c_io - 1) * AW20072_R_IO_MAX + r_io - 1;
}

static int aw20072_write_single_led(struct aw20072_drv_data *ddata, u8 led_index, u32 color)
{
	int ret;
	u8 id[3];
	u8 data[3];

	if (led_index >= ddata->led_nums) {
		dev_err(ddata->dev, "led index is invalid\n");
		return -1;
	}

	ddata->colors[led_index].red = (color >> 16) & 0xFF;
	ddata->colors[led_index].green = (color >> 8) & 0xFF;
	ddata->colors[led_index].blue = color & 0xFF;

	id[0] = aw20072_get_led_id(ddata->leds[led_index].c_io,
					ddata->leds[led_index].r_io_red);
	data[0] = ddata->colors[led_index].red;

	id[1] = aw20072_get_led_id(ddata->leds[led_index].c_io,
					ddata->leds[led_index].r_io_green);
	data[1] = ddata->colors[led_index].green;

	id[2] = aw20072_get_led_id(ddata->leds[led_index].c_io,
					ddata->leds[led_index].r_io_blue);
	data[2] = ddata->colors[led_index].blue;
	ret = aw20072_set_fade(ddata, id, 3, data, 3);
	if (ret < 0)
		return -1;

	return 0;
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct aw20072_drv_data *ddata = aw20072_leds_data;
	unsigned int databuf[2] = {0, 0};

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw20072_i2c_write(ddata, (unsigned char)databuf[0], (unsigned char)databuf[1]);

	return count;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct aw20072_drv_data *ddata = aw20072_leds_data;
	ssize_t len = 0;
	u8 i = 0;
	u8 reg_val = 0;

	for (i = 0; i < AW20072_REG_MAX; i++) {
		if (!(access_reg_table[i] & REG_RD_ACCESS))
			continue;
		aw20072_i2c_read(ddata, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%02x\n", i, reg_val);
	}

	return len;
}

static ssize_t led_nums_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct aw20072_drv_data *ddata = aw20072_leds_data;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", ddata->led_nums);

	return len;
}

static ssize_t single_led_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	u32 ledid, color;
	int ret;
	struct aw20072_drv_data *ddata = aw20072_leds_data;

	mutex_lock(&ddata->lock);
	if (ddata->soft_onoff == 0 || ddata->hw_onoff == 0) {
		mutex_unlock(&ddata->lock);
		return count;
	}

	ret = sscanf(buf, "%d %x", &ledid, &color);
	if (ret != 2) {
		dev_err(ddata->dev, "set single led fail!\n");
		mutex_unlock(&ddata->lock);
		return count;
	}
	aw20072_write_single_led(ddata, ledid, color);
	mutex_unlock(&ddata->lock);

	return count;
}

static ssize_t all_led_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	u32 i, color;
	int ret;
	struct aw20072_drv_data *ddata = aw20072_leds_data;

	mutex_lock(&ddata->lock);
	if (ddata->soft_onoff == 0 || ddata->hw_onoff == 0) {
		mutex_unlock(&ddata->lock);
		return count;
	}

	ret = sscanf(buf, "%x", &color);
	if (ret != 1) {
		dev_err(ddata->dev, "set all led fail!\n");
		mutex_unlock(&ddata->lock);
		return count;
	}
	for (i = 0; i < ddata->led_nums; i++)
		aw20072_write_single_led(ddata, i, color);

	mutex_unlock(&ddata->lock);
	return count;
}

static ssize_t hw_onoff_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	u32 onoff;
	int ret;
	struct aw20072_drv_data *ddata = aw20072_leds_data;

	ret = sscanf(buf, "%d", &onoff);
	if (ret != 1) {
		dev_err(ddata->dev, "hw onoff ctrl fail\n");
		return count;
	}
	mutex_lock(&ddata->lock);
	aw20072_hw_onoff(ddata, onoff);
	mutex_unlock(&ddata->lock);

	return count;
}

static ssize_t hw_onoff_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct aw20072_drv_data *ddata = aw20072_leds_data;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", ddata->hw_onoff);

	return len;
}

static ssize_t current_level_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct aw20072_drv_data *ddata = aw20072_leds_data;
	int level = 0;

	if (sscanf(buf, "%du", &level) == 1)
		aw20072_set_imax(ddata, level);

	return count;
}

static ssize_t current_level_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct aw20072_drv_data *ddata = aw20072_leds_data;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", ddata->led_current_level);

	return len;
}

static ssize_t sw_onoff_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct aw20072_drv_data *ddata = aw20072_leds_data;
	int onoff = 0;
	u32 i = 0;

	mutex_lock(&ddata->lock);
	if (sscanf(buf, "%du", &onoff) == 1) {
		if (onoff == 1) {
			ddata->soft_onoff = 1;
		} else if (onoff == 0) {
			ddata->soft_onoff = 0;
			for (i = 0; i < ddata->led_nums; i++)
				aw20072_write_single_led(ddata, i, 0x0);
		}
	}
	mutex_unlock(&ddata->lock);

	return count;
}

static ssize_t sw_onoff_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct aw20072_drv_data *ddata = aw20072_leds_data;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", ddata->soft_onoff);

	return len;
}

static DEVICE_ATTR_RW(reg);
static DEVICE_ATTR_RO(led_nums);
static DEVICE_ATTR_WO(single_led);
static DEVICE_ATTR_WO(all_led);
static DEVICE_ATTR_RW(hw_onoff);
static DEVICE_ATTR_RW(current_level);
static DEVICE_ATTR_RW(sw_onoff);

static struct attribute *aw20072_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_led_nums.attr,
	&dev_attr_single_led.attr,
	&dev_attr_all_led.attr,
	&dev_attr_hw_onoff.attr,
	&dev_attr_current_level.attr,
	&dev_attr_sw_onoff.attr,
	NULL
};

static struct attribute_group aw20072_attribute_group = {
	.attrs = aw20072_attributes
};

static irqreturn_t aw20072_irq_thread(int irq, void *data)
{
	struct aw20072_drv_data *ddata = data;
	int ret;
	u8 status = 0;

	ret = aw20072_get_int_status(ddata, &status);
	if (ret < 0)
		return IRQ_HANDLED;

	printk(KERN_INFO "%s: get interrupt status 0x%x\n", __func__, status);

	return IRQ_HANDLED;
}

static int aw20072_interrupt_gpio_init(struct aw20072_drv_data *ddata)
{
	int ret = 0;

	ddata->interrupt_gpio = of_get_named_gpio(ddata->dev->of_node, "interrupt-gpio", 0);
	if (ddata->interrupt_gpio < 0) {
		dev_err(ddata->dev, "%s: get interrupt gpio fail\n", __func__);
		return -1;
	}

	ret = gpio_request(ddata->interrupt_gpio, "interrupt-gpio");
	if (ret < 0) {
		dev_err(ddata->dev, "%s: gpio_request fail\n", __func__);
		return ret;
	}

	ret = gpio_direction_input(ddata->interrupt_gpio);
    if (ret < 0) {
        dev_err(ddata->dev, "%s: set interrupt gpio input fail\n", __func__);
        return ret;
    }

	ddata->irq = gpio_to_irq(ddata->interrupt_gpio);
	ret = devm_request_threaded_irq(ddata->dev, ddata->irq, NULL,
						aw20072_irq_thread, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"aw20072-leds-irq", ddata);
	if (ret < 0) {
		dev_err(ddata->dev, "request thread irq failed, ret: %d\n", ret);
		return ret;
	}

	return 0;
}

static int aw20072_enable_gpio_init(struct aw20072_drv_data *ddata)
{
	int ret;

	ddata->enable_gpio = of_get_named_gpio(ddata->dev->of_node, "enable-gpio", 0);
	if (ddata->enable_gpio < 0) {
		dev_err(ddata->dev, "%s: of_get_named_gpio fail\n", __func__);
		return -1;
	}
	ret = gpio_request(ddata->enable_gpio, "enable-gpio");
	if (ret < 0) {
		dev_err(ddata->dev, "%s: gpio_request fail\n", __func__);
		return ret;
	}
	aw20072_hw_onoff(ddata, 1);
	ddata->hw_onoff = 1;

	return 0;
}

static int aw20072_ledon_init(struct aw20072_drv_data *ddata)
{
	u8 ledon_reg[AW20072_REG_LEDON11 - AW20072_REG_LEDON0 + 1] = {0x0};
	u8 i = 0;
	u8 led_id = 0;
	int ret = 0;

	for (i = 0; i < ddata->led_nums; i++) {
		led_id = aw20072_get_led_id(ddata->leds[i].c_io, ddata->leds[i].r_io_red);
		ledon_reg[led_id / 6] |= 0x01 << (led_id % 6);

		led_id = aw20072_get_led_id(ddata->leds[i].c_io, ddata->leds[i].r_io_green);
		ledon_reg[led_id / 6] |= 0x01 << (led_id % 6);

		led_id = aw20072_get_led_id(ddata->leds[i].c_io, ddata->leds[i].r_io_blue);
		ledon_reg[led_id / 6] |= 0x01 << (led_id % 6);
	}

	for (i = AW20072_REG_LEDON0; i <= AW20072_REG_LEDON11; i++) {
		printk(KERN_INFO "write LEDON reg(0x%x)=0x%x ", i, ledon_reg[i - AW20072_REG_LEDON0]);
		ret = aw20072_i2c_write(ddata, i, ledon_reg[i - AW20072_REG_LEDON0]);
		if (ret < 0) {
			dev_err(ddata->dev, "%s: set LEDON reg (0x%x) fail\n", __func__, i);
			return -1;
		}
	}

	return 0;
}

static int aw20072_poweron_light_effect(struct aw20072_drv_data *ddata)
{
	int i = 0;
	int ret;
	int breathLevel = 9;
	// color code for black unit
	u32 color1 = 0xb4b4ff;
	u32 color2 = 0x96c832;

	for(i = 10; i < 22; i++)
    {
		aw20072_write_single_led(ddata, i, color2);
		if(19-i >= 0)
			aw20072_write_single_led(ddata, 19-i, color2);
		else
			aw20072_write_single_led(ddata, 43-i, color2);
        if(i-4 >= 10)
        {
			aw20072_write_single_led(ddata, i-4, color1);
			aw20072_write_single_led(ddata, 23-i, color1);
        }
        msleep(50);
    }
	msleep(1000);

    do
    {
        ret = aw20072_set_imax(ddata, breathLevel);
		if (ret < 0)
			return -1;
        msleep(50);
    }while(breathLevel--);
    
	for (i = 0; i < 24; i++)
		aw20072_write_single_led(ddata, i, 0);

	msleep(50);

	breathLevel = 9;
	ret = aw20072_set_imax(ddata, breathLevel);
	if (ret < 0)
		return -1;

	ret = aw20072_i2c_write(ddata, AW20072_REG_PATCR, 0x01);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set PATCR(0x%x) fail\n", __func__, i);
		return -1;
	}

	ret = aw20072_i2c_write(ddata, AW20072_REG_FADEH0, 0xff);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set FADEH0(0x%x) fail\n", __func__, i);
		return -1;
	}

	ret = aw20072_i2c_write(ddata, AW20072_REG_PAT0T0, 0x60);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set PAT0T0(0x%x) fail\n", __func__, i);
		return -1;
	}

	ret = aw20072_i2c_write(ddata, AW20072_REG_PAT0T1, 0x60);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set PAT0T1(0x%x) fail\n", __func__, i);
		return -1;
	}

	ret = aw20072_i2c_write(ddata, AW20072_REG_PAT0CFG, 0x01);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set PAT0CFG(0x%x) fail\n", __func__, i);
		return -1;
	}

	ret = aw20072_switch_page(ddata, PAGE3);
	if (ret < 0)
		return -1;

	for (i = AW20072_REG_PAT_START; i <= AW20072_REG_PAT_END; i++) {
		ret = aw20072_i2c_write(ddata, i, 0x01);
		if (ret < 0) {
			dev_err(ddata->dev, "%s: set PAT(0x%x) fail\n", __func__, i);
			return -1;
		}
	}

	ret = aw20072_switch_page(ddata, PAGE0);
	if (ret < 0)
		return -1;

	ret = aw20072_i2c_write(ddata, AW20072_REG_PATGO, 0x01);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set PATGO(0x%x) fail\n", __func__, i);
		return -1;
	}

	return 0;
}

static int aw20072_reg_init(struct aw20072_drv_data *ddata)
{
	u8 chip_id = 0;
	u8 i = 0;
	u8 id[AW20072_REG_DIM_END + 1];
	u8 data[AW20072_REG_DIM_END + 1];
	int ret;
	int breathLevel = 9;

	// To Reset all registers afetr a soft reboot
	ret = aw20072_i2c_write(ddata, AW20072_REG_RSTR, 0x01);
	if (ret < 0) {
	 dev_err(ddata->dev, "%s: set RSTR(0x01) fail\n", __func__);
	 return -1;
	}

	//active mode
	ret = aw20072_switch_active_mode(ddata);
	if (ret < 0)
		return -1;

	msleep(5);
	ret = aw20072_get_chip_id(ddata, &chip_id);
	if (ret < 0)
		return -1;

	if (chip_id != AW20072_CHIP_ID) {
		dev_err(ddata->dev, "%s: chip id (0x%x) not match\n", __func__, chip_id);
		return -1;
	}

	ret = aw20072_set_imax(ddata, breathLevel);
	if (ret < 0)
		return -1;

	ret = aw20072_set_display_size(ddata);
	if (ret < 0)
		return -1;

	//set DIM
	for (i = 0; i <= AW20072_REG_DIM_END; i++) {
		id[i] = i;
		data[i] = 0x3F;
	}
	ret = aw20072_set_dim(ddata, id, AW20072_REG_DIM_END + 1, data, AW20072_REG_DIM_END + 1);
	if (ret < 0)
		return -1;

	//set fade
	for (i = 0; i <= AW20072_REG_DIM_END; i++) {
		id[i] = i;
		data[i] = 0x0;
	}
	ret = aw20072_set_fade(ddata, id, AW20072_REG_DIM_END + 1, data, AW20072_REG_DIM_END + 1);
	if (ret < 0)
		return -1;

	//turn on led
	ret = aw20072_ledon_init(ddata);
	if (ret < 0)
		return -1;

    return 0;
}

static int aw20072_led_parse_dt(struct aw20072_drv_data *ddata)
{
	struct device_node *temp;
	u32 colors[3];
	u8 i = 0;
	int ret;

	for_each_child_of_node(ddata->dev->of_node, temp) {
		ret = of_property_read_u32_array(temp, "default_colors",
						colors, ARRAY_SIZE(colors));
		if (ret < 0) {
			dev_err(ddata->dev,
				"read default colors fail, ret = %d\n", ret);
			return -1;
		}

		ret = of_property_read_u32(temp, "c_io", &ddata->leds[i].c_io);
		if (ret < 0) {
			dev_err(ddata->dev,
				"read c_io fail, ret = %d\n", ret);
			return -1;
		}
		if (ddata->leds[i].c_io < AW20072_C_IO_MIN ||
			ddata->leds[i].c_io > AW20072_C_IO_MAX) {
			dev_err(ddata->dev,
				"c_io = %d, is invalid\n", ddata->leds[i].c_io);
			return -1;
		}

		ret = of_property_read_u32(temp, "r_io_red", &ddata->leds[i].r_io_red);
		if (ret < 0) {
			dev_err(ddata->dev,
				"read r_io_red fail, ret = %d\n", ret);
			return -1;
		}
		if (ddata->leds[i].r_io_red < AW20072_R_IO_MIN ||
			ddata->leds[i].r_io_red > AW20072_R_IO_MAX) {
			dev_err(ddata->dev,
				"r_io_red = %d, is invalid\n", ddata->leds[i].r_io_red);
			return -1;
		}

		ret = of_property_read_u32(temp, "r_io_green", &ddata->leds[i].r_io_green);
		if (ret < 0) {
			dev_err(ddata->dev,
				"read r_io_green fail, ret = %d\n", ret);
			return -1;
		}
		if (ddata->leds[i].r_io_green < AW20072_R_IO_MIN ||
			ddata->leds[i].r_io_green > AW20072_R_IO_MAX) {
			dev_err(ddata->dev,
				"r_io_green = %d, is invalid\n", ddata->leds[i].r_io_green);
			return -1;
		}

		ret = of_property_read_u32(temp, "r_io_blue", &ddata->leds[i].r_io_blue);
		if (ret < 0) {
			dev_err(ddata->dev,
				"read r_io_blue fail, ret = %d\n", ret);
			return -1;
		}
		if (ddata->leds[i].r_io_blue < AW20072_R_IO_MIN ||
			ddata->leds[i].r_io_blue > AW20072_R_IO_MAX) {
			dev_err(ddata->dev,
				"r_io_blue = %d, is invalid\n", ddata->leds[i].r_io_blue);
			return -1;
		}

		ddata->colors[i].red = colors[0];
		ddata->colors[i].green = colors[1];
		ddata->colors[i].blue = colors[2];
		i++;
	}

	return 0;
}

static int aw20072_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct aw20072_drv_data *ddata;
	int ret;

	printk(KERN_INFO "aw20072 led probe\n");
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	ddata = devm_kzalloc(&i2c->dev, sizeof(struct aw20072_drv_data), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->i2c = i2c;
	ddata->dev = &i2c->dev;
	ddata->led_nums = device_get_child_node_count(&i2c->dev);
	if (ddata->led_nums <= 0)
		return -ENOMEM;
	ddata->leds = devm_kzalloc(&i2c->dev,
			(ddata->led_nums) * sizeof(struct aw20072_led_io), GFP_KERNEL);
	if (!(ddata->leds))
		return -ENOMEM;
	ddata->colors = devm_kzalloc(&i2c->dev,
			(ddata->led_nums) * sizeof(struct aw20072_rgb_colors), GFP_KERNEL);
	if (!(ddata->colors))
		return -ENOMEM;

	ret = aw20072_enable_gpio_init(ddata);
	if (ret < 0) {
		dev_err(&i2c->dev, "aw20072_enable_gpio_init fail\n");
		return -EINVAL;
	}

	aw20072_interrupt_gpio_init(ddata);

	ret = aw20072_led_parse_dt(ddata);
	if (ret < 0) {
		dev_err(&i2c->dev, "aw20072_led_parse_dt fail\n");
		return -EINVAL;
	}

	ret = aw20072_reg_init(ddata);
	if (ret < 0) {
		dev_err(&i2c->dev, "aw20072_reg_init fail\n");
		return -EINVAL;
	}

	ret = aw20072_poweron_light_effect(ddata);
	if (ret < 0) {
		dev_err(&i2c->dev, "aw20072_poweron_light_effect fail\n");
		return -EINVAL;
	}
	ddata->soft_onoff = 1;
	mutex_init(&ddata->lock);

	i2c_set_clientdata(i2c, ddata);

	//led class
	ddata->cdev.name = "aw20072-leds";
	ddata->cdev.brightness = 0;
	ddata->cdev.max_brightness = 255;
	ret = led_classdev_register(ddata->dev, &ddata->cdev);
	if (ret) {
		dev_err(ddata->dev,
			"register led classdev fail, ret=%d\n", ret);
		return -1;
	}
	ret = sysfs_create_group(&ddata->cdev.dev->kobj,
			&aw20072_attribute_group);
	if (ret) {
		dev_err(ddata->dev, "create led sysfs fail, ret: %d\n", ret);
		led_classdev_unregister(&ddata->cdev);
		return -1;
	}
	aw20072_leds_data = ddata;

	return 0;
}

static int aw20072_i2c_remove(struct i2c_client *i2c)
{
	struct aw20072_drv_data *ddata = i2c_get_clientdata(i2c);

	mutex_destroy(&ddata->lock);
	led_classdev_unregister(&ddata->cdev);

	return 0;
}

static const struct i2c_device_id aw20072_i2c_id[] = {
	{ "aw20072-leds", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw20072_i2c_id);

static const struct of_device_id aw20072_leds_dt_match[] = {
	{ .compatible = "aw20072_leds" },
	{ },
};

static struct i2c_driver aw20072_leds_driver = {
	.driver = {
		.name = "aw20072-leds",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw20072_leds_dt_match),
	},
	.probe = aw20072_i2c_probe,
	.remove = aw20072_i2c_remove,
	.id_table = aw20072_i2c_id,
};

module_i2c_driver(aw20072_leds_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A1semi AW20072 led driver");
MODULE_ALIAS("platform:aw20072-leds");
