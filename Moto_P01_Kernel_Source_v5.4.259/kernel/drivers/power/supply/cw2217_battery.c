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
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/cdev.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <dt-bindings/input/gpio-keys.h>

struct battery_info {
	u8 cap;
	int temp;
	u32 vol;
	int curr;
};

struct cw2217_driver_data {
	struct device *dev;
	struct i2c_client *i2c;
	struct power_supply *battery;
	struct battery_info bat_info;
	struct timer_list timer;
	struct work_struct work;
	int report_period;
};

/**************  reg  ***************/
#define CW2217_REG_VERSION     0x0
#define CW2217_REG_VCELL_HIGH  0x02
#define CW2217_REG_VCELL_LOW   0x03
#define CW2217_REG_SOC_HIGH    0x04
#define CW2217_REG_SOC_LOW     0x05
#define CW2217_REG_TEMP        0x06
#define CW2217_REG_CONFIG      0x08
#define CW2217_REG_INT_CONF    0x0A
#define CW2217_REG_SOC_ALERT   0x0B
#define CW2217_REG_TEMP_MAX    0x0C
#define CW2217_REG_TEMP_MIN    0x0D
#define CW2217_REG_CURRENT_HIGH     0x0E
#define CW2217_REG_CURRENT_LOW      0x0F
#define CW2217_REG_10               0x10
#define CW2217_REG_T_HOST_HIGH      0xA0
#define CW2217_REG_T_HOST_LOW       0xA1
#define CW2217_REG_USER_CONF        0xA2
#define CW2217_REG_CYCLECNT_HIGH    0xA4
#define CW2217_REG_CYCLECNT_LOW     0xA5
#define CW2217_REG_SOH              0xA6
#define CW2217_REG_A7               0xA7
#define CW2217_REG_AVE_CUR_HIGH     0xA8
#define CW2217_REG_AVE_CUR_LOW      0xA9
#define CW2217_REG_FW_VERSION       0xAB
/**************  reg  ***************/
#define AW20072_REG_MAX                 (0xFF)
/**************  reg bits  ***************/
#define CW2217_MODE_REG             CW2217_REG_CONFIG
#define CW2217_MODE_MASK            0xF0
#define CW2217_MODE_SHIFT           4
#define CW2217_EN_INT_REG           CW2217_REG_INT_CONF
#define CW2217_EN_INT_MASK          0x70
#define CW2217_EN_INT_SHIFT         4
#define CW2217_INT_SOURCE_REG       CW2217_REG_INT_CONF
#define CW2217_INT_SOURCE_MASK      0x07
#define CW2217_INT_SOURCE_SHIFT     0
#define CW2217_UPDATE_FLAG_REG      CW2217_REG_SOC_ALERT
#define CW2217_UPDATE_FLAG_MASK     0x80
#define CW2217_UPDATE_FLAG_SHIFT    7
#define CW2217_SOC_ALERT_REG        CW2217_REG_SOC_ALERT
#define CW2217_SOC_ALERT_MASK       0x7F
#define CW2217_SOC_ALERT_SHIFT      0
/**************  reg bits  ***************/
//cw2217 config
#define USER_RSENSE (2 * 1000)
//define
#define CW2217_MODE_RESTART         0x30
#define CW2217_MODE_ACTIVE          0x00
#define CW2217_MODE_SLEEP           0xF0
#define CW2217_INT_EN_ALL           0x07
#define CW2217_INT_DIS_ALL          0x00
#define CW2217_INT_EN_SOC           0x04
#define CW2217_IC_STATE_MASK        0x0C
#define CW2217_UPDATE_FLAG          0x00
#define SIZE_OF_PROFILE             80
#define CW_SLEEP_COUNTS             50
#define CW2217_ERROR_IIC           -1
#define CW2217_ERROR_CHIP_ID       -2
#define CW2217_ERROR_TIME_OUT      -3
#define CW2217_NOT_ACTIVE           1
#define CW2217_PROFILE_NOT_READY    2
#define CW2217_PROFILE_NEED_UPDATE  3
#define REG_RD_ACCESS                   BIT(0)
#define REG_WR_ACCESS                   BIT(1)
#define CW2217_I2C_RETRY_CNT    5
#define CW2217_I2C_RETRY_DELAY  1
//default config
#define DEFAULT_REPORT_PERIOD   2000 //ms

static struct cw2217_driver_data *g_cw2217_data = NULL;
static unsigned char battery_profile[SIZE_OF_PROFILE] = {
	0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xA9, 0xC3, 0xC7, 0xC5, 0xB2, 0xA7, 0xCD, 0xAF,
    0x9E, 0xF4, 0xD1, 0x9C, 0x87, 0x6C, 0x59, 0x4A,
    0x43, 0x3E, 0x35, 0x72, 0x52, 0xDC, 0x3A, 0xDD,
    0xD0, 0xCF, 0xD1, 0xD0, 0xCD, 0xCC, 0xC7, 0xC4,
    0xC1, 0xC4, 0xC6, 0xA6, 0x95, 0x8A, 0x83, 0x78,
    0x6E, 0x75, 0x84, 0x8F, 0xA4, 0x90, 0x55, 0x4D,
    0x20, 0x00, 0xAB, 0x10, 0x00, 0x90, 0x71, 0x00,
    0x00, 0x00, 0x64, 0x23, 0xB3, 0x74, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59,
};

static u8 reg_table[AW20072_REG_MAX] = {
	[CW2217_REG_VERSION] = REG_RD_ACCESS,
	[CW2217_REG_VCELL_HIGH] = REG_RD_ACCESS,
	[CW2217_REG_VCELL_LOW] = REG_RD_ACCESS,
	[CW2217_REG_SOC_HIGH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_SOC_LOW] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_TEMP] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_CONFIG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_INT_CONF] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_SOC_ALERT] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_TEMP_MAX] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_TEMP_MIN] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_CURRENT_HIGH] = REG_RD_ACCESS,
	[CW2217_REG_CURRENT_LOW] = REG_RD_ACCESS,
	[CW2217_REG_10] = REG_RD_ACCESS,
	[CW2217_REG_T_HOST_HIGH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_T_HOST_LOW] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_USER_CONF] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_CYCLECNT_HIGH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_CYCLECNT_LOW] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_SOH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[CW2217_REG_A7] = REG_RD_ACCESS,
	[CW2217_REG_AVE_CUR_HIGH] = REG_RD_ACCESS,
	[CW2217_REG_AVE_CUR_LOW] = REG_RD_ACCESS,
	[CW2217_REG_FW_VERSION] = REG_RD_ACCESS,
};

static int cw2217_i2c_write(struct cw2217_driver_data *ddata, u8 reg_addr, u8 reg_data)
{
	int ret = -1;
	int cnt = 0;

	while (cnt < CW2217_I2C_RETRY_CNT) {
		ret = i2c_smbus_write_byte_data(ddata->i2c, reg_addr, reg_data);
		if (ret < 0)
			dev_err(ddata->dev, "%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
		else
			break;
		cnt++;
		msleep(CW2217_I2C_RETRY_DELAY);
	}

	return ret;
}

static int cw2217_i2c_read(struct cw2217_driver_data *ddata, u8 reg_addr, u8 *reg_data)
{
	int ret = -1;
	int cnt = 0;

	while (cnt < CW2217_I2C_RETRY_CNT) {
		ret = i2c_smbus_read_byte_data(ddata->i2c, reg_addr);
		if (ret < 0) {
			dev_err(ddata->dev, "%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(CW2217_I2C_RETRY_DELAY);
	}

	return ret;
}

static int cw2217_i2c_write_bits(struct cw2217_driver_data *ddata, u8 reg_addr,
		u8 mask, u8 shift, u8 reg_data)
{
	u8 reg_value;
	int ret;

	ret = cw2217_i2c_read(ddata, reg_addr, &reg_value);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: read reg(0x%x) mask(0x%x) shift(0x%x) bits fail\n",
				__func__, reg_addr, mask, shift);
		return -1;
	}
	reg_data = (reg_value & ~mask) | (reg_data << shift);
	ret = cw2217_i2c_write(ddata, reg_addr, reg_data);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: write reg(0x%x) mask(0x%x) shift(0x%x) bits fail\n",
				__func__, reg_addr, mask, shift);
		return -1;
	}

	return 0;
}

static int get_complement_code(unsigned short raw_code)
{
	int complement_code = 0;
	int dir = 0;

	if (0 != (raw_code & 0x8000)){
		dir = -1;
		raw_code =  (0XFFFF - raw_code) + 1;
	}
	else{
		dir = 1;
	}

	complement_code = (int)raw_code * dir;

	return complement_code;
}

static int cw2217_get_ibat(struct cw2217_driver_data *ddata, int *ibat)
{
    u8 reg_val;
    int curr;
    unsigned short raw_current, current_l, current_h;
    int ret;

    ret = cw2217_i2c_read(ddata, CW2217_REG_CURRENT_HIGH, &reg_val);
    if (ret < 0) {
        dev_err(ddata->dev, "read CW2217_REG_CURRENT_HIGH fail\n");
		return -1;
    }
	current_h = reg_val;

    ret = cw2217_i2c_read(ddata, CW2217_REG_CURRENT_LOW, &reg_val);
    if (ret < 0) {
        dev_err(ddata->dev, "read CW2217_REG_CURRENT_LOW fail\n");
		return -1;
    }
	current_l = reg_val;

    raw_current = (current_h << 8) | current_l;

    curr = get_complement_code(raw_current);

    *ibat = curr * 1600 / USER_RSENSE;

    return 0;
}

static int cw2217_get_vcell(struct cw2217_driver_data *ddata, u32 *vcell)
{
    u8 reg_val;
    u32 vcell_l, vcell_h;
    u32 ret;

    ret = cw2217_i2c_read(ddata, CW2217_REG_VCELL_HIGH, &reg_val);
    if (ret < 0) {
        dev_err(ddata->dev, "read CW2217_REG_VCELL_HIGH fail\n");
		return -1;
    }
	vcell_h = reg_val & 0x3F;

    ret = cw2217_i2c_read(ddata, CW2217_REG_VCELL_LOW, &reg_val);
    if (ret < 0) {
        dev_err(ddata->dev, "read CW2217_REG_VCELL_HIGH fail\n");
		return -1;
    }
	vcell_l = reg_val;

    *vcell = (vcell_h << 8) | vcell_l;

    *vcell = (u32)(*vcell * 3125) / 10000;

    return 0;
}

static int cw2217_get_temprature(struct cw2217_driver_data *ddata, int *temprature)
{
	int ret = 0;
    u8 reg_val;

	ret = cw2217_i2c_read(ddata, CW2217_REG_TEMP, &reg_val);
	if(ret < 0){
		dev_err(ddata->dev, "read CW2217_REG_TEMP fail\n");
        return -1;
    }

    *temprature = reg_val / 2 - 40;

	return 0;
}

static int cw2217_get_raw_soc(struct cw2217_driver_data *ddata, u8 *raw_soc)
{
    u8 reg_val;
    u32 soc_l, soc_h;
    u32 ret;

    ret = cw2217_i2c_read(ddata, CW2217_REG_SOC_HIGH, &reg_val);
    if (ret < 0) {
        dev_err(ddata->dev, "read CW2217_REG_SOC_HIGH fail\n");
        return -1;
    }
	soc_h = reg_val;

    ret = cw2217_i2c_read(ddata, CW2217_REG_SOC_LOW, &reg_val);
    if (ret < 0) {
        dev_err(ddata->dev, "read CW2217_REG_SOC_LOW fail\n");
        return -1;
    }
	soc_l = reg_val;
    *raw_soc = soc_h;

    return 0;
}

int cw2217_get_ic_state(struct cw2217_driver_data *ddata, u8 *state)
{
    u8 reg_val;
    u32 ret;

    ret = cw2217_i2c_read(ddata, CW2217_REG_A7, &reg_val);
    if (ret < 0) {
        dev_err(ddata->dev, "read CW2217_REG_CONFIG fail\n");
        return -1;
    }
    
    *state = reg_val & CW2217_IC_STATE_MASK;

    return 0;
}

static int cw2217_get_mode(struct cw2217_driver_data *ddata, u8 *mode)
{
    u8 reg_val;
    u32 ret;

    ret = cw2217_i2c_read(ddata, CW2217_MODE_REG, &reg_val);
    if (ret < 0) {
        dev_err(ddata->dev, "read CW2217_REG_CONFIG fail\n");
        return -1;
    }

    *mode = (reg_val & CW2217_MODE_MASK) >> CW2217_MODE_SHIFT;

    return 0;
}

static int cw2217_get_update_flag(struct cw2217_driver_data *ddata, u8 *flag)
{
    u8 reg_val;
    u32 ret;

    ret = cw2217_i2c_read(ddata, CW2217_UPDATE_FLAG_REG, &reg_val);
    if (ret < 0) {
        dev_err(ddata->dev, "read CW2217_UPDATE_FLAG_REG fail\n");
        return -1;
    }
    
    *flag = (reg_val & CW2217_UPDATE_FLAG_MASK) >> CW2217_UPDATE_FLAG_SHIFT;

    return 0;
}

static int cw2217_set_update_flag(struct cw2217_driver_data *ddata, u8 flag)
{
	return cw2217_i2c_write_bits(ddata, CW2217_UPDATE_FLAG_REG,
                        			CW2217_UPDATE_FLAG_MASK,
									CW2217_UPDATE_FLAG_SHIFT,
									flag & 0x01);
}

static int cw2217_set_interrupt(struct cw2217_driver_data *ddata, u8 en)
{
	return cw2217_i2c_write_bits(ddata, CW2217_EN_INT_REG,
                        			CW2217_EN_INT_MASK,
									CW2217_EN_INT_SHIFT,
									en);
}

static int cw2217_set_soc_alert(struct cw2217_driver_data *ddata)
{
	return cw2217_i2c_write_bits(ddata, CW2217_SOC_ALERT_REG,
                        			CW2217_SOC_ALERT_MASK,
									CW2217_SOC_ALERT_SHIFT,
									0x7F);
}

static int cw2217_get_bat_profile(struct cw2217_driver_data *ddata, int i, u8 *val)
{
    u8 reg_val;
    u32 ret;

    ret = cw2217_i2c_read(ddata, CW2217_REG_10 + i, &reg_val);
    if (ret < 0) {
		dev_err(ddata->dev, "read CW2217_REG_10 fail\n");
        return -1;
    }

    *val = reg_val;

    return 0;
}

static int cw2217_write_profile(struct cw2217_driver_data *ddata, unsigned char const buf[])
{
	int ret;
	int i;

	for (i = 0; i < SIZE_OF_PROFILE; i++) {
		ret = cw2217_i2c_write(ddata, CW2217_REG_10 + i, buf[i]);
		if (ret < 0) {
            dev_err(ddata->dev, "write CW2217_REG_10 + %d fail\n", i);
            return ret;
        }
	}

	return 0;
}

static int cw2217_sleep(struct cw2217_driver_data *ddata)
{
	int ret;

    ret = cw2217_i2c_write(ddata, CW2217_REG_CONFIG, CW2217_MODE_RESTART);
    if (ret < 0) {
		dev_err(ddata->dev, "write CW2217_REG_CONFIG fail\n");
        return -1;
	}
    msleep(20);

    ret = cw2217_i2c_write(ddata, CW2217_REG_CONFIG, CW2217_MODE_SLEEP);
    if (ret < 0) {
		dev_err(ddata->dev, "write CW2217_REG_CONFIG fail\n");
        return -1;
	}
    msleep(10);

	return 0;
}

static int cw2217_active(struct cw2217_driver_data *ddata)
{
    int ret;

    ret = cw2217_i2c_write(ddata, CW2217_REG_CONFIG, CW2217_MODE_RESTART);
    if (ret < 0) {
		dev_err(ddata->dev, "write CW2217_REG_CONFIG fail\n");
        return -1;
	}
    msleep(20);

    ret = cw2217_i2c_write(ddata, CW2217_REG_CONFIG, CW2217_MODE_ACTIVE);
    if (ret < 0) {
		dev_err(ddata->dev, "write CW2217_REG_CONFIG fail\n");
        return -1;
	}
    msleep(10);

    return ret;
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cw2217_driver_data *ddata = g_cw2217_data;
	unsigned int databuf[2] = {0, 0};

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		cw2217_i2c_write(ddata, (unsigned char)databuf[0], (unsigned char)databuf[1]);

	return count;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct cw2217_driver_data *ddata = g_cw2217_data;
	ssize_t len = 0;
	u8 i = 0;
	u8 reg_val = 0;

	for (i = 0; i < AW20072_REG_MAX; i++) {
		if (!(reg_table[i] & REG_RD_ACCESS))
			continue;
		cw2217_i2c_read(ddata, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%02x\n", i, reg_val);
	}

	return len;
}

static ssize_t bat_vol_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct cw2217_driver_data *ddata = g_cw2217_data;
	ssize_t len = 0;

	cw2217_get_vcell(ddata, &ddata->bat_info.vol);
	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", ddata->bat_info.vol);

	return len;
}

static ssize_t bat_curr_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct cw2217_driver_data *ddata = g_cw2217_data;
	ssize_t len = 0;

	cw2217_get_ibat(ddata, &ddata->bat_info.curr);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", ddata->bat_info.curr);

	return len;
}

static ssize_t bat_cap_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct cw2217_driver_data *ddata = g_cw2217_data;
	ssize_t len = 0;

	cw2217_get_raw_soc(ddata, &ddata->bat_info.cap);
	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", ddata->bat_info.cap);

	return len;
}

static ssize_t bat_temp_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct cw2217_driver_data *ddata = g_cw2217_data;
	ssize_t len = 0;

	cw2217_get_temprature(ddata, &ddata->bat_info.temp);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", ddata->bat_info.temp);

	return len;
}

static DEVICE_ATTR_RO(bat_vol);
static DEVICE_ATTR_RO(bat_curr);
static DEVICE_ATTR_RO(bat_cap);
static DEVICE_ATTR_RO(bat_temp);
static DEVICE_ATTR_RW(reg);

static struct attribute *cw2217_attributes[] = {
	&dev_attr_bat_vol.attr,
	&dev_attr_bat_curr.attr,
	&dev_attr_bat_cap.attr,
	&dev_attr_bat_temp.attr,
	&dev_attr_reg.attr,
	NULL
};

static struct attribute_group cw2217_attribute_group = {
	.attrs = cw2217_attributes
};

static int cw2217_battery_get_property(struct power_supply *psy,
						 enum power_supply_property psp,
						 union power_supply_propval *val)
{
	struct cw2217_driver_data *ddata = power_supply_get_drvdata(psy);

	switch (psp) {
		case POWER_SUPPLY_PROP_CAPACITY:
			{
				val->intval = ddata->bat_info.cap;
				break;
			}
		case POWER_SUPPLY_PROP_TEMP:
			{
				val->intval = ddata->bat_info.temp;
				break;
			}
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			{
				val->intval = ddata->bat_info.vol;
				break;
			}
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			{
				val->intval = ddata->bat_info.curr;
				break;
			}
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static enum power_supply_property cw2217_battery_property[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP
};

struct power_supply_desc cw2217_battery_desc = {
	.name = "cw2217_battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = cw2217_battery_property,
	.num_properties = ARRAY_SIZE(cw2217_battery_property),
	.get_property = cw2217_battery_get_property,
};

static int cw2217_power_supply_init(struct cw2217_driver_data *ddata)
{
	int ret = 0;

	struct power_supply_config psy_cfg = { .drv_data = ddata, };

	ddata->battery = power_supply_register(ddata->dev, &cw2217_battery_desc,
						&psy_cfg);
	if (PTR_ERR_OR_ZERO(ddata->battery)) {
		dev_err(ddata->dev, "power_supply_register fail\n");
		return -1;
	}

	ret = sysfs_create_group(&ddata->battery->dev.kobj,
			&cw2217_attribute_group);
	if (ret) {
		dev_err(ddata->dev, "create sysfs fail, ret: %d\n", ret);
		return -1;
	}

	return 0;
}

static void cw2217_battery_work_handler(struct work_struct *work)
{
	struct cw2217_driver_data *ddata = g_cw2217_data;

	cw2217_get_vcell(ddata, &ddata->bat_info.vol);
	cw2217_get_ibat(ddata, &ddata->bat_info.curr);
	cw2217_get_temprature(ddata, &ddata->bat_info.temp);
	cw2217_get_raw_soc(ddata, &ddata->bat_info.cap);

	power_supply_changed(ddata->battery);
	mod_timer(&ddata->timer, jiffies + msecs_to_jiffies(ddata->report_period));
}

static void cw2217_battery_timer_handler(struct timer_list *t)
{
	struct cw2217_driver_data *ddata = g_cw2217_data;

	schedule_work(&ddata->work);
}

static int cw2217_get_state(struct cw2217_driver_data *ddata)
{
	int ret;
	u8 reg_val;
	int i;
	
	ret = cw2217_get_mode(ddata, &reg_val);
	if(ret < 0) {
		dev_err(ddata->dev, "get mode fail\n");
		return CW2217_ERROR_IIC;
	}
	if (reg_val != CW2217_MODE_ACTIVE) {
		dev_err(ddata->dev, "cw2217 not active\n");
		return CW2217_NOT_ACTIVE;
	}
	
	ret = cw2217_get_update_flag(ddata, &reg_val);
	if (ret < 0) {
		dev_err(ddata->dev, "get update flag fail\n");
		return CW2217_ERROR_IIC;
	}
	if (reg_val == CW2217_UPDATE_FLAG) {
		dev_err(ddata->dev, "profile not ready\n");
		return CW2217_PROFILE_NOT_READY;
	}
	
	for (i = 0; i < SIZE_OF_PROFILE; i++) {
		ret = cw2217_get_bat_profile(ddata, i, &reg_val);
		if (ret < 0) {
			dev_err(ddata->dev, "get profile fail\n");
			return CW2217_ERROR_IIC;
		}
		if (battery_profile[i] != reg_val)
			break;
	}
	if ( i != SIZE_OF_PROFILE) {
		dev_err(ddata->dev, "profile need update\n");
		return CW2217_PROFILE_NEED_UPDATE;
	}
	
	return 0;
}

static int cw2217_config_start_ic(struct cw2217_driver_data *ddata)
{
	int ret;
	unsigned char reg_val;
	int count = 0;

	ret = cw2217_sleep(ddata);
	if (ret < 0){
        dev_err(ddata->dev, "cw2217_sleep_1 faile\n");
        return -1;
    }

	/* update new battery info */
	ret = cw2217_write_profile(ddata, battery_profile);
	if (ret < 0){
        dev_err(ddata->dev, "cw2217_write_profile faile\n");
        return -1;
    }

	/* set UPDATE_FLAG AND SOC INTTERRUP VALUE*/
	ret = cw2217_set_update_flag(ddata, 1);
	if (ret < 0){
        dev_err(ddata->dev, "cw2217_update_flag faile\n");
        return -1;
    }

	/*close all interruptes*/
	ret = cw2217_set_interrupt(ddata, CW2217_INT_DIS_ALL);
	if (ret < 0){
        dev_err(ddata->dev, "cw2217_clear_all_inter faile\n");
		return -1;
    }

	ret = cw2217_active(ddata);
	if (ret < 0){
        dev_err(ddata->dev, "cw2217_active faile\n");
		return -1;
    }

	while (1) {
		mdelay(50);
		ret = cw2217_get_ic_state(ddata, &reg_val);
		if (reg_val == CW2217_IC_STATE_MASK) {
            break;
        }
		count++;
		if (count >= CW_SLEEP_COUNTS) {
			cw2217_sleep(ddata);
			dev_err(ddata->dev, "cw2217_get_ic_state faile, timeout\n");
			return -1;
		}
	}

	dev_info(ddata->dev, "cw221x_config_start_ic end~\n");
	return 0;
}

static int cw2217_chip_init(struct cw2217_driver_data *ddata)
{
    int ret;

    ret = cw2217_get_state(ddata);
    if (ret < 0){
        dev_err(ddata->dev, "get state fail\n");
		return -1;
    }

    if (ret != 0) {
		ret = cw2217_config_start_ic(ddata);
		if (ret < 0){
            dev_err(ddata->dev, "config start ic fail\n");
            return -1;
        }
	}

    return 0;
}

static int cw2217_init(struct cw2217_driver_data *ddata)
{
    int ret;

    ret = cw2217_chip_init(ddata);
    if (ret < 0)
		return -1;

	ret = cw2217_set_interrupt(ddata, CW2217_INT_EN_ALL);
	if (ret < 0){
        dev_err(ddata->dev, "enable all intterupt fail\n");
		return -1;
    }

	ret = cw2217_set_soc_alert(ddata);
	if (ret < 0){
        dev_err(ddata->dev, "set soc alert fail\n");
		return -1;
    }

	cw2217_get_vcell(ddata, &ddata->bat_info.vol);
	cw2217_get_ibat(ddata, &ddata->bat_info.curr);
	cw2217_get_temprature(ddata, &ddata->bat_info.temp);
	cw2217_get_raw_soc(ddata, &ddata->bat_info.cap);
    return 0;
}

static int cw2217_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct cw2217_driver_data *ddata;
	int ret;

	dev_info(&i2c->dev, "cw2217 battery probe\n");
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	ddata = devm_kzalloc(&i2c->dev, sizeof(struct cw2217_driver_data), GFP_KERNEL);
	if (!ddata) {
		dev_err(&i2c->dev, "devm_kzalloc failed\n");
		return -ENOMEM;
	}
	ddata->i2c = i2c;
	ddata->dev = &i2c->dev;
	g_cw2217_data = ddata;
	i2c_set_clientdata(i2c, ddata);

	ret = cw2217_power_supply_init(ddata);
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to register power supply\n");
		return -EINVAL;
	}

	ret = cw2217_init(ddata);
	if (ret < 0) {
		power_supply_unregister(ddata->battery);
		dev_err(&i2c->dev, "hw init fail\n");
		return -EINVAL;
	}

	ddata->report_period = DEFAULT_REPORT_PERIOD;
	INIT_WORK(&ddata->work, cw2217_battery_work_handler);
	timer_setup(&ddata->timer, cw2217_battery_timer_handler, 0);
	mod_timer(&ddata->timer, jiffies + msecs_to_jiffies(ddata->report_period));

	return 0;
}

static int cw2217_i2c_remove(struct i2c_client *i2c)
{
	struct cw2217_driver_data *ddata = i2c_get_clientdata(i2c);

	power_supply_unregister(ddata->battery);

	return 0;
}

static const struct i2c_device_id cw2217_i2c_id[] = {
	{ "cw2217-battery", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cw2217_i2c_id);

static const struct of_device_id cw2217_battery_dt_match[] = {
	{ .compatible = "cw2217" },
	{ },
};

static struct i2c_driver cw2217_battery_driver = {
	.driver = {
		.name = "cw2217-battery",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cw2217_battery_dt_match),
	},
	.probe = cw2217_i2c_probe,
	.remove = cw2217_i2c_remove,
	.id_table = cw2217_i2c_id,
};

module_i2c_driver(cw2217_battery_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("cw2217 charge driver");
