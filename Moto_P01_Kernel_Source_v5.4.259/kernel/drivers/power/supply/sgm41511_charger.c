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

enum sys_min {
	SYS_MIN_2600MV = 0,
	SYS_MIN_2800MV,
	SYS_MIN_3000MV,
	SYS_MIN_3200MV,
	SYS_MIN_3400MV,
	SYS_MIN_3500MV,
	SYS_MIN_3600MV,
	SYS_MIN_3700MV,

	SYS_MIN_MAX,
};

struct charge_config {
	u32 ichg;	/* charge current		*/
	u32 vreg;	/* regulation voltage		*/
	u32 iterm;	/* termination current		*/
	u32 iprechg;	/* precharge current		*/
	u32 sysvmin;	/* minimum system voltage limit */
	u32 iindpm;
	u32 vindpm;
	u32 chg_config;
};

struct charger_state {
	u8 online;
	u8 charge_status;
	u8 power_good;
	u8 charge_fault;
	u8 bat_fault;
	u8 ntc_fault;
};

struct sgm41511_driver_data {
	struct device *dev;
	struct i2c_client *i2c;
	struct power_supply *charger;
	int interrupt_gpio;
	int charge_gpio;
	struct charger_state state;
	struct charge_config chg_cfg;
	int irq;
};

/**************  reg  ***************/
#define SGM41511_REG_00  0x0
#define SGM41511_REG_01  0x01
#define SGM41511_REG_02  0x02
#define SGM41511_REG_03  0x03
#define SGM41511_REG_04  0x04
#define SGM41511_REG_05  0x05
#define SGM41511_REG_06  0x06
#define SGM41511_REG_07  0x07
#define SGM41511_REG_08  0x08
#define SGM41511_REG_09  0x09
#define SGM41511_REG_0A  0x0A
#define SGM41511_REG_0B  0x0B
/**************  reg  ***************/
#define SGM41511_REG_MAX  0x0F

//IINDPM
#define SGM41511_REG_IINDPM              SGM41511_REG_00
#define SGM41511_REG_IINDPM_MASK         0x1F
#define SGM41511_REG_IINDPM_SHIFT        0
//enable charge
#define SGM41511_REG_CHG_CONFIG          SGM41511_REG_01
#define SGM41511_REG_CHG_CONFIG_MASK     0x10
#define SGM41511_REG_CHG_CONFIG_SHIFT    4
//min system voltage
#define SGM41511_REG_SYS_MIN          SGM41511_REG_01
#define SGM41511_REG_SYS_MIN_MASK     0x0E
#define SGM41511_REG_SYS_MIN_SHIFT    1
//fast charge current
#define SGM41511_REG_ICHG          SGM41511_REG_02
#define SGM41511_REG_ICHG_MASK     0x3F
#define SGM41511_REG_ICHG_SHIFT    0
//pre-charge current limit
#define SGM41511_REG_IPRECHG          SGM41511_REG_03
#define SGM41511_REG_IPRECHG_MASK     0xF0
#define SGM41511_REG_IPRECHG_SHIFT    4
//Termination charge current limit
#define SGM41511_REG_ITERM          SGM41511_REG_03
#define SGM41511_REG_ITERM_MASK     0x0F
#define SGM41511_REG_ITERM_SHIFT    0
//Charge Voltage Limit
#define SGM41511_REG_VREG          SGM41511_REG_04
#define SGM41511_REG_VREG_MASK     0xF8
#define SGM41511_REG_VREG_SHIFT    3
//Battery Recharge Threshold
#define SGM41511_REG_VRECHG          SGM41511_REG_04
#define SGM41511_REG_VRECHG_MASK     0x01
#define SGM41511_REG_VRECHG_SHIFT    0
//watchdog
#define SGM41511_REG_WATCHDOG          SGM41511_REG_05
#define SGM41511_REG_WATCHDOG_MASK     0x30
#define SGM41511_REG_WATCHDOG_SHIFT    4
//VINDPM
#define SGM41511_REG_VINDPM            SGM41511_REG_06
#define SGM41511_REG_VINDPM_MASK       0x0F
#define SGM41511_REG_VINDPM_SHIFT      0
//Disable BATFET
#define SGM41511_REG_BATFET_DIS          SGM41511_REG_07
#define SGM41511_REG_BATFET_DIS_MASK     0x20
#define SGM41511_REG_BATFET_DIS_SHIFT    5
//BATFET DLY
#define SGM41511_REG_BATFET_DLY          SGM41511_REG_07
#define SGM41511_REG_BATFET_DLY_MASK     0x08
#define SGM41511_REG_BATFET_DLY_SHIFT    3
//VBUS STAT
#define SGM41511_REG_VBUS_STAT          SGM41511_REG_08
#define SGM41511_REG_VBUS_STAT_MASK     0xE0
#define SGM41511_REG_VBUS_STAT_SHIFT    5
//CHRG STAT
#define SGM41511_REG_CHRG_STAT          SGM41511_REG_08
#define SGM41511_REG_CHRG_STAT_MASK     0x18
#define SGM41511_REG_CHRG_STAT_SHIFT    3
//PG STAT
#define SGM41511_REG_PG_STAT          SGM41511_REG_08
#define SGM41511_REG_PG_STAT_MASK     0x04
#define SGM41511_REG_PG_STAT_SHIFT    2
//CHRG FAULT
#define SGM41511_REG_CHRG_FAULT           SGM41511_REG_09
#define SGM41511_REG_CHRG_FAULT_MASK      0x30
#define SGM41511_REG_CHRG_FAULT_SHIFT     4
//BAT FAULT
#define SGM41511_REG_BAT_FAULT           SGM41511_REG_09
#define SGM41511_REG_BAT_FAULT_MASK      0x08
#define SGM41511_REG_BAT_FAULT_SHIFT     3
//NTC FAULT
#define SGM41511_REG_NTC_FAULT           SGM41511_REG_09
#define SGM41511_REG_NTC_FAULT_MASK      0x07
#define SGM41511_REG_NTC_FAULT_SHIFT     0

//
#define SGM41511_I2C_RETRY_CNT    5
#define SGM41511_I2C_RETRY_DELAY  1
#define START_CHARGE  1
#define STOP_CHARGE   0
#define ENTER_SHIP_MODE  1
#define EXIT_SHIP_MODE   0
#define DELAY_ENTER_SHIP_MODE   1
#define IMMEDIATELY_ENTER_SHIP_MODE  0

/*************** charge param threshold ****************/
#define IINDPM_BASE_MA   (100)
#define IINDPM_UNITS_MA  (100)
#define IINDPM_MAX_MA    (3200)
#define VINDPM_BASE_MV   (3900)
#define VINDPM_UNITS_MV  (100)
#define VINDPM_MAX_MV    (5400)
#define ICHG_BASE_MA  (0)
#define ICHG_UNITS_MA  (60)
#define ICHG_MAX_MA  (3000)
#define ITERM_BASE_MA  (60)
#define ITERM_UNITS_MA  (60)
#define ITERM_MAX_MA  (960)
#define IPRECHG_BASE_MA  (60)
#define IPRECHG_UNITS_MA  (60)
#define IPRECHG_MAX_MA  (780)
#define VCHG_BASE_MV  (3856)
#define VCHG_UNITS_MV  (32)
#define VCHG_MAX_MV  (4624)

/*****************default charge config  ************/
#define DEFAULT_CHG_CURRENT   1000  //mA
#define DEFAULT_PRECHG_CURRENT  IPRECHG_MAX_MA    //mA
#define DEFAULT_TERM_CURRENT  420    //mA
#define DEFAULT_CHG_VOLTAGE  4400    //mV
#define DEFAULT_SYS_MIN      SYS_MIN_3500MV
#define DEFAULT_IINDPM       IINDPM_MAX_MA
#define DEFAULT_VINDPM       4800

static struct sgm41511_driver_data *g_sgm41511_data = NULL;

static int sgm41511_i2c_write(struct sgm41511_driver_data *ddata, u8 reg_addr, u8 reg_data)
{
	int ret = -1;
	int cnt = 0;

	while (cnt < SGM41511_I2C_RETRY_CNT) {
		ret = i2c_smbus_write_byte_data(ddata->i2c, reg_addr, reg_data);
		if (ret < 0)
			dev_err(ddata->dev, "%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
		else
			break;
		cnt++;
		msleep(SGM41511_I2C_RETRY_DELAY);
	}

	return ret;
}

static int sgm41511_i2c_read(struct sgm41511_driver_data *ddata, u8 reg_addr, u8 *reg_data)
{
	int ret = -1;
	int cnt = 0;

	while (cnt < SGM41511_I2C_RETRY_CNT) {
		ret = i2c_smbus_read_byte_data(ddata->i2c, reg_addr);
		if (ret < 0) {
			dev_err(ddata->dev, "%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(SGM41511_I2C_RETRY_DELAY);
	}

	return ret;
}

static int sgm41511_i2c_write_bits(struct sgm41511_driver_data *ddata, u8 reg_addr,
		u8 mask, u8 shift, u8 reg_data)
{
	u8 reg_value;
	int ret;

	ret = sgm41511_i2c_read(ddata, reg_addr, &reg_value);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: read reg(0x%x) mask(0x%x) shift(0x%x) bits fail\n",
				__func__, reg_addr, mask, shift);
		return -1;
	}
	reg_data = (reg_value & ~mask) | (reg_data << shift);
	ret = sgm41511_i2c_write(ddata, reg_addr, reg_data);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: write reg(0x%x) mask(0x%x) shift(0x%x) bits fail\n",
				__func__, reg_addr, mask, shift);
		return -1;
	}

	return ret;
}

static int sgm41511_set_chg_config(struct sgm41511_driver_data *ddata, u8 config)
{
	int ret;

	ret = sgm41511_i2c_write_bits(ddata,
								SGM41511_REG_CHG_CONFIG,
								SGM41511_REG_CHG_CONFIG_MASK,
								SGM41511_REG_CHG_CONFIG_SHIFT,
								config);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set SGM41511_REG_CHG_CONFIG fail\n", __func__);
		return -1;
	}

	return 0;
}

static int sgm41511_set_watchdog(struct sgm41511_driver_data *ddata, u8 enable)
{
	int ret;

	ret = sgm41511_i2c_write_bits(ddata,
								SGM41511_REG_WATCHDOG,
								SGM41511_REG_WATCHDOG_MASK,
								SGM41511_REG_WATCHDOG_SHIFT,
								enable);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set SGM41511_REG_WATCHDOG fail\n", __func__);
		return -1;
	}

	return 0;
}

static int sgm41511_set_iindpm(struct sgm41511_driver_data *ddata, u32 curr)
{
	u8 reg = 0;
	int ret;

	if (curr > IINDPM_MAX_MA)
		curr = IINDPM_MAX_MA;
	if (curr < IINDPM_BASE_MA)
		curr = IINDPM_BASE_MA;
	
	reg = (curr - IINDPM_BASE_MA) / IINDPM_UNITS_MA;

	ret = sgm41511_i2c_write_bits(ddata,
								SGM41511_REG_IINDPM,
								SGM41511_REG_IINDPM_MASK,
								SGM41511_REG_IINDPM_SHIFT,
								reg);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set SGM41511_REG_IINDPM fail\n", __func__);
		return -1;
	}

	return 0;
}

static int sgm41511_set_vindpm(struct sgm41511_driver_data *ddata, u32 volt)
{
	u8 reg = 0;
	int ret;

	if (volt > VINDPM_MAX_MV)
		volt = VINDPM_MAX_MV;
	if (volt < VINDPM_BASE_MV)
		volt = VINDPM_BASE_MV;
	
	reg = (volt - VINDPM_BASE_MV) / VINDPM_UNITS_MV;

	ret = sgm41511_i2c_write_bits(ddata,
								SGM41511_REG_VINDPM,
								SGM41511_REG_VINDPM_MASK,
								SGM41511_REG_VINDPM_SHIFT,
								reg);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set SGM41511_REG_VINDPM fail\n", __func__);
		return -1;
	}

	return 0;
}

static int sgm41511_set_ichg(struct sgm41511_driver_data *ddata, u32 ichg)
{
	u8 reg = 0;
	int ret;

	if (ichg > ICHG_MAX_MA)
		ichg = ICHG_MAX_MA;
	if (ichg < ICHG_BASE_MA)
		ichg = ICHG_BASE_MA;

	ddata->chg_cfg.ichg = ichg;
	reg = (ichg - ICHG_BASE_MA) / ICHG_UNITS_MA;

	ret = sgm41511_i2c_write_bits(ddata,
								SGM41511_REG_ICHG,
								SGM41511_REG_ICHG_MASK,
								SGM41511_REG_ICHG_SHIFT,
								reg);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set SGM41511_REG_ICHG fail\n", __func__);
		return -1;
	}

	return 0;
}

static int sgm41511_set_iprechg(struct sgm41511_driver_data *ddata, u32 iprechg)
{
	u8 reg = 0;
	int ret;

	if (iprechg > IPRECHG_MAX_MA)
		iprechg = IPRECHG_MAX_MA;
	if (iprechg < IPRECHG_BASE_MA)
		iprechg = IPRECHG_BASE_MA;
	
	reg = (iprechg - IPRECHG_BASE_MA) / IPRECHG_UNITS_MA;

	ret = sgm41511_i2c_write_bits(ddata,
								SGM41511_REG_IPRECHG,
								SGM41511_REG_IPRECHG_MASK,
								SGM41511_REG_IPRECHG_SHIFT,
								reg);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set SGM41511_REG_IPRECHG fail\n", __func__);
		return -1;
	}

	return 0;
}

static int sgm41511_set_iterm(struct sgm41511_driver_data *ddata, u32 iterm)
{
	u8 reg = 0;
	int ret;

	if (iterm > ITERM_MAX_MA)
		iterm = ITERM_MAX_MA;
	if (iterm < ITERM_BASE_MA)
		iterm = ITERM_BASE_MA;
	
	reg = (iterm - ITERM_BASE_MA) / ITERM_UNITS_MA;

	ret = sgm41511_i2c_write_bits(ddata,
								SGM41511_REG_ITERM,
								SGM41511_REG_ITERM_MASK,
								SGM41511_REG_ITERM_SHIFT,
								reg);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set SGM41511_REG_ITERM fail\n", __func__);
		return -1;
	}

	return 0;
}

static int sgm41511_set_vchg(struct sgm41511_driver_data *ddata, u32 vchg)
{
	u8 reg = 0;
	int ret;

	if (vchg > VCHG_MAX_MV)
		vchg = VCHG_MAX_MV;
	if (vchg < VCHG_BASE_MV)
		vchg = VCHG_BASE_MV;
	
	reg = (vchg - VCHG_BASE_MV) / VCHG_UNITS_MV;

	ret = sgm41511_i2c_write_bits(ddata,
								SGM41511_REG_VREG,
								SGM41511_REG_VREG_MASK,
								SGM41511_REG_VREG_SHIFT,
								reg);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set SGM41511_REG_VREG fail\n", __func__);
		return -1;
	}

	return 0;
}

static int sgm41511_set_sysmin(struct sgm41511_driver_data *ddata, u8 sysmin)
{
	int ret;

	ret = sgm41511_i2c_write_bits(ddata,
								SGM41511_REG_SYS_MIN,
								SGM41511_REG_SYS_MIN_MASK,
								SGM41511_REG_SYS_MIN_SHIFT,
								sysmin);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: set SGM41511_REG_SYS_MIN fail\n", __func__);
		return -1;
	}

	return 0;
}

static int sgm41511_enter_shipmode(struct sgm41511_driver_data *ddata, u8 delay)
{
	int ret;

	ret = sgm41511_i2c_write_bits(ddata,
			SGM41511_REG_BATFET_DLY,
			SGM41511_REG_BATFET_DLY_MASK,
			SGM41511_REG_BATFET_DLY_SHIFT,
			delay > 0 ? DELAY_ENTER_SHIP_MODE : IMMEDIATELY_ENTER_SHIP_MODE);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: enable shipmode delay fail\n", __func__);
		return -1;
	}

	ret = sgm41511_i2c_write_bits(ddata,
								SGM41511_REG_BATFET_DIS,
								SGM41511_REG_BATFET_DIS_MASK,
								SGM41511_REG_BATFET_DIS_SHIFT,
								ENTER_SHIP_MODE);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: enter shipmode fail\n", __func__);
		return -1;
	}

	return 0;
}

static int sgm41511_exit_shipmode(struct sgm41511_driver_data *ddata)
{
	int ret;

	ret = sgm41511_i2c_write_bits(ddata,
								SGM41511_REG_BATFET_DIS,
								SGM41511_REG_BATFET_DIS_MASK,
								SGM41511_REG_BATFET_DIS_SHIFT,
								EXIT_SHIP_MODE);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: exit shipmode fail\n", __func__);
		return -1;
	}

	return 0;
}

static int sgm41511_get_chip_state(struct sgm41511_driver_data *ddata,
							struct charger_state *state)
{
	int ret;
	u8 reg_value;

	ret = sgm41511_i2c_read(ddata, SGM41511_REG_08, &reg_value);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: read status reg fail\n", __func__);
		return -1;
	}
	state->charge_status = (reg_value & SGM41511_REG_CHRG_STAT_MASK)
								>> SGM41511_REG_CHRG_STAT_SHIFT;
	state->online = (reg_value & SGM41511_REG_VBUS_STAT_MASK)
							>> SGM41511_REG_VBUS_STAT_SHIFT;
	state->power_good = (reg_value & SGM41511_REG_PG_STAT_MASK)
							>> SGM41511_REG_PG_STAT_SHIFT;
	
	ret = sgm41511_i2c_read(ddata, SGM41511_REG_09, &reg_value);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: read fault reg fail\n", __func__);
		return -1;
	}
	state->charge_fault = (reg_value & SGM41511_REG_CHRG_FAULT_MASK)
								>> SGM41511_REG_CHRG_FAULT_SHIFT;
	state->bat_fault = (reg_value & SGM41511_REG_BAT_FAULT_MASK)
							>> SGM41511_REG_BAT_FAULT_SHIFT;
	state->ntc_fault = (reg_value & SGM41511_REG_NTC_FAULT_MASK)
							>> SGM41511_REG_NTC_FAULT_SHIFT;

	return 0;
}

static int check_chip_state_changed(struct sgm41511_driver_data *ddata,
							struct charger_state state)
{
	struct charger_state old_state = ddata->state;
	int change_flag = -1;

	if (old_state.charge_status != state.charge_status) {
		dev_info(ddata->dev, "charge status has change: %u\n", state.charge_status);
		change_flag = 0;
	}

	if (old_state.online != state.online) {
		dev_info(ddata->dev, "vbus status has change: %u\n", state.online);
		change_flag = 0;
	}

	if (old_state.power_good != state.power_good) {
		dev_info(ddata->dev, "PG status has change: %u\n", state.power_good);
		change_flag = 0;
	}

	if (old_state.charge_fault != state.charge_fault) {
		dev_info(ddata->dev, "charge fault has change: %u\n", state.charge_fault);
		change_flag = 0;
	}

	if (old_state.bat_fault != state.bat_fault) {
		dev_info(ddata->dev, "bat fault has change: %u\n", state.bat_fault);
		change_flag = 0;
	}

	if (old_state.ntc_fault != state.ntc_fault) {
		dev_info(ddata->dev, "ntc fault has change: %u\n", state.ntc_fault);
		change_flag = 0;
	}

	return change_flag;
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sgm41511_driver_data *ddata = g_sgm41511_data;
	unsigned int databuf[2] = {0, 0};

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		sgm41511_i2c_write(ddata, (unsigned char)databuf[0], (unsigned char)databuf[1]);

	return count;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct sgm41511_driver_data *ddata = g_sgm41511_data;
	ssize_t len = 0;
	u8 i = 0;
	u8 reg_val = 0;

	for (i = SGM41511_REG_00; i <= SGM41511_REG_0B; i++) {
		sgm41511_i2c_read(ddata, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%02x\n", i, reg_val);
	}

	return len;
}

static DEVICE_ATTR_RW(reg);

static struct attribute *sgm41511_attributes[] = {
	&dev_attr_reg.attr,
	NULL
};

static struct attribute_group sgm41511_attribute_group = {
	.attrs = sgm41511_attributes
};

static int sgm41511_start_charge(struct sgm41511_driver_data *ddata)
{
	int ret = 0;

	dev_info(ddata->dev, "start charge\n");
	gpio_direction_output(ddata->charge_gpio, 0);
	ret = sgm41511_set_iindpm(ddata, ddata->chg_cfg.iindpm);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: start charge fail\n", __func__);
		return -1;
	}

	ret = sgm41511_set_chg_config(ddata, START_CHARGE);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: start charge fail\n", __func__);
		return -1;
	}
	ddata->chg_cfg.chg_config = START_CHARGE;

	return 0;
}

static int sgm41511_stop_charge(struct sgm41511_driver_data *ddata)
{
	int ret = 0;

	dev_info(ddata->dev, "stop charge\n");
	gpio_direction_output(ddata->charge_gpio, 1);
	ret = sgm41511_set_chg_config(ddata, STOP_CHARGE);
	if (ret < 0) {
		dev_err(ddata->dev, "%s: stop charge fail\n", __func__);
		return -1;
	}
	ddata->chg_cfg.chg_config = STOP_CHARGE;

	return 0;
}

static int sgm41511_charger_get_property(struct power_supply *psy,
						 enum power_supply_property psp,
						 union power_supply_propval *val)
{
	struct sgm41511_driver_data *ddata = power_supply_get_drvdata(psy);
	struct charger_state state = ddata->state;
	struct charge_config chg_cfg = ddata->chg_cfg;

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			{
				if (!state.charge_status)
					val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
				else if (state.charge_status == 1 || state.charge_status == 2)
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
				else if (state.charge_status == 3)
					val->intval = POWER_SUPPLY_STATUS_FULL;
				else
					val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
				break;
			}
		case POWER_SUPPLY_PROP_ONLINE:
			{
				val->intval = state.power_good;
				break;
			}
			break;
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
			{
				val->intval = chg_cfg.ichg;
			}
			break;
		case POWER_SUPPLY_PROP_CHARGE_ENABLED:
			{
				val->intval = chg_cfg.chg_config;
				break;
			}
			break;
		case POWER_SUPPLY_PROP_SHIP_MODE:
			{
				val->intval = 0;
			}
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int sgm41511_charger_set_property(struct power_supply *psy,
								enum power_supply_property psp,
								const union power_supply_propval *val)
{
	struct sgm41511_driver_data *ddata = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_CHARGE_ENABLED:
			{
				dev_info(ddata->dev, "set charge enabled %d", val->intval);
				if (!val->intval) {
					ret = sgm41511_stop_charge(ddata);
					if (ret < 0) {
						dev_err(ddata->dev, "%s: stop charge fail\n", __func__);
						return -1;
					}
				} else if (val->intval == 1) {
					ret = sgm41511_start_charge(ddata);
					if (ret < 0) {
						dev_err(ddata->dev, "%s: start charge fail\n", __func__);
						return -1;
					}
				}

				break;
			}
			break;
		case POWER_SUPPLY_PROP_SHIP_MODE:
			{
				dev_info(ddata->dev, "set ship mode %d", val->intval);
				if (!val->intval) {
					ret = sgm41511_exit_shipmode(ddata);
					if (ret < 0) {
						dev_err(ddata->dev, "%s: exit shipmode fail\n", __func__);
						return -1;
					}
				} else if (val->intval == 1) {
					ret = sgm41511_enter_shipmode(ddata, 0);
					if (ret < 0) {
						dev_err(ddata->dev, "%s: enter shipmode fail\n", __func__);
						return -1;
					}
				} else if (val->intval == 2) {
					ret = sgm41511_enter_shipmode(ddata, 1);
					if (ret < 0) {
						dev_err(ddata->dev, "%s: enter shipmode fail\n", __func__);
						return -1;
					}
				}
			}
			break;
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
			{
				dev_info(ddata->dev, "set constant current %d", val->intval);
				ret = sgm41511_set_ichg(ddata, val->intval);
				if (ret < 0) {
					dev_err(ddata->dev, "%s: set ichg fail\n", __func__);
					return -1;
				}
			}
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static int sgm41511_charger_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		case POWER_SUPPLY_PROP_SHIP_MODE:
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
			return true;
		default:
			return false;
	}
}

static enum power_supply_property sgm41511_charger_property[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_SHIP_MODE
};

struct power_supply_desc sgm41511_charger_desc = {
	.name = "sgm41511_charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sgm41511_charger_property,
	.num_properties = ARRAY_SIZE(sgm41511_charger_property),
	.get_property = sgm41511_charger_get_property,
	.set_property = sgm41511_charger_set_property,
	.property_is_writeable = sgm41511_charger_property_is_writeable,
};

static int sgm41511_power_supply_init(struct sgm41511_driver_data *ddata)
{
	int ret = 0;

	struct power_supply_config psy_cfg = { .drv_data = ddata, };

	ddata->charger = power_supply_register(ddata->dev, &sgm41511_charger_desc,
						&psy_cfg);
	if (PTR_ERR_OR_ZERO(ddata->charger)) {
		dev_err(ddata->dev, "power_supply_register fail\n");
		return -1;
	}

	ret = sysfs_create_group(&ddata->charger->dev.kobj,
			&sgm41511_attribute_group);
	if (ret) {
		dev_err(ddata->dev, "create sysfs fail, ret: %d\n", ret);
		return -1;
	}

	return 0;
}

static irqreturn_t sgm41511_irq_thread(int irq, void *data)
{
	struct sgm41511_driver_data *ddata = data;
	struct charger_state state;
	int ret = 0;

	//printk(KERN_INFO "irq thread start\n");
	ret = sgm41511_get_chip_state(ddata, &state);
	if (ret < 0)
		return IRQ_HANDLED;
	
	ret = check_chip_state_changed(ddata, state);
	if (ret)
		return IRQ_HANDLED;

	ddata->state = state;
	power_supply_changed(ddata->charger);

	return IRQ_HANDLED;
}

static int sgm41511_interrupt_init(struct sgm41511_driver_data *ddata)
{
	int ret = 0;

	ddata->interrupt_gpio = of_get_named_gpio(ddata->dev->of_node, "interrupt-gpio", 0);
	if (ddata->interrupt_gpio < 0) {
		dev_err(ddata->dev, "get interrupt gpio fail\n");
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
						sgm41511_irq_thread, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"sgm41511-charger-irq", ddata);
	if (ret < 0) {
		dev_err(ddata->dev, "request thread irq failed, ret: %d\n", ret);
		return ret;
	}

	return 0;
}

static int sgm41511_charge_gpio_init(struct sgm41511_driver_data *ddata)
{
	int ret = 0;

	ddata->charge_gpio = of_get_named_gpio(ddata->dev->of_node, "charge-gpio", 0);
	if (ddata->charge_gpio < 0) {
		dev_err(ddata->dev, "get charge gpio fail\n");
		return -1;
	}

	ret = gpio_request(ddata->charge_gpio, "charge-gpio");
	if (ret < 0) {
		dev_err(ddata->dev, "%s: gpio_request fail\n", __func__);
		return ret;
	}

	gpio_direction_output(ddata->charge_gpio, 1);

	return 0;
}

static int sgm41511_hw_init(struct sgm41511_driver_data *ddata)
{
	int ret;

	ddata->chg_cfg.ichg = DEFAULT_CHG_CURRENT;
	ddata->chg_cfg.vreg = DEFAULT_CHG_VOLTAGE;
	ddata->chg_cfg.iterm = DEFAULT_TERM_CURRENT;
	ddata->chg_cfg.iprechg = DEFAULT_PRECHG_CURRENT;
	ddata->chg_cfg.sysvmin = DEFAULT_SYS_MIN;
	ddata->chg_cfg.iindpm = DEFAULT_IINDPM;
	ddata->chg_cfg.vindpm = DEFAULT_VINDPM;

	ret = sgm41511_set_watchdog(ddata, 0);
	if (ret < 0)
		return -1;

	ret = sgm41511_set_ichg(ddata, ddata->chg_cfg.ichg);
	if (ret < 0)
		return -1;
	
	ret = sgm41511_set_vchg(ddata, ddata->chg_cfg.vreg);
	if (ret < 0)
		return -1;
	
	ret = sgm41511_set_iterm(ddata, ddata->chg_cfg.iterm);
	if (ret < 0)
		return -1;

	ret = sgm41511_set_iprechg(ddata, ddata->chg_cfg.iprechg);
	if (ret < 0)
		return -1;

	ret = sgm41511_set_sysmin(ddata, ddata->chg_cfg.sysvmin);
	if (ret < 0)
		return -1;

	ret = sgm41511_set_vindpm(ddata, ddata->chg_cfg.vindpm);
	if (ret < 0)
		return -1;

	ret = sgm41511_get_chip_state(ddata, &ddata->state);
	if (ret < 0)
		return -1;

	return 0;
}

static int sgm41511_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct sgm41511_driver_data *ddata;
	int ret;

	printk(KERN_INFO "sgm41511 charger probe\n");
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	ddata = devm_kzalloc(&i2c->dev, sizeof(struct sgm41511_driver_data), GFP_KERNEL);
	if (!ddata) {
		dev_err(&i2c->dev, "devm_kzalloc failed\n");
		return -ENOMEM;
	}
	ddata->i2c = i2c;
	ddata->dev = &i2c->dev;

	i2c_set_clientdata(i2c, ddata);
	
	ret = sgm41511_charge_gpio_init(ddata);
	if (ret < 0) {
		dev_err(&i2c->dev, "charge gpio init fail\n");
		return -EINVAL;
	}

	ret = sgm41511_hw_init(ddata);
	if (ret < 0) {
		dev_err(&i2c->dev, "sgm41511_hw_init fail\n");
		return -EINVAL;
	}

	ret = sgm41511_power_supply_init(ddata);
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to register power supply\n");
		return -EINVAL;
	}

	ret = sgm41511_interrupt_init(ddata);
	if (ret < 0) {
		dev_err(&i2c->dev, "interrupt init fail\n");
		return -EINVAL;
	}
	g_sgm41511_data = ddata;

	return 0;
}

static int sgm41511_i2c_remove(struct i2c_client *i2c)
{
	struct sgm41511_driver_data *ddata = i2c_get_clientdata(i2c);

	power_supply_unregister(ddata->charger);

	return 0;
}

static const struct i2c_device_id sgm41511_i2c_id[] = {
	{ "sgm41511-charger", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sgm41511_i2c_id);

static const struct of_device_id sgm41511_charger_dt_match[] = {
	{ .compatible = "sgm,sgm41511" },
	{ },
};

static struct i2c_driver sgm41511_charger_driver = {
	.driver = {
		.name = "sgm41511-charger",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sgm41511_charger_dt_match),
	},
	.probe = sgm41511_i2c_probe,
	.remove = sgm41511_i2c_remove,
	.id_table = sgm41511_i2c_id,
};

module_i2c_driver(sgm41511_charger_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sgm41511 charge driver");
