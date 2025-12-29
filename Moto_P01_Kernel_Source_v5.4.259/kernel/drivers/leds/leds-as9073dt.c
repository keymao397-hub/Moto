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
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#define AS9073DT_I2C_RETRY_CNT  5
#define AS9073DT_I2C_RETRY_DELAY  1
#define AS9073DT_COLORS_COUNT    3
#define IS_PRESS_UP     0
#define IS_PRESS_DOWN   1
#define DEFAULT_POLL_MODE  0
#define DEFAULT_POLL_PERION  100
#define AS9073DT_KEY_CHANNEL_MAX  5

/************* reg ***************/
#define AS9073DT_REG_VERSION       0x25
#define AS9073DT_REG_KEY_STATUS    0x26
#define AS9073DT_REG_LED0          0x40
#define AS9073DT_REG_LED1          0x41
#define AS9073DT_REG_LED2          0x42
#define AS9073DT_REG_LED3          0x43
#define AS9073DT_REG_LED4          0x44
#define AS9073DT_REG_LED5          0x45
#define AS9073DT_REG_LED6          0x46
#define AS9073DT_REG_LED7          0x47
#define AS9073DT_REG_LED8          0x48
#define AS9073DT_REG_LED9          0x49
#define AS9073DT_REG_LED10         0x4A
#define AS9073DT_REG_LED11         0x4B
#define AS9073DT_REG_LED12         0x4C
#define AS9073DT_REG_LED13         0x4D
#define AS9073DT_REG_LED14         0x4E
/************* reg ***************/

struct led_io {
	u32 r_io;
	u32 g_io;
	u32 b_io;
};

struct led_colors {
	u32 red;
	u32 green;
	u32 blue;
};

struct led_driver {
	int led_nums;
	struct led_classdev cdev;
	struct led_io *leds;
	struct led_colors *colors;
};

struct key_desc {
	u32 chan;
	u32 type;
	u32 code;
	const char *name;
	u8 current_status;
};

struct key_driver {
	u32 key_size;
	int use_irq;/* 1:irq mode ; 0:polling mode */
	int irq;
	int poll_period;
	struct input_dev *input_dev;
	int interrupt_gpio;
	struct timer_list polling_timer;
	struct work_struct work;
	struct key_desc *key;
	struct key_desc *current_key;
};

struct as9073dt_driver_data {
	struct i2c_client *i2c;
	struct device *dev;
	struct led_driver led_data;
	struct key_driver key_data;
	int soft_onoff;
	struct mutex lock;
};

static u8 led_reg_table[] = {
	AS9073DT_REG_LED0,
	AS9073DT_REG_LED1,
	AS9073DT_REG_LED2,
	AS9073DT_REG_LED3,
	AS9073DT_REG_LED4,
	AS9073DT_REG_LED5,
	AS9073DT_REG_LED6,
	AS9073DT_REG_LED7,
	AS9073DT_REG_LED8,
	AS9073DT_REG_LED9,
	AS9073DT_REG_LED10,
	AS9073DT_REG_LED11,
	AS9073DT_REG_LED12,
	AS9073DT_REG_LED13,
	AS9073DT_REG_LED14
};

//
static struct as9073dt_driver_data *as9073dt_data = NULL;

static int as9073dt_i2c_write(struct as9073dt_driver_data *ddata, u8 reg_addr, u8 reg_data)
{
	int ret = -1;
	int cnt = 0;

	while (cnt < AS9073DT_I2C_RETRY_CNT) {
		ret = i2c_smbus_write_byte_data(ddata->i2c, reg_addr, reg_data);
		if (ret < 0)
			dev_err(ddata->dev, "%s: i2c_write cnt=%d error=%d\n",
					__func__, cnt, ret);
		else
			break;
		cnt++;
		msleep(AS9073DT_I2C_RETRY_DELAY);
	}

	return ret;
}

static int as9073dt_i2c_read(struct as9073dt_driver_data *ddata, u8 reg_addr, u8 *reg_data)
{
	int ret = -1;
	int cnt = 0;

	while (cnt < AS9073DT_I2C_RETRY_CNT) {
		ret = i2c_smbus_read_byte_data(ddata->i2c, reg_addr);
		if (ret < 0) {
			dev_err(ddata->dev, "%s: i2c_read cnt=%d error=%d\n",
					__func__, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(AS9073DT_I2C_RETRY_DELAY);
	}

	return ret;
}

static int as9073dt_read_version(struct as9073dt_driver_data *ddata, u8 *version)
{
	u8 value = 0;
	int ret;

	ret = as9073dt_i2c_read(ddata, AS9073DT_REG_VERSION, &value);
	if (ret < 0) {
		dev_err(ddata->dev, "%s fail\n", __func__);
		return -1;
	}

	*version = value;
	return 0;
}

static int as9073dt_read_key_status(struct as9073dt_driver_data *ddata, u8 *data)
{
	u8 status = 0;
	int ret;

	ret = as9073dt_i2c_read(ddata, AS9073DT_REG_KEY_STATUS, &status);
	if (ret < 0) {
		dev_err(ddata->dev, "read as9073dt key status fail\n");
		return -1;
	}

	*data = status;
	return 0;
}

static int as9073dt_write_led_color(struct as9073dt_driver_data *ddata, u8 led_index, u32 color)
{
	u8 r_io_reg, g_io_reg, b_io_reg;
	u8 red, green, blue;
	struct led_driver *led_data = &ddata->led_data;

	if (led_index >= led_data->led_nums) {
		dev_err(ddata->dev, "led index is invalid\n");
		return -1;
	}

	mutex_lock(&ddata->lock);
	if (ddata->soft_onoff != 0) {
		led_data->colors[led_index].red = ((color >> 16) & 0xFF);
		led_data->colors[led_index].green = ((color >> 8) & 0xFF);
		led_data->colors[led_index].blue = (color & 0xFF);
	}
	mutex_unlock(&ddata->lock);
	red = 0xFF - ((color >> 16) & 0xFF);
	green = 0xFF - ((color >> 8) & 0xFF);
	blue = 0xFF - (color & 0xFF);
	r_io_reg = led_reg_table[led_data->leds[led_index].r_io];
	g_io_reg = led_reg_table[led_data->leds[led_index].g_io];
	b_io_reg = led_reg_table[led_data->leds[led_index].b_io];
	as9073dt_i2c_write(ddata, r_io_reg, red);
	as9073dt_i2c_write(ddata, g_io_reg, green);
	as9073dt_i2c_write(ddata, b_io_reg, blue);

	return 0;
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct as9073dt_driver_data *ddata = as9073dt_data;
	unsigned int databuf[2] = {0, 0};

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		as9073dt_i2c_write(ddata, (unsigned char)databuf[0], (unsigned char)databuf[1]);

	return count;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct as9073dt_driver_data *ddata = as9073dt_data;
	ssize_t len = 0;
	u8 i = 0;
	u8 reg_val = 0;

	as9073dt_i2c_read(ddata, AS9073DT_REG_KEY_STATUS, &reg_val);
	len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%02x\n", AS9073DT_REG_KEY_STATUS, reg_val);

	for (i = 0; i < ARRAY_SIZE(led_reg_table); i++) {
		as9073dt_i2c_read(ddata, led_reg_table[i], &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%02x\n", led_reg_table[i], reg_val);
	}

	return len;
}

static ssize_t led_nums_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct as9073dt_driver_data *ddata = as9073dt_data;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", ddata->led_data.led_nums);

	return len;
}

static ssize_t fw_version_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct as9073dt_driver_data *ddata = as9073dt_data;
	ssize_t len = 0;
	u8 version = 0;

	as9073dt_read_version(ddata, &version);
	len += snprintf(buf + len, PAGE_SIZE - len, "0x%X\n", version);

	return len;
}

static ssize_t single_led_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	u32 ledid, color;
	int ret;
	struct as9073dt_driver_data *ddata = as9073dt_data;

	ret = sscanf(buf, "%d %x", &ledid, &color);
	if (ret != 2) {
		dev_err(ddata->dev, "set led colors fail!\n");
		return count;
	}
	mutex_lock(&ddata->lock);
	if (ddata->soft_onoff == 0 && ledid != 1) {
		mutex_unlock(&ddata->lock);
		return count;
	}
	mutex_unlock(&ddata->lock);
	as9073dt_write_led_color(ddata, ledid, color);

	return count;
}

static ssize_t all_led_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	u32 i, color;
	int ret;
	struct as9073dt_driver_data *ddata = as9073dt_data;

	mutex_lock(&ddata->lock);
	if (ddata->soft_onoff == 0) {
		mutex_unlock(&ddata->lock);
		return count;
	}
	mutex_unlock(&ddata->lock);

	ret = sscanf(buf, "%x", &color);
	if (ret != 1) {
		dev_err(ddata->dev, "set all led fail!\n");
		return count;
	}
	for (i = 0; i < ddata->led_data.led_nums; i++)
		as9073dt_write_led_color(ddata, i, color);

	return count;
}

static ssize_t sw_onoff_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct as9073dt_driver_data *ddata = as9073dt_data;
	int onoff = 0;
	u32 i, color;

	mutex_lock(&ddata->lock);
	if (sscanf(buf, "%du", &onoff) == 1)
		ddata->soft_onoff = onoff;
	mutex_unlock(&ddata->lock);

	if (ddata->soft_onoff == 0) {
		for (i = 0; i < ddata->led_data.led_nums; i++)
			as9073dt_write_led_color(ddata, i, 0x0);
	} else {
		for (i = 0; i < ddata->led_data.led_nums; i++) {
			color = (ddata->led_data.colors[i].red << 16) | \
					(ddata->led_data.colors[i].green << 8) | \
					(ddata->led_data.colors[i].blue);
			as9073dt_write_led_color(ddata, i, color);
		}
	}

	return count;
}

static ssize_t sw_onoff_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct as9073dt_driver_data *ddata = as9073dt_data;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", ddata->soft_onoff);

	return len;
}

static DEVICE_ATTR_RW(reg);
static DEVICE_ATTR_RO(led_nums);
static DEVICE_ATTR_RO(fw_version);
static DEVICE_ATTR_WO(single_led);
static DEVICE_ATTR_WO(all_led);
static DEVICE_ATTR_RW(sw_onoff);

static struct attribute *as9073dt_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_led_nums.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_single_led.attr,
	&dev_attr_all_led.attr,
	&dev_attr_sw_onoff.attr,
	NULL
};

static struct attribute_group as9073dt_attribute_group = {
	.attrs = as9073dt_attributes
};

static int as9073dt_led_parse_dt(struct as9073dt_driver_data *ddata)
{
	struct device_node *temp;
	u32 colors[AS9073DT_COLORS_COUNT];
	u8 i = 0;
	int ret;
	struct led_driver *led_data = &ddata->led_data;

	for_each_child_of_node(ddata->dev->of_node, temp) {
		ret = of_property_read_u32_array(temp, "default_colors",
						colors, AS9073DT_COLORS_COUNT);
		if (ret < 0) {
			dev_err(ddata->dev,
				"read default colors fail, ret = %d\n", ret);
			return -1;
		}

		ret = of_property_read_u32(temp, "r_io", &led_data->leds[i].r_io);
		if (ret < 0 || led_data->leds[i].r_io >= ARRAY_SIZE(led_reg_table)) {
			dev_err(ddata->dev,
				"read r_io fail, ret = %d\n", ret);
			return -1;
		}

		ret = of_property_read_u32(temp, "g_io", &led_data->leds[i].g_io);
		if (ret < 0 || led_data->leds[i].g_io >= ARRAY_SIZE(led_reg_table)) {
			dev_err(ddata->dev,
				"read g_io fail, ret = %d\n", ret);
			return -1;
		}

		ret = of_property_read_u32(temp, "b_io", &led_data->leds[i].b_io);
		if (ret < 0 || led_data->leds[i].b_io >= ARRAY_SIZE(led_reg_table)) {
			dev_err(ddata->dev,
				"read b_io fail, ret = %d\n", ret);
			return -1;
		}

		led_data->colors[i].red = colors[0];
		led_data->colors[i].green = colors[1];
		led_data->colors[i].blue = colors[2];
		i++;
	}

	return 0;
}

static int as9073dt_led_driver_init(struct as9073dt_driver_data *ddata)
{
	int ret = 0;
	struct led_driver *led_data = &ddata->led_data;

	led_data->led_nums = device_get_child_node_count(ddata->dev);
	if (led_data->led_nums <= 0)
		return -ENOMEM;
	led_data->leds = devm_kzalloc(ddata->dev,
			(led_data->led_nums) * sizeof(struct led_io), GFP_KERNEL);
	if (!(led_data->leds))
		return -ENOMEM;
	led_data->colors = devm_kzalloc(ddata->dev,
			(led_data->led_nums) * sizeof(struct led_colors), GFP_KERNEL);
	if (!(led_data->colors))
		return -ENOMEM;

	ret = as9073dt_led_parse_dt(ddata);
	if (ret < 0)
		return -EINVAL;

	//led class
	led_data->cdev.name = "as9073dt-leds";
	led_data->cdev.brightness = 0;
	led_data->cdev.max_brightness = 255;
	ret = led_classdev_register(ddata->dev, &led_data->cdev);
	if (ret) {
		dev_err(ddata->dev,
			"register led classdev fail, ret=%d\n", ret);
		return -1;
	}
	ret = sysfs_create_group(&led_data->cdev.dev->kobj,
			&as9073dt_attribute_group);
	if (ret) {
		dev_err(ddata->dev, "create led sysfs fail, ret: %d\n", ret);
		led_classdev_unregister(&led_data->cdev);
		return -1;
	}

	return 0;
}

static void report_key_code(struct as9073dt_driver_data *ddata, struct key_driver *key_data, int status)
{
	struct key_desc *key = key_data->current_key;

	mutex_lock(&ddata->lock);
	key->current_status = status;
	if (key->current_status == IS_PRESS_UP) {
		if (ddata->soft_onoff == 0) {
			input_event(key_data->input_dev, key->type,
					KEY_WAKEUP, 0);
			dev_info(&key_data->input_dev->dev,
					"key %d up.\n", KEY_WAKEUP);
		} else {
			input_event(key_data->input_dev, key->type,
					key->code, 0);
			dev_info(&key_data->input_dev->dev,
					"key %d up.\n", key->code);
		}
	} else {
		if (ddata->soft_onoff == 0) {
			input_event(key_data->input_dev, key->type,
					KEY_WAKEUP, 1);
			dev_info(&key_data->input_dev->dev,
					"key %d down.\n", KEY_WAKEUP);
		} else {
			input_event(key_data->input_dev, key->type,
					key->code, 1);
			dev_info(&key_data->input_dev->dev,
					"key %d down.\n", key->code);
		}
	}
	mutex_unlock(&ddata->lock);
	input_sync(key_data->input_dev);
}

static void as9073dt_work_handler(struct work_struct *work)
{
	int i, ret;
	u8 reg_value = 0;
	u8 status = 0;
	struct as9073dt_driver_data *ddata = as9073dt_data;
	struct key_driver *key_data = &ddata->key_data;

	ret = as9073dt_read_key_status(ddata, &reg_value);
	if (ret < 0)
		return;

	if (key_data->use_irq == DEFAULT_POLL_MODE) {
		for (i = 0; i < key_data->key_size; i++) {
			status = (reg_value >> key_data->key[i].chan) & 0x01;

			if (key_data->key[i].current_status != status) {
				key_data->current_key = &key_data->key[i];
				report_key_code(ddata, key_data, status);
			}
		}
		mod_timer(&key_data->polling_timer,
			  jiffies + msecs_to_jiffies(key_data->poll_period));
	} else {
		for (i = 0; i < key_data->key_size; i++) {
			status = (reg_value >> key_data->key[i].chan) & 0x01;

			if (key_data->key[i].current_status != status) {
				key_data->current_key = &key_data->key[i];
				report_key_code(ddata, key_data, status);
			}
		}
	}
}

static void polling_timer_handler(struct timer_list *t)
{
	struct as9073dt_driver_data *ddata = as9073dt_data;
	struct key_driver *key_data = &ddata->key_data;

	schedule_work(&key_data->work);
}

static irqreturn_t as9073dt_irq_thread(int irq, void *data)
{
	struct as9073dt_driver_data *ddata = as9073dt_data;
	struct key_driver *key_data = &ddata->key_data;

	//dev_info(ddata->dev, "as9073dt irq\n");
	schedule_work(&key_data->work);
	return IRQ_HANDLED;
}

static int as9073dt_interrupt_init(struct as9073dt_driver_data *ddata)
{
	struct key_driver *key_data = &ddata->key_data;
	int ret = 0;

	key_data->interrupt_gpio = of_get_named_gpio(ddata->dev->of_node, "interrupt-gpio", 0);
	if (key_data->interrupt_gpio < 0) {
		dev_err(ddata->dev, "%s: get interrupt gpio fail\n", __func__);
		return -1;
	}

	ret = gpio_request(key_data->interrupt_gpio, "interrupt-gpio");
	if (ret < 0) {
		dev_err(ddata->dev, "%s: gpio_request fail\n", __func__);
		return ret;
	}

	ret = gpio_direction_input(key_data->interrupt_gpio);
    if (ret < 0) {
        dev_err(ddata->dev, "%s: set interrupt gpio input fail\n", __func__);
        return ret;
    }

	key_data->irq = gpio_to_irq(key_data->interrupt_gpio);
	ret = devm_request_threaded_irq(ddata->dev, key_data->irq, NULL,
						as9073dt_irq_thread, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						"as9073dt-irq", ddata);
	if (ret < 0) {
		dev_err(ddata->dev, "request thread irq failed, ret: %d\n", ret);
		return ret;
	}

	return 0;
}

static int as9073dt_key_driver_init(struct as9073dt_driver_data *ddata)
{
	int ret = 0;
	int i = 0;
	struct input_dev *input_dev;
	struct key_driver *key_data = &ddata->key_data;

	ret = of_property_read_u32(ddata->dev->of_node, "detect_mode", &key_data->use_irq);
	if (ret)
		/* The default mode is polling. */
		key_data->use_irq = DEFAULT_POLL_MODE;
	ret = of_property_read_u32(ddata->dev->of_node, "poll_period", &key_data->poll_period);
	if (ret)
		/* he default scan period is 50. */
		key_data->poll_period = DEFAULT_POLL_PERION;
	ret = of_property_read_u32(ddata->dev->of_node, "key_num", &key_data->key_size);
	if (ret) {
		dev_err(ddata->dev, "failed to get key_num!\n");
		return -EINVAL;
	}
	key_data->key = devm_kzalloc(ddata->dev, (key_data->key_size) * sizeof(struct key_desc),
								GFP_KERNEL);
	if (!(key_data->key))
		return -EINVAL;
	for (i = 0; i < key_data->key_size; i++) {
		key_data->key[i].current_status = IS_PRESS_UP;
		ret = of_property_read_u32_index(ddata->dev->of_node, "key_type",
									i, &key_data->key[i].type);
		if (ret)
			key_data->key[i].type = EV_KEY;

		ret = of_property_read_u32_index(ddata->dev->of_node, "key_code",
									i, &key_data->key[i].code);
		if (ret < 0) {
			dev_err(ddata->dev, "find key_code=%d fail\n", i);
			return -EINVAL;
		}

		ret = of_property_read_u32_index(ddata->dev->of_node, "key_chan",
									i, &key_data->key[i].chan);
		if (ret < 0) {
			dev_err(ddata->dev, "find key_chan=%d fail\n", i);
			return -EINVAL;
		}
		if (key_data->key[i].chan > AS9073DT_KEY_CHANNEL_MAX) {
			dev_err(ddata->dev, "key_chan=%d is more than max value\n", i);
			return -EINVAL;
		}

		ret = of_property_read_string_index(ddata->dev->of_node, "key_name",
									i, &key_data->key[i].name);
		if (ret < 0) {
			dev_err(ddata->dev, "find key_name=%d fail\n", i);
			return -EINVAL;
		}
	}

	/* input */
	input_dev = input_allocate_device();
	if (!input_dev)
		return -EINVAL;
	for (i = 0; i < key_data->key_size; i++) {
		input_set_capability(input_dev, key_data->key[i].type, key_data->key[i].code);

		dev_dbg(ddata->dev, "%s key(%d) type(0x%x) registered.\n",
			 key_data->key[i].name, key_data->key[i].code,
			 key_data->key[i].type);
	}
	input_set_capability(input_dev, key_data->key[0].type, KEY_WAKEUP);
	input_dev->name = "touch_keypad";
	input_dev->phys = "touch_keypad/input0";
	input_dev->dev.parent = ddata->dev;
	input_dev->id.bustype = BUS_ISA;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->rep[REP_DELAY] = 0xffffffff;
	input_dev->rep[REP_PERIOD] = 0xffffffff;
	input_dev->keycodesize = sizeof(unsigned short);
	input_dev->keycodemax = 0x1ff;
	key_data->input_dev = input_dev;
	ret = input_register_device(key_data->input_dev);
	if (ret < 0) {
		input_free_device(key_data->input_dev);
		return -EINVAL;
	}

	INIT_WORK(&key_data->work, as9073dt_work_handler);
	timer_setup(&key_data->polling_timer, polling_timer_handler, 0);
	if (key_data->use_irq == DEFAULT_POLL_MODE) {
		mod_timer(&key_data->polling_timer,
				jiffies + msecs_to_jiffies(key_data->poll_period));
	} else {
		ret = as9073dt_interrupt_init(ddata);
		if (ret) {
			input_free_device(key_data->input_dev);
			del_timer(&key_data->polling_timer);
			return -EINVAL;
		}
	}

	return 0;
}

static int as9073dt_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct as9073dt_driver_data *ddata;
	u8 version = 0;
	u8 reg_val = 0;
	int ret;

	dev_info(&i2c->dev, "as9073dt probe\n");
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	ddata = devm_kzalloc(&i2c->dev, sizeof(struct as9073dt_driver_data), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->i2c = i2c;
	ddata->dev = &i2c->dev;
	ddata->soft_onoff = 1;
	i2c_set_clientdata(i2c, ddata);
	as9073dt_data = ddata;

	ret = as9073dt_read_version(ddata, &version);
	if (ret < 0)
		return -1;

	dev_info(ddata->dev, "as9073dt fw version: 0x%x\n", version);
	as9073dt_i2c_read(ddata, AS9073DT_REG_KEY_STATUS, &reg_val);
	dev_info(ddata->dev, "key status: 0x%x\n", reg_val);
	mutex_init(&ddata->lock);
	ret = as9073dt_led_driver_init(ddata);
	if (ret < 0) {
		dev_err(ddata->dev, "led driver init failed\n");
		return -1;
	}

	ret = as9073dt_key_driver_init(ddata);
	if (ret < 0) {
		led_classdev_unregister(&ddata->led_data.cdev);
		dev_err(ddata->dev, "key driver init failed\n");
		return -1;
	}

	return 0;
}

static int as9073dt_i2c_remove(struct i2c_client *i2c)
{
	struct as9073dt_driver_data *ddata = i2c_get_clientdata(i2c);

	mutex_destroy(&ddata->lock);
	led_classdev_unregister(&ddata->led_data.cdev);
	return 0;
}

static const struct i2c_device_id as9073dt_i2c_id[] = {
	{ "as9073dt-leds", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, as9073dt_i2c_id);

static const struct of_device_id as9073dt_leds_dt_match[] = {
	{ .compatible = "as9073dt_leds" },
	{ },
};

static struct i2c_driver as9073dt_leds_driver = {
	.driver = {
		.name = "as9073dt-leds",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(as9073dt_leds_dt_match),
	},
	.probe = as9073dt_i2c_probe,
	.remove = as9073dt_i2c_remove,
	.id_table = as9073dt_i2c_id,
};

module_i2c_driver(as9073dt_leds_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A1semi AS9073DT led driver");
MODULE_ALIAS("platform:as9073dt-leds");
