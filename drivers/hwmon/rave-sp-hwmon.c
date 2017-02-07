/*
 * zii-pic-hwmon.c - HWMON driver for Zodiac Inflight Innovations PIC sensors
 *
 * Copyright (C) 2017 Nikita Yushchenko <nikita.yoush@cogentembedded.com>
 *
 * based on work by Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

/* #define DEBUG */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <linux/rave-sp.h>

enum rave_sp_sensor {
	RAVE_SP_SENSOR_FIRST,
	RAVE_SP_SENSOR_28V = RAVE_SP_SENSOR_FIRST,
	RAVE_SP_SENSOR_12V,
	RAVE_SP_SENSOR_5V,
	RAVE_SP_SENSOR_3V3,
	RAVE_SP_SENSOR_RMB_3V3_PMIC,
	RAVE_SP_SENSOR_RMB_3V3_MCU,
	RAVE_SP_SENSOR_RMB_5V_MAIN,
	RAVE_SP_SENSOR_RMB_12V_MAIN,
	RAVE_SP_SENSOR_RMB_28V_FIL,
	RAVE_SP_SENSOR_RMB_28V_HOTSWAP,
	RAVE_SP_SENSOR_DEB_1V8,
	RAVE_SP_SENSOR_DEB_3V3,
	RAVE_SP_SENSOR_DEB_28V_DEB,
	RAVE_SP_SENSOR_DEB_28V_RDU,
	RAVE_SP_SENSOR_TEMPERATURE,
	RAVE_SP_SENSOR_TEMPERATURE_2,
	RAVE_SP_SENSOR_BACKLIGHT_CURRENT,
	RAVE_SP_SENSOR_RMB_28V_CURRENT,
	RAVE_SP_SENSORS_COUNT
};

static const char * const input_names[] = {
	[RAVE_SP_SENSOR_28V]			= "28V",
	[RAVE_SP_SENSOR_12V]			= "12V",
	[RAVE_SP_SENSOR_5V]			= "5V",
	[RAVE_SP_SENSOR_3V3]			= "3V3",
	[RAVE_SP_SENSOR_RMB_3V3_PMIC]		= "RMB_3V3_PMIC",
	[RAVE_SP_SENSOR_RMB_3V3_MCU]		= "RMB_3V3_MCU",
	[RAVE_SP_SENSOR_RMB_5V_MAIN]		= "RMB_5V_MAIN",
	[RAVE_SP_SENSOR_RMB_12V_MAIN]		= "RMB_12V_MAIN",
	[RAVE_SP_SENSOR_RMB_28V_FIL]		= "RMB_28V_FIL",
	[RAVE_SP_SENSOR_RMB_28V_HOTSWAP]	= "RMB_28V_HOTSWAP",
	[RAVE_SP_SENSOR_DEB_1V8]		= "DEB_1V8",
	[RAVE_SP_SENSOR_DEB_3V3]		= "DEB_3V3",
	[RAVE_SP_SENSOR_DEB_28V_DEB]		= "DEB_28V_DEB",
	[RAVE_SP_SENSOR_DEB_28V_RDU]		= "DEB_28V_RDU",
	[RAVE_SP_SENSOR_TEMPERATURE]		= "TEMPERATURE",
	[RAVE_SP_SENSOR_TEMPERATURE_2]		= "TEMPERATURE_2",
	[RAVE_SP_SENSOR_BACKLIGHT_CURRENT]	= "BACKLIGHT_CURRENT",
	[RAVE_SP_SENSOR_RMB_28V_CURRENT]	= "RMB_28V_CURRENT"
};

struct rave_sp_hwmon {
	struct rave_sp		*zp;
	struct attribute	*attrs[];
};

static int f88_to_1000x(u8 *data)
{
	return (data[1] * 256 + data[0]) * 1000 / 256;
}

static int halfdg_to_mdg(u8 *data)
{
	return (data[1] * 256 + data[0]) * 500;
}

#define CMD_RDU2_GET_VOLTAGE	0x22

#define CMD_RDU2_GET_CURRENT	0x23

#define CMD_RDU2_GET_TEMPERATURE	0x24

static int rdu2_read_sensor(struct rave_sp_hwmon *hwmon,
		enum rave_sp_sensor sensor)
{
	u8 cmd[3], rsp[2];
	int ret;

	/* Voltage */
	if (sensor >= RAVE_SP_SENSOR_RMB_3V3_PMIC &&
	    sensor <= RAVE_SP_SENSOR_DEB_28V_RDU) {
		cmd[0] = CMD_RDU2_GET_VOLTAGE;
		cmd[1] = 0;
		cmd[2] = sensor - RAVE_SP_SENSOR_RMB_3V3_PMIC;
		ret = rave_sp_exec(hwmon->zp,
				   cmd, sizeof(cmd),
				   rsp, sizeof(rsp));
		if (ret)
			return ret;

		return f88_to_1000x(rsp);
	}

	/* Current */
	if (sensor == RAVE_SP_SENSOR_RMB_28V_CURRENT) {
		cmd[0] = CMD_RDU2_GET_CURRENT;
		cmd[1] = 0;
		cmd[2] = 0;
		ret = rave_sp_exec(hwmon->zp,
				   cmd, sizeof(cmd),
				   rsp, sizeof(rsp));
		if (ret)
			return ret;

		return f88_to_1000x(rsp);
	}

	/* Temperature */
	if (sensor >= RAVE_SP_SENSOR_TEMPERATURE &&
	    sensor <= RAVE_SP_SENSOR_TEMPERATURE_2) {
		cmd[0] = CMD_RDU2_GET_TEMPERATURE;
		cmd[1] = 0;
		cmd[2] = sensor - RAVE_SP_SENSOR_TEMPERATURE;
		ret = rave_sp_exec(hwmon->zp,
				   cmd, sizeof(cmd),
				   rsp, sizeof(rsp));
		if (ret)
			return ret;

		/* le16, 0.5 degree per int => millidegrees */
		return halfdg_to_mdg(rsp);
	}

	return -EINVAL;
}

#define CMD_RDU1_GET_STATUS	0xA0

static int rdu1_read_sensor(struct rave_sp_hwmon *hwmon,
		enum rave_sp_sensor sensor)
{
	struct {
		u8 _pad[22];
		u8 t1[2];
		u8 t2[2];
		u8 bk[3];
		u8 _pad2[2];
		u8 v[2];
	} __packed rsp;

	u8 cmd[] = { CMD_RDU1_GET_STATUS, 0 };

	int ret = rave_sp_exec(hwmon->zp,
			       cmd, sizeof(cmd),
			       &rsp, sizeof(rsp));
	if (ret)
		return ret;

	switch (sensor) {
	case RAVE_SP_SENSOR_28V:
		return f88_to_1000x(rsp.v);
	case RAVE_SP_SENSOR_TEMPERATURE:
		return halfdg_to_mdg(rsp.t1);
	case RAVE_SP_SENSOR_TEMPERATURE_2:
		return halfdg_to_mdg(rsp.t2);
	case RAVE_SP_SENSOR_BACKLIGHT_CURRENT:
		return rsp.bk[1] + 256 * rsp.bk[2];
	default:
		return -EINVAL;
	}
}

#define OCMD_GET_28V		0x1A

#define OCMD_GET_12V		0x2C

#define OCMD_GET_5V		0x2E

#define OCMD_GET_3V3		0x2F

#define OCMD_GET_TERMERATURE	0x19

static int old_read_sensor(struct rave_sp_hwmon *hwmon,
		enum rave_sp_sensor sensor)
{
	u8 cmd[2];
	u8 rsp[2];
	int ret;

	switch (sensor) {
	case RAVE_SP_SENSOR_28V:
		cmd[0] = OCMD_GET_28V;
		ret = rave_sp_exec(hwmon->zp,
				   cmd, sizeof(cmd),
				   rsp, sizeof(rsp));
		if (ret)
			return ret;
		else
			return f88_to_1000x(rsp);
	case RAVE_SP_SENSOR_12V:
		cmd[0] = OCMD_GET_12V;
		ret = rave_sp_exec(hwmon->zp,
				   cmd, sizeof(cmd),
				   rsp, sizeof(rsp));
		if (ret)
			return ret;
		else
			return f88_to_1000x(rsp);
	case RAVE_SP_SENSOR_5V:
		cmd[0] = OCMD_GET_5V;
		ret = rave_sp_exec(hwmon->zp,
				   cmd, sizeof(cmd),
				   rsp, sizeof(rsp));
		if (ret)
			return ret;
		else
			return f88_to_1000x(rsp);
	case RAVE_SP_SENSOR_3V3:
		cmd[0] = OCMD_GET_3V3;
		ret = rave_sp_exec(hwmon->zp,
				   cmd, sizeof(cmd),
				   rsp, sizeof(rsp));
		if (ret)
			return ret;
		else
			return f88_to_1000x(rsp);
	case RAVE_SP_SENSOR_TEMPERATURE:
		cmd[0] = OCMD_GET_TERMERATURE;
		ret = rave_sp_exec(hwmon->zp,
				   cmd, sizeof(cmd),
				   rsp, sizeof(rsp));
		if (ret)
			return ret;
		else
			return halfdg_to_mdg(rsp);
	default:
		return -EINVAL;
	}
}

static ssize_t rave_sp_read_sensor(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rave_sp_hwmon *hwmon = dev_get_drvdata(dev);
	int idx = to_sensor_dev_attr(attr)->index;
	int ret;
#if 0
	if (hwmon->zp->hw_id == RAVE_SP_HW_ID_RDU2)
		ret = rdu2_read_sensor(hwmon, idx);
	else if (hwmon->zp->hw_id == RAVE_SP_HW_ID_RDU1)
		ret = rdu1_read_sensor(hwmon, idx);
	else
		ret = old_read_sensor(hwmon, idx);
#else
	ret = -ENOTSUPP;
#endif
	if (ret < 0)
		return ret;
	else
		return sprintf(buf, "%d\n", ret);
}

static ssize_t rave_sp_show_label(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	int idx = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%s\n", input_names[idx]);
}

/* Voltage sensors */
static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_28V);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_12V);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_5V);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_3V3);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_RMB_3V3_PMIC);
static SENSOR_DEVICE_ATTR(in5_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_RMB_3V3_MCU);
static SENSOR_DEVICE_ATTR(in6_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_RMB_5V_MAIN);
static SENSOR_DEVICE_ATTR(in7_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_RMB_12V_MAIN);
static SENSOR_DEVICE_ATTR(in8_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_RMB_28V_FIL);
static SENSOR_DEVICE_ATTR(in9_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_RMB_28V_HOTSWAP);
static SENSOR_DEVICE_ATTR(in10_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_DEB_1V8);
static SENSOR_DEVICE_ATTR(in11_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_DEB_3V3);
static SENSOR_DEVICE_ATTR(in12_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_DEB_28V_DEB);
static SENSOR_DEVICE_ATTR(in13_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_DEB_28V_RDU);

/* Voltage sensor labels */
static SENSOR_DEVICE_ATTR(in0_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_28V);
static SENSOR_DEVICE_ATTR(in1_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_12V);
static SENSOR_DEVICE_ATTR(in2_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_5V);
static SENSOR_DEVICE_ATTR(in3_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_3V3);
static SENSOR_DEVICE_ATTR(in4_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_RMB_3V3_PMIC);
static SENSOR_DEVICE_ATTR(in5_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_RMB_3V3_MCU);
static SENSOR_DEVICE_ATTR(in6_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_RMB_5V_MAIN);
static SENSOR_DEVICE_ATTR(in7_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_RMB_12V_MAIN);
static SENSOR_DEVICE_ATTR(in8_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_RMB_28V_FIL);
static SENSOR_DEVICE_ATTR(in9_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_RMB_28V_HOTSWAP);
static SENSOR_DEVICE_ATTR(in10_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_DEB_1V8);
static SENSOR_DEVICE_ATTR(in11_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_DEB_3V3);
static SENSOR_DEVICE_ATTR(in12_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_DEB_28V_DEB);
static SENSOR_DEVICE_ATTR(in13_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_DEB_28V_RDU);

/* Temperature sensors */
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_TEMPERATURE);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_TEMPERATURE_2);

static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_TEMPERATURE);
static SENSOR_DEVICE_ATTR(temp2_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_TEMPERATURE_2);

/* Current sensors */
static SENSOR_DEVICE_ATTR(curr1_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_BACKLIGHT_CURRENT);
static SENSOR_DEVICE_ATTR(curr2_input, S_IRUGO, rave_sp_read_sensor,
			NULL, RAVE_SP_SENSOR_RMB_28V_CURRENT);
static SENSOR_DEVICE_ATTR(curr1_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_BACKLIGHT_CURRENT);
static SENSOR_DEVICE_ATTR(curr2_label, S_IRUGO, rave_sp_show_label,
			NULL, RAVE_SP_SENSOR_RMB_28V_CURRENT);

static struct attribute *rave_sp_hwmon_sensors[] = {
	[RAVE_SP_SENSOR_28V]	= &sensor_dev_attr_in0_input.dev_attr.attr,
	[RAVE_SP_SENSOR_12V]	= &sensor_dev_attr_in1_input.dev_attr.attr,
	[RAVE_SP_SENSOR_5V]	= &sensor_dev_attr_in2_input.dev_attr.attr,
	[RAVE_SP_SENSOR_3V3]	= &sensor_dev_attr_in3_input.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_3V3_PMIC]	= &sensor_dev_attr_in4_input.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_3V3_MCU]	= &sensor_dev_attr_in5_input.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_5V_MAIN]	= &sensor_dev_attr_in6_input.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_12V_MAIN]	= &sensor_dev_attr_in7_input.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_28V_FIL]	= &sensor_dev_attr_in8_input.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_28V_HOTSWAP] = &sensor_dev_attr_in9_input.dev_attr.attr,
	[RAVE_SP_SENSOR_DEB_1V8]	= &sensor_dev_attr_in10_input.dev_attr.attr,
	[RAVE_SP_SENSOR_DEB_3V3]	= &sensor_dev_attr_in11_input.dev_attr.attr,
	[RAVE_SP_SENSOR_DEB_28V_DEB]	= &sensor_dev_attr_in12_input.dev_attr.attr,
	[RAVE_SP_SENSOR_DEB_28V_RDU]	= &sensor_dev_attr_in13_input.dev_attr.attr,
	[RAVE_SP_SENSOR_TEMPERATURE]	= &sensor_dev_attr_temp1_input.dev_attr.attr,
	[RAVE_SP_SENSOR_TEMPERATURE_2]	= &sensor_dev_attr_temp2_input.dev_attr.attr,
	[RAVE_SP_SENSOR_BACKLIGHT_CURRENT] = &sensor_dev_attr_curr1_input.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_28V_CURRENT] = &sensor_dev_attr_curr2_input.dev_attr.attr
};

static struct attribute *rave_sp_hwmon_labels[] = {
	[RAVE_SP_SENSOR_28V]	= &sensor_dev_attr_in0_label.dev_attr.attr,
	[RAVE_SP_SENSOR_12V]	= &sensor_dev_attr_in1_label.dev_attr.attr,
	[RAVE_SP_SENSOR_5V]	= &sensor_dev_attr_in2_label.dev_attr.attr,
	[RAVE_SP_SENSOR_3V3]	= &sensor_dev_attr_in3_label.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_3V3_PMIC]	= &sensor_dev_attr_in4_label.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_3V3_MCU]	= &sensor_dev_attr_in5_label.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_5V_MAIN]	= &sensor_dev_attr_in6_label.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_12V_MAIN]	= &sensor_dev_attr_in7_label.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_28V_FIL]	= &sensor_dev_attr_in8_label.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_28V_HOTSWAP] = &sensor_dev_attr_in9_label.dev_attr.attr,
	[RAVE_SP_SENSOR_DEB_1V8]	= &sensor_dev_attr_in10_label.dev_attr.attr,
	[RAVE_SP_SENSOR_DEB_3V3]	= &sensor_dev_attr_in11_label.dev_attr.attr,
	[RAVE_SP_SENSOR_DEB_28V_DEB]	= &sensor_dev_attr_in12_label.dev_attr.attr,
	[RAVE_SP_SENSOR_DEB_28V_RDU]	= &sensor_dev_attr_in13_label.dev_attr.attr,
	[RAVE_SP_SENSOR_TEMPERATURE] = &sensor_dev_attr_temp1_label.dev_attr.attr,
	[RAVE_SP_SENSOR_TEMPERATURE_2] = &sensor_dev_attr_temp2_label.dev_attr.attr,
	[RAVE_SP_SENSOR_BACKLIGHT_CURRENT] = &sensor_dev_attr_curr1_label.dev_attr.attr,
	[RAVE_SP_SENSOR_RMB_28V_CURRENT] = &sensor_dev_attr_curr2_label.dev_attr.attr
};

static struct attribute_group rave_sp_hwmon_group;

__ATTRIBUTE_GROUPS(rave_sp_hwmon);

static const struct of_device_id rave_sp_hwmon_of_match[] = {
	{ .compatible = "zii,pic-hwmon" },
	{}
};

static int rave_sp_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rave_sp *zp = dev_get_drvdata(dev->parent);
	struct rave_sp_hwmon *hwmon;
	struct device *hwmon_dev;
	struct device_node *node = dev->of_node;
	const char *sensor_name;
	int count, i, attr_idx = 0;

	if (!node || !zp)
		return -EINVAL;

	count = of_property_count_strings(node, "sensors");
	if (count <= 0) {
		dev_err(dev, "sensor list not provided\n");
		return -EINVAL;
	}

	/* add one more for NULL-terminator */
	count++;

	hwmon = devm_kzalloc(dev, sizeof(struct rave_sp_hwmon) +
			count * sizeof(struct attribute *), GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	hwmon->zp = zp;

	/* prepare attribute group */
	for (i = 0; i < count; i++) {
		enum rave_sp_sensor j;

		if (of_property_read_string_index(node, "sensors",
				i, &sensor_name))
			continue;
		for (j = RAVE_SP_SENSOR_FIRST; j < RAVE_SP_SENSORS_COUNT; j++) {
			if (strcmp(sensor_name, input_names[j]))
				continue;

			hwmon->attrs[attr_idx++] = rave_sp_hwmon_sensors[j];
			hwmon->attrs[attr_idx++] = rave_sp_hwmon_labels[j];
			break;
		}
	}
	rave_sp_hwmon_group.attrs = hwmon->attrs;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, "rave_sp_hwmon",
					hwmon, rave_sp_hwmon_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static struct platform_driver rave_sp_hwmon_driver = {
	.probe = rave_sp_hwmon_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = rave_sp_hwmon_of_match,
	},
};

module_platform_driver(rave_sp_hwmon_driver);

MODULE_DEVICE_TABLE(of, rave_sp_hwmon_of_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_DESCRIPTION("Rave SP HWMON driver");
MODULE_ALIAS("platform:rave-sp-hwmon");
