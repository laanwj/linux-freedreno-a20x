/*
 * rave_sp_bl.c - LCD Backlight driver for Zodiac Inflight Innovations
 * PIC MCU that is connected via dedicated UART port
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
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/rave-sp.h>

#define CMD_SET_BACKLIGHT	0xA6

static int rave_sp_bl_update_status(struct backlight_device *bd)
{
	struct rave_sp *zp = dev_get_drvdata(&bd->dev);
	int intensity = bd->props.brightness;
	u8 cmd[5];

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;

	cmd[0] = CMD_SET_BACKLIGHT;
	cmd[1] = 0;
	cmd[2] = intensity ? 0x80 | intensity : 0;
	cmd[3] = 0;
	cmd[4] = 0;

	return rave_sp_exec(zp, cmd, sizeof(cmd),
			    NULL, 0);
}

static const struct backlight_ops rave_sp_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status  = rave_sp_bl_update_status,
};

static struct backlight_properties rave_sp_bl_props = {
	.type = BACKLIGHT_FIRMWARE,
	.max_brightness = 100,
	.brightness = 50,
};

static const struct of_device_id rave_sp_bl_of_match[] = {
	{ .compatible = "zii,pic-backlight" },
	{}
};

static int rave_sp_backlight_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rave_sp *zp = dev_get_drvdata(dev->parent);
	struct backlight_device *bd;

	if (!zp)
		return -EINVAL;

	bd = devm_backlight_device_register(dev, KBUILD_MODNAME,
					    dev, zp, &rave_sp_bl_ops,
					    &rave_sp_bl_props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);
	backlight_update_status(bd);

	return 0;
}

static struct platform_driver rave_sp_backlight_driver = {
	.probe		= rave_sp_backlight_probe,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = rave_sp_bl_of_match,
	},
};
module_platform_driver(rave_sp_backlight_driver);

MODULE_DEVICE_TABLE(of, rave_sp_bl_of_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_DESCRIPTION("Rave SP LCD Backlight driver");
MODULE_ALIAS("platform:zii-pic-backlight");
