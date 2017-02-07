/*
 *  zii-pic-pwrbutton.c - driver for power button on Zodiac Inflight Innovation
 *  PIC.
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

#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rave-spg.h>

#define EVT_BUTTON_PRESS	0xE0

struct zii_pic_power_button {
	struct input_dev *idev;
	struct notifier_block nb;
};

static struct zii_pic_power_button *
to_zii_pic_power_button(struct notifier_block *nb)
{
	return container_of(nb, struct zii_pic_power_button, nb);
}

static int zii_pic_power_button_event(struct notifier_block *nb,
				      unsigned long action, void *data)
{
	if (zii_pic_action_get_event(action) == EVT_BUTTON_PRESS) {
		struct zii_pic_power_button *picpb;

		picpb = to_zii_pic_power_button(nb);

		input_report_key(picpb->idev, KEY_POWER,
				 zii_pic_action_get_value(action));
		input_sync(picpb->idev);

		return NOTIFY_STOP;
	}

	return NOTIFY_DONE;
}

static const struct of_device_id zii_pic_pwrbutton_of_match[] = {
	{ .compatible = "zii,pic-pwrbutton" },
	{}
};

static int zii_pic_pwrbutton_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zii_pic *zp = dev_get_drvdata(dev->parent);
	struct zii_pic_power_button *picpb;
	int ret;

	if (!zp)
		return -EINVAL;

	picpb = devm_kzalloc(&pdev->dev, sizeof(*picpb), GFP_KERNEL);
	dev_set_drvdata(&pdev->dev, picpb);
	if (!picpb)
		return -ENOMEM;

	picpb->idev = devm_input_allocate_device(&pdev->dev);
	if (!picpb->idev)
		return -ENOMEM;

	picpb->nb.notifier_call = zii_pic_power_button_event;
	picpb->nb.priority = 128;

	picpb->idev->name = KBUILD_MODNAME;
	picpb->idev->dev.parent = dev;

	input_set_capability(picpb->idev, EV_KEY, KEY_POWER);

	ret = input_register_device(picpb->idev);
	if (ret)
		return ret;

	return devm_zii_pic_register_event_notifier(dev, &picpb->nb);
}

static struct platform_driver zii_pic_pwrbutton_driver = {
	.probe		= zii_pic_pwrbutton_probe,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = zii_pic_pwrbutton_of_match,
	},
};
module_platform_driver(zii_pic_pwrbutton_driver);

MODULE_DEVICE_TABLE(of, zii_pic_pwrbutton_of_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC Power Button driver");
MODULE_ALIAS("platform:zii-pic-pwrbutton");
