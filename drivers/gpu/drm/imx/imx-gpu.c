/*
 * Freescale i.MX drm driver
 *
 * Copyright (C) 2015 Martin Fuzzey, Parkeon
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/component.h>
#include <linux/module.h>

#include "../msm/msm_gpu.h" /* XXX This sucks */

static void load_gpu(struct drm_device *dev)
{
	static DEFINE_MUTEX(init_lock);
	struct msm_plat_private *priv = dev->dev_private;

	mutex_lock(&init_lock);

	if (!priv->gpu)
		priv->gpu = adreno_load_gpu(dev);

	mutex_unlock(&init_lock);
}

int imxgpu_open(struct drm_device *dev, struct drm_file *file)
{
	struct msm_file_private *ctx;

	/* For now, load gpu on open.. to avoid the requirement of having
	 * firmware in the initrd.
	 */
	load_gpu(dev);

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	file->driver_priv = ctx;

	return 0;
}

static int __init imxgpu_register(void)
{
	adreno_register();

	return 0;
}

static void __exit imxgpu_unregister(void)
{
	adreno_unregister();
}

module_init(imxgpu_register);
module_exit(imxgpu_unregister);

MODULE_AUTHOR("Martin Fuzzey <mfuzzey@parkeon.com");
MODULE_DESCRIPTION("i.MX Adreno GPU Driver");
MODULE_LICENSE("GPL");
