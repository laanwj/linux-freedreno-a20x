/*
 * Freescale i.MX gpu driver
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
#include <linux/debugfs.h>
#include <linux/module.h>

#include "../msm/msm_gpu.h" /* XXX This sucks */


#ifdef CONFIG_DEBUG_FS
static int imxgpu_gpu_show(struct drm_device *dev, struct seq_file *m)
{
	struct msm_plat_private *priv = dev->dev_private;
	struct msm_gpu *gpu = priv->gpu;

	if (gpu) {
		seq_printf(m, "%s Status:\n", gpu->name);
		gpu->funcs->show(gpu, m);
	}

	return 0;
}

static int imxgpu_gem_show(struct drm_device *dev, struct seq_file *m)
{
	struct msm_plat_private *priv = dev->dev_private;
	struct msm_gpu *gpu = priv->gpu;

	if (gpu) {
		seq_printf(m, "Active Objects (%s):\n", gpu->name);
		msm_gem_describe_objects(&gpu->active_list, m);
	}

	seq_printf(m, "Inactive Objects:\n");
	msm_gem_describe_objects(&priv->inactive_list, m);

	return 0;
}

static int imxgpu_mm_show(struct drm_device *dev, struct seq_file *m)
{
	return drm_mm_dump_table(m, &dev->vma_offset_manager->vm_addr_space_mm);
}

static int show_locked(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	int (*show)(struct drm_device *dev, struct seq_file *m) =
			node->info_ent->data;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	ret = show(dev, m);

	mutex_unlock(&dev->struct_mutex);

	return ret;
}

static struct drm_info_list imxgpu_debugfs_list[] = {
		{"gpu", show_locked, 0, imxgpu_gpu_show},
		{"gem", show_locked, 0, imxgpu_gem_show},
		{ "mm", show_locked, 0, imxgpu_mm_show },
};

static int late_init_minor(struct drm_minor *minor)
{
	int ret;

	if (!minor)
		return 0;

	ret = msm_rd_debugfs_init(minor);
	if (ret) {
		dev_err(minor->dev->dev, "could not install rd debugfs\n");
		return ret;
	}

	ret = msm_perf_debugfs_init(minor);
	if (ret) {
		dev_err(minor->dev->dev, "could not install perf debugfs\n");
		return ret;
	}

	return 0;
}

struct imxgpu_gmem_ctx {
	struct drm_device *dev;
	struct resource	*res;
	void __iomem 	*gmem;
};

static struct imxgpu_gmem_ctx *imxgpu_gmem_open(struct drm_device *dev)
{
	struct msm_plat_private *priv = dev->dev_private;
	struct 	imxgpu_gmem_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->res = platform_get_resource_byname(priv->gpu_pdev, IORESOURCE_MEM, "gmem");
	if (!ctx->res) {
		dev_err(dev->dev, "No gmem defined in DT %p '%s'\n", priv->gpu_pdev, priv->gpu_pdev->name);
		ret = -ENODEV;
		goto out_err;
	}
	ctx->gmem = ioremap_nocache(ctx->res->start, resource_size(ctx->res));
	if (!ctx->gmem) {
		dev_err(dev->dev, "Unable to map gmem\n");
		ret = -ENOMEM;
		goto out_err;
	}

	ctx->dev = dev;

	return ctx;

out_err:
	kfree(ctx);

	return ERR_PTR(ret);
}

static void imxgpu_gmem_close(struct imxgpu_gmem_ctx *ctx)
{
	iounmap(ctx->gmem);
	kfree(ctx);
}

static int gmem_open(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;
	struct imxgpu_gmem_ctx *ctx;

	ctx = imxgpu_gmem_open(dev);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	file->private_data = ctx;

	return 0;
}

static int gmem_release(struct inode *inode, struct file *file)
{
	struct imxgpu_gmem_ctx *ctx = file->private_data;

	imxgpu_gmem_close(ctx);

	return 0;
}

static ssize_t gmem_read(struct file *file, char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	struct imxgpu_gmem_ctx *ctx = file->private_data;
	loff_t pos = *ppos;
	u32 val;
	unsigned long remain;
	size_t i;

	if (pos < 0)
		return -EINVAL;

	if (pos >= resource_size(ctx->res))
		return 0;

	if (count > resource_size(ctx->res) - pos)
		count = resource_size(ctx->res) - pos;

	count &= ~3;

	for (i=0; i < count; i += sizeof(val)) {
		val = readl(ctx->gmem + pos);
		remain = copy_to_user(user_buf, &val, sizeof(val));
		if (remain)
			return -EFAULT;
		user_buf += sizeof(val);
		pos += sizeof(val);
	}

	*ppos = pos;

	return count;
}

static const struct file_operations fops_gmem = {
	.open		= gmem_open,
	.release	= gmem_release,
	.read		= gmem_read,
	.llseek		= default_llseek,
};

static int imxgpu_debugfs_gmem_init(struct drm_device *dev)
{
	struct msm_plat_private *priv = dev->dev_private;
	struct dentry *dentry;

	dentry = debugfs_create_file("gmem", S_IRUGO,
					dev->primary->debugfs_root,
					dev, &fops_gmem);

	if (IS_ERR(dentry))
		return PTR_ERR(dentry);

	priv->gmem_debugfs = dentry;

	return 0;
}

int imxgpu_debugfs_late_init(struct drm_device *dev)
{
	int ret;

	printk(KERN_INFO "@MF@ %s\n", __func__);

	ret = late_init_minor(dev->primary);
	if (ret)
		return ret;
	ret = late_init_minor(dev->render);
	if (ret)
		return ret;
	ret = late_init_minor(dev->control);
	if (ret)
		return ret;

	ret = imxgpu_debugfs_gmem_init(dev);

	return ret;
}

int imxgpu_debugfs_init(struct drm_minor *minor)
{
	struct drm_device *dev = minor->dev;
	int ret;

	printk(KERN_INFO "@MF@ %s\n", __func__);

	ret = drm_debugfs_create_files(imxgpu_debugfs_list,
			ARRAY_SIZE(imxgpu_debugfs_list),
			minor->debugfs_root, minor);

	if (ret) {
		dev_err(dev->dev, "could not install imxgpu_debugfs_list\n");
		return ret;
	}

	return 0;
}

void imxgpu_debugfs_cleanup(struct drm_minor *minor)
{
	struct msm_plat_private *priv = minor->dev->dev_private;

	drm_debugfs_remove_files(imxgpu_debugfs_list,
			ARRAY_SIZE(imxgpu_debugfs_list), minor);
	if (!priv)
		return;
	msm_rd_debugfs_cleanup(minor);
	msm_perf_debugfs_cleanup(minor);

	if (priv->gmem_debugfs) {
		debugfs_remove(priv->gmem_debugfs);
		priv->gmem_debugfs = NULL;
	}
}
#endif

static int imxgpu_allocate_vram_carveout(struct drm_device *dev)
{
	struct msm_plat_private *priv = dev->dev_private;

	DEFINE_DMA_ATTRS(attrs);
	unsigned long size;
	void *p;
	char *vram = "16m";

	size = memparse(vram, NULL);
	priv->vram.size = size;

	drm_mm_init(&priv->vram.mm, 0, (size >> PAGE_SHIFT) - 1);

	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
	dma_set_attr(DMA_ATTR_WRITE_COMBINE, &attrs);

	/* note that for no-kernel-mapping, the vaddr returned
	 * is bogus, but non-null if allocation succeeded:
	 */
	p = dma_alloc_attrs(dev->dev, size,
			&priv->vram.paddr, GFP_KERNEL, &attrs);
	if (!p) {
		dev_err(dev->dev, "failed to allocate VRAM\n");
		priv->vram.paddr = 0;
		return -ENOMEM;
	}

	dev_info(dev->dev, "GPU VRAM: %08x->%08x\n",
			(uint32_t)priv->vram.paddr,
			(uint32_t)(priv->vram.paddr + size));

	return 0;
}

static int imxgpu_clear_gmem(struct drm_device *dev)
{
	struct imxgpu_gmem_ctx *ctx;
	int i;

	ctx = imxgpu_gmem_open(dev);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	for (i=0; i < resource_size(ctx->res); i += 4)
		writel(0, ctx->gmem + i);

	imxgpu_gmem_close(ctx);

	return 0;
}

int imxgpu_load(struct drm_device *dev)
{
	// XXX: cleanup
	struct msm_plat_private *priv = dev->dev_private;
	int ret;

	priv->wq = alloc_ordered_workqueue("imx-gpu", 0);
	init_waitqueue_head(&priv->fence_event);

	INIT_LIST_HEAD(&priv->inactive_list);
	INIT_LIST_HEAD(&priv->fence_cbs);

	ret = imxgpu_debugfs_late_init(dev);  /* MF: maybe to early?? */
	if (ret)
		return ret;

	return imxgpu_allocate_vram_carveout(dev);
}

static void load_gpu(struct drm_device *dev)
{
	static DEFINE_MUTEX(init_lock);
	struct msm_plat_private *priv = dev->dev_private;

	mutex_lock(&init_lock);

	if (!priv->gpu)
		priv->gpu = adreno_load_gpu(dev);

	imxgpu_clear_gmem(dev);

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
