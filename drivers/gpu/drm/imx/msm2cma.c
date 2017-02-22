/*
 * Bridge msm gem objects to cma ones wanted by imx-drm
 *
 * This stinks
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>

#include "msm_gem.h"
#include "msm_plat.h"


/* XXX: this must be the same as the version in drm_fb_cma_helper.c - extract it */
struct drm_fb_cma {
	struct drm_framebuffer		fb;
	struct drm_gem_cma_object	*obj[4];
};

struct drm_fbdev_cma {
	struct drm_fb_helper	fb_helper;
	struct drm_fb_cma	*fb;
};


static inline struct drm_fb_cma *to_fb_cma(struct drm_framebuffer *fb)
{
	return container_of(fb, struct drm_fb_cma, fb);
}

static void drm_fb_msm2cma_destroy(struct drm_framebuffer *fb)
{
	struct drm_fb_cma *fb_cma = to_fb_cma(fb);
	int i;

	for (i = 0; i < 4; i++) {
		if (fb_cma->obj[i]) {
			drm_gem_object_unreference_unlocked(&fb_cma->obj[i]->base);
			kfree(fb_cma->obj[i]);
		}
	}

	drm_framebuffer_cleanup(fb);
	kfree(fb_cma);
}

static int drm_fb_msm2cma_create_handle(struct drm_framebuffer *fb,
	struct drm_file *file_priv, unsigned int *handle)
{
	struct drm_fb_cma *fb_cma = to_fb_cma(fb);

	return drm_gem_handle_create(file_priv,
			&fb_cma->obj[0]->base, handle);
}

static struct drm_framebuffer_funcs drm_fb_msm2cma_funcs = {
	.destroy	= drm_fb_msm2cma_destroy,
	.create_handle	= drm_fb_msm2cma_create_handle,
};

static struct drm_fb_cma *drm_fb_msm2cma_alloc(struct drm_device *dev,
	struct drm_mode_fb_cmd2 *mode_cmd, struct msm_gem_object **msm_obj,
	unsigned int num_planes)
{
	struct drm_fb_cma *fb_cma;
	int ret;
	int i;

	fb_cma = kzalloc(sizeof(*fb_cma), GFP_KERNEL);
	if (!fb_cma)
		return ERR_PTR(-ENOMEM);

	drm_helper_mode_fill_fb_struct(&fb_cma->fb, mode_cmd);

	for (i = 0; i < num_planes; i++) {
		struct drm_gem_cma_object *cma_obj;
		struct msm_plat_private *msm_priv = dev->dev_private;

		cma_obj = kzalloc(sizeof(*cma_obj), GFP_KERNEL);
		if (!cma_obj)
			return ERR_PTR(-ENOMEM);

		cma_obj->paddr = (((dma_addr_t)msm_obj[i]->vram_node->start) << PAGE_SHIFT) +
						msm_priv->vram.paddr;
		cma_obj->vaddr = msm_obj[i]->vaddr;

		printk(KERN_INFO "@MF@ %s: plane=%d phys=%08x\n", __func__, i, cma_obj->paddr);

		fb_cma->obj[i] = cma_obj;
	}

	ret = drm_framebuffer_init(dev, &fb_cma->fb, &drm_fb_msm2cma_funcs);
	if (ret) {
		dev_err(dev->dev, "Failed to initialize framebuffer: %d\n", ret);

		for (i=0; i< num_planes; i++)
			kfree(fb_cma->obj[i]);

		kfree(fb_cma);
		return ERR_PTR(ret);
	}

	return fb_cma;
}

struct drm_framebuffer *drm_fb_msm2cma_create(struct drm_device *dev,
	struct drm_file *file_priv, struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_fb_cma *fb_cma;
	struct msm_gem_object *objs[4];
	struct drm_gem_object *obj;
	unsigned int hsub;
	unsigned int vsub;
	int ret;
	int i;

	hsub = drm_format_horz_chroma_subsampling(mode_cmd->pixel_format);
	vsub = drm_format_vert_chroma_subsampling(mode_cmd->pixel_format);

	for (i = 0; i < drm_format_num_planes(mode_cmd->pixel_format); i++) {
		unsigned int width = mode_cmd->width / (i ? hsub : 1);
		unsigned int height = mode_cmd->height / (i ? vsub : 1);
		unsigned int min_size;

		obj = drm_gem_object_lookup(dev, file_priv, mode_cmd->handles[i]);
		if (!obj) {
			dev_err(dev->dev, "Failed to lookup GEM object\n");
			ret = -ENXIO;
			goto err_gem_object_unreference;
		}

		min_size = (height - 1) * mode_cmd->pitches[i]
			 + width * drm_format_plane_cpp(mode_cmd->pixel_format, i)
			 + mode_cmd->offsets[i];

		if (obj->size < min_size) {
			drm_gem_object_unreference_unlocked(obj);
			ret = -EINVAL;
			goto err_gem_object_unreference;
		}
		objs[i] = to_msm_bo(obj);
	}

	fb_cma = drm_fb_msm2cma_alloc(dev, mode_cmd, objs, i);
	if (IS_ERR(fb_cma)) {
		ret = PTR_ERR(fb_cma);
		goto err_gem_object_unreference;
	}

	return &fb_cma->fb;

err_gem_object_unreference:
	for (i--; i >= 0; i--)
		drm_gem_object_unreference_unlocked(&objs[i]->base);
	return ERR_PTR(ret);
}
