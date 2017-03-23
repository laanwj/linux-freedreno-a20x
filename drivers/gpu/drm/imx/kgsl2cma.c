/*
 * Bridge kgsl handles to cma ones wanted by imx-drm
 *
 * This stinks
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>

unsigned int kgsl_sharedmem_export(unsigned int handle, dma_addr_t *paddr_out, void **vaddr_out);

/* XXX: this must be the same as the version in drm_fb_cma_helper.c - extract it */
struct drm_fb_cma {
	struct drm_framebuffer		fb;
	struct drm_gem_cma_object	*obj[4];
};

static inline struct drm_fb_cma *to_fb_cma(struct drm_framebuffer *fb)
{
	return container_of(fb, struct drm_fb_cma, fb);
}

static void drm_fb_kgsl2cma_destroy(struct drm_framebuffer *fb)
{
	struct drm_fb_cma *fb_cma = to_fb_cma(fb);
	int i;

	for (i = 0; i < 4; i++) {
		if (fb_cma->obj[i]) {
			// It is not an actual GEM object
			// drm_gem_object_unreference_unlocked(&fb_cma->obj[i]->base);
			kfree(fb_cma->obj[i]);
		}
	}

	drm_framebuffer_cleanup(fb);
	kfree(fb_cma);
}

static int drm_fb_kgsl2cma_create_handle(struct drm_framebuffer *fb,
	struct drm_file *file_priv, unsigned int *handle)
{
	struct drm_fb_cma *fb_cma = to_fb_cma(fb);

	return drm_gem_handle_create(file_priv,
			&fb_cma->obj[0]->base, handle);
}

static struct drm_framebuffer_funcs drm_fb_kgsl2cma_funcs = {
	.destroy	= drm_fb_kgsl2cma_destroy,
	.create_handle	= drm_fb_kgsl2cma_create_handle,
};

static struct drm_fb_cma *drm_fb_kgsl2cma_alloc(struct drm_device *dev,
	const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_fb_cma *fb_cma;
	int ret;
	int i;
	int num_planes = drm_format_num_planes(mode_cmd->pixel_format);

	fb_cma = kzalloc(sizeof(*fb_cma), GFP_KERNEL);
	if (!fb_cma)
		return ERR_PTR(-ENOMEM);

	drm_helper_mode_fill_fb_struct(dev, &fb_cma->fb, mode_cmd);

	for (i = 0; i < num_planes; i++) {
		struct drm_gem_cma_object *cma_obj;

		cma_obj = kzalloc(sizeof(*cma_obj), GFP_KERNEL);
		if (!cma_obj) {
			ret = ENOMEM;
			goto err_free_fb_cma;
		}

		if (kgsl_sharedmem_export(mode_cmd->handles[i], &cma_obj->paddr, &cma_obj->vaddr) < 0) {
			dev_err(dev->dev, "Failed to lookup KGSL handle\n");
			ret = ENXIO;
			goto err_free_fb_cma;
		}
		printk(KERN_INFO "@MF@ %s: plane=%d phys=%08x\n", __func__, i, cma_obj->paddr);

		fb_cma->obj[i] = cma_obj;
	}

	ret = drm_framebuffer_init(dev, &fb_cma->fb, &drm_fb_kgsl2cma_funcs);
	if (ret) {
		dev_err(dev->dev, "Failed to initialize framebuffer: %d\n", ret);
		goto err_free_fb_cma;
	}

	return fb_cma;
err_free_fb_cma:
	for (i=0; i< num_planes; i++)
		kfree(fb_cma->obj[i]);
	kfree(fb_cma);
	return ERR_PTR(ret);
}

struct drm_framebuffer *drm_fb_kgsl2cma_create(struct drm_device *dev,
	struct drm_file *file_priv, const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_fb_cma *fb_cma;
	int ret;
	int i;

	fb_cma = drm_fb_kgsl2cma_alloc(dev, mode_cmd);
	if (IS_ERR(fb_cma)) {
		ret = PTR_ERR(fb_cma);
		goto err_gem_object_unreference;
	}

	return &fb_cma->fb;

err_gem_object_unreference:
	return ERR_PTR(ret);
}

