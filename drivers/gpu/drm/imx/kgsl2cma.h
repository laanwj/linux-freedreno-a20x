#ifndef _KGSL2CMA_H_
#define _KGSL2CMA_H_

#include <drm/drmP.h>
#include <drm/drm_mode.h>

struct drm_framebuffer *drm_fb_kgsl2cma_create(struct drm_device *dev,
	struct drm_file *file_priv, const struct drm_mode_fb_cmd2 *mode_cmd);

#endif

