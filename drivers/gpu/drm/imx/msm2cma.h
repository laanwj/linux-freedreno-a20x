#ifndef _MSM2CMA_H_
#define _MSM2CMA_H_

#include <drm/drmP.h>
#include <drm/drm_mode.h>

struct drm_framebuffer *drm_fb_msm2cma_create(struct drm_device *dev,
	struct drm_file *file_priv, struct drm_mode_fb_cmd2 *mode_cmd);

#endif
