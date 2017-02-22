#ifndef _IMX_GPU_H_
#define _IMX_GPU_H_

#include <drm/drmP.h>

int imxgpu_load(struct drm_device *dev);
int imxgpu_open(struct drm_device *dev, struct drm_file *file);
int imxgpu_debugfs_init(struct drm_minor *minor);
void imxgpu_debugfs_cleanup(struct drm_minor *minor);

#endif
