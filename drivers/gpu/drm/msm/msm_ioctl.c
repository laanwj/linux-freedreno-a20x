#include "msm_fence.h"
#include "msm_gem.h"
#include "msm_gpu.h"
#include "msm_ioctl.h"
#include "msm_plat.h"

static int msm_ioctl_get_param(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct msm_plat_private *priv = dev->dev_private;
	struct drm_msm_param *args = data;
	struct msm_gpu *gpu;

	/* for now, we just have 3d pipe.. eventually this would need to
	 * be more clever to dispatch to appropriate gpu module:
	 */
	if (args->pipe != MSM_PIPE_3D0)
		return -EINVAL;

	gpu = priv->gpu;

	if (!gpu)
		return -ENXIO;

	return gpu->funcs->get_param(gpu, args->param, &args->value);
}

static int msm_ioctl_gem_new(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_gem_new *args = data;

	if (args->flags & ~MSM_BO_FLAGS) {
		DRM_ERROR("invalid flags: %08x\n", args->flags);
		return -EINVAL;
	}

	return msm_gem_new_handle(dev, file, args->size,
			args->flags, &args->handle);
}

#define TS(t) ((struct timespec){ .tv_sec = (t).tv_sec, .tv_nsec = (t).tv_nsec })

static int msm_ioctl_gem_cpu_prep(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_gem_cpu_prep *args = data;
	struct drm_gem_object *obj;
	int ret;

	if (args->op & ~MSM_PREP_FLAGS) {
		DRM_ERROR("invalid op: %08x\n", args->op);
		return -EINVAL;
	}

	obj = drm_gem_object_lookup(dev, file, args->handle);
	if (!obj)
		return -ENOENT;

	ret = msm_gem_cpu_prep(obj, args->op, &TS(args->timeout));

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int msm_ioctl_gem_cpu_fini(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_gem_cpu_fini *args = data;
	struct drm_gem_object *obj;
	int ret;

	obj = drm_gem_object_lookup(dev, file, args->handle);
	if (!obj)
		return -ENOENT;

	ret = msm_gem_cpu_fini(obj);

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int msm_ioctl_gem_info(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_gem_info *args = data;
	struct drm_gem_object *obj;
	int ret = 0;

	if (args->pad)
		return -EINVAL;

	obj = drm_gem_object_lookup(dev, file, args->handle);
	if (!obj)
		return -ENOENT;

	args->offset = msm_gem_mmap_offset(obj);

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int msm_ioctl_wait_fence(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_wait_fence *args = data;
	struct msm_plat_private *priv = dev->dev_private;

	if (args->pad) {
		DRM_ERROR("invalid pad: %08x\n", args->pad);
		return -EINVAL;
	}

	if (!priv->gpu)
		return 0;

	return msm_wait_fence_interruptable(dev, args->fence,
			&TS(args->timeout));
}

const struct drm_ioctl_desc msm_drm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(MSM_GET_PARAM,    msm_ioctl_get_param,    DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_NEW,      msm_ioctl_gem_new,      DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_INFO,     msm_ioctl_gem_info,     DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_CPU_PREP, msm_ioctl_gem_cpu_prep, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_CPU_FINI, msm_ioctl_gem_cpu_fini, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_SUBMIT,   msm_ioctl_gem_submit,   DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_WAIT_FENCE,   msm_ioctl_wait_fence,   DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
};
