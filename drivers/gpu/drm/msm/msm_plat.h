#ifndef __MSM_PLAT_H__
#define __MSM_PLAT_H__

#include <drm/drmP.h>

struct msm_mmu;

int msm_register_mmu(struct drm_device *dev, struct msm_mmu *mmu);

#endif
