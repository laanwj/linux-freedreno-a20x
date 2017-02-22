#include "msm_drv.h"

#include "msm_plat.h"

int msm_register_mmu(struct drm_device *dev, struct msm_mmu *mmu)
{
	struct msm_plat_private *priv = dev->dev_private;
	int idx = priv->num_mmus++;

	if (WARN_ON(idx >= ARRAY_SIZE(priv->mmus)))
		return -EINVAL;

	priv->mmus[idx] = mmu;

	return idx;
}
