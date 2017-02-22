#ifndef __MSM_UTIL_H__
#define __MSM_UTIL_H__

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#define DBG(fmt, ...) DRM_DEBUG(fmt"\n", ##__VA_ARGS__)
#define VERB(fmt, ...) if (0) DRM_DEBUG(fmt"\n", ##__VA_ARGS__)

void __iomem *msm_ioremap(struct platform_device *pdev, const char *name,
		const char *dbgname);
void msm_writel(u32 data, void __iomem *addr);
u32 msm_readl(const void __iomem *addr);

#endif
