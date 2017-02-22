#include <linux/io.h>

#include "msm_util.h"

static bool reglog = false;  /* XXX attach to module param */

void __iomem *msm_ioremap(struct platform_device *pdev, const char *name,
		const char *dbgname)
{
	struct resource *res;
	unsigned long size;
	void __iomem *ptr;

	if (name)
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	else
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res) {
		dev_err(&pdev->dev, "failed to get memory resource: %s\n", name);
		return ERR_PTR(-EINVAL);
	}

	size = resource_size(res);

	ptr = devm_ioremap_nocache(&pdev->dev, res->start, size);
	if (!ptr) {
		dev_err(&pdev->dev, "failed to ioremap: %s\n", name);
		return ERR_PTR(-ENOMEM);
	}

	if (reglog)
		printk(KERN_DEBUG "IO:region %s %08x %08lx\n", dbgname, (u32)ptr, size);

	return ptr;
}

void msm_writel(u32 data, void __iomem *addr)
{
	if (reglog)
		printk(KERN_DEBUG "IO:W %08x %08x\n", (u32)addr, data);
	writel(data, addr);
}

u32 msm_readl(const void __iomem *addr)
{
	u32 val = readl(addr);
	if (reglog)
		printk(KERN_ERR "IO:R %08x %08x\n", (u32)addr, val);
	return val;
}
