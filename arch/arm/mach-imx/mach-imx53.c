/*
 * Copyright 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/pmu.h>

#include "common.h"
#include "hardware.h"

static void __init imx53_init_early(void)
{
	mxc_set_cpu_type(MXC_CPU_MX53);
}


#define MXC_CORTEXA8_PLAT_GPC 0x63fa0004 /* Hard code as this is i.Mx53 only file */
#define GPC_DBG_EN (1 << 16)

static void __iomem *imx53_pmu_get_gpc(void)
{
	static void __iomem *gpc = NULL;

	if (!gpc) {
		gpc = ioremap(MXC_CORTEXA8_PLAT_GPC, 4);
		if (!gpc)
			printk_once(KERN_INFO "unable to map GPC to enable perf\n");
	}

	return gpc;
}

static int imx53_pmu_resume(struct device *dev)
{
	struct arm_pmu *arm_pmu = dev_get_drvdata(dev);
	void __iomem *gpc_reg;
	u32 gpc;

	gpc_reg = imx53_pmu_get_gpc();
	if (!gpc_reg)
		return 0;

	gpc = __raw_readl(gpc_reg);
	if (gpc & GPC_DBG_EN) {
		arm_pmu->activated_flags.platform_enabled = 0;
	} else {
		gpc |= GPC_DBG_EN;
		__raw_writel(gpc, gpc_reg);
		arm_pmu->activated_flags.platform_enabled = 1;
	}

	return 0;
}

static int imx53_pmu_suspend(struct device *dev)
{
	struct arm_pmu *arm_pmu = dev_get_drvdata(dev);
	void __iomem *gpc_reg;
	u32 gpc;

	gpc_reg = imx53_pmu_get_gpc();
	if (!gpc_reg)
		return 0;

	if (arm_pmu->activated_flags.platform_enabled) {
		gpc = __raw_readl(gpc_reg);
		gpc &= ~GPC_DBG_EN;
		__raw_writel(gpc, gpc_reg);
		arm_pmu->activated_flags.platform_enabled = 0;
	}

	return 0;
}


static struct arm_pmu_platdata imx53_pmu_platdata = {
	.runtime_resume = imx53_pmu_resume,
	.runtime_suspend = imx53_pmu_suspend,
};

static struct of_dev_auxdata imx53_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("arm,cortex-a8-pmu", 0, "arm-pmu", &imx53_pmu_platdata),
	{}
};

static void __init imx53_dt_init(void)
{
	imx_src_init();

	of_platform_populate(NULL, of_default_bus_match_table,
					imx53_auxdata_lookup, NULL);
	imx_aips_allow_unprivileged_access("fsl,imx53-aipstz");
}

static void __init imx53_init_late(void)
{
	imx53_pm_init();

	platform_device_register_simple("cpufreq-dt", -1, NULL, 0);
}

static const char * const imx53_dt_board_compat[] __initconst = {
	"fsl,imx53",
	NULL
};

DT_MACHINE_START(IMX53_DT, "Freescale i.MX53 (Device Tree Support)")
	.init_early	= imx53_init_early,
	.init_irq	= tzic_init_irq,
	.init_machine	= imx53_dt_init,
	.init_late	= imx53_init_late,
	.dt_compat	= imx53_dt_board_compat,
MACHINE_END
