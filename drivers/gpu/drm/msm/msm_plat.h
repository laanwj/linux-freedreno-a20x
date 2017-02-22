#ifndef __MSM_PLAT_H__
#define __MSM_PLAT_H__

#include <linux/list.h>
#include <linux/workqueue.h>

#include <drm/drmP.h>

#define NUM_DOMAINS 2    /* one for KMS, then one per gpu core (?) */

struct msm_mmu;
struct msm_rd_state;

struct msm_file_private {
	/* currently we don't do anything useful with this.. but when
	 * per-context address spaces are supported we'd keep track of
	 * the context's page-tables here.
	 */
	int dummy;
};

/* Contains common (to msm/imx) parts of data to be embedded in drm driver */
struct msm_plat_private {
	/* subordinate devices, if present: */
	struct platform_device *gpu_pdev;

	/* when we have more than one 'msm_gpu' these need to be an array: */
	struct msm_gpu *gpu;
	struct msm_file_private *lastctx;

	uint32_t next_fence, completed_fence;
	wait_queue_head_t fence_event;

	struct msm_rd_state *rd;
	struct msm_perf_state *perf;

	/* list of GEM objects: */
	struct list_head inactive_list;

	struct workqueue_struct *wq;

	/* callbacks deferred until bo is inactive: */
	struct list_head fence_cbs;

	/* registered MMUs: */
	unsigned int num_mmus;
	struct msm_mmu *mmus[NUM_DOMAINS];

	/* VRAM carveout, used when no IOMMU: */
	struct {
		unsigned long size;
		dma_addr_t paddr;
		/* NOTE: mm managed at the page level, size is in # of pages
		 * and position mm_node->start is in # of pages:
		 */
		struct drm_mm mm;
	} vram;
};

int msm_register_mmu(struct drm_device *dev, struct msm_mmu *mmu);

#endif
