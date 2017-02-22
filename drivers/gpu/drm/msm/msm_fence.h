#ifndef _MSM_FENCE_H_
#define _MSM_FENCE_H_

#include <linux/list.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <drm/drmP.h>

/* callback from wq once fence has passed: */
struct msm_fence_cb {
	struct work_struct work;
	uint32_t fence;
	void (*func)(struct msm_fence_cb *cb);
};

void __msm_fence_worker(struct work_struct *work);

#define INIT_FENCE_CB(_cb, _func)  do {                     \
		INIT_WORK(&(_cb)->work, __msm_fence_worker); \
		(_cb)->func = _func;                         \
	} while (0)

int msm_wait_fence_interruptable(struct drm_device *dev, uint32_t fence,
		struct timespec *timeout);
int msm_queue_fence_cb(struct drm_device *dev,
		struct msm_fence_cb *cb, uint32_t fence);
void msm_update_fence(struct drm_device *dev, uint32_t fence);

#endif
