ccflags-$(CONFIG_DRM_IMX_ADRENO) := -Idrivers/gpu/drm/msm

imxdrm-objs := imx-drm-core.o

obj-$(CONFIG_DRM_IMX) += imxdrm.o

obj-$(CONFIG_DRM_IMX_PARALLEL_DISPLAY) += parallel-display.o
obj-$(CONFIG_DRM_IMX_TVE) += imx-tve.o
obj-$(CONFIG_DRM_IMX_LDB) += imx-ldb.o

imx-ipuv3-crtc-objs  := ipuv3-crtc.o ipuv3-plane.o
obj-$(CONFIG_DRM_IMX_IPUV3)	+= imx-ipuv3-crtc.o
obj-$(CONFIG_DRM_IMX_HDMI) += imx-hdmi.o
obj-$(CONFIG_DRM_IMX_ADRENO) += imx-gpu.o \
	../msm/adreno/adreno_device.o \
	../msm/adreno/adreno_gpu.o \
	../msm/adreno/a3xx_gpu.o \
	../msm/adreno/a4xx_gpu.o \
	../msm/msm_fence.o \
	../msm/msm_gem.o \
	../msm/msm_gem_submit.o \
	../msm/msm_gpu.o \
	../msm/msm_ioctl.o \
	../msm/msm_perf.o \
	../msm/msm_plat.o \
	../msm/msm_rd.o \
	../msm/msm_ringbuffer.o \
	../msm/msm_util.o
