/* Copyright (c) 2008-2010, Advanced Micro Devices. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

/*
 * Martin Fuzzey: 5/5/2014
 *
 * The original Freescale driver handles allocations by using
 * __vmalloc() to obtain a virtually contiguous (but phusically non contiguous)
 * block of memory. Each physical page is then mapped to the GPU using the
 * GPU's MMU.
 *
 * However this approach causes GPU buffer corruption since the caches are not
 * handled properly.
 * This typically manifests itself as a GPU crash or interrupt storm
 * (but the system still responds to SYSRQ)
 *
 * By enabling MF_USE_DMA_API below the buffers are allocated using the
 * linux DMA API (dma_alloc_writecombine())
 *
 * This avoids the GPU crash but means that the memory region is now
 * physically contiguous (making use of the GPU MMU less interesting)
 *
 * Note that, based on my current understaning the DMA API does *not* actually
 * guaruntee that the kernel direct mapping does not have an alias for the
 * range unless CMA is used (not currently enabled in our config as requires
 * reserving memory up front.
 *
 * However this is the simplest approach for the moment.
 * If memory fragmentation is an issue possiblities are:
 *
 * # dma_alloc_writecombine() multiple page size chunks (but will fragament
 * the CPU page tables)
 *
 * # use the dma_pool API for page size chunks
 *
 * The original internal API of gsl_linux_map has been modified to allow
 * various implementations local to this file.
 *
 */
#define MF_USE_DMA_API

#include "gsl.h"

#include <linux/dma-mapping.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include "gsl_linux_map.h"

struct gsl_linux_map
{
	struct list_head list;
	unsigned int gpu_addr;
	void *kernel_virtual_addr;
	unsigned int size;
#ifdef MF_USE_DMA_API
	dma_addr_t dma_addr;
#endif
};

static LIST_HEAD(gsl_linux_map_list);
static DEFINE_MUTEX(gsl_linux_map_mutex);

int gsl_linux_map_init()
{
	mutex_lock(&gsl_linux_map_mutex);
	INIT_LIST_HEAD(&gsl_linux_map_list);
	mutex_unlock(&gsl_linux_map_mutex);

	return 0;
}

struct gsl_linux_map *gsl_linux_map_alloc(unsigned int gpu_addr, unsigned int size)
{
	struct device *linux_dev = gsl_driver.osdep_dev;
	struct gsl_linux_map * map;
	struct list_head *p;
	void *va;
	dma_addr_t dma_addr;

	mutex_lock(&gsl_linux_map_mutex);

	list_for_each(p, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		if(map->gpu_addr == gpu_addr){
			mutex_unlock(&gsl_linux_map_mutex);
			return map->kernel_virtual_addr;
		}
	}

#ifdef MF_USE_DMA_API
	va = dma_alloc_writecombine(linux_dev, size, &dma_addr, GFP_KERNEL);
#else
	va = __vmalloc(size, GFP_KERNEL, pgprot_writecombine(pgprot_kernel));
#endif

	if(va == NULL){
		mutex_unlock(&gsl_linux_map_mutex);
		return NULL;
	}
	map = (struct gsl_linux_map *)kmalloc(sizeof(*map), GFP_KERNEL);
	map->gpu_addr = gpu_addr;
	map->kernel_virtual_addr = va;
	map->size = size;
#ifdef MF_USE_DMA_API
	map->dma_addr = dma_addr;
#endif

	INIT_LIST_HEAD(&map->list);
	list_add_tail(&map->list, &gsl_linux_map_list);

	mutex_unlock(&gsl_linux_map_mutex);
	return map;
}

void gsl_linux_map_free(unsigned int gpu_addr)
{
	struct device *linux_dev = gsl_driver.osdep_dev;
	int found = 0;
	struct gsl_linux_map * map;
	struct list_head *p;

	mutex_lock(&gsl_linux_map_mutex);

	list_for_each(p, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		if(map->gpu_addr == gpu_addr){
			found = 1;
			break;
		}
	}

	if(found){
#ifdef MF_USE_DMA_API
		dma_free_writecombine(linux_dev, map->size, map->kernel_virtual_addr, map->dma_addr);
#else
		vfree(map->kernel_virtual_addr);
#endif
		list_del(&map->list);
		kfree(map);
	}

	mutex_unlock(&gsl_linux_map_mutex);
}


void gsl_linux_map_fill_sg(struct gsl_linux_map *map, unsigned int scattergatterlist[])
{
	int i;
	int numpages = map->size / PAGE_SIZE;

#ifdef MF_USE_DMA_API
	struct device *linux_dev = gsl_driver.osdep_dev;
	phys_addr_t phys = dma_to_phys(linux_dev, map->dma_addr);

	for (i = 0; i < numpages; i++) {
		scattergatterlist[i] = phys;
		phys += PAGE_SIZE;
	}
#else
	void *va = map->kernel_virtual_addr;

	for (i = 0; i < numpages; i++) {
		scattergatterlist[i] = page_to_phys(vmalloc_to_page(va));
		va += PAGE_SIZE;
	}
#endif

}


static struct gsl_linux_map *gsl_linux_map_find_entry(unsigned int gpu_addr)
{
	struct gsl_linux_map * map;
	struct list_head *p;

	mutex_lock(&gsl_linux_map_mutex);

	list_for_each(p, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		if(map->gpu_addr == gpu_addr){
			mutex_unlock(&gsl_linux_map_mutex);
			return map;
		}
	}

	mutex_unlock(&gsl_linux_map_mutex);
	return NULL;
}

void *gsl_linux_map_read(void *dst, unsigned int gpuoffset, unsigned int sizebytes, unsigned int touserspace)
{
	struct gsl_linux_map * map;
	struct list_head *p;

	mutex_lock(&gsl_linux_map_mutex);

	list_for_each(p, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		if(map->gpu_addr <= gpuoffset &&
			(map->gpu_addr +  map->size) > gpuoffset){
			void *src = map->kernel_virtual_addr + (gpuoffset - map->gpu_addr);
			mutex_unlock(&gsl_linux_map_mutex);
                        if (touserspace)
                        {
                            return (void *)copy_to_user(dst, map->kernel_virtual_addr + gpuoffset - map->gpu_addr, sizebytes);
                        }
                        else
                        {
	                    return memcpy(dst, src, sizebytes);
                        }
		}
	}

	mutex_unlock(&gsl_linux_map_mutex);
	return NULL;
}

void *gsl_linux_map_write(void *src, unsigned int gpuoffset, unsigned int sizebytes, unsigned int fromuserspace)
{
	struct gsl_linux_map * map;
	struct list_head *p;

	mutex_lock(&gsl_linux_map_mutex);

	list_for_each(p, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		if(map->gpu_addr <= gpuoffset &&
			(map->gpu_addr +  map->size) > gpuoffset){
			void *dst = map->kernel_virtual_addr + (gpuoffset - map->gpu_addr);
			mutex_unlock(&gsl_linux_map_mutex);
                        if (fromuserspace)
                        {
                            return (void *)copy_from_user(map->kernel_virtual_addr + gpuoffset - map->gpu_addr, src, sizebytes);
                        }
                        else
                        {
                            return memcpy(dst, src, sizebytes);
                        }
		}
	}

	mutex_unlock(&gsl_linux_map_mutex);
	return NULL;
}

void *gsl_linux_map_set(unsigned int gpuoffset, unsigned int value, unsigned int sizebytes)
{
	struct gsl_linux_map * map;
	struct list_head *p;

	mutex_lock(&gsl_linux_map_mutex);

	list_for_each(p, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		if(map->gpu_addr <= gpuoffset &&
			(map->gpu_addr +  map->size) > gpuoffset){
			void *ptr = map->kernel_virtual_addr + (gpuoffset - map->gpu_addr);
			mutex_unlock(&gsl_linux_map_mutex);
			return memset(ptr, value, sizebytes);
		}
	}

	mutex_unlock(&gsl_linux_map_mutex);
	return NULL;
}

int gsl_linux_map_destroy()
{
	struct device *linux_dev = gsl_driver.osdep_dev;
	struct gsl_linux_map * map;
	struct list_head *p, *tmp;

	mutex_lock(&gsl_linux_map_mutex);

	list_for_each_safe(p, tmp, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
#ifdef MF_USE_DMA_API
		dma_free_writecombine(linux_dev, map->size, map->kernel_virtual_addr, map->dma_addr);
#else
		vfree(map->kernel_virtual_addr);
#endif
		list_del(&map->list);
		kfree(map);
	}

	INIT_LIST_HEAD(&gsl_linux_map_list);

	mutex_unlock(&gsl_linux_map_mutex);
	return 0;
}


int gsl_linux_map_mmap(unsigned long gpu_addr, struct vm_area_struct *vma, unsigned long size)
{
	struct device *linux_dev = gsl_driver.osdep_dev;
	struct gsl_linux_map * entry;

#ifndef MF_USE_DMA_API
	unsigned long prot = pgprot_writecombine(vma->vm_page_prot);
	unsigned long start = vma->vm_start;
	void *va;
#endif

	entry = gsl_linux_map_find_entry(gpu_addr);
	if (!entry)
		return -EINVAL;

#ifdef MF_USE_DMA_API
	return 	dma_mmap_writecombine(linux_dev, vma,
			entry->kernel_virtual_addr, entry->dma_addr,size);

#else
	va = entry->kernel_virtual_addr;
	while (size > 0) {
	    if (remap_pfn_range(vma, start, vmalloc_to_pfn(va), PAGE_SIZE, prot)) {
		return -EAGAIN;
	    }
	    start += PAGE_SIZE;
	    va += PAGE_SIZE;
	    size -= PAGE_SIZE;
	}
#endif

	return 0;
}
