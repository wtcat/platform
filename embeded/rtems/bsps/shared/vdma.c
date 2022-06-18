/*
 * Copyright(c) 2022 wtcat
 */
#include "bsp/vdma.h"

static const struct dma_slave_map *dma_filter_match(
	struct dma_device *device, const char *name) {
	if (!device->filter.mapcnt)
		return NULL;
	for (int i = 0; i < device->filter.mapcnt; i++) {
		const struct dma_slave_map *map = &device->filter.map[i];
		if (!strcmp(map->slave, name))
			return map;
	}
	return NULL;
}

struct dma_chan *dma_request_chan(struct drvmgr_dev *dev, 
	const char *name) {
	const struct dma_slave_map *map;
	struct dma_device *dma;
	if (name == NULL)
		return NULL;
	dma = (struct dma_device *)dev->priv;
	_Assert(dma->filter.fn != NULL);
	map = dma_filter_match(dma, name);
	if (map) {
		struct dma_chan *chan;
		rtems_mutex_lock(&dma->lock);
		list_for_each_entry(chan, &dma->channels, node) {
			if (!dma->filter.fn(chan, map->param)) {
				dev_vdbg("%s: %s filter said false\n",
					 __func__, chan->name);
				continue;
			}
			if (!chan->refcnt && dma->device_alloc_chan_resources)
				dma->device_alloc_chan_resources(chan);
			chan->refcnt++;
			rtems_mutex_unlock(&dma->lock);
			return chan;
		}
		rtems_mutex_unlock(&dma->lock);
	}
	return NULL;
}

void dma_release_chan(struct dma_chan *chan) {
	if (chan == NULL)
		return;
	struct dma_device *dma = chan->device;
	rtems_mutex_lock(&dma->lock);
	if (chan->refcnt > 0) {
		chan->refcnt--;
		if (chan->refcnt == 0) {
			if (dma->device_free_chan_resources)
				dma->device_free_chan_resources(chan);
		}
	}
	rtems_mutex_unlock(&dma->lock);
}
