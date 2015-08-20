/**
 * Copyright (c) 2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Mark Greer
 * @brief Pseudo DMA driver that uses memcpy instead of real DMA
 */

#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <nuttx/device.h>
#include <nuttx/device_dma.h>

#include "up_arch.h"

#define TSB_DMA_MEMCPY_DMA_CHANNELS     8 /* Emulate 8 channel DMA ctlr */

struct tsb_dma_memcpy_chan_info {
    sem_t               lock;
    device_dma_callback callback;
};

struct tsb_dma_memcpy_info {
    struct tsb_dma_memcpy_chan_info chan_info[TSB_DMA_MEMCPY_DMA_CHANNELS];
};

static int tsb_dma_memcpy_register_callback(struct device *dev,
                                            unsigned int chan,
                                            device_dma_callback callback)
{
    struct tsb_dma_memcpy_info *info = device_get_private(dev);

    if (!info) {
        return -EIO;
    }

    if (chan >= TSB_DMA_MEMCPY_DMA_CHANNELS) {
        return -EINVAL;
    }

    sem_wait(&info->chan_info[chan].lock);

    info->chan_info[chan].callback = callback;

    sem_post(&info->chan_info[chan].lock);

    return 0;
}

static int tsb_dma_memcpy_transfer(struct device *dev, unsigned int chan,
                                   void *src, void *dst, size_t len,
                                   void *eom_addr, void *arg)
{
    struct tsb_dma_memcpy_info *info = device_get_private(dev);

    if (!info) {
        return -EIO;
    }

    if ((chan >= TSB_DMA_MEMCPY_DMA_CHANNELS) || !src || !dst || !len) {
        return -EINVAL;
    }

    sem_wait(&info->chan_info[chan].lock);

    memcpy(dst, src, len);

    if (info->chan_info[chan].callback) {
        /* Ignore the cmd returned */
        info->chan_info[chan].callback(dev, chan, DEVICE_DMA_EVENT_COMPLETED,
                                       arg);
    }

    if (eom_addr) {
        putreg8(1, eom_addr);
    }

    sem_post(&info->chan_info[chan].lock);

    return 0;
}

static int tsb_dma_memcpy_open(struct device *dev)
{
    struct tsb_dma_memcpy_info *info;
    unsigned int chan;
    int ret, rc = 0;

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    for (chan = 0; chan < TSB_DMA_MEMCPY_DMA_CHANNELS; chan++) {
        ret = sem_init(&info->chan_info[chan].lock, 0, 1);
        if (ret != OK) {
            rc = -errno;
            break;
        }
    }

    if (rc) {
        for (; chan; chan--) {
            sem_destroy(&info->chan_info[chan - 1].lock);
        }

        free(info);
        return rc;
    }

    device_set_private(dev, info);

    return 0;
}

static void tsb_dma_memcpy_close(struct device *dev)
{
    struct tsb_dma_memcpy_info *info = device_get_private(dev);
    unsigned int chan;

    if (!info) {
        return;
    }

    for (chan = 0; chan < TSB_DMA_MEMCPY_DMA_CHANNELS; chan++) {
        sem_destroy(&info->chan_info[chan].lock);
    }

    device_set_private(dev, NULL);
    free(info);
}

static struct device_dma_type_ops tsb_dma_memcpy_type_ops = {
    .register_callback  = tsb_dma_memcpy_register_callback,
    .transfer           = tsb_dma_memcpy_transfer,
};

static struct device_driver_ops tsb_dma_memcpy_driver_ops = {
    .open       = tsb_dma_memcpy_open,
    .close      = tsb_dma_memcpy_close,
    .type_ops   = &tsb_dma_memcpy_type_ops,
};

struct device_driver tsb_dma_memcpy_driver = {
    .type   = DEVICE_TYPE_DMA_HW,
    .name   = "tsb_dma_memcpy",
    .desc   = "TSB memcpy Psuedo DMA Driver",
    .ops    = &tsb_dma_memcpy_driver_ops,
};
