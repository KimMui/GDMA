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
 */

#ifndef __INCLUDE_NUTTX_DEVICE_DMA_H
#define __INCLUDE_NUTTX_DEVICE_DMA_H

#include <errno.h>

#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_DMA_HW                 "dma"

enum device_dma_event {
    DEVICE_DMA_EVENT_INVALID,
    DEVICE_DMA_EVENT_BLOCKED,
    DEVICE_DMA_EVENT_COMPLETED,
    DEVICE_DMA_EVENT_ERROR_UNKOWN,
};

enum device_dma_cmd {
    DEVICE_DMA_CMD_INVALID,
    DEVICE_DMA_CMD_NONE,
    DEVICE_DMA_CMD_CONTINUE,
    DEVICE_DMA_CMD_STOP,
};

typedef enum device_dma_event (*device_dma_callback)(struct device *dev,
                                                     unsigned int chan,
                                                     enum device_dma_event
                                                                         event,
                                                     void *arg);

struct device_dma_type_ops {
    int(*register_callback)(struct device *dev, unsigned int chan,
                            device_dma_callback callback);
    int (*transfer)(struct device *dev, unsigned int chan, void *src, void *dst,
                    size_t len, void *eom_addr, void *arg);
};

/**
 * @brief Register a callback for the DMA controller
 * @param dev DMA device callback is registered for
 * @param chan DMA channel callback is registered for
 * @param callback The callback routine
 * @return 0: DMA callback is registered for that DMA controller & channel
 *         -errno: Cause of failure
 */
static inline int device_dma_register_callback(struct device *dev,
                                               unsigned int chan,
                                               device_dma_callback callback)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, dma)->register_callback)
        return DEVICE_DRIVER_GET_OPS(dev, dma)->register_callback(dev, chan,
                                                                  callback);

    return -ENOSYS;
}

/**
 * @brief Start a DMA transfer
 * @param dev DMA device to use
 * @param chan DMA channel to use
 * @param src Address of data source
 * @param dst Address of data destination
 * @param len Number of bytes to transfer
 * @param eom_addr Address of EOM area (NULL means don't write to EOM area)
 * @param arg Argument to be passed to the callback
 * @return 0: DMA transfer started
 *         -errno: Cause of failure
 */
static inline int device_dma_transfer(struct device *dev, unsigned int chan,
                                      void *src, void *dst, size_t len,
                                      void *eom_addr, void *arg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, dma)->transfer)
        return DEVICE_DRIVER_GET_OPS(dev, dma)->transfer(dev, chan, src, dst,
                                                         len, eom_addr, arg);

    return -ENOSYS;
}

#endif /* __INCLUDE_NUTTX_DEVICE_DMA_H */
