// DMA functions on stm32
//
// Copyright (C) 2024  Bevan Weiss <bevan.weiss@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_STM32F1
#include "command.h" // shutdown
#include "gpio.h" // parallel_setup
#include "board/misc.h" // timer_is_before

#if CONFIG_MACH_STM32F1 || CONFIG_MACH_STM32F2 || CONFIG_MACH_STM32F4
DECL_ENUMERATION("dma_channel", "dma1_1", 0);
DECL_ENUMERATION("dma_channel", "dma1_2", 1);
DECL_ENUMERATION("dma_channel", "dma1_3", 2);
DECL_ENUMERATION("dma_channel", "dma1_4", 3);
DECL_ENUMERATION("dma_channel", "dma1_5", 4);
DECL_ENUMERATION("dma_channel", "dma1_6", 5);
DECL_ENUMERATION("dma_channel", "dma1_7", 6);
DECL_ENUMERATION("dma_channel", "dma2_1", 7);
DECL_ENUMERATION("dma_channel", "dma2_2", 8);
DECL_ENUMERATION("dma_channel", "dma2_3", 9);
DECL_ENUMERATION("dma_channel", "dma2_4", 10);
DECL_ENUMERATION("dma_channel", "dma2_5", 11);
#endif

static const struct dma_config dma_channels[] = {
#if CONFIG_MACH_STM32F1 || CONFIG_MACH_STM32F2 || CONFIG_MACH_STM32F4
    { .chan = DMA1_Channel1 },
    { .chan = DMA1_Channel2 },
    { .chan = DMA1_Channel3 },
    { .chan = DMA1_Channel4 },
    { .chan = DMA1_Channel5 },
    { .chan = DMA1_Channel6 },
    { .chan = DMA1_Channel7 },
    { .chan = DMA2_Channel1 },
    { .chan = DMA2_Channel2 },
    { .chan = DMA2_Channel3 },
    { .chan = DMA2_Channel4 },
    { .chan = DMA2_Channel5 },
#endif
};

struct dma_config
dma_setup(uint32_t channel)
{
    // Lookup requested dma channel
    if (channel >= ARRAY_SIZE(dma_channels))
        shutdown("Unsupported dma channel");
    return dma_channels[channel];
}