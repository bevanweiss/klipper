// Parallel Bus functions on stm32
//
// Copyright (C) 2024  Bevan Weiss <bevan.weiss@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_STM32F1
#include "board/misc.h" // timer_is_before
#include "command.h" // shutdown
#include "gpio.h" // parallel_setup
#include "internal.h" // GPIO
#include "sched.h" // sched_shutdown
#include "board/irq.h" //irq_disable
#include "parallelcmds.h"
#include "parallel.h"

#define FSMC_FLAGS_BANK_TYPE_MASK 0x07

enum fsmc_flags {
    PARALLEL_FSMC_Bank1     = 0 & FSMC_FLAGS_BANK_TYPE_MASK,
    PARALLEL_FSMC_Bank1E    = 1 & FSMC_FLAGS_BANK_TYPE_MASK,
    PARALLEL_FSMC_Bank2     = 2 & FSMC_FLAGS_BANK_TYPE_MASK,
    PARALLEL_FSMC_Bank3     = 3 & FSMC_FLAGS_BANK_TYPE_MASK,
    PARALLEL_FSMC_Bank4     = 4 & FSMC_FLAGS_BANK_TYPE_MASK,

    PARALLEL_FSMC_SYNC      = 1 << 5,
};

struct fsmc_info {
    void* base_reg_addr;
    uint8_t flags;
    uint8_t bus_width;
};

struct fsmc_cs_info {
    void* base_reg_addr;
    int16_t cs_pin;
    uint8_t bank_flags;
};

#if CONFIG_MACH_STM32F1 || CONFIG_MACH_STM32F4
#define FSMC1_8bit        "PD4,PD5,PD14,PD15,PD0,PD1,PE7,PE8,PE9,PE10"
#define FSMC1_16bit       "PD4,PD5,PD14,PD15,PD0,PD1,PE7,PE8,PE9,PE10,PE11,PE12,PE13,PE14,PE15,PD8,PD9,PD10"
#define FSMC_D0_GPIO      GPIO('D',14)
#define FSMC_D1_GPIO      GPIO('D',15)
#define FSMC_D2_GPIO      GPIO('D',0)
#define FSMC_D3_GPIO      GPIO('D',1)
#define FSMC_D4_GPIO      GPIO('E',7)
#define FSMC_D5_GPIO      GPIO('E',8)
#define FSMC_D6_GPIO      GPIO('E',9)
#define FSMC_D7_GPIO      GPIO('E',10)
#define FSMC_D8_GPIO      GPIO('E',11)
#define FSMC_D9_GPIO      GPIO('E',12)
#define FSMC_D10_GPIO     GPIO('E',13)
#define FSMC_D11_GPIO     GPIO('E',14)
#define FSMC_D12_GPIO     GPIO('E',15)
#define FSMC_D13_GPIO     GPIO('D',8)
#define FSMC_D14_GPIO     GPIO('D',9)
#define FSMC_D15_GPIO     GPIO('D',10)
#define FSMC_CLK_GPIO     GPIO('D',3)
#define FSMC_NOE_GPIO     GPIO('D',4)
#define FSMC_NWE_GPIO     GPIO('D',5)
#define FSMC_NE1_GPIO     GPIO('D',7)
#define FSMC_NE2_GPIO     GPIO('G',9)
#define FSMC_NE3_GPIO     GPIO('G',10)
#define FSMC_NE4_GPIO     GPIO('G',12)
#define FSMC_A0_GPIO      GPIO('F',0)
#define FSMC_A1_GPIO      GPIO('F',1)
#define FSMC_A2_GPIO      GPIO('F',2)
#define FSMC_A3_GPIO      GPIO('F',3)
#define FSMC_A4_GPIO      GPIO('F',4)
#define FSMC_A5_GPIO      GPIO('F',5)
#define FSMC_A6_GPIO      GPIO('F',12)
#define FSMC_A7_GPIO      GPIO('F',13)
#define FSMC_A8_GPIO      GPIO('F',14)
#define FSMC_A9_GPIO      GPIO('F',15)
#define FSMC_A10_GPIO     GPIO('G',0)
#define FSMC_A11_GPIO     GPIO('G',1)
#define FSMC_A12_GPIO     GPIO('G',2)
#define FSMC_A13_GPIO     GPIO('G',3)
#define FSMC_A14_GPIO     GPIO('G',4)
#define FSMC_A15_GPIO     GPIO('G',5)
#define FSMC_A16_GPIO     GPIO('D',11)
#define FSMC_A17_GPIO     GPIO('D',12)
#define FSMC_A18_GPIO     GPIO('D',13)
#define FSMC_A19_GPIO     GPIO('E',3)
#define FSMC_A20_GPIO     GPIO('E',4)
#define FSMC_A21_GPIO     GPIO('E',5)
#define FSMC_A22_GPIO     GPIO('E',6)
#define FSMC_A23_GPIO     GPIO('E',2)
#define FSMC_A24_GPIO     GPIO('G',13)
#define FSMC_A25_GPIO     GPIO('G',14)

#if CONFIG_MACH_STM32F101 ||  CONFIG_MACH_STM32F103 || CONFIG_MACH_STM32F2 || CONFIG_MACH_STM32F4
#define FSMC_NCE2_GPIO    GPIO('D',7)
#define FSMC_NCE3_GPIO    GPIO('G',9)
#define FSMC_NCE4_1_GPIO  GPIO('G',10)
#define FSMC_NCE4_2_GPIO  GPIO('G',11)
#endif
#endif


#if CONFIG_MACH_STM32F1 || CONFIG_MACH_STM32F2 || CONFIG_MACH_STM32F4
DECL_ENUMERATION("parallel_bus", "fsmc1_8bit", 0);
DECL_CONSTANT_STR("BUS_PINS_fsmc1_8bit", FSMC1_8bit);

DECL_ENUMERATION("parallel_bus", "fsmc1_16bit", 1);
DECL_CONSTANT_STR("BUS_PINS_fsmc1_16bit", FSMC1_16bit);
#endif

static const struct fsmc_info parallel_bus[] = {
#if CONFIG_MACH_STM32F1 || CONFIG_MACH_STM32F2 || CONFIG_MACH_STM32F4
    { (void*)FSMC_Bank1, PARALLEL_FSMC_Bank1, 8 },
    { (void*)FSMC_Bank1, PARALLEL_FSMC_Bank1, 16 },
#endif
};

static const uint32_t address_pins[] = {
    FSMC_A0_GPIO,
    FSMC_A1_GPIO,
    FSMC_A2_GPIO,
    FSMC_A3_GPIO,
    FSMC_A4_GPIO,
    FSMC_A5_GPIO,
    FSMC_A6_GPIO,
    FSMC_A7_GPIO,
    FSMC_A8_GPIO,
    FSMC_A9_GPIO,
    FSMC_A10_GPIO,
    FSMC_A11_GPIO,
    FSMC_A12_GPIO,
    FSMC_A13_GPIO,
    FSMC_A14_GPIO,
    FSMC_A15_GPIO,
    FSMC_A16_GPIO,
    FSMC_A17_GPIO,
    FSMC_A18_GPIO,
    FSMC_A19_GPIO,
    FSMC_A20_GPIO,
    FSMC_A21_GPIO,
    FSMC_A22_GPIO,
    FSMC_A23_GPIO,
    FSMC_A24_GPIO,
    FSMC_A25_GPIO,
};


static const struct fsmc_cs_info parallel_bus_cs[] = {
#if CONFIG_MACH_STM32F1 || CONFIG_MACH_STM32F2 || CONFIG_MACH_STM32F4
    { (void*)FSMC_BANK1_1, FSMC_NE1_GPIO, PARALLEL_FSMC_Bank1 },
    { (void*)FSMC_BANK1_2, FSMC_NE2_GPIO, PARALLEL_FSMC_Bank1 },
    { (void*)FSMC_BANK1_3, FSMC_NE3_GPIO, PARALLEL_FSMC_Bank1 },
    { (void*)FSMC_BANK1_4, FSMC_NE4_GPIO, PARALLEL_FSMC_Bank1 },
#if CONFIG_MACH_STM32F101 ||  CONFIG_MACH_STM32F103 || CONFIG_MACH_STM32F2 || CONFIG_MACH_STM32F4
    { (void*)FSMC_BANK2, FSMC_NCE2_GPIO, PARALLEL_FSMC_Bank2 },
    { (void*)FSMC_BANK3, FSMC_NCE3_GPIO, PARALLEL_FSMC_Bank3 },
    { (void*)FSMC_BANK4, FSMC_NCE4_1_GPIO, PARALLEL_FSMC_Bank4 },
  #endif
#endif
};

void
paralleldev_set_timings(struct paralleldev_s *dev, uint_fast8_t addressSetup, uint_fast8_t addressHold, 
                        uint_fast8_t dataSetup, uint_fast8_t dataLatency, uint_fast8_t clockDivision, uint_fast8_t busTurnAround);


struct parallel_hw_config
parallelbus_hwmode_setup(uint32_t bus, uint32_t addr_mask)
{
    // Lookup requested parallel bus
    if (bus >= ARRAY_SIZE(parallel_bus))
        shutdown("Unsupported parallel bus");
    const struct fsmc_info *fsmc = &parallel_bus[bus];

    // TODO: This FSMC shouldn't be hard coded, there may be chips with multiple
    // FSMC units exposed, we should capture their base elsewhere
    if (!is_enabled_pclock((uint32_t)fsmc->base_reg_addr)) {
        // Enable parallel clock and gpio
        enable_pclock((uint32_t)fsmc->base_reg_addr);
    }

    gpio_peripheral(FSMC_NOE_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
    gpio_peripheral(FSMC_NWE_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);

    gpio_peripheral(FSMC_D0_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
    gpio_peripheral(FSMC_D1_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
    gpio_peripheral(FSMC_D2_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
    gpio_peripheral(FSMC_D3_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
    gpio_peripheral(FSMC_D4_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
    gpio_peripheral(FSMC_D5_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
    gpio_peripheral(FSMC_D6_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
    gpio_peripheral(FSMC_D7_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
    if (fsmc->bus_width == 16)
    {
        gpio_peripheral(FSMC_D8_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
        gpio_peripheral(FSMC_D9_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
        gpio_peripheral(FSMC_D10_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
        gpio_peripheral(FSMC_D11_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
        gpio_peripheral(FSMC_D12_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
        gpio_peripheral(FSMC_D13_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
        gpio_peripheral(FSMC_D14_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
        gpio_peripheral(FSMC_D15_GPIO, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
    }

    for(uint8_t addr_idx = 0; addr_idx < ARRAY_SIZE(address_pins); ++addr_idx)
    {
        if (1 << addr_idx & addr_mask)
        {
            gpio_peripheral(address_pins[addr_idx], GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
        }
    }

    RCC->AHBENR |= RCC_AHBPeriph_FSMC;
    output("AHB_clock=%u, APB_clock=%u", RCC->AHBENR, RCC->APB2ENR);

    return (struct parallel_hw_config){
        .base_reg_addr = fsmc->base_reg_addr,
        .data_width = fsmc->bus_width,
        .flags = fsmc->flags,
    };
}

void
paralleldev_setup(struct paralleldev_s *dev)
{
    uint8_t mode = (dev->flags & PARDEV_BUS_MODE_MSK);
    if (mode != intel8080 && mode != motorola6800 )
        shutdown("Invalid parallel device mode, supported options are 'intel8080', or 'motorola6800'");
    if (dev->data_width > dev->bus->hw_config->data_width)
        shutdown("Parallel device data width wider than bus width");

    if (dev->cs_pin_def != 0)
    {
        for(uint8_t cs_idx = 0; cs_idx < ARRAY_SIZE(parallel_bus_cs); ++cs_idx)
        {
            if (dev->cs_pin_def == parallel_bus_cs[cs_idx].cs_pin && ((dev->bus->hw_config->flags & FSMC_FLAGS_BANK_TYPE_MASK) == parallel_bus_cs[cs_idx].bank_flags))
            {
                dev->flags |= PARDEV_CS_HW_DRIVEN;
                gpio_peripheral(dev->cs_pin_def, GPIO_FUNCTION(0) | GPIO_HIGH_SPEED, 0);
                dev->bus_addr = parallel_bus_cs[cs_idx].base_reg_addr;
                break;
            }
        }
        if (!(dev->flags & PARDEV_CS_HW_DRIVEN))
            dev->cs_pin = gpio_out_setup(dev->cs_pin_def, (dev->flags & PARDEV_CS_ACTIVE_HIGH) ? 0 : 1);
    }
    
    FSMC_Bank1_TypeDef* bank = dev->bus->hw_config->base_reg_addr;

    /* Bank1 NOR/SRAM control register configuration */ 
    bank->BTCR[0] = 
                (uint32_t)FSMC_DataAddressMux_Disable |
                FSMC_MemoryType_SRAM |
                FSMC_MemoryDataWidth_16b |
                FSMC_BurstAccessMode_Disable |
                FSMC_AsynchronousWait_Disable |
                FSMC_WaitSignalPolarity_Low |
                FSMC_WrapMode_Disable |
                FSMC_WaitSignalActive_BeforeWaitState |
                FSMC_WriteOperation_Enable |
                FSMC_WaitSignal_Disable |
                FSMC_ExtendedMode_Enable |
                FSMC_WriteBurst_Disable;

    // set default timings
    paralleldev_set_timings(dev, 15, 15, 24, 17, 16, 0);

    // enable
    bank->BTCR[0] |= BCR_MBKEN_Set;
}

void
paralleldev_set_timings(struct paralleldev_s *dev, uint_fast8_t addressSetup, uint_fast8_t addressHold, 
                        uint_fast8_t dataSetup, uint_fast8_t dataLatency, uint_fast8_t clockDivision, uint_fast8_t busTurnAround)
{
    FSMC_Bank1_TypeDef* bank = dev->bus->hw_config->base_reg_addr;
    /* Bank1 NOR/SRAM timing register configuration */
    bank->BTCR[0+1] = 
            (uint32_t)addressSetup |
            (addressHold << 4) |
            (dataSetup << 8) |
            (busTurnAround << 16) |
            (clockDivision << 20) |
            (dataLatency << 24) |
             0;
}

void
paralleldev_write(struct paralleldev_s *dev, uint32_t dest_start, uint8_t dest_inc, uint8_t src_inc, uint16_t data_len, const uint8_t *src_data)
{
    if(!(dev->flags & PARDEV_CS_HW_DRIVEN))
        gpio_out_write(dev->cs_pin, (dev->flags & PARDEV_CS_ACTIVE_HIGH));

    uint16_t* dptr = (uint16_t*)src_data;

    *(uint16_t*)(dev->bus_addr+dest_start) = *dptr;

    if(!(dev->flags & PARDEV_CS_HW_DRIVEN))
        gpio_out_write(dev->cs_pin, !(dev->flags & PARDEV_CS_ACTIVE_HIGH));
}

void
paralleldev_read(struct paralleldev_s *dev, uint32_t src_start, uint8_t src_inc, uint8_t dest_inc, uint16_t data_len, uint8_t *dest_data)
{
    // if the HW isn't driving the CS, we do it manually here
    if(!(dev->flags & PARDEV_CS_HW_DRIVEN))
        gpio_out_write(dev->cs_pin, (dev->flags & PARDEV_CS_ACTIVE_HIGH));

    uint16_t* dptr = (uint16_t*)dest_data;

    *dptr = *(uint16_t*)(dev->bus_addr+src_start);

    // if the HW isn't driving the CS, we do it manually here
    if(!(dev->flags & PARDEV_CS_HW_DRIVEN))
        gpio_out_write(dev->cs_pin, !(dev->flags & PARDEV_CS_ACTIVE_HIGH));
}