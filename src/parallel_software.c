// Software PARALLEL emulation
//
// Copyright (C) 2024  Bevan Weiss <bevan.weiss@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/gpio.h" // gpio_out_setup
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_shutdown
#include "parallelcmds.h" // paralleldev_set_software_bus

struct parallel_sw_config {
    struct gpio_out nwr; // nwr (i8080) or rd/nwr (m6800)
    struct gpio_out nrd_en; // nrd (i8080) or en (m6800)
    struct gpio_out* addr_out;
    struct gpio_in* data_in;
    struct gpio_out* data_out;
    uint8_t addr_width;
    uint8_t data_width;
    uint8_t state;  // used to sequence gpio operations to perform the bus cycle
};

void
command_config_parallelbus_software(const uint32_t *args)
{
    struct parallelbus_s *bus = config_parallelbus(args[0]);
    bus->flags = PARBUS_SOFTWARE_MODE;
    bus->sw_config = alloc_chunk(sizeof(*bus->sw_config));
    bus->sw_config->state = 0;
    bus->sw_config->nwr = gpio_out_setup(args[1], 1);
    bus->sw_config->nrd_en = gpio_out_setup(args[2], 1);
    uint8_t num_addr_pins = args[3];
    bus->sw_config->addr_width = num_addr_pins;
    uint8_t *addr_pins = command_decode_ptr(args[4]);
    uint8_t num_data_pins = args[5];
    bus->sw_config->data_width = num_data_pins;
    uint8_t *data_pins = command_decode_ptr(args[6]);

    bus->sw_config->addr_out = alloc_chunk(num_addr_pins*sizeof(*bus->sw_config->addr_out));
    for(uint8_t addr_pin_idx = 0; addr_pin_idx < num_addr_pins; ++addr_pin_idx) {
        bus->sw_config->addr_out[addr_pin_idx] = gpio_out_setup(addr_pins[addr_pin_idx],0);
    }

    bus->sw_config->data_in = alloc_chunk(num_data_pins*sizeof(*bus->sw_config->data_in));
    bus->sw_config->data_out = alloc_chunk(num_data_pins*sizeof(*bus->sw_config->data_out));
    for(uint8_t data_pin_idx = 0; data_pin_idx < num_data_pins; ++data_pin_idx) {
        bus->sw_config->data_in[data_pin_idx] = gpio_in_setup(addr_pins[data_pin_idx], 0);
        bus->sw_config->data_out[data_pin_idx] = gpio_out_setup(addr_pins[data_pin_idx], 0);
    }
}
DECL_COMMAND(command_config_parallelbus_software, "config_parallelbus_software oid=%c nwr_pin=%c nrd_en_pin=%c addr_pins=%*s data_pins=%*s");

void
paralleldev_software_setup(struct paralleldev_s *dev)
{
    uint8_t mode = (dev->flags & PARDEV_BUS_MODE_MSK);
    if (mode != intel8080 && mode != motorola6800 )
        shutdown("Invalid parallel device mode, supported options are 'intel8080', or 'motorola6800'");
    if (dev->data_width > dev->bus->sw_config->data_width)
        shutdown("Parallel device data width wider than bus width");

    if (dev->flags & PARDEV_HAS_CS) {
        dev->cs_pin = gpio_out_setup(dev->cs_pin_def, !(dev->flags & PARDEV_CS_ACTIVE_HIGH));
    }
}

void
paralleldev_software_write(const struct paralleldev_s *dev, uint32_t dest_start, uint8_t dest_inc, uint8_t src_inc, uint16_t data_len, const uint8_t *src_data)
{
    struct parallel_sw_config* bus = dev->bus->sw_config;
    uint8_t addr_width = bus->addr_width;
    uint8_t data_width = dev->data_width;
    
    uint32_t dest = dest_start;
    uint32_t dest_addr_inc = dest_inc ? data_width >> 3 : 0;

    switch(dev->flags & PARDEV_BUS_MODE_MSK) {
        case intel8080:
            switch(bus->state) {
                case 0:
            }
        break;
        case motorola6800:

        break;
        default:
            
    }
}

void
paralleldev_software_read(const struct paralleldev_s *dev, uint32_t src_start, uint8_t src_inc, uint8_t dest_inc, uint16_t data_len, uint8_t *dest_data)
{
    struct parallel_sw_config* bus = dev->bus->sw_config;
    uint8_t addr_width = bus->addr_width;
    uint8_t data_width = dev->data_width;
    
    uint32_t src = src_start;
    uint32_t src_addr_inc = src_inc ? data_width >> 3 : 0;

    uint8_t *dest = dest_data;
    // we put the 
    switch(dev->flags & PARDEV_BUS_MODE_MSK) {
        case intel8080:


            break;

        case motorola6800:


            break;
    }
}
