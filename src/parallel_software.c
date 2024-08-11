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

struct parallel_software {
    struct gpio_out nwr; // nwr (i8080) or rd/nwr (m6800)
    struct gpio_out nrd_en; // nrd (i8080) or en (m6800)
    struct gpio_out* addr_out;
    struct gpio_in* data_in;
    struct gpio_out* data_out;
    uint8_t addr_width;
    uint8_t data_width;
};

void
command_config_parallelbus_software(const uint32_t *args)
{
    struct parallelbus_s *bus = config_parallelbus(args[0]);
    par_bus->flags = PARBUS_SOFTWARE_MODE;
    par_bus->parallel_software = alloc_chunk(sizeof(*par_bus->parallel_software));
    par_bus->parallel_software->nwr = gpio_out_setup(args[1], 1);
    par_bus->parallel_software->nrd_en = gpio_out_setup(args[2], 1);
    uint8_t num_addr_pins = args[3];
    par_bus->parallel_software->addr_width = num_addr_pins;
    uint8_t *addr_pins = command_decode_ptr(args[4]);
    uint8_t num_data_pins = args[5];
    par_bus->parallel_software->data_width = num_data_pins;
    uint8_t *data_pins = command_decode_ptr(args[6]);

    par_bus->parallel_software->addr_out = alloc_chunk(num_addr_pins*sizeof(*par_bus->parallel_software->addr_out));
    for(uint8_t addr_pin_idx = 0; addr_pin_idx < num_addr_pins; ++addr_pin_idx)
    {
        par_bus->parallel_software->addr_out[addr_pin_idx] = gpio_out_setup(addr_pins[addr_pin_idx],0);
    }

    par_bus->parallel_software->data_in = alloc_chunk(num_data_pins*sizeof(*par_bus->parallel_software->data_in));
    par_bus->parallel_software->data_out = alloc_chunk(num_data_pins*sizeof(*par_bus->parallel_software->data_out));
    for(uint8_t data_pin_idx = 0; data_pin_idx < num_data_pins; ++data_pin_idx)
    {
        par_bus->parallel_software->data_in[data_pin_idx] = gpio_in_setup(addr_pins[data_pin_idx], 0);
        par_bus->parallel_software->data_out[data_pin_idx] = gpio_out_setup(addr_pins[data_pin_idx], 0);
    }
}
DECL_COMMAND(command_config_parallelbus_software, "config_parallelbus_software oid=%c nwr_pin=%c nrd_en_pin=%c addr_pins=%*c data_pins=%*c");

void
paralleldev_software_setup(struct paralleldev_s *dev)
{
    uint8_t mode = (dev->flags & PARDEV_BUS_MODE);
    if (mode != i80 && mode != m68 )
        shutdown("Invalid parallel device mode");
    if (dev->data_width > dev->bus->parallel_software->data_width)
        shutdown("Parallel device data width wider than bus width");

    if (dev->cs_pin)
    {
        gpio_out_setup(dev->cs_pin, 1);
    }
}

void
paralleldev_software_write(const struct paralleldev_s *dev, uint32_t addr_start, uint8_t addr_inc, uint16_t data_len, const uint8_t *data)
{
    uint8_t addr_width = dev->bus->parallel_software->addr_width;
    uint8_t data_width = dev->bus->parallel_software->data_width;
    uint32_t addr = addr_start;
    struct gpio_out cs_pin = gpio_out_setup(dev->cs_pin, 1);
    struct parallel_software* bus = dev->bus->parallel_software;
    switch(dev->flags & PARDEV_BUS_MODE)
    {
        case i80:

        break;
        case m68:

        break;
        default:
            shutdown("Invalid parallel device mode")
    }


    switch(dev->flags & PARDEV_BUS_MODE)
    {
        case i80:

        break;
        case m68:

        break;
        default:
            shutdown("Invalid parallel device mode")
    }

}

void
paralleldev_software_read(const struct paralleldev_s *dev, uint32_t addr_start, uint8_t addr_inc, uint16_t data_len, uint8_t *data)
{
    uint8_t addr_width = dev->bus->parallel_software->addr_width;
    uint8_t data_width = dev->bus->parallel_software->data_width;
    
    uint32_t addr = addr_start;
    // we put the 
    switch(parallel->mode)
    {
        case i80:


            break;

        case m68:


            break;
    }
}