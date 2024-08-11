// Commands for sending messages on a Memory Mapped IO Bus
//
// Copyright (C) 2024  Bevan Weiss <bevan.weiss@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_WANT_SOFTWARE_PARALLEL
#include "board/gpio.h" // gpio_out_write
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_SHUTDOWN
#include "parallel_software.h" // parallel_software_setup
#include "parallelcmds.h" // paralleldev_transfer

DECL_ENUMERATION("parallel_bus_mode", "intel8080", i80);
DECL_ENUMERATION("parallel_bus_mode", "motorola6800", m68);

struct parallelbus_s *
config_parallelbus(uint8_t oid)
{
    return oid_alloc(oid, config_parallelbus, sizeof(*bus));
}

struct parallelbus_s *
parallelbus_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, config_parallelbus);
}

void
command_config_parallelbus_hardware(const uint32_t *args)
{
    struct parallelbus_s *bus = config_parallelbus(args[0]);
    bus->parallel_config = alloc_chunk(sizeof(*bus->parallel_config));
    *bus->parallel_config = parallel_setup(args[1], args[2]);
    bus->flags |= PARBUS_HARDWARE_MODE;
}
DECL_COMMAND(command_config_parallelbus_hardware, "config_parallelbus_hardware oid=%c bus=%c addr_mask=%u");

void
command_config_paralleldev(const uint32_t *args)
{
    struct paralleldev_s *dev = oid_alloc(args[0], command_config_paralleldev, sizeof(*dev));
    dev->bus = parallelbus_oid_lookup(args[1]);
    dev->cs_pin = args[2];
    dev->flags = args[3];
    dev->data_width = args[4];
    if (dev->bus->flags & PARBUS_SOFTWARE_MODE)
        paralleldev_software_setup();
}
DECL_COMMAND(command_parallel_set_bus,
             "config_paralleldev oid=%c bus=%c cs_pin=%c flags=%c data_width=%c");

struct paralleldev_s *
paralleldev_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_paralleldev);
}

void
paralleldev_write(struct paralleldev_s *dev, uint32_t addr, uint8_t addr_inc, uint16_t data_len, const uint8_t *data)
{
    uint_fast8_t flags = dev->flags;

    if (dev->bus->flags & IF_SOFTWARE)
        parallel_software_write(dev, addr, addr_inc, data_len, data);
    else
        parallel_write(dev, addr, addr_inc, data_len, data);
}

void
command_paralleldev_write(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct paralleldev_s *dev = paralleldev_oid_lookup(oid);
    uint32_t addr = args[1];
    uint8_t addr_inc = args[2] != 0; // true / false
    uint16_t data_len = args[3];
    uint8_t *data = command_decode_ptr(args[4]);
    paralleldev_write(dev, addr, addr_inc, data_len, data);
}
DECL_COMMAND(command_paralleldev_write, "paralleldev_write oid=%c addr=%u addr_inc=%c data=%*s");

void
command_paralleldev_read(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct paralleldev_s *dev = paralleldev_oid_lookup(oid);
    uint32_t addr = args[1];
    uint8_t addr_inc = args[2] != 0; // true / false
    uint16_t data_len = args[3];
    uint8_t data[data_len];
    paralleldev_read(dev, addr, addr_inc, data_len, data);
    sendf("paralleldev_read_response oid=%c response=%*s", oid, data_len, data);
}
DECL_COMMAND(command_paralleldev_read, "paralleldev_read_read oid=%c addr=%u addr_inc=%c data_len=%u");


/****************************************************************
 * Shutdown handling
 ****************************************************************/

struct paralleldev_shutdown_s {
    struct paralleldev_s *dev;
    uint32_t shutdown_addr;
    uint8_t shutdown_addrinc;
    uint16_t shutdown_data_len;
    uint8_t shutdown_data[];
};

void
command_config_parallel_shutdown(const uint32_t *args)
{
    struct paralleldev_s *dev = paralleldev_oid_lookup(args[1]);
    uint16_t shutdown_data_len = args[4];

    struct paralleldev_shutdown_s *sd = oid_alloc(
        args[0], command_config_parallel_shutdown, sizeof(*sd) + shutdown_data_len);
    sd->dev = dev;
    sd->shutdown_data_len = shutdown_data_len;
    sd->shutdown_addr = args[2];
    sd->shutdown_addrinc = (args[3] != 0);
    uint8_t *shutdown_data = command_decode_ptr(args[5]);
    memcpy(sd->shutdown_data, shutdown_data, shutdown_data_len);
}
DECL_COMMAND(command_config_parallel_shutdown,
             "config_parallel_shutdown oid=%c dev_oid=%c shutdown_addr=%u shutdown_addrinc=%c shutdown_data=%*s");

void
paralleldev_shutdown(void)
{
    // Cancel any transmissions that may be in progress
    uint8_t oid;
    struct paralleldev_s *dev;
    foreach_oid(oid, dev, command_config_paralleldev) {
        if ( !(dev->flags & PARDEV_CS_HW_DRIVEN) && dev->cs_pin != 0)
            gpio_out_setup(dev->cs_pin, !(dev->flags & PARDEV_CS_ACTIVE_HIGH));
    }

    // Send shutdown messages
    struct paralleldev_shutdown_s *sd;
    foreach_oid(oid, sd, command_config_parallel_shutdown) {
        paralleldev_write(sd->dev, sd->shutdown_addr, sd->shutdown_addrinc, sd->shutdown_data_len, sd->shutdown_data);
    }
}
DECL_SHUTDOWN(paralleldev_shutdown);
