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

DECL_ENUMERATION("parallel_bus_mode", "intel8080", intel8080);
DECL_ENUMERATION("parallel_bus_mode", "motorola6800", motorola6800);

struct parallelbus_s *
config_parallelbus(uint8_t oid)
{
    struct parallelbus_s * bus = oid_alloc(oid, config_parallelbus, sizeof(*bus));
    return bus;
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
    bus->hw_config = alloc_chunk(sizeof(*bus->hw_config));
    *bus->hw_config = parallelbus_hwmode_setup(args[1], args[2]);
    bus->flags |= PARBUS_HARDWARE_MODE;
}
DECL_COMMAND(command_config_parallelbus_hardware, "config_parallelbus_hardware oid=%c bus=%c addr_mask=%u");

void
command_config_paralleldev(const uint32_t *args)
{
    struct paralleldev_s *dev = oid_alloc(args[0], command_config_paralleldev,
                                            sizeof(*dev));
    dev->bus = parallelbus_oid_lookup(args[1]);
    dev->cs_pin_def = args[2];
    dev->flags = args[3];
    dev->data_width = args[4];
    if (dev->bus->flags & PARBUS_SOFTWARE_MODE)
        paralleldev_software_setup(dev);
    else
        paralleldev_setup(dev);
}
DECL_COMMAND(command_config_paralleldev,
             "config_paralleldev oid=%c bus=%c cs_pin=%c flags=%c data_width=%c");

struct paralleldev_s *
paralleldev_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_paralleldev);
}

void
command_paralleldev_write(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct paralleldev_s *dev = paralleldev_oid_lookup(oid);
    uint32_t dest_addr = args[1];
    uint8_t dest_inc = args[2] != 0; // true / false
    uint8_t src_inc = args[3] != 0; // true / false
    uint16_t data_len = args[4];
    uint8_t *src_data = command_decode_ptr(args[5]);

    if (dev->bus->flags & PARBUS_SOFTWARE_MODE)
        paralleldev_software_write(dev, dest_addr, dest_inc, src_inc, data_len, src_data);
    else
        paralleldev_write(dev, dest_addr, dest_inc, src_inc, data_len, src_data);
}
DECL_COMMAND(command_paralleldev_write, "paralleldev_write oid=%c dest_addr=%u dest_inc=%c src_inc=%c src_data=%*s");

void
command_paralleldev_read(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct paralleldev_s *dev = paralleldev_oid_lookup(oid);
    uint32_t src_addr = args[1];
    uint8_t src_inc = args[2] != 0; // true / false
    uint8_t dest_inc = args[3] != 0; // true / false
    uint16_t data_len = args[4];
    uint8_t dest_data[data_len];
    paralleldev_read(dev, src_addr, src_inc, dest_inc, data_len, dest_data);
    sendf("paralleldev_read_response oid=%c response=%*s", oid, data_len, dest_data);
}
DECL_COMMAND(command_paralleldev_read, "paralleldev_read_read oid=%c src_addr=%u src_inc=%c dest_inc=%c data_len=%u");


/****************************************************************
 * Shutdown handling
 ****************************************************************/

struct paralleldev_shutdown_s {
    struct paralleldev_s *dev;
    uint32_t shutdown_dest_addr;
    uint16_t shutdown_data_len;
    uint8_t shutdown_dest_inc;
    uint8_t shutdown_src_inc;
    uint8_t shutdown_src_data[];
};

void
command_config_parallel_shutdown(const uint32_t *args)
{
    struct paralleldev_s *dev = paralleldev_oid_lookup(args[1]);
    uint16_t shutdown_data_len = args[5];

    struct paralleldev_shutdown_s *sd = oid_alloc(
        args[0], command_config_parallel_shutdown, sizeof(*sd) + shutdown_data_len);
    sd->dev = dev;
    sd->shutdown_data_len = shutdown_data_len;
    sd->shutdown_dest_addr = args[2];
    sd->shutdown_dest_inc = (args[3] != 0);
    sd->shutdown_src_inc = (args[4] != 0);
    uint8_t *shutdown_src_data = command_decode_ptr(args[6]);
    memcpy(sd->shutdown_src_data, shutdown_src_data, shutdown_data_len);
}
DECL_COMMAND(command_config_parallel_shutdown,
             "config_parallel_shutdown oid=%c dev_oid=%c shutdown_dest_addr=%u shutdown_dest_inc=%c shutdown_src_inc=%c shutdown_src_data=%*s");

void
paralleldev_shutdown(void)
{
    // Cancel any transmissions that may be in progress
    uint8_t oid;
    struct paralleldev_s *dev;
    foreach_oid(oid, dev, command_config_paralleldev) {
        if ((dev->flags & PARDEV_HAS_CS) 
        && !(dev->flags & PARDEV_CS_HW_DRIVEN))
            gpio_out_write(dev->cs_pin, !(dev->flags & PARDEV_CS_ACTIVE_HIGH));
    }

    // Send shutdown messages
    struct paralleldev_shutdown_s *sd;
    foreach_oid(oid, sd, command_config_parallel_shutdown) {
        if (dev->bus->flags & PARBUS_SOFTWARE_MODE) {

        } else {
            paralleldev_write(sd->dev, sd->shutdown_dest_addr, 
                                        sd->shutdown_dest_inc,
                                        sd->shutdown_src_inc, 
                                        sd->shutdown_data_len, 
                                        sd->shutdown_src_data);
        }
    }
}
DECL_SHUTDOWN(paralleldev_shutdown);
