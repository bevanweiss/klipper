# Support for Parallel Bus interfaces
#
# Copyright (C) 2024  Bevan Weiss <bevan.weiss@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

class paralleldev:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]

        ppins = printer.lookup_object('pins')

        # Bus Options
        # Find enumerations for the given bus
        par_buses = [ (x) for x,y in printer.lookup_objects('parallelbus')]
        par_bus = ('parallelbus ' + config.get('par_bus', '')).strip()
        if par_bus not in par_buses:
            raise config.error("par_bus '%s' not found, options are: %s" % (par_bus, par_buses))
        self.bus = printer.lookup_object(par_bus)
        self.mcu = mcu = self.bus.get_mcu()
        self.oid = mcu.create_oid()

        self.bus_oid = self.bus.get_oid()

        # Chip Select (CS) Pin
        cs_pin = config.get('cs_pin')
        cs_ppin = ppins.lookup_pin(cs_pin)
        self.cs_pin = cs_ppin['pin']
        cs_pin_mcu = cs_ppin['chip']
        if cs_pin_mcu != self.mcu:
            raise config.error("Parallel Device %s config error: Parallel Bus (%s) and"
                               " cs_pin (%s) must be on the same MCU"
                                % (self.name, self.bus, self.cs_pin) )
        cs_active_high = config.get('cs_active_high', False)
        
        # Bus Mode (i8080, m6800, future others)
        enumerations = mcu.get_enumerations()
        modes = dict(enumerations.get('parallel_bus_mode', [ ('intel8080',0), ('motorola6800',1) ]))
        mode = config.get('bus_mode')
        if mode not in modes:
            raise config.error("Parallel Device %s config error: Unsupported bus mode %s, supported modes are %s"
                               % (self.name, modes.keys() ) )
        bus_mode = modes[mode]
        # Bus Width
        data_width = config.getint('data_width', minval=4)
        if (data_width % 8) != 0:
            raise config.error("Parallel Device %s config error: Data Width (%s) must be multiple of 8"
                               % (self.name, data_width) )
        self.data_width = data_width

        flags = bus_mode & (cs_active_high << 4)

        mcu.add_config_cmd("config_paralleldev oid=%d bus=%d cs_pin=%s flags=%d data_width=%d"
                               % (self.oid, self.bus.get_oid(), self.cs_pin, flags, self.data_width))
        self.mcu.register_config_callback(self._build_config)

        self.cmd_queue = self.bus.get_command_queue()

    def _build_config(self):

        self.pdev_write_cmd = self.mcu.lookup_command(
            "paralleldev_write oid=%c addr=%u addr_inc=%c data=%*s",
            cq=self.cmd_queue)
        self.pdev_read_cmd = self.mcu.lookup_query_command(
            "paralleldev_read_read oid=%c addr=%u addr_inc=%c data_len=%u",
            "paralleldev_read_response oid=%c response=%*s",
            oid=self.oid,
            cq=self.cmd_queue, is_async=True)
    def setup_shutdown_msg(self, shutdown_seq):
        shutdown_msg = "".join(["%02x" % (x,) for x in shutdown_seq])
        self.mcu.add_config_cmd(
            "config_spi_shutdown oid=%d spi_oid=%d shutdown_msg=%s"
            % (self.mcu.create_oid(), self.oid, shutdown_msg))
    def get_oid(self):
        return self.oid
    def get_mcu(self):
        return self.mcu
    def get_command_queue(self):
        return self.cmd_queue
    def pdev_write(self, addr, addr_inc, data, minclock=0, reqclock=0):
        self.pdev_write_cmd.send([self.oid, addr, addr_inc, data],
                               minclock=minclock, reqclock=reqclock)
    def pdev_read(self, addr, addr_inc, data_len):
        return self.pdev_read_cmd.send([self.oid, addr, addr_inc, data_len])


def load_config(config):
    return paralleldev(config)

def load_config_prefix(config):
    return paralleldev(config)