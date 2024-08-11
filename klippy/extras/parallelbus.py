# Support for Parallel Bus interfaces
#
# Copyright (C) 2024  Bevan Weiss <bevan.weiss@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, mcu, re
from . import bus

re_split = re.compile(r'[;,\||\s]+')

class parallelbus:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]

        ppins = printer.lookup_object('pins')

        self.par_bus = config.get('mcu_bus')
        if self.par_bus == 'software':
            # NWR/RD Pin
            nwr_pin = config.get('nwr_pin')
            nwr_ppin = ppins.lookup_pin(nwr_pin)
            self.mcu = nwr_ppin['chip']

            # NRD/EN pin
            nrd_en_pin = config.get('nrd_en_pin')
            nrd_en_ppin = ppins.lookup_pin(nrd_en_pin)
            if nrd_en_ppin['chip'] != self.mcu:
                raise ppins.error("%s: parallel bus pins must be on same mcu" % (
                config.get_name(),))

            # Data pins (8/16/32)
            data_pins = config.get('data_pins')
            data_pin_list = [x.strip() for x in re.split(';|,|\|| ', data_pins)]
            num_data_pins = len(data_pin_list)
            if (num_data_pins % 8) != 0:
                raise config.error("%s: parallel bus must have multiple of 8 data pins %s"
                                % (self.name, data_pin_list) )
            data_ppins = [ppins.lookup_pin(x) for x in data_pin_list]

            for ppin in data_ppins:
                if ppin['chip'] != self.mcu:
                    raise ppins.error("%s: parallel bus pins must be on same mcu" % (
                    config.get_name(),))

            # Address pins
            addr_pins = config.get('addr_pins', '')
            addr_pin_list = [x.strip() for x in re_split.split(addr_pins)]
            addr_ppins = [ppins.lookup_pin(x) for x in addr_pin_list]

            for ppin in addr_ppins:
                if ppin['chip'] != self.mcu:
                    raise ppins.error("%s: parallel bus pins must be on same mcu" % (
                    config.get_name(),))


            self.oid = oid = self.mcu.create_oid()

            mcu_addr_pins = ",".join([x['pin'] for x in addr_ppins])
            mcu_data_pins = ",".join([x['pin'] for x in data_ppins])
            self.config_fmt = ("config_parallelbus_software oid=%c nwr_pin=%s nrd_en_pin=%s addr_pins=%s data_pins=%s"
                % (oid, nwr_ppin['pin'], nrd_en_ppin['pin'], mcu_addr_pins, mcu_data_pins) )
        else:
            self.mcu = mcu.get_printer_mcu(printer, config.get('mcu', 'mcu'))

            self.addr_mask = addr_mask = config.get('addr_mask', 0xFFFFFFFF)

            self.oid = oid = self.mcu.create_oid()

            self.config_fmt = ("config_parallelbus_hardware oid=%c bus=%%s addr_mask=%u"
                % (oid, addr_mask) )

        self.cmd_queue = self.mcu.alloc_command_queue()
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        if '%' in self.config_fmt:
            self.par_bus = bus.resolve_bus_name(self.mcu, "parallel_bus", self.par_bus)
            self.config_fmt = self.config_fmt % (self.par_bus,)
        self.mcu.add_config_cmd(self.config_fmt)

    def get_mcu(self):
        return self.mcu

    def get_oid(self):
        return self.oid

    def get_command_queue(self):
        return self.cmd_queue

def load_config(config):
    return parallelbus(config)

def load_config_prefix(config):
    return parallelbus(config)

