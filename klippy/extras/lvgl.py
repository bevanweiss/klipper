# Support for LVGL LCDs
#
# Copyright (C) 2024  Bevan Weiss <bevan.weiss@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000

class lvgl:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]

        self.mcu = mcu = config.get

        ppins = printer.lookup_object('pins')

        # pin config
        pbus = self.printer.lookup_object('bus')

        enumerations = mcu.get_enumerations()
        driver_choices = enumerations.get('lvgl_driver')
        driver = config.get('driver', 'auto')
        if driver not in driver_choices:
            raise ppins.error("Unknown %s '%s'" % (param, bus))
        self.driver = config.getchoice('driver', driver_choices)
        self.oid = self.mcu.create_oid()

        self.mcu.add_config_cmd("config_lvgl_display oid=%c bus_dev=%c reset_pin=%c backlight_pin=%c driver=%c horizontal=%u vertical=%u flags=%c rotation=%c"
                % (self.oid, nwr_ppin['pin'], nrd_en_ppin['pin'], mcu_addr_pins, mcu_data_pins) )



def load_config(config):
    return lvgl(config)

def load_config_prefix(config):
    return lvgl(config)