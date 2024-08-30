# Support for LGVL LCD/OLED displays
#
# Copyright (C) 2024 Bevan Weiss<bevan.weiss@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import mcu

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000

LVGL_HAS_RESET              =   1 << 0
LVGL_RESET_ACTIVE_HIGH      =   1 << 1
LVGL_IS_SPI                 =   1 << 2

class LvglError(Exception):
    pass

class lvgl:
    def __init__(self, config):
        self._printer = config.get_printer()
        ppins = self._printer.lookup_object('pins')
        self._name = config.get_name().split()[-1]
        self._mcu = mcu.get_printer_mcu(self._printer, config.get('lvgl_mcu', 'mcu'))

        driver = config.get('driver')
        avail_drivers = self._mcu.get_enumerations().get('lvgl_driver')
        if avail_drivers is not None and driver not in avail_drivers:
                raise LvglError("Invalid driver '%s', options are %s" % (driver, ",".join( ["'%s'" % (x) for x in avail_drivers] )))
        self._driver = driver

        # bus oid
        self._bus = 0

        self._h_res = config.getint('h_res')
        self._v_res = config.getint('v_res')

        # screen rotation
        rotation = config.get('rotation', '0deg')
        self._rotation = 0

        # lvgl flags
        self._lvgl_flags = 0
        mirrorX = config.get('mirrorX', False)
        if mirrorX:
            self._lvgl_flags |= 1
        mirrorY = config.get('mirrorY', False)
        if mirrorY:
            self._lvgl_flags |= 2
        BGR_mode = config.get('BGR_mode', False)
        if BGR_mode:
            self._lvgl_flags |= 4
        RGB666_mode = config.get('RGB666_mode', False)
        if RGB666_mode:
            self._lvgl_flags |= 8

        # other flags
        self._flags = 0


        # reset pin
        reset_pin = config.get('reset_pin', None)
        self._reset_pin = ""
        if reset_pin is not None:
            self._reset_pin = ppins.lookup_pin(reset_pin) #ppins.setup_pin('digital_out', reset_pin)
            self._flags |= LVGL_HAS_RESET
            if self._reset_pin['chip'] != self._mcu:
                raise LvglError("'reset_pin' must be on same mcu as lvgl bus ('%s')" % self._mcu)

        if reset_pin is not None:
            if config.get('reset_active_high', False):
                self._flags |= LVGL_RESET_ACTIVE_HIGH


        # Register commands
        gcode = self._printer.lookup_object('gcode')
        gcode.register_mux_command('LVGL_SET_COLOR', 'LVGL', self._name,
                               self._cmd_LVGL_SET_COLOR,
                               desc=self.cmd_LVGL_SET_COLOR_help)
        gcode.register_mux_command('LVGL_ADD', 'LVGL', self._name,
                               self._cmd_LVGL_ADD,
                               desc=self.cmd_LVGL_ADD_help)
        self._mcu.register_config_callback(self._build_config)

    cmd_LVGL_SET_COLOR_help = "Sets the background color for the display"
    cmd_LVGL_ADD_help = "Add a graphical object to LVGL display"

    def _build_config(self):
        # Setup config
        self._oid = self._mcu.create_oid()
        self._cmdqueue = self._mcu.alloc_command_queue()
        self._mcu.add_config_cmd("config_lvgl_display oid=%d lvgl_driver=%s bus=%d horizontal=%d vertical=%d rotation=%d lvgl_flags=%d flags=%d reset_pin=%s"
                                 % (self._oid, self._driver, self._bus, self._h_res, self._v_res, self._rotation, self._lvgl_flags, self._flags, self._reset_pin['pin']))
        self._add = self._mcu.lookup_query_command("lvgl_add oid=%c class=%u flags=%u",
                                                   "lvgl_add_response oid=%c lv_obj_t=%u class=%u flags=%u",
                                                   cq=self._cmdqueue)
    def _cmd_LVGL_SET_COLOR(self, gcmd):
        # Set LVGL color
        temp = gcmd.get_int('C', 0)
    def _cmd_LVGL_ADD(self, gcmd):
        lvgl_class = gcmd.get_int('CLASS')
        flags = gcmd.get_int('FLAGS')
        add = self._add.send([self._oid, lvgl_class, flags])
        logging.info("LVGL_ADD Succeeded: lv_obj_t=%d class=%d flags=%d", add['lv_obj_t'], add['class'], add['flags'])

def load_config(config):
    return lvgl(config)

def load_config_prefix(config):
     return lvgl(config)
