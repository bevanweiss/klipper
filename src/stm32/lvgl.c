// LVGL Display support
//
// Copyright (C) 2024 Bevan Weiss <bevan.weiss@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "lvglcmds.h"

/* Send short command to the LCD. This function shall wait until the transaction finishes. */
void lvgl_send_cmd(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, const uint8_t *param, size_t param_size)
{

}

/* Send large array of pixel data to the LCD. If necessary, this function has to do the byte-swapping. This function can do the transfer in the background. */
void lvgl_send_color(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, uint8_t *param, size_t param_size)
{

}

// /* Callback is called when background transfer finished */
static void lvgl_send_color_ready_cb(void* hspi)
{
    
}
