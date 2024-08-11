// LVGL Display support
//
// Copyright (C) 2024 Bevan Weiss <bevan.weiss@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // struct gpio_in
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "lvgl.h"
#include "../lib/lv_conf.h"
#include "src/drivers/display/lcd/lv_lcd_generic_mipi.h"
#include "src/drivers/display/ili9341/lv_ili9341.h"
#include "src/drivers/display/st7735/lv_st7735.h"
#include "src/drivers/display/st7789/lv_st7789.h"
#include "src/drivers/display/st7796/lv_st7796.h"


static bool lvgvl_initialised = false;


struct lvgl_display {
    union {
        struct {
            uint16_t** REG;
            uint16_t** RAM;
        } parallel;
    } registers;
    void* bus_dev;
    struct gpio_out reset_pin;
    struct gpio_pwm backlight_pin;
    uint8_t flags;
    struct timer timer;
    uint32_t rest_time, sample_time, next_begin_time;
    lv_display_t *lvgl_display;
};
//static lv_display_t *lvgl_display;

extern void lvgl_send_cmd(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, const uint8_t *param, size_t param_size);
extern void lvgl_send_color(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, uint8_t *param, size_t param_size);

static struct task_wake lvgl_wake;

typedef enum lvgl_driver {
    lvgl_driver_auto,
    lvgl_driver_mipi,
    lvgl_driver_ili9341,
    lvgl_driver_st7735,
    lvgl_driver_st7789,
    lvgl_driver_st7796
} lvgl_driver_t;

#if LV_USE_GENERIC_MIPI
DECL_ENUMERATION("lvgl_driver", "auto", lvgl_driver_auto);
DECL_ENUMERATION("lvgl_driver", "mipi", lvgl_driver_mipi);
#endif
#if LV_USE_ILI9341
DECL_ENUMERATION("lvgl_driver", "ili9341", lvgl_driver_ili9341);
#endif
#if LV_USE_ST7735
DECL_ENUMERATION("lvgl_driver", "st7735", lvgl_driver_st7735);
#endif
#if LV_USE_ST7789
DECL_ENUMERATION("lvgl_driver", "st7789", lvgl_driver_st7789);
#endif
#if LV_USE_ST7796
DECL_ENUMERATION("lvgl_driver", "st7796", lvgl_driver_st7796);
#endif

DECL_CONSTANT("lvgl_flag_mirrorX", LV_LCD_FLAG_MIRROR_X);
DECL_CONSTANT("lvgl_flag_mirrorY", LV_LCD_FLAG_MIRROR_Y);
DECL_CONSTANT("lvgl_flag_BGR_mode", LV_LCD_FLAG_BGR);
DECL_CONSTANT("lvgl_flag_RGB666_mode", LV_LCD_FLAG_RGB666);

DECL_ENUMERATION("lvgl_rotation", "0deg", LV_DISPLAY_ROTATION_0);
DECL_ENUMERATION("lvgl_rotation", "90deg", LV_DISPLAY_ROTATION_90);
DECL_ENUMERATION("lvgl_rotation", "180deg", LV_DISPLAY_ROTATION_180);
DECL_ENUMERATION("lvgl_rotation", "270deg", LV_DISPLAY_ROTATION_270);

DECL_ENUMERATION("lvgl_input_type", "pointer", LV_INDEV_TYPE_POINTER); /**< Touch pad, mouse, external button*/
DECL_ENUMERATION("lvgl_input_type", "keypad", LV_INDEV_TYPE_KEYPAD);
DECL_ENUMERATION("lvgl_input_type", "button", LV_INDEV_TYPE_BUTTON);
DECL_ENUMERATION("lvgl_input_type", "encoder", LV_INDEV_TYPE_ENCODER);


static uint_fast8_t
lvgl_event(struct timer *timer)
{
    struct lvgl_display *lv = container_of(timer, struct lvgl_display, timer);

    uint32_t wait_time_us = 1000*lv_timer_handler();
    sched_wake_task(&lvgl_wake);
    lv->timer.waketime += timer_from_us(wait_time_us);
    return SF_RESCHEDULE;
}


// uint32_t lvgl_get_id(void)
// {
//   uint32_t id;
//   writeReg(0x0000);
//   id = LCD->RAM;

//   if (id == 0)
//     id = readID(LCD_READ_ID);
//   if ((id & 0xFFFF) == 0 || (id & 0xFFFF) == 0xFFFF)
//     id = readID(LCD_READ_ID4);
//   if ((id & 0xFF00) == 0 && (id & 0xFF) != 0)
//     id = readID(LCD_READ_ID4);
//   return id;
// }

//  uint32_t lvgl_read_id(const uint16_t inReg)
//  {
//    uint32_t id;
//    writeReg(inReg);
//    id = LCD->RAM; // dummy read
//    id = inReg << 24;
//    id |= (LCD->RAM & 0x00FF) << 16;
//    id |= (LCD->RAM & 0x00FF) << 8;
//    id |= LCD->RAM & 0x00FF;
//    return id;
//  }

/**
 * Basic example to create a "Hello world" label
 */
void lv_example_get_started_1(void)
{
    /*Change the active screen's background color*/
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x003a57), LV_PART_MAIN);

    /*Create a white label, set its text and align it to the center*/
    lv_obj_t * label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello world");
    lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void
command_config_lvgl_display(uint32_t *args)
{
    struct lvgl_display *lv = oid_alloc(args[0], command_config_lvgl_display, sizeof(*lv));

    lv->reset_pin = gpio_out_setup(args[2], 0);
    lv->backlight_pin = gpio_pwm_setup(args[3], 10000, 0);
    lvgl_driver_t driver_type = args[4];
    int8_t horizontal_res = args[5];
    int8_t vertical_res = args[6];
    uint32_t flags = args[7];
    lv_display_rotation_t rotation = args[8];

    /* Initialize LVGL */
    if (!lvgvl_initialised) {
        lvgvl_initialised = true;
        lv_init();
        lv->timer.func = lvgl_event;
    }

    gpio_out_write(lv->reset_pin, 1);
    lv_delay_ms(10);


    switch(driver_type){
#if LV_USE_GENERIC_MIPI
        case lvgl_driver_auto:

        break;

        case lvgl_driver_mipi:
            lv->lvgl_display = lv_lcd_generic_mipi_create(horizontal_res, vertical_res, flags, lvgl_send_cmd, lvgl_send_color);
        break;
#endif
#if LV_USE_ILI9341
        case lvgl_driver_ili9341:
            lv->lvgl_display = lv_ili9341_create(horizontal_res, vertical_res, flags, lvgl_send_cmd, lvgl_send_color);
        break;
#endif
#if LV_USE_ST7735
        case lvgl_driver_st7735:
            lv->lvgl_display = lv_st7735_create(horizontal_res, vertical_res, flags, lvgl_send_cmd, lvgl_send_color);
        break;
#endif
#if LV_USE_ST7789
        case lvgl_driver_st7789:
            lv->lvgl_display = lv_st7789_create(horizontal_res, vertical_res, flags, lvgl_send_cmd, lvgl_send_color);
        break;
#endif
#if LV_USE_ST7796
        case lvgl_driver_st7796:
            lv->lvgl_display = lv_st7796_create(horizontal_res, vertical_res, flags, lvgl_send_cmd, lvgl_send_color);
        break;
#endif
        default:
            shutdown("Invalid lvgl driver specified");
    }

    /* Set display orientation */
    lv_display_set_rotation(lv->lvgl_display, rotation);

    /* Configure draw buffers, etc. */
    uint8_t * buf1 = NULL;
    uint8_t * buf2 = NULL;
    uint32_t buf_size = horizontal_res * vertical_res * lv_color_format_get_size(lv_display_get_color_format(lv->lvgl_display)) / 10;

    buf1 = lv_malloc(buf_size);
    if(buf1 == NULL) {
            shutdown("display draw buffer malloc failed");
            return;
    }

    lv_display_set_buffers(lv->lvgl_display, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_example_get_started_1();
}
DECL_COMMAND(command_config_lvgl_display, "config_lvgl_display oid=%c bus_dev=%c reset_pin=%c backlight_pin=%c driver=%c horizontal=%u vertical=%u flags=%c rotation=%c");

struct lvgl_display *
lvgl_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_lvgl_display);
}

void
command_lvgl_send_mipi_cmds(uint32_t *args)
{
    struct lvgl_display *lv = lvgl_oid_lookup(args[0]);
#ifdef LV_USE_GENERIC_MIPI

    uint16_t data_len = args[1];
    uint8_t *data = command_decode_ptr(args[2]);
    if(data_len > 0 ) {
        // the last element MUST be LV_LCD_CMD_EOF... we're going to overwrite it to ENSURE
        *(data+data_len-1) = LV_LCD_CMD_EOF;
        lv_lcd_generic_mipi_send_cmd_list(lv->lvgl_display, data);
    }
#endif
}
DECL_COMMAND(command_lvgl_send_mipi_cmds, "lvgl_send_mipi_cmds oid=%c cmds=%*s");

void
command_config_lvgl_input(uint32_t *args)
{
    // lv_indev_t * indev = lv_indev_create();           /*Create an input device*/
    // lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);  /*Touch pad is a pointer-like device*/
    // lv_ondev_set_read_cb(indev, my_touchpad_read);    /*Set your driver function*/

    // uint8_t button_count = args[1];
    // if (button_count > 8)
    //     shutdown("Max of 8 buttons");
    // struct buttons *b = oid_alloc(
    //     args[0], command_config_buttons
    //     , sizeof(*b) + sizeof(b->pins[0]) * button_count);
    // b->button_count = button_count;
    // b->time.func = buttons_event;
}
DECL_COMMAND(command_config_lvgl_input, "config_lvgl_input oid=%c horizontal=%u vertical=%u ");

void
command_lvgl_add(uint32_t *args)
{
    // struct buttons *b = oid_lookup(args[0], command_config_buttons);
    // uint8_t pos = args[1];
    // if (pos >= b->button_count)
    //     shutdown("Set button past maximum button count");
    // b->pins[pos] = gpio_in_setup(args[2], args[3]);
}
DECL_COMMAND(command_lvgl_add, "lvgl_add oid=%c pos=%c");

void
command_lvgl_query(uint32_t *args)
{
    // struct buttons *b = oid_lookup(args[0], command_config_buttons);
    // sched_del_timer(&b->time);
    // b->time.waketime = args[1];
    // b->rest_ticks = args[2];
    // b->pressed = b->last_pressed = args[4];
    // b->ack_count = b->report_count = 0;
    // b->retransmit_state = BF_ACKED;
    // b->retransmit_count = args[3];
    // if (b->retransmit_count >= BF_NO_RETRANSMIT)
    //     shutdown("Invalid buttons retransmit count");
    // if (! b->rest_ticks)
    //     return;
    // sched_add_timer(&b->time);
}
DECL_COMMAND(command_lvgl_query, "lvgl_query oid=%c clock=%u rest_ticks=%u retransmit_count=%c invert=%c");

void
command_lvgl_ack(uint32_t *args)
{
    // struct buttons *b = oid_lookup(args[0], command_config_buttons);
    // uint8_t count = args[1];
    // b->ack_count += count;
    // irq_disable();
    // if (count >= b->report_count) {
    //     b->report_count = 0;
    //     b->retransmit_state = BF_ACKED;
    // } else {
    //     uint8_t pending = b->report_count - count, i;
    //     for (i=0; i<pending; i++)
    //         b->reports[i] = b->reports[i+count];
    //     b->report_count = pending;
    // }
    // irq_enable();
}
DECL_COMMAND(command_lvgl_ack, "lvgl_ack oid=%c count=%c");

void
lvgl_task(void)
{
    if (!sched_check_wake(&lvgl_wake))
        return;
    lv_timer_handler();


    // if (!sched_check_wake(&buttons_wake))
    //     return;
    // uint8_t oid;
    // struct buttons *b;
    // foreach_oid(oid, b, command_config_buttons) {
    //     // See if need to transmit buttons_state
    //     if (b->retransmit_state != BF_PENDING)
    //         continue;
    //     // Generate message
    //     irq_disable();
    //     uint8_t report_count = b->report_count;
    //     if (!report_count) {
    //         irq_enable();
    //         continue;
    //     }
    //     b->retransmit_state = b->retransmit_count;
    //     irq_enable();
    //     sendf("buttons_state oid=%c ack_count=%c state=%*s"
    //           , oid, b->ack_count, report_count, b->reports);
    // }

    //sendf("lvgl_event oid=%c error=%c response=%*s", oid, err, response_len, response);
}
DECL_TASK(lvgl_task);
