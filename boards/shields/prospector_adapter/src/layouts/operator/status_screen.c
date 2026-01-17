#include <lvgl.h>

#include "modifier_indicator.h"
#include "wpm_meter.h"
#include "layer_display.h"
#ifndef CONFIG_PROSPECTOR_ESB_MODE
#include "battery_circles.h"
#include "output.h"
#endif

#include <fonts.h>

static struct zmk_widget_modifier_indicator modifier_indicator_widget;
static struct zmk_widget_wpm_meter wpm_meter_widget;
static struct zmk_widget_layer_display layer_display_widget;
#ifndef CONFIG_PROSPECTOR_ESB_MODE
static struct zmk_widget_battery_circles battery_circles_widget;
static struct zmk_widget_output output_widget;
#endif

lv_obj_t *zmk_display_status_screen() {
    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, 255, LV_PART_MAIN);

    zmk_widget_modifier_indicator_init(&modifier_indicator_widget, screen);
    lv_obj_set_pos(zmk_widget_modifier_indicator_obj(&modifier_indicator_widget), 25, 8);

    zmk_widget_wpm_meter_init(&wpm_meter_widget, screen);
#ifdef CONFIG_PROSPECTOR_ESB_MODE
    lv_obj_set_pos(zmk_widget_wpm_meter_obj(&wpm_meter_widget), 10, 42);
#else
    lv_obj_set_pos(zmk_widget_wpm_meter_obj(&wpm_meter_widget), 10, 42);
#endif

    zmk_widget_layer_display_init(&layer_display_widget, screen);
#ifdef CONFIG_PROSPECTOR_ESB_MODE
    lv_obj_set_pos(zmk_widget_layer_display_obj(&layer_display_widget), 10, 180);
#else
    lv_obj_set_pos(zmk_widget_layer_display_obj(&layer_display_widget), 10, 142);
#endif

#ifndef CONFIG_PROSPECTOR_ESB_MODE
    zmk_widget_battery_circles_init(&battery_circles_widget, screen);
    lv_obj_set_pos(zmk_widget_battery_circles_obj(&battery_circles_widget), 11, 170);

    zmk_widget_output_init(&output_widget, screen);
    lv_obj_set_pos(zmk_widget_output_obj(&output_widget), 148, 170);
#endif

    return screen;
}
