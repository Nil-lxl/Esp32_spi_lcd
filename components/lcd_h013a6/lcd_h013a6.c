/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

#include "lcd_h013a6.h"

static const char *TAG = "h013a6";

static esp_err_t panel_h013a6_del(esp_lcd_panel_t *panel);
static esp_err_t panel_h013a6_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_h013a6_init(esp_lcd_panel_t *panel);
static esp_err_t panel_h013a6_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_h013a6_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_h013a6_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_h013a6_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_h013a6_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_h013a6_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_val; // save current value of LCD_CMD_COLMOD register
    const h013a6_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
} h013a6_panel_t;

esp_err_t esp_lcd_new_panel_h013a6(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel) {
    esp_err_t ret = ESP_OK;
    h013a6_panel_t *h013a6 = NULL;
    gpio_config_t io_conf = { 0 };

    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    h013a6 = (h013a6_panel_t *)calloc(1, sizeof(h013a6_panel_t));
    ESP_GOTO_ON_FALSE(h013a6, ESP_ERR_NO_MEM, err, TAG, "no mem for h013a6 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num;
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    switch (panel_dev_config->color_space) {
        case ESP_LCD_COLOR_SPACE_RGB:
            h013a6->madctl_val = 0;
            break;
        case ESP_LCD_COLOR_SPACE_BGR:
            h013a6->madctl_val |= LCD_CMD_BGR_BIT;
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
            break;
    }
#else
    switch (panel_dev_config->rgb_endian) {
        case LCD_RGB_ENDIAN_RGB:
            h013a6->madctl_val = 0;
            break;
        case LCD_RGB_ENDIAN_BGR:
            h013a6->madctl_val |= LCD_CMD_BGR_BIT;
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported rgb endian");
            break;
    }
#endif

    switch (panel_dev_config->bits_per_pixel) {
        case 16: // RGB565
            h013a6->colmod_val = 0x55;
            h013a6->fb_bits_per_pixel = 16;
            break;
        case 18: // RGB666
            h013a6->colmod_val = 0x66;
            // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
            h013a6->fb_bits_per_pixel = 24;
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
            break;
    }

    h013a6->io = io;
    h013a6->reset_gpio_num = panel_dev_config->reset_gpio_num;
    h013a6->reset_level = panel_dev_config->flags.reset_active_high;
    if (panel_dev_config->vendor_config) {
        h013a6->init_cmds = ((h013a6_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds;
        h013a6->init_cmds_size = ((h013a6_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds_size;
    }
    h013a6->base.del = panel_h013a6_del;
    h013a6->base.reset = panel_h013a6_reset;
    h013a6->base.init = panel_h013a6_init;
    h013a6->base.draw_bitmap = panel_h013a6_draw_bitmap;
    h013a6->base.invert_color = panel_h013a6_invert_color;
    h013a6->base.set_gap = panel_h013a6_set_gap;
    h013a6->base.mirror = panel_h013a6_mirror;
    h013a6->base.swap_xy = panel_h013a6_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    h013a6->base.disp_off = panel_h013a6_disp_on_off;
#else
    h013a6->base.disp_on_off = panel_h013a6_disp_on_off;
#endif
    *ret_panel = &(h013a6->base);
    ESP_LOGD(TAG, "new h013a6 panel @%p", h013a6);

    // ESP_LOGI(TAG, "LCD panel create success, version: %d.%d.%d", ESP_LCD_h013a6_VER_MAJOR, ESP_LCD_h013a6_VER_MINOR,
    //          ESP_LCD_h013a6_VER_PATCH);

    return ESP_OK;

err:
    if (h013a6) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(h013a6);
    }
    return ret;
}

static esp_err_t panel_h013a6_del(esp_lcd_panel_t *panel) {
    h013a6_panel_t *h013a6 = __containerof(panel, h013a6_panel_t, base);

    if (h013a6->reset_gpio_num >= 0) {
        gpio_reset_pin(h013a6->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del h013a6 panel @%p", h013a6);
    free(h013a6);
    return ESP_OK;
}

static esp_err_t panel_h013a6_reset(esp_lcd_panel_t *panel) {
    h013a6_panel_t *h013a6 = __containerof(panel, h013a6_panel_t, base);
    esp_lcd_panel_io_handle_t io = h013a6->io;

    // perform hardware reset
    if (h013a6->reset_gpio_num >= 0) {
        gpio_set_level(15, 1);
        gpio_set_level(h013a6->reset_gpio_num, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(h013a6->reset_gpio_num, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
    } else { // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

static const h013a6_lcd_init_cmd_t vendor_specific_init_default[] = {
//  {cmd, { data }, data_size, delay_ms}
    {0xFE, (uint8_t[]) { 0x00 },0,0},
    {0xEF, (uint8_t[]) { 0x00 },0,0},
    {0xEB, (uint8_t[]) { 0x14 },1,0},
    {0x84, (uint8_t[]) { 0x40 },1,0},
    {0x85, (uint8_t[]) { 0xFF },1,0},
    {0x86, (uint8_t[]) { 0xFF },1,0},
    {0x87, (uint8_t[]) { 0xFF },1,0},
    {0x8E, (uint8_t[]) { 0xFF },1,0},
    {0x8F, (uint8_t[]) { 0xFF },1,0},
    {0x88, (uint8_t[]) { 0x0A },1,0},
    {0x89, (uint8_t[]) { 0x21 },1,0},
    {0x8A, (uint8_t[]) { 0x00 },1,0},
    {0x8B, (uint8_t[]) { 0x80 },1,0},
    {0x8C, (uint8_t[]) { 0x01 },1,0},
    {0x8D, (uint8_t[]) { 0x01 },1,0},
    {0xB6, (uint8_t[]) { 0x00,0x20 },2,0},
    {0x36, (uint8_t[]) { 0x48 },1,0},
    {0x3A, (uint8_t[]) { 0x05 },1,0},
    {0x90, (uint8_t[]) { 0x08,0x08,0x08,0x08 },4,0},
    {0xBD, (uint8_t[]) { 0x06 },1,0},
    {0xBC, (uint8_t[]) { 0x00 },1,0},
    {0xFF, (uint8_t[]) { 0x60,0x01,0x04 },3,0},
    {0xC3, (uint8_t[]) { 0x1d },1,0},
    {0xC4, (uint8_t[]) { 0x1d },1,0},
    {0xC9, (uint8_t[]) { 0x25 },1,0},
    {0xBE, (uint8_t[]) { 0x11 },1,0},
    {0xE1, (uint8_t[]) { 0x10,0x0E },2,0},
    {0xDF, (uint8_t[]) { 0x21,0x0c,0x02 },3,0},
    {0xF0, (uint8_t[]) { 0x45,0x09,0x08,0x08,0x26,0x2A },6,0},
    {0xF1, (uint8_t[]) { 0x43,0x70,0x72,0x36,0x37,0x6F },6,0},
    {0xF2, (uint8_t[]) { 0x45,0x09,0x08,0x08,0x26,0x2A },6,0},
    {0xF3, (uint8_t[]) { 0x43,0x70,0x72,0x36,0x37,0x6F },6,0},
    {0xED, (uint8_t[]) { 0x1B,0x0B },2,0},
    {0xAE, (uint8_t[]) { 0x77 },1,0},
    {0xCD, (uint8_t[]) { 0x63 },1,0},
    {0x70, (uint8_t[]) { 0x07,0x07,0x04,0x0E,0x0F,0x09,0x07,0x08,0x03 },9,0},
    {0xE8, (uint8_t[]) { 0x34 },1,0},
    {0x60, (uint8_t[]) { 0x38,0x0B,0x6D,0x6D,0x39,0xF0,0x6D,0x6D },8,0},
    {0x61, (uint8_t[]) { 0x38,0xF4,0x6D,0x6D,0x38,0xF7,0x6D,0x6D },8,0},
    {0x62, (uint8_t[]) { 0x38,0x0D,0x71,0xED,0x70,0x70,0x38,0x0F,0x71,0xEF,0x70,0x70 },12,0},
    {0x63, (uint8_t[]) { 0x38,0x11,0x71,0xF1,0x70,0x70,0x38,0x13,0x71,0xF3,0x70,0x70 },12,0},
    {0x64, (uint8_t[]) { 0x28,0x29,0xF1,0x01,0xF1,0x00,0x07 },7,0},
    {0x66, (uint8_t[]) { 0x3C,0x00,0xCD,0x67,0x45,0x45,0x10,0x00,0x00,0x00 },10,0},
    {0x67, (uint8_t[]) { 0x00,0x3C,0x00,0x00,0x00,0x01,0x54,0x10,0x32,0x98 },10,0},
    {0x74, (uint8_t[]) { 0x10,0x85,0x80,0x00,0x00,0x4E,0x00 },7,0},
    {0x98, (uint8_t[]) { 0x3e,0x07 },2,0},
    {0x35, (uint8_t[]) { 0x00 },1,0},

    {0x21, (uint8_t[]) { 0x00 },0,120},
    {0x11, (uint8_t[]) { 0x00 },0,120},
    {0x29, (uint8_t[]) { 0x00 },0,120},
    {0x2C, (uint8_t[]) { 0x00 },0,120},
    // {0x29, (uint8_t[]) { 0x00 }, 0, 120},
};

static esp_err_t panel_h013a6_init(esp_lcd_panel_t *panel) {
    h013a6_panel_t *h013a6 = __containerof(panel, h013a6_panel_t, base);
    esp_lcd_panel_io_handle_t io = h013a6->io;

    const h013a6_lcd_init_cmd_t *init_cmds = NULL;
    uint16_t init_cmds_size = 0;
    if (h013a6->init_cmds) {
        init_cmds = h013a6->init_cmds;
        init_cmds_size = h013a6->init_cmds_size;
    } else {
        init_cmds = vendor_specific_init_default;
        init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(h013a6_lcd_init_cmd_t);
    }

    bool is_cmd_overwritten = false;
    for (int i = 0; i < init_cmds_size; i++) {
        // Check if the command has been used or conflicts with the internal
        switch (init_cmds[i].cmd) {
        // case LCD_CMD_MADCTL:
        //     is_cmd_overwritten = true;
        //     h013a6->madctl_val = ((uint8_t *)init_cmds[i].data)[0];
        //     break;
        // case LCD_CMD_COLMOD:
            // is_cmd_overwritten = true;
            // h013a6->colmod_val = ((uint8_t *)init_cmds[i].data)[0];
            // break;
            default:
                is_cmd_overwritten = false;
                break;
        }

        if (is_cmd_overwritten) {
            ESP_LOGW(TAG, "The %02Xh command has been used and will be overwritten by external initialization sequence", init_cmds[i].cmd);
        }

        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));
    }
    ESP_LOGW(TAG, "send init commands success");

    return ESP_OK;
}

static esp_err_t panel_h013a6_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data) {
    h013a6_panel_t *h013a6 = __containerof(panel, h013a6_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = h013a6->io;

    x_start += h013a6->x_gap;
    x_end += h013a6->x_gap;  //x-start and x-end is offset by 35 pixels
    y_start += h013a6->y_gap;
    y_end += h013a6->y_gap;

    // define an area of frame memory where MCU can access
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
            x_start & 0xFF,
            ((x_end - 1) >> 8) & 0xFF,
            (x_end - 1) & 0xFF,
    }, 4), TAG, "send command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF,
            y_start & 0xFF,
            ((y_end - 1) >> 8) & 0xFF,
            (y_end - 1) & 0xFF,
    }, 4), TAG, "send command failed");
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * h013a6->fb_bits_per_pixel / 8;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len), TAG, "send color failed");

    return ESP_OK;
}

static esp_err_t panel_h013a6_invert_color(esp_lcd_panel_t *panel, bool invert_color_data) {
    h013a6_panel_t *h013a6 = __containerof(panel, h013a6_panel_t, base);
    esp_lcd_panel_io_handle_t io = h013a6->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_h013a6_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y) {
    // h013a6_panel_t *h013a6 = __containerof(panel, h013a6_panel_t, base);
    // esp_lcd_panel_io_handle_t io = h013a6->io;
    // if (mirror_x) {
    //     h013a6->madctl_val |= LCD_CMD_MX_BIT;
    // } else {
    //     h013a6->madctl_val &= ~LCD_CMD_MX_BIT;
    // }
    // if (mirror_y) {
    //     h013a6->madctl_val |= LCD_CMD_MY_BIT;
    // } else {
    //     h013a6->madctl_val &= ~LCD_CMD_MY_BIT;
    // }
    // ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
    //     h013a6->madctl_val
    // }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_h013a6_swap_xy(esp_lcd_panel_t *panel, bool swap_axes) {
    // h013a6_panel_t *h013a6 = __containerof(panel, h013a6_panel_t, base);
    // esp_lcd_panel_io_handle_t io = h013a6->io;
    // if (swap_axes) {
    //     h013a6->madctl_val |= LCD_CMD_MV_BIT;
    // } else {
    //     h013a6->madctl_val &= ~LCD_CMD_MV_BIT;
    // }
    // ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
    //     h013a6->madctl_val
    // }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_h013a6_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap) {
    // h013a6_panel_t *h013a6 = __containerof(panel, h013a6_panel_t, base);
    // h013a6->x_gap = x_gap;
    // h013a6->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_h013a6_disp_on_off(esp_lcd_panel_t *panel, bool on_off) {
    h013a6_panel_t *h013a6 = __containerof(panel, h013a6_panel_t, base);
    esp_lcd_panel_io_handle_t io = h013a6->io;
    int command = 0;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    on_off = !on_off;
#endif

    if (on_off) {
        command = LCD_CMD_DISPON;
    } else {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}
