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

#include "lcd_h020a05.h"

static const char *TAG = "h020a05";

static esp_err_t panel_h020a05_del(esp_lcd_panel_t *panel);
static esp_err_t panel_h020a05_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_h020a05_init(esp_lcd_panel_t *panel);
static esp_err_t panel_h020a05_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_h020a05_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_h020a05_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_h020a05_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_h020a05_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_h020a05_disp_on_off(esp_lcd_panel_t *panel, bool off);

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
    const h020a05_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
} h020a05_panel_t;

esp_err_t esp_lcd_new_panel_h020a05(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    h020a05_panel_t *h020a05 = NULL;
    gpio_config_t io_conf = { 0 };

    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    h020a05 = (h020a05_panel_t *)calloc(1, sizeof(h020a05_panel_t));
    ESP_GOTO_ON_FALSE(h020a05, ESP_ERR_NO_MEM, err, TAG, "no mem for h020a05 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num;
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    switch (panel_dev_config->color_space) {
    case ESP_LCD_COLOR_SPACE_RGB:
        h020a05->madctl_val = 0;
        break;
    case ESP_LCD_COLOR_SPACE_BGR:
        h020a05->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }
#else
    switch (panel_dev_config->rgb_endian) {
    case LCD_RGB_ENDIAN_RGB:
        h020a05->madctl_val = 0;
        break;
    case LCD_RGB_ENDIAN_BGR:
        h020a05->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported rgb endian");
        break;
    }
#endif

    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        h020a05->colmod_val = 0x55;
        h020a05->fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        h020a05->colmod_val = 0x66;
        // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
        h020a05->fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    h020a05->io = io;
    h020a05->reset_gpio_num = panel_dev_config->reset_gpio_num;
    h020a05->reset_level = panel_dev_config->flags.reset_active_high;
    if (panel_dev_config->vendor_config) {
        h020a05->init_cmds = ((h020a05_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds;
        h020a05->init_cmds_size = ((h020a05_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds_size;
    }
    h020a05->base.del = panel_h020a05_del;
    h020a05->base.reset = panel_h020a05_reset;
    h020a05->base.init = panel_h020a05_init;
    h020a05->base.draw_bitmap = panel_h020a05_draw_bitmap;
    h020a05->base.invert_color = panel_h020a05_invert_color;
    h020a05->base.set_gap = panel_h020a05_set_gap;
    h020a05->base.mirror = panel_h020a05_mirror;
    h020a05->base.swap_xy = panel_h020a05_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    h020a05->base.disp_off = panel_h020a05_disp_on_off;
#else
    h020a05->base.disp_on_off = panel_h020a05_disp_on_off;
#endif
    *ret_panel = &(h020a05->base);
    ESP_LOGD(TAG, "new h020a05 panel @%p", h020a05);

    // ESP_LOGI(TAG, "LCD panel create success, version: %d.%d.%d", ESP_LCD_h020a05_VER_MAJOR, ESP_LCD_h020a05_VER_MINOR,
    //          ESP_LCD_h020a05_VER_PATCH);

    return ESP_OK;

err:
    if (h020a05) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(h020a05);
    }
    return ret;
}

static esp_err_t panel_h020a05_del(esp_lcd_panel_t *panel)
{
    h020a05_panel_t *h020a05 = __containerof(panel, h020a05_panel_t, base);

    if (h020a05->reset_gpio_num >= 0) {
        gpio_reset_pin(h020a05->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del h020a05 panel @%p", h020a05);
    free(h020a05);
    return ESP_OK;
}

static esp_err_t panel_h020a05_reset(esp_lcd_panel_t *panel)
{
    h020a05_panel_t *h020a05 = __containerof(panel, h020a05_panel_t, base);
    esp_lcd_panel_io_handle_t io = h020a05->io;

    // perform hardware reset
    if (h020a05->reset_gpio_num >= 0) {
        gpio_set_level(15,1);
        gpio_set_level(h020a05->reset_gpio_num, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(h020a05->reset_gpio_num, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
    } else { // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

static const h020a05_lcd_init_cmd_t vendor_specific_init_default[] = {
//  {cmd, { data }, data_size, delay_ms}
    {0x11, (uint8_t []){0x00}, 0, 120},
    {0x36, (uint8_t []){0x00}, 1, 0},
    {0x3A, (uint8_t []){0x05}, 1, 0},
    {0xB2, (uint8_t []){0x0C,0x0C,0x00,0x33,0x33}, 5, 0},
    {0xB7, (uint8_t []){0x35}, 1, 0},
    {0xBB, (uint8_t []){0x1A}, 1, 0},
    {0xC0, (uint8_t []){0x2C}, 1, 0},
    {0xC2, (uint8_t []){0x01}, 1, 0},
    {0xC3, (uint8_t []){0x0b}, 1, 0},
    {0xC4, (uint8_t []){0x20}, 1, 0},
    {0xC6, (uint8_t []){0x0f}, 1, 0},
    {0xD0, (uint8_t []){0xa4,0xa1}, 2, 0},
    {0xD6, (uint8_t []){0xa1}, 1, 0},

    {0xE0, (uint8_t []){0xf0,0x06,0x0b,0x07,0x07,0x24,0x2e,0x32,0x46,0x37,0x13,0x13,0x2d,0x33}, 14, 0},
    {0xE1, (uint8_t []){0xf0,0x02,0x06,0x09,0x08,0x05,0x29,0x44,0x42,0x38,0x14,0x14,0x2a,0x30}, 14, 0},
    
    {0x21, (uint8_t []){0x00}, 0, 0},
    
    {0x29, (uint8_t []){0x00}, 0, 120},
};

static esp_err_t panel_h020a05_init(esp_lcd_panel_t *panel)
{
    h020a05_panel_t *h020a05 = __containerof(panel, h020a05_panel_t, base);
    esp_lcd_panel_io_handle_t io = h020a05->io;

    const h020a05_lcd_init_cmd_t *init_cmds = NULL;
    uint16_t init_cmds_size = 0;
    if (h020a05->init_cmds) {
        init_cmds = h020a05->init_cmds;
        init_cmds_size = h020a05->init_cmds_size;
    } else {
        init_cmds = vendor_specific_init_default;
        init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(h020a05_lcd_init_cmd_t);
    }

    bool is_cmd_overwritten = false;
    for (int i = 0; i < init_cmds_size; i++) {
        // Check if the command has been used or conflicts with the internal
        switch (init_cmds[i].cmd) {
        // case LCD_CMD_MADCTL:
        //     is_cmd_overwritten = true;
        //     h020a05->madctl_val = ((uint8_t *)init_cmds[i].data)[0];
        //     break;
        // case LCD_CMD_COLMOD:
            // is_cmd_overwritten = true;
            // h020a05->colmod_val = ((uint8_t *)init_cmds[i].data)[0];
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

static esp_err_t panel_h020a05_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    h020a05_panel_t *h020a05 = __containerof(panel, h020a05_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = h020a05->io;

    x_start += h020a05->x_gap+35;
    x_end += h020a05->x_gap+35;  //x-start and x-end is offset by 35 pixels
    y_start += h020a05->y_gap;
    y_end += h020a05->y_gap;

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
    size_t len = (x_end - x_start) * (y_end - y_start) * h020a05->fb_bits_per_pixel / 8;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len), TAG, "send color failed");

    return ESP_OK;
}

static esp_err_t panel_h020a05_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    h020a05_panel_t *h020a05 = __containerof(panel, h020a05_panel_t, base);
    esp_lcd_panel_io_handle_t io = h020a05->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_h020a05_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    // h020a05_panel_t *h020a05 = __containerof(panel, h020a05_panel_t, base);
    // esp_lcd_panel_io_handle_t io = h020a05->io;
    // if (mirror_x) {
    //     h020a05->madctl_val |= LCD_CMD_MX_BIT;
    // } else {
    //     h020a05->madctl_val &= ~LCD_CMD_MX_BIT;
    // }
    // if (mirror_y) {
    //     h020a05->madctl_val |= LCD_CMD_MY_BIT;
    // } else {
    //     h020a05->madctl_val &= ~LCD_CMD_MY_BIT;
    // }
    // ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
    //     h020a05->madctl_val
    // }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_h020a05_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    // h020a05_panel_t *h020a05 = __containerof(panel, h020a05_panel_t, base);
    // esp_lcd_panel_io_handle_t io = h020a05->io;
    // if (swap_axes) {
    //     h020a05->madctl_val |= LCD_CMD_MV_BIT;
    // } else {
    //     h020a05->madctl_val &= ~LCD_CMD_MV_BIT;
    // }
    // ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
    //     h020a05->madctl_val
    // }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_h020a05_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    // h020a05_panel_t *h020a05 = __containerof(panel, h020a05_panel_t, base);
    // h020a05->x_gap = x_gap;
    // h020a05->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_h020a05_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    h020a05_panel_t *h020a05 = __containerof(panel, h020a05_panel_t, base);
    esp_lcd_panel_io_handle_t io = h020a05->io;
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
