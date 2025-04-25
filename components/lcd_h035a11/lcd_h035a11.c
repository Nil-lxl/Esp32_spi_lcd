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

#include "lcd_h035a11.h"

static const char *TAG = "h035a11";

static esp_err_t panel_h035a11_del(esp_lcd_panel_t *panel);
static esp_err_t panel_h035a11_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_h035a11_init(esp_lcd_panel_t *panel);
static esp_err_t panel_h035a11_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_h035a11_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_h035a11_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_h035a11_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_h035a11_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_h035a11_disp_on_off(esp_lcd_panel_t *panel, bool off);

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
    const h035a11_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
} h035a11_panel_t;

esp_err_t esp_lcd_new_panel_h035a11(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    h035a11_panel_t *h035a11 = NULL;
    gpio_config_t io_conf = { 0 };

    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    h035a11 = (h035a11_panel_t *)calloc(1, sizeof(h035a11_panel_t));
    ESP_GOTO_ON_FALSE(h035a11, ESP_ERR_NO_MEM, err, TAG, "no mem for h035a11 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num;
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    switch (panel_dev_config->color_space) {
    case ESP_LCD_COLOR_SPACE_RGB:
        h035a11->madctl_val = 0;
        break;
    case ESP_LCD_COLOR_SPACE_BGR:
        h035a11->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }
#else
    switch (panel_dev_config->rgb_endian) {
    case LCD_RGB_ENDIAN_RGB:
        h035a11->madctl_val = 0;
        break;
    case LCD_RGB_ENDIAN_BGR:
        h035a11->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported rgb endian");
        break;
    }
#endif

    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        h035a11->colmod_val = 0x55;
        h035a11->fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        h035a11->colmod_val = 0x66;
        // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
        h035a11->fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    h035a11->io = io;
    h035a11->reset_gpio_num = panel_dev_config->reset_gpio_num;
    h035a11->reset_level = panel_dev_config->flags.reset_active_high;
    if (panel_dev_config->vendor_config) {
        h035a11->init_cmds = ((h035a11_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds;
        h035a11->init_cmds_size = ((h035a11_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds_size;
    }
    h035a11->base.del = panel_h035a11_del;
    h035a11->base.reset = panel_h035a11_reset;
    h035a11->base.init = panel_h035a11_init;
    h035a11->base.draw_bitmap = panel_h035a11_draw_bitmap;
    h035a11->base.invert_color = panel_h035a11_invert_color;
    h035a11->base.set_gap = panel_h035a11_set_gap;
    h035a11->base.mirror = panel_h035a11_mirror;
    h035a11->base.swap_xy = panel_h035a11_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    h035a11->base.disp_off = panel_h035a11_disp_on_off;
#else
    h035a11->base.disp_on_off = panel_h035a11_disp_on_off;
#endif
    *ret_panel = &(h035a11->base);
    ESP_LOGD(TAG, "new h035a11 panel @%p", h035a11);

    // ESP_LOGI(TAG, "LCD panel create success, version: %d.%d.%d", ESP_LCD_h035a11_VER_MAJOR, ESP_LCD_h035a11_VER_MINOR,
    //          ESP_LCD_h035a11_VER_PATCH);

    return ESP_OK;

err:
    if (h035a11) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(h035a11);
    }
    return ret;
}

static esp_err_t panel_h035a11_del(esp_lcd_panel_t *panel)
{
    h035a11_panel_t *h035a11 = __containerof(panel, h035a11_panel_t, base);

    if (h035a11->reset_gpio_num >= 0) {
        gpio_reset_pin(h035a11->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del h035a11 panel @%p", h035a11);
    free(h035a11);
    return ESP_OK;
}

static esp_err_t panel_h035a11_reset(esp_lcd_panel_t *panel)
{
    h035a11_panel_t *h035a11 = __containerof(panel, h035a11_panel_t, base);
    esp_lcd_panel_io_handle_t io = h035a11->io;

    // perform hardware reset
    if (h035a11->reset_gpio_num >= 0) {
        gpio_set_level(15,1);
        gpio_set_level(h035a11->reset_gpio_num, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(h035a11->reset_gpio_num, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
    } else { // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

static const h035a11_lcd_init_cmd_t vendor_specific_init_default[] = {
//  {cmd, { data }, data_size, delay_ms}
    {0x11, (uint8_t []){0x00}, 0, 120},

    {0xf0,(uint8_t []){0xc3},1,0},
    {0xf0,(uint8_t []){0x96},1,0},
    {0x36,(uint8_t []){0x48},1,0},
    {0x3A,(uint8_t []){0x55},1,0},
    {0xB4,(uint8_t []){0x01},1,0},
    {0xe8,(uint8_t []){0x40,0x8a,0x00,0x00,0x29,0x19,0xa5,0x33},8,0},
    {0xc1,(uint8_t []){0x06},1,0},
    {0xc2,(uint8_t []){0xa7},1,0},
    {0xc5,(uint8_t []){0x18},1,0},
    {0xe0,(uint8_t []){0xf0,0x09,0x0b,0x06,0x04,0x15,0x2f,0x54,0x42,0x3c,0x17,0x14,0x18,0x1b},14,0},
    {0xe1,(uint8_t []){0xf0,0x09,0x0b,0x06,0x04,0x03,0x2d,0x43,0x42,0x3b,0x16,0x14,0x17,0x1b},14,0},
    {0xf0,(uint8_t []){0x3c},1,0},
    {0xf0,(uint8_t []){0x69},1,0},
    
    // {0x21, (uint8_t []){0x00}, 0, 120},
    
    {0x29, (uint8_t []){0x00}, 0, 120},
};

static esp_err_t panel_h035a11_init(esp_lcd_panel_t *panel)
{
    h035a11_panel_t *h035a11 = __containerof(panel, h035a11_panel_t, base);
    esp_lcd_panel_io_handle_t io = h035a11->io;

    const h035a11_lcd_init_cmd_t *init_cmds = NULL;
    uint16_t init_cmds_size = 0;
    if (h035a11->init_cmds) {
        init_cmds = h035a11->init_cmds;
        init_cmds_size = h035a11->init_cmds_size;
    } else {
        init_cmds = vendor_specific_init_default;
        init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(h035a11_lcd_init_cmd_t);
    }

    bool is_cmd_overwritten = false;
    for (int i = 0; i < init_cmds_size; i++) {
        // Check if the command has been used or conflicts with the internal
        switch (init_cmds[i].cmd) {
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

static esp_err_t panel_h035a11_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    h035a11_panel_t *h035a11 = __containerof(panel, h035a11_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = h035a11->io;

    x_start += h035a11->x_gap;
    x_end += h035a11->x_gap;
    y_start += h035a11->y_gap;
    y_end += h035a11->y_gap;

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
    size_t len = (x_end - x_start) * (y_end - y_start) * h035a11->fb_bits_per_pixel / 8;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len), TAG, "send color failed");

    return ESP_OK;
}

static esp_err_t panel_h035a11_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    h035a11_panel_t *h035a11 = __containerof(panel, h035a11_panel_t, base);
    esp_lcd_panel_io_handle_t io = h035a11->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_h035a11_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    // h035a11_panel_t *h035a11 = __containerof(panel, h035a11_panel_t, base);
    // esp_lcd_panel_io_handle_t io = h035a11->io;
    // if (mirror_x) {
    //     h035a11->madctl_val |= LCD_CMD_MX_BIT;
    // } else {
    //     h035a11->madctl_val &= ~LCD_CMD_MX_BIT;
    // }
    // if (mirror_y) {
    //     h035a11->madctl_val |= LCD_CMD_MY_BIT;
    // } else {
    //     h035a11->madctl_val &= ~LCD_CMD_MY_BIT;
    // }
    // ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
    //     h035a11->madctl_val
    // }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_h035a11_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    // h035a11_panel_t *h035a11 = __containerof(panel, h035a11_panel_t, base);
    // esp_lcd_panel_io_handle_t io = h035a11->io;
    // if (swap_axes) {
    //     h035a11->madctl_val |= LCD_CMD_MV_BIT;
    // } else {
    //     h035a11->madctl_val &= ~LCD_CMD_MV_BIT;
    // }
    // ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
    //     h035a11->madctl_val
    // }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_h035a11_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    h035a11_panel_t *h035a11 = __containerof(panel, h035a11_panel_t, base);
    h035a11->x_gap = x_gap;
    h035a11->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_h035a11_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    h035a11_panel_t *h035a11 = __containerof(panel, h035a11_panel_t, base);
    esp_lcd_panel_io_handle_t io = h035a11->io;
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
