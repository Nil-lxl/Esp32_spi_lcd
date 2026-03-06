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

#include "spi_lcd.h"
#include "init_code.h"

static const char *TAG = "spi_lcd";

static esp_err_t panel_spi_del(esp_lcd_panel_t *panel);
static esp_err_t panel_spi_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_spi_init(esp_lcd_panel_t *panel);
static esp_err_t panel_spi_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_spi_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_spi_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_spi_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_spi_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_spi_disp_on_off(esp_lcd_panel_t *panel, bool off);

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
    const spi_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
} spi_panel_t;

esp_err_t esp_lcd_new_panel_spi(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel) {
    esp_err_t ret = ESP_OK;
    spi_panel_t *spilcd = NULL;
    gpio_config_t io_conf = { 0 };

    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    spilcd = (spi_panel_t *)calloc(1, sizeof(spi_panel_t));
    ESP_GOTO_ON_FALSE(spilcd, ESP_ERR_NO_MEM, err, TAG, "no mem for spi panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num;
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }
    switch (panel_dev_config->rgb_endian) {
        case LCD_RGB_ENDIAN_RGB:
            spilcd->madctl_val = 0;
            break;
        case LCD_RGB_ENDIAN_BGR:
            spilcd->madctl_val |= LCD_CMD_BGR_BIT;
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported rgb endian");
            break;
    }
    switch (panel_dev_config->bits_per_pixel) {
        case 16: // RGB565
            spilcd->colmod_val = 0x55;
            spilcd->fb_bits_per_pixel = 16;
            break;
        case 18: // RGB666
            spilcd->colmod_val = 0x66;
            // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
            spilcd->fb_bits_per_pixel = 24;
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
            break;
    }

    spilcd->io = io;
    spilcd->reset_gpio_num = panel_dev_config->reset_gpio_num;
    spilcd->reset_level = panel_dev_config->flags.reset_active_high;
    if (panel_dev_config->vendor_config) {
        spilcd->init_cmds = ((spi_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds;
        spilcd->init_cmds_size = ((spi_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds_size;
    }
    spilcd->base.del = panel_spi_del;
    spilcd->base.reset = panel_spi_reset;
    spilcd->base.init = panel_spi_init;
    spilcd->base.draw_bitmap = panel_spi_draw_bitmap;
    spilcd->base.invert_color = panel_spi_invert_color;
    spilcd->base.set_gap = panel_spi_set_gap;
    spilcd->base.mirror = panel_spi_mirror;
    spilcd->base.swap_xy = panel_spi_swap_xy;
    spilcd->base.disp_on_off = panel_spi_disp_on_off;
    *ret_panel = &(spilcd->base);
    ESP_LOGD(TAG, "new spilcd panel @%p", spilcd);

    return ESP_OK;

err:
    if (spilcd) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(spilcd);
    }
    return ret;
}

static esp_err_t panel_spi_del(esp_lcd_panel_t *panel) {
    spi_panel_t *spilcd = __containerof(panel, spi_panel_t, base);

    if (spilcd->reset_gpio_num >= 0) {
        gpio_reset_pin(spilcd->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del spilcd panel @%p", spilcd);
    free(spilcd);
    return ESP_OK;
}

static esp_err_t panel_spi_reset(esp_lcd_panel_t *panel) {
    spi_panel_t *spilcd = __containerof(panel, spi_panel_t, base);
    esp_lcd_panel_io_handle_t io = spilcd->io;

    // perform hardware reset
    if (spilcd->reset_gpio_num >= 0) {
        gpio_set_level(spilcd->reset_gpio_num, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(spilcd->reset_gpio_num, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(spilcd->reset_gpio_num, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
    } else { // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

static esp_err_t panel_spi_init(esp_lcd_panel_t *panel) {
    spi_panel_t *spilcd = __containerof(panel, spi_panel_t, base);
    esp_lcd_panel_io_handle_t io = spilcd->io;

    const spi_lcd_init_cmd_t *init_cmds = NULL;
    uint16_t init_cmds_size = 0;
    if (spilcd->init_cmds) {
        init_cmds = spilcd->init_cmds;
        init_cmds_size = spilcd->init_cmds_size;
    } else {
        init_cmds = vendor_specific_init_default;
        init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(spi_lcd_init_cmd_t);
    }

    bool is_cmd_overwritten = false;
    for (int i = 0; i < init_cmds_size; i++) {
        // Check if the command has been used or conflicts with the internal
        switch (init_cmds[i].cmd) {
            default:
                is_cmd_overwritten = false;
                break;
        }
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));
    }
    ESP_LOGW(TAG, "send init commands success");

    return ESP_OK;
}

static esp_err_t panel_spi_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data) {
    spi_panel_t *spilcd = __containerof(panel, spi_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = spilcd->io;

    x_start += spilcd->x_gap;
    x_end += spilcd->x_gap;  //x-start and x-end is offset by 35 pixels
    y_start += spilcd->y_gap;
    y_end += spilcd->y_gap;

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
    size_t len = (x_end - x_start) * (y_end - y_start) * spilcd->fb_bits_per_pixel / 8;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len), TAG, "send color failed");

    return ESP_OK;
}

static esp_err_t panel_spi_invert_color(esp_lcd_panel_t *panel, bool invert_color_data) {
    spi_panel_t *spilcd = __containerof(panel, spi_panel_t, base);
    esp_lcd_panel_io_handle_t io = spilcd->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_spi_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y) {
    // spi_panel_t *spilcd = __containerof(panel, spi_panel_t, base);
    // esp_lcd_panel_io_handle_t io = spilcd->io;
    // if (mirror_x) {
    //     spilcd->madctl_val |= LCD_CMD_MX_BIT;
    // } else {
    //     spilcd->madctl_val &= ~LCD_CMD_MX_BIT;
    // }
    // if (mirror_y) {
    //     spilcd->madctl_val |= LCD_CMD_MY_BIT;
    // } else {
    //     spilcd->madctl_val &= ~LCD_CMD_MY_BIT;
    // }
    // ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
    //     spilcd->madctl_val
    // }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_spi_swap_xy(esp_lcd_panel_t *panel, bool swap_axes) {
    // spi_panel_t *spilcd = __containerof(panel, spi_panel_t, base);
    // esp_lcd_panel_io_handle_t io = spilcd->io;
    // if (swap_axes) {
    //     spilcd->madctl_val |= LCD_CMD_MV_BIT;
    // } else {
    //     spilcd->madctl_val &= ~LCD_CMD_MV_BIT;
    // }
    // ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
    //     spilcd->madctl_val
    // }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_spi_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap) {
    // spi_panel_t *spilcd = __containerof(panel, spi_panel_t, base);
    // spilcd->x_gap = x_gap;
    // spilcd->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_spi_disp_on_off(esp_lcd_panel_t *panel, bool on_off) {
    spi_panel_t *spilcd = __containerof(panel, spi_panel_t, base);
    esp_lcd_panel_io_handle_t io = spilcd->io;
    int command = 0;

    if (on_off) {
        command = LCD_CMD_DISPON;
    } else {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}
