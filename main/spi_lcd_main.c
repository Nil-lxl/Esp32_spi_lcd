/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "lvgl.h"
#include "lcd_config.h"

esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;

static esp_err_t spi_lcd_init(void) {

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
#if CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
    ESP_LOGI(TAG, "Install GC9A01 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9107
    ESP_LOGI(TAG, "Install GC9107 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9107(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_H013A6
    ESP_LOGI(TAG, "Install H013A6 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_h013a6(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_H020A05
    ESP_LOGI(TAG, "Install H020A05 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_h020a05(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_H032A05
    ESP_LOGI(TAG, "Install H032A05 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_h032a05(io_handle, &panel_config, &panel_handle));
#endif
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
#if CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    return ESP_OK;
}


extern esp_err_t lvgl_init(void);

void app_main(void) {
#if PIN_NUM_BACKLIGHT>=0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BACKLIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_BACKLIGHT, 0));
#endif
    ESP_ERROR_CHECK(spi_lcd_init());
    ESP_ERROR_CHECK(lvgl_init());

#if PIN_NUM_BACKLIGHT>=0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(PIN_NUM_BACKLIGHT, 1);
#endif
}
