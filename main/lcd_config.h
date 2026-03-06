#ifndef _LCD_DEFINES_H_
#define _LCD_DEFINES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "spi_lcd.h"

#define SPI_LCD_USE_H010A11         0
#define SPI_LCD_USE_H011A1          0
#define SPI_LCD_USE_H013A6          0
#define SPI_LCD_USE_H015A06         0
#define SPI_LCD_USE_H020A05         0
#define SPI_LCD_USE_H027A02         0
#define SPI_LCD_USE_H028A27         1
#define SPI_LCD_USE_H032A05         0

// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LCD_PIXEL_CLOCK_HZ          (30 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL       1
#define LCD_BK_LIGHT_OFF_LEVEL      !LCD_BK_LIGHT_ON_LEVEL
#define PIN_NUM_SCLK                6
#define PIN_NUM_MOSI                7
#define PIN_NUM_MISO                -1
#define PIN_NUM_LCD_DC              15
#define PIN_NUM_LCD_RST             5
#define PIN_NUM_LCD_CS              4
#define PIN_NUM_BACKLIGHT           10

// The pixel number in horizontal and vertical
#if SPI_LCD_USE_H010A11
#define SPI_LCD_H_RES               80
#define SPI_LCD_V_RES               160

#elif SPI_LCD_USE_H011A1
#define SPI_LCD_H_RES               135
#define SPI_LCD_V_RES               240

#elif SPI_LCD_USE_H013A6
#define SPI_LCD_H_RES               240
#define SPI_LCD_V_RES               240

#elif SPI_LCD_USE_H015A06
#define SPI_LCD_H_RES               170
#define SPI_LCD_V_RES               320

#elif SPI_LCD_USE_H020A05
#define SPI_LCD_H_RES               170
#define SPI_LCD_V_RES               320

#elif SPI_LCD_USE_H027A02
#define SPI_LCD_H_RES               142
#define SPI_LCD_V_RES               428

#elif SPI_LCD_USE_H028A27
#define SPI_LCD_H_RES               200
#define SPI_LCD_V_RES               648

#elif SPI_LCD_USE_H032A05
#define SPI_LCD_H_RES               240
#define SPI_LCD_V_RES               320

#endif
// Bit number used to represent command and parameter
#define SPI_LCD_CMD_BITS            8
#define SPI_LCD_PARAM_BITS          8

#define LVGL_DRAW_BUF_LINES         20 // number of display lines in each draw buffer
#define LVGL_TICK_PERIOD_MS         2
#define LVGL_TASK_MAX_DELAY_MS      500
#define LVGL_TASK_MIN_DELAY_MS      1

#ifdef __cplusplus
}
#endif

#endif //_LCD_DEFINES_H_