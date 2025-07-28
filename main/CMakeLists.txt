set(main_srcs "spi_lcd_main.c" "lvgl_demo_ui.c" "lvgl_init.c")

idf_component_register(SRCS ${main_srcs} INCLUDE_DIRS ".")
