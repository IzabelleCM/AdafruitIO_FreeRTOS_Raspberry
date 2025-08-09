add_library(oled STATIC ssd1306_i2c.c ssd1306.h)
target_link_libraries(oled
        pico_stdlib
        hardware_i2c
        )