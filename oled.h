#ifndef __OLED_H__
#define __OLED_H__

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <sdkconfig.h>
#include "ssd1306.h"


static const char *TAG = "Test-I2C";


esp_err_t i2c_master_init(void);

void ssd1306_init();

void task_ssd1306_display_text(const void *arg_text, uint8_t a[]);

void task_ssd1306_display_clear(void *ignore);



#endif
