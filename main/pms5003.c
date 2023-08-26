#include <stdio.h>
#include "pms5003.h"

void app_main(void)
{
    static pms5003_data_t particles;

    pms5003_begin();
    pms5003_zero_values(&particles);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uint16_t count = 0;
    do {
        pms5003_passive_read(&particles);
        pms5003_print_values(particles);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        count++;
    } while (count < 30);

    pms5003_zero_values(&particles);                            // reset values
    ESP_LOGI(__func__, "Stop measuring, put sensor to sleep for 30sec");
    pms5003_sleep();
    vTaskDelay(30000 / portTICK_PERIOD_MS);
    ESP_LOGI(__func__, "Enable active mode....");
    pms5003_wakeup();
    vTaskDelay(300 / portTICK_PERIOD_MS);
    pms5003_set_active();
    vTaskDelay(2000 / portTICK_PERIOD_MS);


    count = 0;
    do {
        pms5003_active_read(&particles);
        pms5003_print_values(particles);
        count++;
        vTaskDelay(750 / portTICK_PERIOD_MS);
    } while ( count < 50);
    pms5003_sleep();
}
