#pragma once

#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"


#ifdef __cplusplus
extern "C" {
#endif


#ifdef CONFIG_PMS5003_SENSOR_ENABLED
#define UART_PORT_NUM       CONFIG_UART_PORT_NUMBER
#define UART_GPIO_RX        CONFIG_UART_GPIO_RX
#define UART_GPIO_TX        CONFIG_UART_GPIO_TX
#define UART_BAUD_RATE      CONFIG_PMS5003_UART_BAUDRATE

#else
#error  "run idp.py menuconfig to enable the sensor"
#endif

/* *********   ************ */
#define makeWord(x,y)       ((x << 8) | y)


/* *********   ************ */
static const uint8_t cmd_wakeup[] =  {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74 };;
static const uint8_t cmd_sleep[]  =  {0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73 };
static const uint8_t cmd_active[] =  {0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71 };
static const uint8_t cmd_passive[] = { 0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70 };
static const uint8_t cmd_passive_read[] = { 0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71 };


/**
 * @brief
 */
typedef struct {
    uint16_t pm_std_1_0;
    uint16_t pm_std_2_5;
    uint16_t pm_std_10;

    uint16_t pm_env_1_0;
    uint16_t pm_env_2_5;
    uint16_t pm_env_10;

    uint16_t pm_part_0_3um;
    uint16_t pm_part_0_5um;
    uint16_t pm_part_1_0um;
    uint16_t pm_part_2_5um;
    uint16_t pm_part_5_0um;
    uint16_t pm_part_10um;

} pms5003_data_t;



/* ********* Functions ************** */
bool pms5003_begin(void);
void pms5003_wakeup(void);
void pms5003_sleep(void);
void pms5003_set_passive(void);
void pms5003_set_active(void);
void pms5003_passive_read(pms5003_data_t *values);
void pms5003_zero_values(pms5003_data_t *values);
void pms5003_print_values(pms5003_data_t particles);
void pms5003_active_read(pms5003_data_t *values);

#ifdef __cplusplus
}
#endif