#include <stdio.h>
#include <string.h>
#include "pms5003.h"

static esp_err_t _uart_init(void);



/**
 * @brief Initialize the UART
 * @param
 */
static esp_err_t _uart_init(void)
{
    esp_err_t err = ESP_OK;
    static const char *TAG = __func__;

    ESP_LOGI( TAG, "Initializing UART" );

    const uart_port_t _uart_num = UART_PORT_NUM;
    const int uart_buffer_size = ( 1024 );

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
#if CONFIG_UART_MORE_PRIO
    intr_alloc_flags = ESP_INTR_FLAG_LEVEL3;
#endif

    ESP_LOGI( TAG, "UART set pins, mode and install driver." );
    err = uart_driver_install( _uart_num, uart_buffer_size, 0, 0, NULL, intr_alloc_flags );
    err +=  uart_param_config( _uart_num, &uart_config );
    err += uart_set_pin( _uart_num, UART_GPIO_TX, UART_GPIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE );

    return err;
}



/**
 * @brief Initialize the Sensor
 * @param
 */
bool pms5003_begin(void)
{

    // Init the UART
    if (_uart_init() != ESP_OK) {
        ESP_LOGE(__func__, "Uart Init failed");
        return false;
    }

    pms5003_wakeup();
    vTaskDelay(250 / portTICK_PERIOD_MS);
    pms5003_set_passive();
    vTaskDelay(250 / portTICK_PERIOD_MS);

    uart_flush(UART_PORT_NUM);

    return true;
}


/**
 * @brief Send wakeup cmd to the sensor
 * @param  none
 */
void pms5003_wakeup(void)
{
    uint16_t txbytes = uart_write_bytes(UART_PORT_NUM, cmd_wakeup, sizeof(cmd_wakeup));
    if (txbytes == 0) {
        ESP_LOGE(__func__, "Problem sending wakeup command");
    }
}


/**
 * @brief Send sensor sleep cmd
 * @param  none
 */
void pms5003_sleep(void)
{
    uint16_t txbytes = uart_write_bytes(UART_PORT_NUM, cmd_sleep, sizeof(cmd_sleep));
    if (txbytes == 0) {
        ESP_LOGE(__func__, "Problem sending sleep command");
    }

}


/**
 * @brief Set sensor in passive mode
 * @param none
 */
void pms5003_set_passive(void)
{
    uint16_t txbytes = uart_write_bytes(UART_PORT_NUM, cmd_passive, sizeof(cmd_passive));
    if (txbytes == 0) {
        ESP_LOGE(__func__, "Problem setting passive mode");
    }
}



/**
 * @brief Set sensor in active mode
 * @param  none
 */
void pms5003_set_active(void)
{
    uint16_t txbytes = uart_write_bytes(UART_PORT_NUM, cmd_active, sizeof(cmd_active));
    if (txbytes == 0) {
        ESP_LOGE(__func__, "Problem setting active mode");
    }
}



/**
 * @brief Reading values in passive mode
 * @param values struct of values pms5003_data_t
 */
void pms5003_passive_read(pms5003_data_t *values)
{
    static const uint8_t rxlen = 32;
    static uint8_t rxbuf[32];
    memset(rxbuf, 0, rxlen);


    // send the passive read commando, the the sensor we expect measurements
    uint16_t txlen = uart_write_bytes(UART_PORT_NUM, cmd_passive_read, sizeof(cmd_passive_read));
    ESP_LOGD(__func__, "Executing passive read command, Wrote %d bytes to the sensor", txlen);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint16_t rxbytes =  uart_read_bytes(UART_PORT_NUM, rxbuf, rxlen, pdMS_TO_TICKS(400));
    ESP_LOGD(__func__, "And received %d bytes back", rxbytes);

    if (rxbytes > 0) {
        // houston we received something, might be garbage
        ESP_LOGD(__func__, "Bytes 1 and 2:  0x%X - 0x%X", rxbuf[0], rxbuf[1]);


        if ((rxbuf[0] != 0x42) && (rxbuf[1] != 0x4D)) {                          // if first byte is not eq start char 1 bail out
            uart_flush(UART_PORT_NUM);
            pms5003_zero_values(values);
            return;
        }

        uint16_t frame_length = makeWord(rxbuf[2], rxbuf[3]);
        if (frame_length != 0x1C) {                     // frame length is always 28 Bytes (data+check bytes).
            return;
        }
        ESP_LOGD(__func__, "Frame length: %d", frame_length);


        // crc validation
        uint16_t _calc_checksum = 0;
        uint16_t _ret_checksum = 0;
        for (uint8_t q = 0 ; q < 30 ; q ++) {
            _calc_checksum += rxbuf[q];
        }
        _ret_checksum = makeWord(rxbuf[30], rxbuf[31]);

        if (_calc_checksum == _ret_checksum) {
            values->pm_std_1_0 = makeWord(rxbuf[4], rxbuf[5]);
            values->pm_std_2_5 = makeWord(rxbuf[6], rxbuf[7]);
            values->pm_std_10 = makeWord(rxbuf[8], rxbuf[9]);

            values->pm_env_1_0 = makeWord(rxbuf[10], rxbuf[11]);
            values->pm_env_2_5 = makeWord(rxbuf[12], rxbuf[13]);
            values->pm_env_10 = makeWord(rxbuf[14], rxbuf[15]);

            values->pm_part_0_3um = makeWord(rxbuf[16], rxbuf[17]);
            values->pm_part_0_5um = makeWord(rxbuf[18], rxbuf[19]);
            values->pm_part_1_0um = makeWord(rxbuf[20], rxbuf[21]);
            values->pm_part_2_5um = makeWord(rxbuf[22], rxbuf[23]);
            values->pm_part_5_0um = makeWord(rxbuf[24], rxbuf[25]);
            values->pm_part_10um = makeWord(rxbuf[26], rxbuf[27]);
            // 28, 29 reserved
            // 30, 31 check
        } else {
            ESP_LOGE(__func__, "Checksum comparison failed !!!: calculated 0x%X - received: 0x%X", _calc_checksum, _ret_checksum);
        }

    }

}



/**
 * @brief
 * @param values
 */
void pms5003_active_read(pms5003_data_t *values)
{
    static const uint8_t rxlen = 32;
    static uint8_t rxbuf[32];


    uint16_t rxbytes =  uart_read_bytes(UART_PORT_NUM, rxbuf, rxlen, pdMS_TO_TICKS(400));
    ESP_LOGD(__func__, "And received %d bytes back", rxbytes);

    if (rxbytes > 0) {
        ESP_LOGD(__func__, "Bytes 1 and 2:  0x%X - 0x%X", rxbuf[0], rxbuf[1]);
        // houston we received something, might be garbage
        if ((rxbuf[0] != 0x42) && (rxbuf[1] != 0x4D)) {                          // if first byte is not eq start char 1 bail out
            uart_flush(UART_PORT_NUM);
            pms5003_zero_values(values);
            return;
        }

        uint16_t frame_length = makeWord(rxbuf[2], rxbuf[3]);
        if (frame_length != 0x1C) {                     // frame length is always 28 Bytes (data+check bytes).
            return;
        }
        ESP_LOGD(__func__, "Frame length: %d", frame_length);


        // crc validation
        uint16_t _calc_checksum = 0;
        uint16_t _ret_checksum = 0;
        for (uint8_t q = 0 ; q < 30 ; q ++) {
            _calc_checksum += rxbuf[q];
        }
        _ret_checksum = makeWord(rxbuf[30], rxbuf[31]);

        if (_calc_checksum == _ret_checksum) {
            values->pm_std_1_0 = makeWord(rxbuf[4], rxbuf[5]);
            values->pm_std_2_5 = makeWord(rxbuf[6], rxbuf[7]);
            values->pm_std_10 = makeWord(rxbuf[8], rxbuf[9]);

            values->pm_env_1_0 = makeWord(rxbuf[10], rxbuf[11]);
            values->pm_env_2_5 = makeWord(rxbuf[12], rxbuf[13]);
            values->pm_env_10 = makeWord(rxbuf[14], rxbuf[15]);

            values->pm_part_0_3um = makeWord(rxbuf[16], rxbuf[17]);
            values->pm_part_0_5um = makeWord(rxbuf[18], rxbuf[19]);
            values->pm_part_1_0um = makeWord(rxbuf[20], rxbuf[21]);
            values->pm_part_2_5um = makeWord(rxbuf[22], rxbuf[23]);
            values->pm_part_5_0um = makeWord(rxbuf[24], rxbuf[25]);
            values->pm_part_10um = makeWord(rxbuf[26], rxbuf[27]);
            // 28, 29 reserved
            // 30, 31 check
        } else {
            ESP_LOGE(__func__, "Checksum comparison failed !!!: calculated 0x%X - received: 0x%X", _calc_checksum, _ret_checksum);
        }
    }
}

/**
 * @brief Print measured values
 * @param particles struct
 */
void pms5003_print_values(pms5003_data_t particles)
{
    ESP_LOGI(__func__, "Standard  1.0:\t %d", particles.pm_std_1_0);
    ESP_LOGI(__func__, "Standard  2.5:\t %d", particles.pm_std_2_5);
    ESP_LOGI(__func__, "Standard   10:\t %d", particles.pm_std_10);
    ESP_LOGI(__func__, "Atm Env.  1.0:\t %d", particles.pm_env_1_0);
    ESP_LOGI(__func__, "Atm Env.  2.5:\t %d", particles.pm_env_2_5);
    ESP_LOGI(__func__, "Atm Env    10:\t %d", particles.pm_env_10);
    ESP_LOGI(__func__, "Particles 0.1L 0.3um: %d", particles.pm_part_0_3um);
    ESP_LOGI(__func__, "Particles 0.1L 0.5um: %d", particles.pm_part_0_5um);
    ESP_LOGI(__func__, "Particles 0.1L 1.0um: %d", particles.pm_part_1_0um);
    ESP_LOGI(__func__, "Particles 0.1L 2.5um: %d", particles.pm_part_2_5um);
    ESP_LOGI(__func__, "Particles 0.1L 5.0um: %d", particles.pm_part_5_0um);
    ESP_LOGI(__func__, "Particles 0.1L  10um: %d", particles.pm_part_10um);
}


/**
 * @brief Initialize and zero-out all values in struct
 * @param values struct
 */
void pms5003_zero_values(pms5003_data_t *values)
{
    values->pm_env_10 = 0;
    values->pm_env_1_0 = 0;
    values->pm_env_2_5 = 0;
    values->pm_part_0_3um = 0;
    values->pm_part_0_5um = 0;
    values->pm_part_10um = 0;
    values->pm_part_1_0um = 0;
    values->pm_part_2_5um = 0;
    values->pm_part_5_0um = 0;
    values->pm_std_10 = 0;
    values->pm_std_1_0 = 0;
    values->pm_std_2_5 = 0;

}