/**
 * @file main.c
 * @brief Example for sending frames over CAN using the CAN 2.0 protocol
 *
 * Product used is www.solde.red/333020
 *
 * @author Soldered Electronics
 */

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soldered_mcp2518fd.h"

static const char *TAG = "CAN20_SEND";

// Change these to match how your breakout is wired
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_SCK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

void app_main(void)
{
    mcp2518fd_t can;

    // The SPI bus belongs to the application, not to the driver, so that other
    // devices can share it. Create it first, then hand the host to the driver.
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    ESP_ERROR_CHECK(mcp2518fd_init(&can, SPI2_HOST, PIN_NUM_CS));

    // Initialize the CAN bus at a bit rate of 125 kbps. This sits in a retry
    // loop because the MCP2518FD needs some time after power-up before it
    // answers over SPI.
    while (CAN_OK != mcp2518fd_begin(&can, CAN_125KBPS, MCP2518FD_20MHz)) {
        ESP_LOGE(TAG, "CAN init fail, retry...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "CAN init ok!");

    uint8_t stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7}; // Buffer which stores the data to send

    while (1) {
        // First parameter  - which ID to put in the frame (ID of the transmitter)
        // Second parameter - frame format (0 - standard frame, 1 - extended frame)
        // Third parameter  - length of the buffer in bytes
        // Fourth parameter - buffer holding the data to send
        mcp2518fd_send_msg_buf(&can, 0x01, 0, 8, stmp);
        vTaskDelay(pdMS_TO_TICKS(10)); // Wait a bit for the CAN module to send the data

        mcp2518fd_send_msg_buf(&can, 0x04, 0, 8, stmp); // Send the same data again
        vTaskDelay(pdMS_TO_TICKS(500));                 // Wait a bit so the bus does not get flooded

        ESP_LOGI(TAG, "CAN BUS sendMsgBuf ok!");
    }
}
