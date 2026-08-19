/**
 * @file main.c
 * @brief Example for sending frames over CAN using the CAN FD protocol
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

static const char *TAG = "CANFD_SEND";

#define MAX_DATA_SIZE 64

// Change these to match how your breakout is wired
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_SCK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

void app_main(void)
{
    mcp2518fd_t can;
    uint8_t stmp[MAX_DATA_SIZE] = {0}; // Buffer which stores the data to send

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

    // Put the transceiver into normal mode. The driver defaults to classic mode,
    // which cannot send CAN FD frames, so this has to happen before begin().
    mcp2518fd_set_mode(&can, CAN_NORMAL_MODE);

    // Initialize the CAN bus with an arbitration rate of 125 kbps and a data
    // rate of 500 kbps. This sits in a retry loop because the MCP2518FD needs
    // some time after power-up before it answers over SPI.
    while (CAN_OK != mcp2518fd_begin(&can, CAN_125K_500K, MCP2518FD_20MHz)) {
        ESP_LOGE(TAG, "CAN init fail, retry...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "CAN init ok!");

    for (int i = 0; i < MAX_DATA_SIZE; i++) { // Fill the buffer with ascending numbers
        stmp[i] = i;
    }

    while (1) {
        // First parameter  - which ID to put in the frame (ID of the transmitter)
        // Second parameter - frame format (0 - standard frame, 1 - extended frame)
        // Third parameter  - length of the buffer, converted into a data length code
        // Fourth parameter - buffer holding the data to send
        mcp2518fd_send_msg_buf(&can, 0x01, 0, mcp2518fd_len2dlc(MAX_DATA_SIZE), stmp);
        vTaskDelay(pdMS_TO_TICKS(10)); // Wait a bit for the CAN module to send the data

        mcp2518fd_send_msg_buf(&can, 0x04, 0, mcp2518fd_len2dlc(MAX_DATA_SIZE), stmp);
        vTaskDelay(pdMS_TO_TICKS(500)); // Wait a bit so the bus does not get flooded

        ESP_LOGI(TAG, "CAN BUS sendMsgBuf ok!");
    }
}
