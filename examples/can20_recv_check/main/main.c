/**
 * @file main.c
 * @brief Example for receiving data over CAN using the CAN 2.0 protocol, by polling
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

static const char *TAG = "CAN20_RECV_CHECK";

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

    while (1) {
        uint8_t len = 0; // Variable to store the length of the incoming data
        uint8_t buf[8];  // Buffer to store the incoming data

        if (CAN_MSGAVAIL == mcp2518fd_check_receive(&can)) { // Check if data is coming in
            // Read the frame before asking for its ID; this call is what latches
            // the ID, and it fills buf with len bytes of payload.
            mcp2518fd_read_msg_buf(&can, &len, buf);

            uint32_t id = mcp2518fd_get_can_id(&can); // Get the ID of the transmitter
            ESP_LOGI(TAG, "Get Data From id: %lu", (unsigned long)id);
            ESP_LOGI(TAG, "Len = %u", len);

            for (int i = 0; i < len; i++) {
                printf("%u\t", buf[i]); // Print the received data
            }
            printf("\n");
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Yield so the idle task can run
    }
}
