/**
 * @file main.c
 * @brief Example for receiving data over CAN using the CAN 2.0 protocol and an interrupt
 *
 * Product used is www.solde.red/333020
 *
 * @author Soldered Electronics
 */

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soldered_mcp2518fd.h"

static const char *TAG = "CAN20_RECV_INT";

// Change these to match how your breakout is wired
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_SCK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5
#define PIN_NUM_INT  GPIO_NUM_4

// Signals from the interrupt handler that a frame has arrived
static SemaphoreHandle_t frame_ready;

/**
 * @brief Interrupt handler for the breakout's INT pin
 *
 * Must stay as short as possible. SPI cannot be used from an interrupt, so this
 * only wakes app_main, which then reads the frame out.
 */
static void IRAM_ATTR can_isr(void *arg)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    xSemaphoreGiveFromISR(frame_ready, &higher_priority_task_woken);

    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

void app_main(void)
{
    mcp2518fd_t can;
    uint8_t len = 0; // Length of the data
    uint8_t buf[8];  // Buffer to store the data

    frame_ready = xSemaphoreCreateBinary();

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

    // The breakout's INT pin is open drain and idles high, so it needs a pull-up
    // and fires on the falling edge.
    const uint64_t int_pin_mask = 1ULL << PIN_NUM_INT;
    gpio_config_t int_cfg = {
        .pin_bit_mask = int_pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&int_cfg));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_NUM_INT, can_isr, NULL));

    while (1) {
        if (xSemaphoreTake(frame_ready, portMAX_DELAY) == pdTRUE) {
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
    }
}
