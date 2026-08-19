/**
 * @file main.c
 * @brief Query a vehicle's OBD-II PIDs over CAN
 *
 * Type a PID in hex on the serial console and press enter; the vehicle answers
 * with the corresponding value.
 *
 *     Query
 *         send id: 0x7df
 *         dta: 0x02, 0x01, PID_CODE, 0, 0, 0, 0, 0
 *
 *     Response
 *         From id: 0x7E9 or 0x7EA or 0x7EB
 *         dta: len, 0x41, PID_CODE, byte0, byte1(option), byte2(option), byte3(option), byte4(option)
 *
 * Product used is www.solde.red/333020
 *
 * @author Soldered Electronics
 */

#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soldered_mcp2518fd.h"

static const char *TAG = "OBDII_PIDS";

// Change these to match how your breakout is wired
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_SCK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

#define PID_ENGIN_PRM     0x0C // Engine RPM
#define PID_VEHICLE_SPEED 0x0D // Vehicle speed
#define PID_COOLANT_TEMP  0x05 // Engine coolant temperature

#define CAN_ID_PID 0x7DF

// The console UART, used to type in PIDs
#define CONSOLE_UART UART_NUM_0

static mcp2518fd_t can;

static uint8_t pid_input;
static uint8_t get_pid;

/**
 * @brief Only let through the ECU responses, which all come from ID 0x7E8
 *
 * These are OBD-II details that are just reproduced here. If you want to know
 * more, look them up; information about OBD-II is hard to find because it is
 * only used in automotive systems.
 */
static void set_mask_filt(void)
{
    // Set both masks to 0x7FC
    mcp2518fd_init_mask(&can, 0, 0, 0x7FC);
    mcp2518fd_init_mask(&can, 1, 0, 0x7FC);

    // Set the filters, so IDs 0x7E8 through 0x7EB are accepted
    mcp2518fd_init_filt(&can, 0, 0, 0x7E8);
    mcp2518fd_init_filt(&can, 1, 0, 0x7E8);
    mcp2518fd_init_filt(&can, 2, 0, 0x7E8);
    mcp2518fd_init_filt(&can, 3, 0, 0x7E8);
    mcp2518fd_init_filt(&can, 4, 0, 0x7E8);
    mcp2518fd_init_filt(&can, 5, 0, 0x7E8);
}

/**
 * @brief Send a mode 01 request for one PID
 */
static void send_pid(uint8_t pid)
{
    uint8_t tmp[8] = {0x02, 0x01, pid, 0, 0, 0, 0, 0}; // Build the request frame

    ESP_LOGI(TAG, "SEND PID: 0x%02X", pid);

    // First parameter  - which ID to put in the frame (ID of the transmitter)
    // Second parameter - frame format (0 - standard frame, 1 - extended frame)
    // Third parameter  - length of the buffer in bytes
    // Fourth parameter - buffer holding the data to send
    mcp2518fd_send_msg_buf(&can, CAN_ID_PID, 0, 8, tmp);
}

/**
 * @brief Read anything the vehicle has sent back
 */
static void task_can_recv(void)
{
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
}

/**
 * @brief Collect a PID typed on the console, one hex digit at a time
 *
 * Each digit shifts the value left by one hex place, so typing "0C" then enter
 * builds 0x0C. A newline ends the entry and marks the PID ready to send.
 */
static void task_dbg(void)
{
    uint8_t c;

    while (uart_read_bytes(CONSOLE_UART, &c, 1, 0) == 1) {
        if (c >= '0' && c <= '9') {
            pid_input *= 0x10;
            pid_input += c - '0';
        } else if (c >= 'A' && c <= 'F') {
            pid_input *= 0x10;
            pid_input += 10 + c - 'A';
        } else if (c >= 'a' && c <= 'f') {
            pid_input *= 0x10;
            pid_input += 10 + c - 'a';
        } else if (c == '\n' || c == '\r') {
            get_pid = 1;
        }
    }
}

void app_main(void)
{
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

    // Take over the console UART so PIDs can be typed in. Logs keep going out
    // over the same UART.
    ESP_ERROR_CHECK(uart_driver_install(CONSOLE_UART, 256, 0, 0, NULL, 0));

    // Initialize the CAN bus at a bit rate of 125 kbps. This sits in a retry
    // loop because the MCP2518FD needs some time after power-up before it
    // answers over SPI.
    while (CAN_OK != mcp2518fd_begin(&can, CAN_125KBPS, MCP2518FD_20MHz)) {
        ESP_LOGE(TAG, "CAN init fail, retry...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "CAN init ok!");

    set_mask_filt(); // Only listen to the ECU's responses

    ESP_LOGI(TAG, "Type a PID in hex and press enter, e.g. %02X for engine RPM", PID_ENGIN_PRM);

    while (1) {
        task_can_recv(); // Check whether data is coming in from the CAN bus
        task_dbg();      // Collect a PID typed on the console

        if (get_pid) { // A full PID was entered
            get_pid = 0;
            send_pid(pid_input);
            pid_input = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Yield so the idle task can run
    }
}
