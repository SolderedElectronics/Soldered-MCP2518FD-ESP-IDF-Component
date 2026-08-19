/**
 * @file soldered_mcp2518fd.h
 * @brief Public API for the soldered-mcp2518fd component
 *
 * ESP-IDF driver for the Soldered CAN Bus Breakout (MCP2518FD) over SPI.
 *
 * @author Soldered Electronics
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "mcp2518fd_dfs.h"

/** Default SPI clock used for the MCP2518FD */
#define MCP2518FD_DEFAULT_SPI_CLOCK_HZ (4 * 1000 * 1000)

/**
 * @brief Handle for one MCP2518FD controller
 *
 * Create one per breakout board. All fields are managed by the driver; treat
 * the struct as opaque and read state through the accessor functions.
 */
typedef struct {
    spi_device_handle_t spi; /**< SPI device handle, created by mcp2518fd_init() */
    gpio_num_t cs_pin;       /**< Chip select pin, driven by the SPI peripheral */
    uint8_t *spi_tx;         /**< DMA capable SPI transmit buffer */
    uint8_t *spi_rx;         /**< DMA capable SPI receive buffer */

    /* State of the frame most recently handed to / read from the controller */
    uint32_t can_id; /**< ID of the last received frame */
    uint8_t ext_flg; /**< 1 if the last received frame used a 29 bit extended ID */
    uint8_t rtr;     /**< 1 if the last received frame was a remote transmission request */

    uint8_t mcp_mode;      /**< Operation mode the controller should run in, a CAN_OPERATION_MODE */
    uint8_t n_reserved_tx; /**< Number of TX buffers held back for reserved sends */

    /* Bit timing, computed by mcp2518fd_begin() from the requested bit rate */
    uint32_t sys_clock;                    /**< Oscillator frequency in Hz */
    uint32_t desired_arbitration_bit_rate; /**< Requested arbitration phase bit rate in bps */
    uint8_t data_bit_rate_factor;          /**< Data phase bit rate as a multiple of the arbitration bit rate */
    uint8_t data_phase_segment1;           /**< Data phase segment 1, in time quanta */
    uint8_t data_phase_segment2;           /**< Data phase segment 2, in time quanta */
    uint8_t data_sjw;                      /**< Data phase synchronization jump width */
    uint16_t bit_rate_prescaler;           /**< Prescaler shared by the arbitration and data phases */
    uint16_t arbitration_phase_segment1;   /**< Arbitration phase segment 1, in time quanta */
    uint8_t arbitration_phase_segment2;    /**< Arbitration phase segment 2, in time quanta */
    uint8_t arbitration_sjw;               /**< Arbitration phase synchronization jump width */
    bool arbitration_bit_rate_ok;          /**< false if the computed bit rate misses the request by more than 1% */
    int8_t tdco;                           /**< Transmitter delay compensation offset, -64 to 63 */

    /* Scratch objects for the frames currently being moved in and out */
    CAN_RX_MSGOBJ rx_obj;                 /**< Header of the frame most recently read out of the RX FIFO */
    CAN_TX_MSGOBJ tx_obj;                 /**< Header of the frame most recently pushed into the TX FIFO */
    uint8_t rxd[MCP2518FD_MAX_DATA_BYTES]; /**< Payload of the frame most recently received */
    uint8_t txd[MCP2518FD_MAX_DATA_BYTES]; /**< Payload of the frame most recently queued for sending */
} mcp2518fd_t;

/**
 * @brief Attach a MCP2518FD to an already initialized SPI bus
 *
 * Adds the controller as a device on `host` and allocates the DMA capable
 * transfer buffers it needs. The SPI bus itself must already exist, created
 * with spi_bus_initialize(); this leaves the bus free to be shared with other
 * devices. Call mcp2518fd_begin() afterwards to actually bring the CAN
 * controller up.
 *
 * Uses ::MCP2518FD_DEFAULT_SPI_CLOCK_HZ; use mcp2518fd_init_with_clock() to
 * pick a different SPI clock.
 *
 * @param[out] dev Handle to initialize
 * @param[in] host SPI host the breakout is wired to, previously initialized
 *                 with spi_bus_initialize()
 * @param[in] cs_pin GPIO wired to the breakout's NCS pin
 *
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the transfer buffers could not
 *         be allocated, or the error returned by spi_bus_add_device()
 */
esp_err_t mcp2518fd_init(mcp2518fd_t *dev, spi_host_device_t host, gpio_num_t cs_pin);

/**
 * @brief Attach a MCP2518FD to an already initialized SPI bus at a given SPI clock
 *
 * Same as mcp2518fd_init(), but lets you pick the SPI clock. The MCP2518FD is
 * rated for up to 17 MHz; long jumper wires will need something slower.
 *
 * @param[out] dev Handle to initialize
 * @param[in] host SPI host the breakout is wired to, previously initialized
 *                 with spi_bus_initialize()
 * @param[in] cs_pin GPIO wired to the breakout's NCS pin
 * @param[in] spi_clock_hz SPI clock in Hz
 *
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the transfer buffers could not
 *         be allocated, or the error returned by spi_bus_add_device()
 */
esp_err_t mcp2518fd_init_with_clock(mcp2518fd_t *dev, spi_host_device_t host, gpio_num_t cs_pin,
                                    uint32_t spi_clock_hz);

/**
 * @brief Detach the controller from the SPI bus and free its buffers
 *
 * Does not deinitialize the SPI bus itself, since the bus is owned by the caller.
 *
 * @param[in,out] dev Handle previously initialized with mcp2518fd_init()
 *
 * @return ESP_OK on success, or the error returned by spi_bus_remove_device()
 */
esp_err_t mcp2518fd_deinit(mcp2518fd_t *dev);

/**
 * @brief Reset and configure the CAN controller, and set the bit rate
 *
 * Resets the chip, initializes its RAM, sets up one TX and one RX FIFO, applies
 * the bit timing for `speedset`, and switches into the mode last requested with
 * mcp2518fd_set_mode() (CAN_CLASSIC_MODE if it was never called). Finally it
 * verifies the SPI link with a write-then-read test, so a failure here usually
 * means miswiring rather than a bad configuration.
 *
 * The chip needs a moment after power-up, so call this in a retry loop like the
 * examples do.
 *
 * @param[in,out] dev Handle previously initialized with mcp2518fd_init()
 * @param[in] speedset Bit rate: either a classic CAN preset from
 *                     ::MCP_BITTIME_SETUP (e.g. CAN_125KBPS), or a CAN FD bit
 *                     rate word such as CAN_125K_500K / MCP2518FD_BITRATE()
 * @param[in] clockset Oscillator frequency populated on the board, from
 *                     ::MCP_CLOCK_T. The Soldered breakout uses MCP2518FD_20MHz
 *
 * @return CAN_OK on success, CAN_FAILINIT if the controller does not respond
 */
uint8_t mcp2518fd_begin(mcp2518fd_t *dev, uint32_t speedset, uint8_t clockset);

/**
 * @brief Choose the operation mode mcp2518fd_begin() will switch into
 *
 * Only records the mode; nothing is written over SPI. Call this before
 * mcp2518fd_begin() - the CAN FD examples use it to select CAN_NORMAL_MODE,
 * since the default CAN_CLASSIC_MODE cannot send or receive FD frames. To
 * change mode on a running controller use mcp2518fd_switch_mode().
 *
 * @param[in,out] dev Handle
 * @param[in] op_mode Mode to run in, a ::CAN_OPERATION_MODE
 *
 * @return CAN_OK
 */
uint8_t mcp2518fd_set_mode(mcp2518fd_t *dev, uint8_t op_mode);

/**
 * @brief Switch the running controller into an operation mode
 *
 * Writes the mode to the chip immediately. CAN_SLEEP_MODE is applied but not
 * remembered, so mcp2518fd_wake() can restore the previous mode.
 *
 * @param[in,out] dev Handle
 * @param[in] op_mode Mode to switch into, a ::CAN_OPERATION_MODE
 *
 * @return 0 on success, negative on SPI error
 */
uint8_t mcp2518fd_switch_mode(mcp2518fd_t *dev, uint8_t op_mode);

/**
 * @brief Read back the mode the controller is currently in
 *
 * @param[in] dev Handle
 *
 * @return Current ::CAN_OPERATION_MODE, or CAN_INVALID_MODE on SPI error
 */
uint8_t mcp2518fd_get_mode(mcp2518fd_t *dev);

/**
 * @brief Set the acceptance mask of one filter
 *
 * Briefly drops the controller into configuration mode and back. A mask bit set
 * to 1 means the matching ID bit must equal the filter; a 0 means "don't care".
 *
 * @param[in,out] dev Handle
 * @param[in] num Filter index, 0 to 31
 * @param[in] ext 0 for standard (11 bit) IDs, 1 for extended (29 bit) IDs
 * @param[in] ul_data Mask value
 *
 * @return 0 on success, negative on SPI error
 */
uint8_t mcp2518fd_init_mask(mcp2518fd_t *dev, uint8_t num, uint8_t ext, uint32_t ul_data);

/**
 * @brief Set the acceptance filter value of one filter
 *
 * Briefly drops the controller into configuration mode and back.
 *
 * @param[in,out] dev Handle
 * @param[in] num Filter index, 0 to 31
 * @param[in] ext 0 for standard (11 bit) IDs, 1 for extended (29 bit) IDs
 * @param[in] ul_data ID the filter matches against
 *
 * @return 0 on success, negative on SPI error
 */
uint8_t mcp2518fd_init_filt(mcp2518fd_t *dev, uint8_t num, uint8_t ext, uint32_t ul_data);

/**
 * @brief Set a filter and its mask in one call, and link it to the RX FIFO
 *
 * The convenience call the mask/filter examples use: disables the filter,
 * writes both the filter value and the mask, then enables the filter and points
 * it at the driver's RX FIFO.
 *
 * @param[in,out] dev Handle
 * @param[in] num Filter index, 0 to 31
 * @param[in] ext 0 for standard (11 bit) IDs, 1 for extended (29 bit) IDs
 * @param[in] f ID the filter matches against
 * @param[in] m Mask applied before matching
 *
 * @return 0 on success
 */
uint8_t mcp2518fd_init_filt_mask(mcp2518fd_t *dev, uint8_t num, uint8_t ext, uint32_t f, uint32_t m);

/**
 * @brief Disable one acceptance filter
 *
 * @param[in,out] dev Handle
 * @param[in] filter Filter to disable, a ::CAN_FILTER
 *
 * @return 0 on success, negative on SPI error
 */
uint8_t mcp2518fd_filter_disable(mcp2518fd_t *dev, CAN_FILTER filter);

/**
 * @brief Enable or disable the transmit interrupt
 *
 * @param[in,out] dev Handle
 * @param[in] enable true to raise INT when the TX FIFO has room
 */
void mcp2518fd_enable_tx_interrupt(mcp2518fd_t *dev, bool enable);

/**
 * @brief Hold back some TX buffers for reserved sends
 *
 * @param[in,out] dev Handle
 * @param[in] n_tx_buf Number of buffers to reserve, clamped to 2
 */
void mcp2518fd_reserve_tx_buffers(mcp2518fd_t *dev, uint8_t n_tx_buf);

/**
 * @brief Get the index of the last TX buffer
 *
 * @param[in] dev Handle
 *
 * @return Index of the last TX buffer
 */
uint8_t mcp2518fd_get_last_tx_buffer(mcp2518fd_t *dev);

/**
 * @brief Enable or disable low power mode, which CAN bus activity can wake from
 *
 * @param[in,out] dev Handle
 * @param[in] enable non-zero to enable low power mode
 */
void mcp2518fd_set_sleep_wakeup(mcp2518fd_t *dev, uint8_t enable);

/**
 * @brief Put the controller to sleep
 *
 * @param[in,out] dev Handle
 *
 * @return CAN_OK if already asleep, otherwise 0 on success and negative on SPI error
 */
uint8_t mcp2518fd_sleep(mcp2518fd_t *dev);

/**
 * @brief Wake the controller, returning it to the mode it ran in before sleeping
 *
 * @param[in,out] dev Handle
 *
 * @return CAN_OK if already awake, otherwise 0 on success and negative on SPI error
 */
uint8_t mcp2518fd_wake(mcp2518fd_t *dev);

/**
 * @brief Read the controller's error state
 *
 * @param[in,out] dev Handle
 * @param[out] err_ptr Optional, receives the same ::CAN_ERROR_STATE flags as the return value
 *
 * @return ::CAN_ERROR_STATE flags, CAN_ERROR_FREE_STATE (0) when the bus is healthy
 */
uint8_t mcp2518fd_check_error(mcp2518fd_t *dev, uint8_t *err_ptr);

/**
 * @brief Check whether a frame is waiting to be read
 *
 * @param[in,out] dev Handle
 *
 * @return CAN_MSGAVAIL if a frame is waiting, CAN_NOMSG if not
 */
uint8_t mcp2518fd_check_receive(mcp2518fd_t *dev);

/**
 * @brief Read the next frame from the RX FIFO into a buffer
 *
 * Also latches the frame's ID, extended flag and RTR flag, readable afterwards
 * with mcp2518fd_get_can_id(), mcp2518fd_is_extended_frame() and
 * mcp2518fd_is_remote_request(). `buf` must be large enough for the incoming
 * frame - 8 bytes is enough for CAN 2.0, CAN FD needs up to
 * ::MCP2518FD_MAX_DATA_BYTES.
 *
 * @param[in,out] dev Handle
 * @param[out] len Receives the payload length in bytes
 * @param[out] buf Receives the payload
 *
 * @return 0 on success
 */
uint8_t mcp2518fd_read_msg_buf(mcp2518fd_t *dev, uint8_t *len, uint8_t *buf);

/**
 * @brief Read the next frame from the RX FIFO, along with its ID
 *
 * @param[in,out] dev Handle
 * @param[out] id Receives the frame's ID
 * @param[out] len Receives the payload length in bytes
 * @param[out] buf Receives the payload
 *
 * @return 0 on success
 */
uint8_t mcp2518fd_read_msg_buf_id(mcp2518fd_t *dev, uint32_t *id, uint8_t *len, uint8_t *buf);

/**
 * @brief Read the next frame from the RX FIFO, along with every header field
 *
 * @param[in,out] dev Handle
 * @param[in] status Status word from mcp2518fd_read_rx_tx_status(); unused, the
 *                   controller does not need it to pick the frame to read
 * @param[out] id Optional, receives the frame's ID
 * @param[out] ext Optional, receives 1 for an extended ID frame
 * @param[out] rtr Optional, receives 1 for a remote transmission request
 * @param[out] len Receives the payload length in bytes
 * @param[out] buf Receives the payload
 *
 * @return 0 on success
 */
uint8_t mcp2518fd_read_msg_buf_id_status(mcp2518fd_t *dev, uint8_t status, uint32_t *id, uint8_t *ext,
                                         uint8_t *rtr, uint8_t *len, uint8_t *buf);

/**
 * @brief Get the ID of the frame read by the last mcp2518fd_read_msg_buf* call
 *
 * @param[in] dev Handle
 *
 * @return CAN ID of the last received frame
 */
uint32_t mcp2518fd_get_can_id(mcp2518fd_t *dev);

/**
 * @brief Check whether the last received frame used an extended (29 bit) ID
 *
 * @param[in] dev Handle
 *
 * @return true if the last received frame was an extended frame
 */
bool mcp2518fd_is_extended_frame(mcp2518fd_t *dev);

/**
 * @brief Check whether the last received frame was a remote transmission request
 *
 * @param[in] dev Handle
 *
 * @return true if the last received frame was an RTR frame
 */
bool mcp2518fd_is_remote_request(mcp2518fd_t *dev);

/**
 * @brief Send a data frame
 *
 * For CAN 2.0 pass the payload length directly as `dlc` (0 to 8). For CAN FD
 * payloads longer than 8 bytes, pass a data length code - use
 * mcp2518fd_len2dlc() to convert a byte count into one.
 *
 * @param[in,out] dev Handle
 * @param[in] id CAN ID to send under
 * @param[in] ext 0 for a standard (11 bit) ID, 1 for an extended (29 bit) ID
 * @param[in] dlc Payload length in bytes for CAN 2.0, or a ::CAN_DLC for CAN FD
 * @param[in] buf Payload to send
 *
 * @return CAN_OK on success, CAN_SENDMSGTIMEOUT if the TX FIFO never freed up,
 *         CAN_FAILTX or CAN_FAILINIT on other failures
 */
uint8_t mcp2518fd_send_msg_buf(mcp2518fd_t *dev, uint32_t id, uint8_t ext, uint8_t dlc, const uint8_t *buf);

/**
 * @brief Send a data or remote frame, with control over waiting
 *
 * @param[in,out] dev Handle
 * @param[in] id CAN ID to send under
 * @param[in] ext 0 for a standard (11 bit) ID, 1 for an extended (29 bit) ID
 * @param[in] rtr 1 to send a remote transmission request instead of data
 * @param[in] dlc Payload length in bytes for CAN 2.0, or a ::CAN_DLC for CAN FD
 * @param[in] buf Payload to send
 * @param[in] wait_sent true to request immediate transmission of the FIFO
 *
 * @return CAN_OK on success, or one of the CAN_FAIL* codes
 */
uint8_t mcp2518fd_send_msg_buf_ex(mcp2518fd_t *dev, uint32_t id, uint8_t ext, uint8_t rtr, uint8_t dlc,
                                  const uint8_t *buf, bool wait_sent);

/**
 * @brief Queue a frame without waiting for a free TX buffer
 *
 * @param[in,out] dev Handle
 * @param[in] id CAN ID to send under
 * @param[in] ext 0 for a standard (11 bit) ID, 1 for an extended (29 bit) ID
 * @param[in] rtr 1 to send a remote transmission request instead of data
 * @param[in] dlc Payload length in bytes for CAN 2.0, or a ::CAN_DLC for CAN FD
 * @param[in] buf Payload to send
 * @param[in] i_tx_buf TX buffer index; unused, kept for source compatibility
 *
 * @return CAN_OK on success, or one of the CAN_FAIL* codes
 */
uint8_t mcp2518fd_try_send_msg_buf(mcp2518fd_t *dev, uint32_t id, uint8_t ext, uint8_t rtr, uint8_t dlc,
                                   const uint8_t *buf, uint8_t i_tx_buf);

/**
 * @brief Clear the "transmit attempts exhausted" flag on the TX FIFO
 *
 * Worth calling when there is nothing left to send and the TX interrupt is in
 * use, otherwise INT never returns to idle.
 *
 * @param[in,out] dev Handle
 * @param[in] flags Unused, the driver always clears the flag on its own TX FIFO
 */
void mcp2518fd_clear_buffer_transmit_if_flags(mcp2518fd_t *dev, uint8_t flags);

/**
 * @brief Read the RX FIFO event flags
 *
 * @param[in,out] dev Handle
 *
 * @return ::CAN_RX_FIFO_EVENT flags
 */
uint8_t mcp2518fd_read_rx_tx_status(mcp2518fd_t *dev);

/**
 * @brief Return and clear the first RX status bit found
 *
 * The MCP2518FD tracks its RX FIFO itself, so there is no status word to consume
 * a bit from; this always reports one pending RX.
 *
 * @param[in,out] dev Handle
 * @param[in,out] status Status word to consume a bit from
 *
 * @return 1
 */
uint8_t mcp2518fd_check_clear_rx_status(mcp2518fd_t *dev, uint8_t *status);

/**
 * @brief Return and clear the first TX status bit found
 *
 * The MCP2518FD tracks its TX FIFO itself, so there is no status word to consume
 * a bit from; this always reports one free TX buffer.
 *
 * @param[in,out] dev Handle
 * @param[in,out] status Status word to consume a bit from
 * @param[in] i_tx_buf TX buffer index; unused
 *
 * @return 1
 */
uint8_t mcp2518fd_check_clear_tx_status(mcp2518fd_t *dev, uint8_t *status, uint8_t i_tx_buf);

/**
 * @brief Switch one of the controller's two pins between interrupt and GPIO mode
 *
 * @param[in,out] dev Handle
 * @param[in] pin MCP2518FD_GPIO_PIN_0 or MCP2518FD_GPIO_PIN_1
 * @param[in] mode MCP2518FD_GPIO_MODE_INT or MCP2518FD_GPIO_MODE_GPIO
 *
 * @return true on success
 */
bool mcp2518fd_pin_mode(mcp2518fd_t *dev, uint8_t pin, uint8_t mode);

/**
 * @brief Drive one of the controller's pins high or low
 *
 * The pin must first be put in GPIO mode with mcp2518fd_pin_mode().
 *
 * @param[in,out] dev Handle
 * @param[in] pin MCP2518FD_GPIO_PIN_0 or MCP2518FD_GPIO_PIN_1
 * @param[in] mode MCP2518FD_GPIO_LOW or MCP2518FD_GPIO_HIGH
 *
 * @return true on success
 */
bool mcp2518fd_digital_write(mcp2518fd_t *dev, uint8_t pin, uint8_t mode);

/**
 * @brief Read the level on one of the controller's pins
 *
 * @param[in,out] dev Handle
 * @param[in] pin MCP2518FD_GPIO_PIN_0 or MCP2518FD_GPIO_PIN_1
 *
 * @return MCP2518FD_GPIO_LOW or MCP2518FD_GPIO_HIGH
 */
uint8_t mcp2518fd_digital_read(mcp2518fd_t *dev, uint8_t pin);

/**
 * @brief Convert a CAN FD data length code into a byte count
 *
 * @param[in] dlc Data length code, a ::CAN_DLC
 *
 * @return Payload length in bytes
 */
uint8_t mcp2518fd_dlc2len(uint8_t dlc);

/**
 * @brief Convert a byte count into a CAN FD data length code
 *
 * CAN FD only supports certain payload lengths, so the result rounds up to the
 * next supported size.
 *
 * @param[in] len Payload length in bytes, 0 to 64
 *
 * @return Matching ::CAN_DLC
 */
uint8_t mcp2518fd_len2dlc(uint8_t len);

#ifdef __cplusplus
}
#endif
