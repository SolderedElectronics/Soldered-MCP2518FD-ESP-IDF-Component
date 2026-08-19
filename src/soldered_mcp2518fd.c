/**
 * @file soldered_mcp2518fd.c
 * @brief Implementation for the soldered-mcp2518fd component
 *
 * Most of the register level code derives from the Microchip MCP2518FD SDK, and
 * the bit timing calculation from https://github.com/pierremolinaro/acan2517FD.
 *
 * @author Soldered Electronics
 */

#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soldered_mcp2518fd.h"

static const char *TAG = "MCP2518FD";

/* Largest single SPI transfer the driver issues: a full RAM init block plus the
 * two byte command header, rounded up to a multiple of four for DMA. */
#define SPI_DEFAULT_BUFFER_LENGTH 96
#define SPI_BUF_LEN               (SPI_DEFAULT_BUFFER_LENGTH + 8)

/* CRC-16/USB parameters used by the WRITE_SAFE and READ_CRC instructions */
#define CRCBASE  0xFFFF
#define CRCUPPER 1

/* How many times to poll the TX FIFO for free space before giving up */
#define MAX_TXQUEUE_ATTEMPTS 50

/* FIFO channels the driver uses. FIFO 0 is the TX queue and cannot be used for RX. */
#define APP_TX_FIFO CAN_FIFO_CH2
#define APP_RX_FIFO CAN_FIFO_CH1

// *****************************************************************************
// Section: Reset values
//
// Register reset values, straight out of the MCP2518FD datasheet. Kept here
// rather than in the header so that including the header does not drag unused
// constants into every translation unit.

/** Control register reset values, addresses 0x000 to 0x04C */
static const uint32_t canControlResetValues[] = {
    /* Address 0x000 to 0x00C */
    0x04980760, 0x003E0F0F, 0x000E0303, 0x00021000,
    /* Address 0x010 to 0x01C */
    0x00000000, 0x00000000, 0x40400040, 0x00000000,
    /* Address 0x020 to 0x02C */
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* Address 0x030 to 0x03C */
    0x00000000, 0x00200000, 0x00000000, 0x00000000,
    /* Address 0x040 to 0x04C */
    0x00000400, 0x00000000, 0x00000000, 0x00000000
};

/** FIFO register reset values */
static const uint32_t canFifoResetValues[] = {0x00600400, 0x00000000, 0x00000000};

/** Look-up table for the SPI CRC calculation */
static const uint16_t crc16_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D,
    0x8027, 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A,
    0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5,
    0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D,
    0x8197, 0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA,
    0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145,
    0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132, 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E,
    0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A,
    0x832B, 0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356,
    0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD,
    0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A,
    0x829B, 0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6,
    0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E,
    0x0264, 0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
    0x0208, 0x820D, 0x8207, 0x0202
};

// *****************************************************************************
// Section: Small helpers

/**
 * @brief Block for at least the given number of milliseconds
 *
 * pdMS_TO_TICKS() rounds down, which turns short waits into no wait at all on a
 * coarse tick rate. Round up to one tick instead.
 */
static void mcp_delay_ms(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay(ticks ? ticks : 1);
}

/**
 * @brief CRC-16 over a buffer, as expected by the controller's safe-write instructions
 */
static uint16_t calculate_crc16(const uint8_t *data, uint16_t size)
{
    uint16_t init = CRCBASE;
    uint8_t index;

    while (size-- != 0) {
        index = ((uint8_t *)&init)[CRCUPPER] ^ *data++;
        init = (init << 8) ^ crc16_table[index];
    }

    return init;
}

/**
 * @brief Run one full duplex SPI transfer out of the handle's buffers
 *
 * One transaction per register access, so the SPI peripheral drives chip select
 * itself and holds it low for the whole command.
 *
 * @param[in,out] dev Handle
 * @param[in] len Number of bytes to clock, including the command header
 *
 * @return 0 on success, -1 if the transfer failed or would overrun the buffers
 */
static int8_t spi_xfer(mcp2518fd_t *dev, size_t len)
{
    if (len > SPI_BUF_LEN) {
        ESP_LOGE(TAG, "SPI transfer of %u bytes exceeds the %d byte buffer", (unsigned)len, SPI_BUF_LEN);
        return -1;
    }

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = dev->spi_tx,
        .rx_buffer = dev->spi_rx,
    };

    return (spi_device_polling_transmit(dev->spi, &t) == ESP_OK) ? 0 : -1;
}

/**
 * @brief Convert a data length code into the number of payload bytes it stands for
 */
static uint32_t dlc_to_data_bytes(CAN_DLC dlc)
{
    uint32_t dataBytesInObject = 0;

    if (dlc < CAN_DLC_12) {
        dataBytesInObject = dlc;
    } else {
        switch (dlc) {
        case CAN_DLC_12:
            dataBytesInObject = 12;
            break;
        case CAN_DLC_16:
            dataBytesInObject = 16;
            break;
        case CAN_DLC_20:
            dataBytesInObject = 20;
            break;
        case CAN_DLC_24:
            dataBytesInObject = 24;
            break;
        case CAN_DLC_32:
            dataBytesInObject = 32;
            break;
        case CAN_DLC_48:
            dataBytesInObject = 48;
            break;
        case CAN_DLC_64:
            dataBytesInObject = 64;
            break;
        default:
            break;
        }
    }

    return dataBytesInObject;
}

// *****************************************************************************
// Section: Raw register access

/**
 * @brief Reset the controller
 */
static int8_t mcp_reset(mcp2518fd_t *dev)
{
    int8_t err;

    dev->spi_tx[0] = (uint8_t)(cINSTRUCTION_RESET << 4);
    dev->spi_tx[1] = 0;

    err = spi_xfer(dev, 2);
    mcp_delay_ms(10);

    return err;
}

/**
 * @brief Read a single byte register
 */
static int8_t mcp_read_byte(mcp2518fd_t *dev, uint16_t address, uint8_t *rxd)
{
    int8_t err;

    dev->spi_tx[0] = (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
    dev->spi_tx[1] = (uint8_t)(address & 0xFF);
    dev->spi_tx[2] = 0;

    err = spi_xfer(dev, 3);
    *rxd = dev->spi_rx[2];

    return err;
}

/**
 * @brief Write a single byte register
 */
static int8_t mcp_write_byte(mcp2518fd_t *dev, uint16_t address, uint8_t txd)
{
    dev->spi_tx[0] = (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
    dev->spi_tx[1] = (uint8_t)(address & 0xFF);
    dev->spi_tx[2] = txd;

    return spi_xfer(dev, 3);
}

/**
 * @brief Read a 32 bit register
 */
static int8_t mcp_read_word(mcp2518fd_t *dev, uint16_t address, uint32_t *rxd)
{
    uint8_t i;
    int8_t err;

    dev->spi_tx[0] = (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
    dev->spi_tx[1] = (uint8_t)(address & 0xFF);
    memset(&dev->spi_tx[2], 0, 4);

    err = spi_xfer(dev, 6);

    *rxd = 0;
    for (i = 2; i < 6; i++) {
        *rxd += ((uint32_t)dev->spi_rx[i]) << ((i - 2) * 8);
    }

    return err;
}

/**
 * @brief Write a 32 bit register
 */
static int8_t mcp_write_word(mcp2518fd_t *dev, uint16_t address, uint32_t txd)
{
    uint8_t i;

    dev->spi_tx[0] = (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
    dev->spi_tx[1] = (uint8_t)(address & 0xFF);

    for (i = 0; i < 4; i++) {
        dev->spi_tx[i + 2] = (uint8_t)((txd >> (i * 8)) & 0xFF);
    }

    return spi_xfer(dev, 6);
}

/**
 * @brief Read a 16 bit register
 */
static int8_t mcp_read_half_word(mcp2518fd_t *dev, uint16_t address, uint16_t *rxd)
{
    uint8_t i;
    int8_t err;

    dev->spi_tx[0] = (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
    dev->spi_tx[1] = (uint8_t)(address & 0xFF);
    memset(&dev->spi_tx[2], 0, 2);

    err = spi_xfer(dev, 4);

    *rxd = 0;
    for (i = 2; i < 4; i++) {
        *rxd += ((uint16_t)dev->spi_rx[i]) << ((i - 2) * 8);
    }

    return err;
}

/**
 * @brief Write a 16 bit register
 */
static int8_t mcp_write_half_word(mcp2518fd_t *dev, uint16_t address, uint16_t txd)
{
    uint8_t i;

    dev->spi_tx[0] = (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
    dev->spi_tx[1] = (uint8_t)(address & 0xFF);

    for (i = 0; i < 2; i++) {
        dev->spi_tx[i + 2] = (uint8_t)((txd >> (i * 8)) & 0xFF);
    }

    return spi_xfer(dev, 4);
}

/**
 * @brief Read a run of bytes, typically out of the controller's message RAM
 */
static int8_t mcp_read_byte_array(mcp2518fd_t *dev, uint16_t address, uint8_t *rxd, uint16_t nBytes)
{
    uint16_t i;
    uint16_t spiTransferSize = nBytes + 2;
    int8_t err;

    dev->spi_tx[0] = (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
    dev->spi_tx[1] = (uint8_t)(address & 0xFF);
    memset(&dev->spi_tx[2], 0, nBytes);

    err = spi_xfer(dev, spiTransferSize);
    if (err) {
        return err;
    }

    for (i = 0; i < nBytes; i++) {
        rxd[i] = dev->spi_rx[i + 2];
    }

    return err;
}

/**
 * @brief Write a run of bytes, typically into the controller's message RAM
 */
static int8_t mcp_write_byte_array(mcp2518fd_t *dev, uint16_t address, const uint8_t *txd, uint16_t nBytes)
{
    uint16_t spiTransferSize = nBytes + 2;

    dev->spi_tx[0] = (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
    dev->spi_tx[1] = (uint8_t)(address & 0xFF);
    memcpy(&dev->spi_tx[2], txd, nBytes);

    return spi_xfer(dev, spiTransferSize);
}

/**
 * @brief Write a single byte register, protected by a CRC
 */
static int8_t __attribute__((unused)) mcp_write_byte_safe(mcp2518fd_t *dev, uint16_t address, uint8_t txd)
{
    uint16_t crcResult;

    dev->spi_tx[0] = (uint8_t)((cINSTRUCTION_WRITE_SAFE << 4) + ((address >> 8) & 0xF));
    dev->spi_tx[1] = (uint8_t)(address & 0xFF);
    dev->spi_tx[2] = txd;

    crcResult = calculate_crc16(dev->spi_tx, 3);
    dev->spi_tx[3] = (crcResult >> 8) & 0xFF;
    dev->spi_tx[4] = crcResult & 0xFF;

    return spi_xfer(dev, 5);
}

/**
 * @brief Write a 32 bit register, protected by a CRC
 */
static int8_t __attribute__((unused)) mcp_write_word_safe(mcp2518fd_t *dev, uint16_t address, uint32_t txd)
{
    uint8_t i;
    uint16_t crcResult;

    dev->spi_tx[0] = (uint8_t)((cINSTRUCTION_WRITE_SAFE << 4) + ((address >> 8) & 0xF));
    dev->spi_tx[1] = (uint8_t)(address & 0xFF);

    for (i = 0; i < 4; i++) {
        dev->spi_tx[i + 2] = (uint8_t)((txd >> (i * 8)) & 0xFF);
    }

    crcResult = calculate_crc16(dev->spi_tx, 6);
    dev->spi_tx[6] = (crcResult >> 8) & 0xFF;
    dev->spi_tx[7] = crcResult & 0xFF;

    return spi_xfer(dev, 8);
}

/**
 * @brief Read a run of bytes and verify the CRC the controller appends
 */
static int8_t __attribute__((unused))
mcp_read_byte_array_with_crc(mcp2518fd_t *dev, uint16_t address, uint8_t *rxd, uint16_t nBytes, bool fromRam,
                             bool *crcIsCorrect)
{
    uint16_t i;
    uint16_t crcFromSpiSlave;
    uint16_t crcAtController;
    /* Two bytes of command and address, one of size, and two of CRC */
    uint16_t spiTransferSize = nBytes + 5;
    int8_t err;

    dev->spi_tx[0] = (uint8_t)((cINSTRUCTION_READ_CRC << 4) + ((address >> 8) & 0xF));
    dev->spi_tx[1] = (uint8_t)(address & 0xFF);
    dev->spi_tx[2] = fromRam ? (nBytes >> 2) : nBytes;
    memset(&dev->spi_tx[3], 0, spiTransferSize - 3);

    err = spi_xfer(dev, spiTransferSize);
    if (err) {
        return err;
    }

    crcFromSpiSlave =
        (uint16_t)(dev->spi_rx[spiTransferSize - 2] << 8) + (uint16_t)(dev->spi_rx[spiTransferSize - 1]);

    /* The CRC covers the command bytes too, so put them back over the garbage
     * that was clocked in while they were going out. */
    dev->spi_rx[0] = dev->spi_tx[0];
    dev->spi_rx[1] = dev->spi_tx[1];
    dev->spi_rx[2] = dev->spi_tx[2];
    crcAtController = calculate_crc16(dev->spi_rx, nBytes + 3);

    *crcIsCorrect = (crcFromSpiSlave == crcAtController);

    for (i = 0; i < nBytes; i++) {
        rxd[i] = dev->spi_rx[i + 3];
    }

    return err;
}

/**
 * @brief Write a run of bytes, protected by a CRC
 */
static int8_t __attribute__((unused))
mcp_write_byte_array_with_crc(mcp2518fd_t *dev, uint16_t address, const uint8_t *txd, uint16_t nBytes, bool fromRam)
{
    uint16_t crcResult;
    uint16_t spiTransferSize = nBytes + 5;

    dev->spi_tx[0] = (uint8_t)((cINSTRUCTION_WRITE_CRC << 4) + ((address >> 8) & 0xF));
    dev->spi_tx[1] = (uint8_t)(address & 0xFF);
    dev->spi_tx[2] = fromRam ? (nBytes >> 2) : nBytes;
    memcpy(&dev->spi_tx[3], txd, nBytes);

    crcResult = calculate_crc16(dev->spi_tx, spiTransferSize - 2);
    dev->spi_tx[spiTransferSize - 2] = (uint8_t)((crcResult >> 8) & 0xFF);
    dev->spi_tx[spiTransferSize - 1] = (uint8_t)(crcResult & 0xFF);

    return spi_xfer(dev, spiTransferSize);
}

/**
 * @brief Read a run of 32 bit registers
 */
static int8_t mcp_read_word_array(mcp2518fd_t *dev, uint16_t address, uint32_t *rxd, uint16_t nWords)
{
    uint16_t i, j, n;
    REG_t w;
    uint16_t spiTransferSize = nWords * 4 + 2;
    int8_t err;

    dev->spi_tx[0] = (cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF);
    dev->spi_tx[1] = address & 0xFF;
    memset(&dev->spi_tx[2], 0, nWords * 4);

    err = spi_xfer(dev, spiTransferSize);
    if (err) {
        return err;
    }

    n = 2;
    for (i = 0; i < nWords; i++) {
        w.word = 0;
        for (j = 0; j < 4; j++, n++) {
            w.byte[j] = dev->spi_rx[n];
        }
        rxd[i] = w.word;
    }

    return err;
}

/**
 * @brief Write a run of 32 bit registers
 */
static int8_t __attribute__((unused))
mcp_write_word_array(mcp2518fd_t *dev, uint16_t address, const uint32_t *txd, uint16_t nWords)
{
    uint16_t i, j, n;
    REG_t w;
    uint16_t spiTransferSize = nWords * 4 + 2;

    dev->spi_tx[0] = (cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF);
    dev->spi_tx[1] = address & 0xFF;

    n = 2;
    for (i = 0; i < nWords; i++) {
        w.word = txd[i];
        for (j = 0; j < 4; j++, n++) {
            dev->spi_tx[n] = w.byte[j];
        }
    }

    return spi_xfer(dev, spiTransferSize);
}

// *****************************************************************************
// Section: Controller configuration

/**
 * @brief Turn on the message RAM's error correcting code
 */
static int8_t mcp_ecc_enable(mcp2518fd_t *dev)
{
    int8_t err;
    uint8_t d = 0;

    err = mcp_read_byte(dev, cREGADDR_ECCCON, &d);
    if (err) {
        return -1;
    }

    d |= 0x01;

    err = mcp_write_byte(dev, cREGADDR_ECCCON, d);
    if (err) {
        return -2;
    }

    return 0;
}

/**
 * @brief Fill the whole message RAM with a known value, so the ECC has valid parity
 */
static int8_t mcp_ram_init(mcp2518fd_t *dev, uint8_t d)
{
    uint8_t txd[SPI_DEFAULT_BUFFER_LENGTH];
    uint32_t k;
    uint16_t a = cRAMADDR_START;
    int8_t err = 0;

    memset(txd, d, sizeof(txd));

    for (k = 0; k < (cRAM_SIZE / SPI_DEFAULT_BUFFER_LENGTH); k++) {
        err = mcp_write_byte_array(dev, a, txd, SPI_DEFAULT_BUFFER_LENGTH);
        if (err) {
            return -1;
        }
        a += SPI_DEFAULT_BUFFER_LENGTH;
    }

    return err;
}

/**
 * @brief Fill a config struct with the controller's reset defaults
 */
static int8_t mcp_configure_object_reset(CAN_CONFIG *config)
{
    REG_CiCON ciCon;
    ciCon.word = canControlResetValues[cREGADDR_CiCON / 4];

    config->DNetFilterCount = ciCon.bF.DNetFilterCount;
    config->IsoCrcEnable = ciCon.bF.IsoCrcEnable;
    config->ProtocolExpectionEventDisable = ciCon.bF.ProtocolExceptionEventDisable;
    config->WakeUpFilterEnable = ciCon.bF.WakeUpFilterEnable;
    config->WakeUpFilterTime = ciCon.bF.WakeUpFilterTime;
    config->BitRateSwitchDisable = ciCon.bF.BitRateSwitchDisable;
    config->RestrictReTxAttempts = ciCon.bF.RestrictReTxAttempts;
    config->EsiInGatewayMode = ciCon.bF.EsiInGatewayMode;
    config->SystemErrorToListenOnly = ciCon.bF.SystemErrorToListenOnly;
    config->StoreInTEF = ciCon.bF.StoreInTEF;
    config->TXQEnable = ciCon.bF.TXQEnable;
    config->TxBandWidthSharing = ciCon.bF.TxBandWidthSharing;

    return 0;
}

/**
 * @brief Push a config struct into the controller's CiCON register
 */
static int8_t mcp_configure(mcp2518fd_t *dev, CAN_CONFIG *config)
{
    REG_CiCON ciCon;
    int8_t err;

    ciCon.word = canControlResetValues[cREGADDR_CiCON / 4];

    ciCon.bF.DNetFilterCount = config->DNetFilterCount;
    ciCon.bF.IsoCrcEnable = config->IsoCrcEnable;
    ciCon.bF.ProtocolExceptionEventDisable = config->ProtocolExpectionEventDisable;
    ciCon.bF.WakeUpFilterEnable = config->WakeUpFilterEnable;
    ciCon.bF.WakeUpFilterTime = config->WakeUpFilterTime;
    ciCon.bF.BitRateSwitchDisable = config->BitRateSwitchDisable;
    ciCon.bF.RestrictReTxAttempts = config->RestrictReTxAttempts;
    ciCon.bF.EsiInGatewayMode = config->EsiInGatewayMode;
    ciCon.bF.SystemErrorToListenOnly = config->SystemErrorToListenOnly;
    ciCon.bF.StoreInTEF = config->StoreInTEF;
    ciCon.bF.TXQEnable = config->TXQEnable;
    ciCon.bF.TxBandWidthSharing = config->TxBandWidthSharing;

    err = mcp_write_word(dev, cREGADDR_CiCON, ciCon.word);
    if (err) {
        return -1;
    }

    return err;
}

/**
 * @brief Fill a TX FIFO config struct with the controller's reset defaults
 */
static int8_t mcp_tx_channel_configure_object_reset(CAN_TX_FIFO_CONFIG *config)
{
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = canFifoResetValues[0];

    config->RTREnable = ciFifoCon.txBF.RTREnable;
    config->TxPriority = ciFifoCon.txBF.TxPriority;
    config->TxAttempts = ciFifoCon.txBF.TxAttempts;
    config->FifoSize = ciFifoCon.txBF.FifoSize;
    config->PayLoadSize = ciFifoCon.txBF.PayLoadSize;

    return 0;
}

/**
 * @brief Set up one FIFO channel for transmitting
 */
static int8_t mcp_tx_channel_configure(mcp2518fd_t *dev, CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_CONFIG *config)
{
    uint16_t a;
    REG_CiFIFOCON ciFifoCon;

    ciFifoCon.word = canFifoResetValues[0];
    ciFifoCon.txBF.TxEnable = 1;
    ciFifoCon.txBF.FifoSize = config->FifoSize;
    ciFifoCon.txBF.PayLoadSize = config->PayLoadSize;
    ciFifoCon.txBF.TxAttempts = config->TxAttempts;
    ciFifoCon.txBF.TxPriority = config->TxPriority;
    ciFifoCon.txBF.RTREnable = config->RTREnable;

    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

    return mcp_write_word(dev, a, ciFifoCon.word);
}

/**
 * @brief Fill an RX FIFO config struct with the controller's reset defaults
 */
static int8_t mcp_rx_channel_configure_object_reset(CAN_RX_FIFO_CONFIG *config)
{
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = canFifoResetValues[0];

    config->FifoSize = ciFifoCon.rxBF.FifoSize;
    config->PayLoadSize = ciFifoCon.rxBF.PayLoadSize;
    config->RxTimeStampEnable = ciFifoCon.rxBF.RxTimeStampEnable;

    return 0;
}

/**
 * @brief Set up one FIFO channel for receiving
 */
static int8_t mcp_rx_channel_configure(mcp2518fd_t *dev, CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_CONFIG *config)
{
    uint16_t a;
    REG_CiFIFOCON ciFifoCon;

    /* Channel 0 is the TX queue and can never receive */
    if (channel == CAN_TXQUEUE_CH0) {
        return -100;
    }

    ciFifoCon.word = canFifoResetValues[0];
    ciFifoCon.rxBF.TxEnable = 0;
    ciFifoCon.rxBF.FifoSize = config->FifoSize;
    ciFifoCon.rxBF.PayLoadSize = config->PayLoadSize;
    ciFifoCon.rxBF.RxTimeStampEnable = config->RxTimeStampEnable;

    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

    return mcp_write_word(dev, a, ciFifoCon.word);
}

/**
 * @brief Write one acceptance filter's ID
 */
static int8_t mcp_filter_object_configure(mcp2518fd_t *dev, CAN_FILTER filter, CAN_FILTEROBJ_ID *id)
{
    uint16_t a;
    REG_CiFLTOBJ fObj;

    fObj.word = 0;
    fObj.bF = *id;
    a = cREGADDR_CiFLTOBJ + (filter * CiFILTER_OFFSET);

    ESP_LOGD(TAG, "filter %d at 0x%03X = 0x%08X", (int)filter, a, (unsigned int)fObj.word);

    return mcp_write_word(dev, a, fObj.word);
}

/**
 * @brief Write one acceptance filter's mask
 */
static int8_t mcp_filter_mask_configure(mcp2518fd_t *dev, CAN_FILTER filter, CAN_MASKOBJ_ID *mask)
{
    uint16_t a;
    REG_CiMASK mObj;

    mObj.word = 0;
    mObj.bF = *mask;
    a = cREGADDR_CiMASK + (filter * CiFILTER_OFFSET);

    ESP_LOGD(TAG, "mask %d at 0x%03X = 0x%08X", (int)filter, a, (unsigned int)mObj.word);

    return mcp_write_word(dev, a, mObj.word);
}

/**
 * @brief Point one acceptance filter at a FIFO, and enable or disable it
 */
static int8_t mcp_filter_to_fifo_link(mcp2518fd_t *dev, CAN_FILTER filter, CAN_FIFO_CHANNEL channel, bool enable)
{
    uint16_t a;
    REG_CiFLTCON_BYTE fCtrl;

    fCtrl.byte = 0;
    fCtrl.bF.Enable = !!enable;
    fCtrl.bF.BufferPointer = channel;
    a = cREGADDR_CiFLTCON + filter;

    return mcp_write_byte(dev, a, fCtrl.byte);
}

// *****************************************************************************
// Section: Bit timing
//
// Ported from https://github.com/pierremolinaro/acan2517FD

static const uint16_t MAX_BRP = 256;
static const uint16_t MAX_ARBITRATION_PHASE_SEGMENT_1 = 256;
static const uint8_t MAX_ARBITRATION_PHASE_SEGMENT_2 = 128;
static const uint16_t MAX_DATA_PHASE_SEGMENT_1 = 32;
static const uint8_t MAX_DATA_PHASE_SEGMENT_2 = 16;

/**
 * @brief Search for the prescaler and segment lengths that best hit the requested bit rate
 *
 * Aims for a sample point at 80% of the bit. When a data phase factor is set,
 * the data phase is solved first and the arbitration phase derived from it, so
 * that both share one prescaler as the controller requires.
 *
 * @param[in,out] dev Handle
 * @param[in] inDesiredArbitrationBitRate Arbitration phase bit rate in bps
 * @param[in] inTolerancePPM Acceptable deviation in parts per million
 *
 * @return 1 if the result is within tolerance, 0 if not
 */
static int mcp_calc_bittime(mcp2518fd_t *dev, const uint32_t inDesiredArbitrationBitRate, const uint32_t inTolerancePPM)
{
    if (dev->data_bit_rate_factor <= 1) { // Single bit rate
        const uint32_t maxTQCount =
            MAX_ARBITRATION_PHASE_SEGMENT_1 + MAX_ARBITRATION_PHASE_SEGMENT_2 + 1; // Setting for slowest bit rate
        uint32_t BRP = MAX_BRP;
        uint32_t smallestError = UINT32_MAX;
        uint32_t bestBRP = 1;      // Setting for highest bit rate
        uint32_t bestTQCount = 4;  // Setting for highest bit rate
        uint32_t TQCount = dev->sys_clock / inDesiredArbitrationBitRate / BRP;

        //--- Loop for finding best BRP and best TQCount
        while ((TQCount <= (MAX_ARBITRATION_PHASE_SEGMENT_1 + MAX_ARBITRATION_PHASE_SEGMENT_2 + 1)) && (BRP > 0)) {
            //--- Compute error using TQCount
            if ((TQCount >= 4) && (TQCount <= maxTQCount)) {
                const uint32_t error =
                    dev->sys_clock - inDesiredArbitrationBitRate * TQCount * BRP; // error is always >= 0
                if (error <= smallestError) {
                    smallestError = error;
                    bestBRP = BRP;
                    bestTQCount = TQCount;
                }
            }
            //--- Compute error using TQCount+1
            if ((TQCount >= 3) && (TQCount < maxTQCount)) {
                const uint32_t error =
                    inDesiredArbitrationBitRate * (TQCount + 1) * BRP - dev->sys_clock; // error is always >= 0
                if (error <= smallestError) {
                    smallestError = error;
                    bestBRP = BRP;
                    bestTQCount = TQCount + 1;
                }
            }
            //--- Continue with next value of BRP
            BRP--;
            TQCount = (BRP == 0) ? (maxTQCount + 1) : (dev->sys_clock / inDesiredArbitrationBitRate / BRP);
        }
        //--- Compute PS2 (1 <= PS2 <= 128)
        uint32_t PS2 = bestTQCount / 5; // For sampling point at 80%
        if (PS2 == 0) {
            PS2 = 1;
        } else if (PS2 > MAX_ARBITRATION_PHASE_SEGMENT_2) {
            PS2 = MAX_ARBITRATION_PHASE_SEGMENT_2;
        }
        //--- Compute PS1 (1 <= PS1 <= 256)
        uint32_t PS1 = bestTQCount - PS2 - 1 /* Sync Seg */;
        if (PS1 > MAX_ARBITRATION_PHASE_SEGMENT_1) {
            PS2 += PS1 - MAX_ARBITRATION_PHASE_SEGMENT_1;
            PS1 = MAX_ARBITRATION_PHASE_SEGMENT_1;
        }
        //---
        dev->bit_rate_prescaler = (uint16_t)bestBRP;
        dev->arbitration_phase_segment1 = (uint16_t)PS1;
        dev->arbitration_phase_segment2 = (uint8_t)PS2;
        dev->arbitration_sjw = dev->arbitration_phase_segment2; // Always 1 <= SJW <= 128, and SJW <= PS2
        //--- Final check of the nominal configuration
        const uint32_t W = bestTQCount * inDesiredArbitrationBitRate * bestBRP;
        const uint64_t diff = (dev->sys_clock > W) ? (dev->sys_clock - W) : (W - dev->sys_clock);
        const uint64_t ppm = (uint64_t)(1000UL * 1000UL);
        dev->arbitration_bit_rate_ok = (diff * ppm) <= (((uint64_t)W) * inTolerancePPM);
    } else { // Dual bit rate, first compute data bit rate
        const uint32_t maxDataTQCount = MAX_DATA_PHASE_SEGMENT_1 + MAX_DATA_PHASE_SEGMENT_2; // Slowest bit rate
        const uint32_t desiredDataBitRate = inDesiredArbitrationBitRate * (uint8_t)(dev->data_bit_rate_factor);
        uint32_t smallestError = UINT32_MAX;
        uint32_t bestBRP = MAX_BRP;                 // Setting for lowest bit rate
        uint32_t bestDataTQCount = maxDataTQCount;  // Setting for lowest bit rate
        uint32_t dataTQCount = 4;
        uint32_t brp = dev->sys_clock / desiredDataBitRate / dataTQCount;

        //--- Loop for finding best BRP and best TQCount
        while ((dataTQCount <= maxDataTQCount) && (brp > 0)) {
            //--- Compute error using brp
            if (brp <= MAX_BRP) {
                const uint32_t error = dev->sys_clock - desiredDataBitRate * dataTQCount * brp; // always >= 0
                if (error <= smallestError) {
                    smallestError = error;
                    bestBRP = brp;
                    bestDataTQCount = dataTQCount;
                }
            }
            //--- Compute error using brp+1
            if (brp < MAX_BRP) {
                const uint32_t error = desiredDataBitRate * dataTQCount * (brp + 1) - dev->sys_clock; // always >= 0
                if (error <= smallestError) {
                    smallestError = error;
                    bestBRP = brp + 1;
                    bestDataTQCount = dataTQCount;
                }
            }
            //--- Continue with next value of BRP
            dataTQCount += 1;
            brp = dev->sys_clock / desiredDataBitRate / dataTQCount;
        }
        //--- Compute data PS2 (1 <= PS2 <= 16)
        uint32_t dataPS2 = bestDataTQCount / 5; // For sampling point at 80%
        if (dataPS2 == 0) {
            dataPS2 = 1;
        }
        //--- Compute data PS1 (1 <= PS1 <= 32)
        uint32_t dataPS1 = bestDataTQCount - dataPS2 - 1 /* Sync Seg */;
        if (dataPS1 > MAX_DATA_PHASE_SEGMENT_1) {
            dataPS2 += dataPS1 - MAX_DATA_PHASE_SEGMENT_1;
            dataPS1 = MAX_DATA_PHASE_SEGMENT_1;
        }
        //---
        const int TDCO = bestBRP * dataPS1; // According to DS20005678D, section 3.4.8 page 20
        dev->tdco = (TDCO > 63) ? 63 : (int8_t)TDCO;
        dev->data_phase_segment1 = (uint8_t)dataPS1;
        dev->data_phase_segment2 = (uint8_t)dataPS2;
        dev->data_sjw = dev->data_phase_segment2;
        const uint32_t arbitrationTQCount = bestDataTQCount * (uint8_t)(dev->data_bit_rate_factor);
        //--- Compute arbitration PS2 (1 <= PS2 <= 128)
        uint32_t arbitrationPS2 = arbitrationTQCount / 5; // For sampling point at 80%
        if (arbitrationPS2 == 0) {
            arbitrationPS2 = 1;
        }
        //--- Compute PS1 (1 <= PS1 <= 256)
        uint32_t arbitrationPS1 = arbitrationTQCount - arbitrationPS2 - 1 /* Sync Seg */;
        if (arbitrationPS1 > MAX_ARBITRATION_PHASE_SEGMENT_1) {
            arbitrationPS2 += arbitrationPS1 - MAX_ARBITRATION_PHASE_SEGMENT_1;
            arbitrationPS1 = MAX_ARBITRATION_PHASE_SEGMENT_1;
        }
        //---
        dev->bit_rate_prescaler = (uint16_t)bestBRP;
        dev->arbitration_phase_segment1 = (uint16_t)arbitrationPS1;
        dev->arbitration_phase_segment2 = (uint8_t)arbitrationPS2;
        dev->arbitration_sjw = dev->arbitration_phase_segment2;
        //--- Final check of the nominal configuration
        const uint32_t W = arbitrationTQCount * inDesiredArbitrationBitRate * bestBRP;
        const uint64_t diff = (dev->sys_clock > W) ? (dev->sys_clock - W) : (W - dev->sys_clock);
        const uint64_t ppm = (uint64_t)(1000UL * 1000UL);
        dev->arbitration_bit_rate_ok = (diff * ppm) <= (((uint64_t)W) * inTolerancePPM);
    }

    return dev->arbitration_bit_rate_ok;
}

/**
 * @brief Write the arbitration phase bit timing registers
 */
static int8_t mcp_bit_time_configure_nominal(mcp2518fd_t *dev)
{
    int8_t err;
    REG_CiNBTCFG ciNbtcfg;

    ciNbtcfg.word = canControlResetValues[cREGADDR_CiNBTCFG / 4];

    ciNbtcfg.bF.BRP = dev->bit_rate_prescaler - 1;
    ciNbtcfg.bF.TSEG1 = dev->arbitration_phase_segment1;
    ciNbtcfg.bF.TSEG2 = dev->arbitration_phase_segment2;
    ciNbtcfg.bF.SJW = dev->arbitration_sjw - 1;

    err = mcp_write_word(dev, cREGADDR_CiNBTCFG, ciNbtcfg.word);
    if (err) {
        return -2;
    }

    return err;
}

/**
 * @brief Write the data phase bit timing and transmitter delay compensation registers
 */
static int8_t mcp_bit_time_configure_data(mcp2518fd_t *dev, CAN_SSP_MODE sspMode)
{
    int8_t err;
    REG_CiDBTCFG ciDbtcfg;
    REG_CiTDC ciTdc;

    ciDbtcfg.word = canControlResetValues[cREGADDR_CiDBTCFG / 4];
    ciDbtcfg.bF.BRP = dev->bit_rate_prescaler - 1;
    ciDbtcfg.bF.TSEG1 = dev->data_phase_segment1 - 1;
    ciDbtcfg.bF.TSEG2 = dev->data_phase_segment2 - 1;
    ciDbtcfg.bF.SJW = dev->data_sjw - 1;

    err = mcp_write_word(dev, cREGADDR_CiDBTCFG, ciDbtcfg.word);
    if (err) {
        return -2;
    }

    ciTdc.word = canControlResetValues[cREGADDR_CiTDC / 4];
    ciTdc.bF.TDCMode = sspMode;
    ciTdc.bF.TDCOffset = dev->tdco;

    err = mcp_write_word(dev, cREGADDR_CiTDC, ciTdc.word);
    if (err) {
        return -3;
    }

    return err;
}

/**
 * @brief Decode a bit rate word, solve the bit timing, and write it to the controller
 */
static int8_t mcp_bit_time_configure(mcp2518fd_t *dev, uint32_t speedset, CAN_SSP_MODE sspMode, CAN_SYSCLK_SPEED clk)
{
    dev->desired_arbitration_bit_rate = speedset & 0xFFFFFUL;
    dev->data_bit_rate_factor = (speedset >> 24) & 0xFF;

    switch (clk) {
    case CAN_SYSCLK_10M:
        dev->sys_clock = 10UL * 1000UL * 1000UL;
        break;
    case CAN_SYSCLK_20M:
        dev->sys_clock = 20UL * 1000UL * 1000UL;
        break;
    case CAN_SYSCLK_40M:
    default:
        dev->sys_clock = 40UL * 1000UL * 1000UL;
        break;
    }

    /* 10000 ppm, i.e. accept up to 1% off the requested bit rate */
    mcp_calc_bittime(dev, dev->desired_arbitration_bit_rate, 10000);
    if (!dev->arbitration_bit_rate_ok) {
        ESP_LOGW(TAG, "Requested bit rate of %u bps cannot be hit within 1%% from a %u Hz oscillator",
                 (unsigned int)dev->desired_arbitration_bit_rate, (unsigned int)dev->sys_clock);
    }

    mcp_bit_time_configure_nominal(dev);
    mcp_bit_time_configure_data(dev, sspMode);

    return 0;
}

// *****************************************************************************
// Section: Events and modes

/**
 * @brief Set what the controller's two pins do
 */
static int8_t mcp_gpio_mode_configure(mcp2518fd_t *dev, GPIO_PIN_MODE gpio0, GPIO_PIN_MODE gpio1)
{
    int8_t err;
    uint16_t a = cREGADDR_IOCON + 3;
    REG_IOCON iocon;

    iocon.word = 0;

    err = mcp_read_byte(dev, a, &iocon.byte[3]);
    if (err) {
        return -1;
    }

    iocon.bF.PinMode0 = gpio0;
    iocon.bF.PinMode1 = gpio1;

    err = mcp_write_byte(dev, a, iocon.byte[3]);
    if (err) {
        return -2;
    }

    return err;
}

/**
 * @brief Enable interrupt sources on a TX FIFO
 */
static int8_t mcp_tx_channel_event_enable(mcp2518fd_t *dev, CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_EVENT flags)
{
    int8_t err;
    uint16_t a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    REG_CiFIFOCON ciFifoCon;

    ciFifoCon.word = 0;

    err = mcp_read_byte(dev, a, &ciFifoCon.byte[0]);
    if (err) {
        return -1;
    }

    ciFifoCon.byte[0] |= (flags & CAN_TX_FIFO_ALL_EVENTS);

    err = mcp_write_byte(dev, a, ciFifoCon.byte[0]);
    if (err) {
        return -2;
    }

    return err;
}

/**
 * @brief Enable interrupt sources on an RX FIFO
 */
static int8_t mcp_rx_channel_event_enable(mcp2518fd_t *dev, CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_EVENT flags)
{
    int8_t err;
    uint16_t a;
    REG_CiFIFOCON ciFifoCon;

    if (channel == CAN_TXQUEUE_CH0) {
        return -100;
    }

    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    ciFifoCon.word = 0;

    err = mcp_read_byte(dev, a, &ciFifoCon.byte[0]);
    if (err) {
        return -1;
    }

    ciFifoCon.byte[0] |= (flags & CAN_RX_FIFO_ALL_EVENTS);

    err = mcp_write_byte(dev, a, ciFifoCon.byte[0]);
    if (err) {
        return -2;
    }

    return err;
}

/**
 * @brief Enable module level interrupt sources
 */
static int8_t mcp_module_event_enable(mcp2518fd_t *dev, CAN_MODULE_EVENT flags)
{
    int8_t err;
    uint16_t a = cREGADDR_CiINTENABLE;
    REG_CiINTENABLE intEnables;

    intEnables.word = 0;

    err = mcp_read_half_word(dev, a, &intEnables.word);
    if (err) {
        return -1;
    }

    intEnables.word |= (flags & CAN_ALL_EVENTS);

    err = mcp_write_half_word(dev, a, intEnables.word);
    if (err) {
        return -2;
    }

    return err;
}

/**
 * @brief Request an operation mode from the controller
 */
static int8_t mcp_operation_mode_select(mcp2518fd_t *dev, CAN_OPERATION_MODE opMode)
{
    uint8_t d = 0;
    int8_t err;

    err = mcp_read_byte(dev, cREGADDR_CiCON + 3, &d);
    if (err) {
        return -1;
    }

    d &= ~0x07;
    d |= opMode;

    err = mcp_write_byte(dev, cREGADDR_CiCON + 3, d);
    if (err) {
        return -2;
    }

    return err;
}

/**
 * @brief Read back the operation mode the controller is in
 */
static CAN_OPERATION_MODE mcp_operation_mode_get(mcp2518fd_t *dev)
{
    uint8_t d = 0;

    if (mcp_read_byte(dev, cREGADDR_CiCON + 2, &d)) {
        return CAN_INVALID_MODE;
    }

    d = (d >> 5) & 0x7;

    switch (d) {
    case CAN_NORMAL_MODE:
    case CAN_SLEEP_MODE:
    case CAN_INTERNAL_LOOPBACK_MODE:
    case CAN_EXTERNAL_LOOPBACK_MODE:
    case CAN_LISTEN_ONLY_MODE:
    case CAN_CONFIGURATION_MODE:
    case CAN_CLASSIC_MODE:
    case CAN_RESTRICTED_MODE:
        return (CAN_OPERATION_MODE)d;
    default:
        return CAN_INVALID_MODE;
    }
}

/**
 * @brief Read the pending interrupt flags of a TX FIFO
 */
static int8_t mcp_tx_channel_event_get(mcp2518fd_t *dev, CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_EVENT *flags)
{
    int8_t err;
    uint16_t a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
    REG_CiFIFOSTA ciFifoSta;

    ciFifoSta.word = 0;

    err = mcp_read_byte(dev, a, &ciFifoSta.byte[0]);
    if (err) {
        return -1;
    }

    *flags = (CAN_TX_FIFO_EVENT)(ciFifoSta.byte[0] & CAN_TX_FIFO_ALL_EVENTS);

    return err;
}

/**
 * @brief Read the transmit and receive error counters, and the resulting error state
 */
static int8_t mcp_error_count_state_get(mcp2518fd_t *dev, uint8_t *tec, uint8_t *rec, CAN_ERROR_STATE *flags)
{
    int8_t err;
    REG_CiTREC ciTrec;

    ciTrec.word = 0;

    err = mcp_read_word(dev, cREGADDR_CiTREC, &ciTrec.word);
    if (err) {
        return -1;
    }

    *tec = ciTrec.byte[1];
    *rec = ciTrec.byte[0];
    *flags = (CAN_ERROR_STATE)(ciTrec.byte[2] & CAN_ERROR_ALL);

    return err;
}

/**
 * @brief Read the controller's error state
 */
static int8_t mcp_error_state_get(mcp2518fd_t *dev, CAN_ERROR_STATE *flags)
{
    int8_t err;
    uint8_t f = 0;

    err = mcp_read_byte(dev, cREGADDR_CiTREC + 2, &f);
    if (err) {
        return -1;
    }

    *flags = (CAN_ERROR_STATE)(f & CAN_ERROR_ALL);

    return err;
}

// *****************************************************************************
// Section: FIFO traffic

/**
 * @brief Bump a TX FIFO's write pointer, optionally requesting transmission
 */
static int8_t mcp_tx_channel_update(mcp2518fd_t *dev, CAN_FIFO_CHANNEL channel, bool flush)
{
    uint16_t a;
    REG_CiFIFOCON ciFifoCon;

    /* Byte 1 of CiFIFOCON holds UINC and TXREQ */
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1;
    ciFifoCon.word = 0;
    ciFifoCon.txBF.UINC = 1;

    if (flush) {
        ciFifoCon.txBF.TxRequest = 1;
    }

    if (mcp_write_byte(dev, a, ciFifoCon.byte[1])) {
        return -1;
    }

    return 0;
}

/**
 * @brief Bump an RX FIFO's read pointer, freeing the slot just read
 */
static int8_t mcp_rx_channel_update(mcp2518fd_t *dev, CAN_FIFO_CHANNEL channel)
{
    uint16_t a;
    REG_CiFIFOCON ciFifoCon;

    ciFifoCon.word = 0;

    /* Byte 1 of CiFIFOCON holds UINC */
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1;
    ciFifoCon.rxBF.UINC = 1;

    return mcp_write_byte(dev, a, ciFifoCon.byte[1]);
}

/**
 * @brief Copy a message object and its payload into a TX FIFO's RAM slot
 */
static int8_t mcp_tx_channel_load(mcp2518fd_t *dev, CAN_FIFO_CHANNEL channel, CAN_TX_MSGOBJ *txObj, uint8_t *txd,
                                  uint32_t txdNumBytes, bool flush)
{
    uint16_t a;
    uint32_t fifoReg[3];
    uint32_t dataBytesInObject;
    REG_CiFIFOCON ciFifoCon;
    REG_CiFIFOUA ciFifoUa;
    uint8_t txBuffer[MCP2518FD_MAX_MSG_SIZE];
    uint8_t i;
    uint16_t n = 0;
    uint8_t j;

    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

    if (mcp_read_word_array(dev, a, fifoReg, 3)) {
        return -1;
    }

    /* Refuse to load a FIFO that is not set up for transmitting */
    ciFifoCon.word = fifoReg[0];
    if (!ciFifoCon.txBF.TxEnable) {
        return -2;
    }

    /* Refuse a payload that does not fit the DLC the caller asked for */
    dataBytesInObject = dlc_to_data_bytes((CAN_DLC)txObj->bF.ctrl.DLC);
    if (dataBytesInObject < txdNumBytes) {
        return -3;
    }

    ciFifoUa.word = fifoReg[2];
    a = ciFifoUa.bF.UserAddress;
    a += cRAMADDR_START;

    memcpy(txBuffer, txObj->byte, 8);
    for (i = 0; i < txdNumBytes; i++) {
        txBuffer[i + 8] = txd[i];
    }

    /* Message RAM is only writable in whole words, so pad up to a multiple of four */
    if (txdNumBytes % 4) {
        n = 4 - (txdNumBytes % 4);
        i = txdNumBytes + 8;

        for (j = 0; j < n; j++) {
            txBuffer[i + j] = 0;
        }
    }

    if (mcp_write_byte_array(dev, a, txBuffer, txdNumBytes + 8 + n)) {
        return -4;
    }

    if (mcp_tx_channel_update(dev, channel, flush)) {
        return -5;
    }

    return 0;
}

/**
 * @brief Read the pending interrupt flags of an RX FIFO
 */
static int8_t mcp_rx_channel_event_get(mcp2518fd_t *dev, CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_EVENT *flags)
{
    int8_t err;
    uint16_t a;
    REG_CiFIFOSTA ciFifoSta;

    if (channel == CAN_TXQUEUE_CH0) {
        return -100;
    }

    ciFifoSta.word = 0;
    a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);

    err = mcp_read_byte(dev, a, &ciFifoSta.byte[0]);
    if (err) {
        return -1;
    }

    *flags = (CAN_RX_FIFO_EVENT)(ciFifoSta.byte[0] & CAN_RX_FIFO_ALL_EVENTS);

    return err;
}

/**
 * @brief Read the oldest message object and payload out of an RX FIFO
 */
static int8_t mcp_receive_message_get(mcp2518fd_t *dev, CAN_FIFO_CHANNEL channel, CAN_RX_MSGOBJ *rxObj, uint8_t *rxd,
                                      uint8_t nBytes)
{
    uint8_t n = 0;
    uint8_t i = 0;
    uint16_t a;
    uint32_t fifoReg[3];
    REG_CiFIFOCON ciFifoCon;
    REG_CiFIFOUA ciFifoUa;
    REG_t myReg;
    uint8_t ba[MCP2518FD_MAX_MSG_SIZE];

    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

    if (mcp_read_word_array(dev, a, fifoReg, 3)) {
        return -1;
    }

    ciFifoCon.word = fifoReg[0];

    ciFifoUa.word = fifoReg[2];
    a = ciFifoUa.bF.UserAddress;
    a += cRAMADDR_START;

    /* Eight header bytes, plus four more if timestamps are on */
    n = nBytes + 8;
    if (ciFifoCon.rxBF.RxTimeStampEnable) {
        n += 4;
    }

    /* Message RAM is only readable in whole words */
    if (n % 4) {
        n = n + 4 - (n % 4);
    }

    if (n > MCP2518FD_MAX_MSG_SIZE) {
        n = MCP2518FD_MAX_MSG_SIZE;
    }

    if (mcp_read_byte_array(dev, a, ba, n)) {
        return -3;
    }

    myReg.byte[0] = ba[0];
    myReg.byte[1] = ba[1];
    myReg.byte[2] = ba[2];
    myReg.byte[3] = ba[3];
    rxObj->word[0] = myReg.word;

    myReg.byte[0] = ba[4];
    myReg.byte[1] = ba[5];
    myReg.byte[2] = ba[6];
    myReg.byte[3] = ba[7];
    rxObj->word[1] = myReg.word;

    if (ciFifoCon.rxBF.RxTimeStampEnable) {
        myReg.byte[0] = ba[8];
        myReg.byte[1] = ba[9];
        myReg.byte[2] = ba[10];
        myReg.byte[3] = ba[11];
        rxObj->word[2] = myReg.word;

        for (i = 0; i < nBytes; i++) {
            rxd[i] = ba[i + 12];
        }
    } else {
        rxObj->word[2] = 0;

        for (i = 0; i < nBytes; i++) {
            rxd[i] = ba[i + 8];
        }
    }

    if (mcp_rx_channel_update(dev, channel)) {
        return -4;
    }

    return 0;
}

/**
 * @brief Read how full an RX FIFO is
 */
static int8_t mcp_rx_channel_status_get(mcp2518fd_t *dev, CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_STATUS *status)
{
    uint16_t a;
    REG_CiFIFOSTA ciFifoSta;
    int8_t err;

    ciFifoSta.word = 0;
    a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);

    err = mcp_read_byte(dev, a, &ciFifoSta.byte[0]);
    if (err) {
        return -1;
    }

    *status = (CAN_RX_FIFO_STATUS)(ciFifoSta.byte[0] & 0x0F);

    return err;
}

/**
 * @brief Clear the "transmit attempts exhausted" flag of a TX FIFO
 */
static int8_t mcp_tx_channel_event_attempt_clear(mcp2518fd_t *dev, CAN_FIFO_CHANNEL channel)
{
    int8_t err;
    uint16_t a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
    REG_CiFIFOSTA ciFifoSta;

    ciFifoSta.word = 0;

    err = mcp_read_byte(dev, a, &ciFifoSta.byte[0]);
    if (err) {
        return -1;
    }

    ciFifoSta.byte[0] &= ~CAN_TX_FIFO_ATTEMPTS_EXHAUSTED_EVENT;

    err = mcp_write_byte(dev, a, ciFifoSta.byte[0]);
    if (err) {
        return -2;
    }

    return err;
}

/**
 * @brief Enter the chip's low power mode
 */
static int8_t mcp_low_power_mode_enable(mcp2518fd_t *dev)
{
    int8_t err;
    uint8_t d = 0;

    err = mcp_read_byte(dev, cREGADDR_OSC, &d);
    if (err) {
        return -1;
    }

    d |= 0x08;

    err = mcp_write_byte(dev, cREGADDR_OSC, d);
    if (err) {
        return -2;
    }

    return err;
}

/**
 * @brief Leave the chip's low power mode
 */
static int8_t mcp_low_power_mode_disable(mcp2518fd_t *dev)
{
    int8_t err;
    uint8_t d = 0;

    err = mcp_read_byte(dev, cREGADDR_OSC, &d);
    if (err) {
        return -1;
    }

    d &= ~0x08;

    err = mcp_write_byte(dev, cREGADDR_OSC, d);
    if (err) {
        return -2;
    }

    return err;
}

/**
 * @brief Wait for a free TX slot, then load and transmit the staged message object
 */
static int8_t mcp_transmit_message_queue(mcp2518fd_t *dev)
{
    uint8_t attempts = MAX_TXQUEUE_ATTEMPTS;
    CAN_TX_FIFO_EVENT txFlags;
    CAN_ERROR_STATE errorFlags;
    uint8_t tec, rec;
    uint8_t n;

    do {
        if (mcp_tx_channel_event_get(dev, APP_TX_FIFO, &txFlags) < 0) {
            return -1; // SPI communication error
        }
        if (attempts == 0) {
            mcp_error_count_state_get(dev, &tec, &rec, &errorFlags);
            ESP_LOGW(TAG, "TX FIFO stayed full: tec %u, rec %u, error state 0x%02X", tec, rec, errorFlags);
            return -2; // Timeout waiting for FIFO to become available
        }
        attempts--;
    } while (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT));

    n = dlc_to_data_bytes((CAN_DLC)dev->tx_obj.bF.ctrl.DLC);

    return mcp_tx_channel_load(dev, APP_TX_FIFO, &dev->tx_obj, dev->txd, n, true);
}

/**
 * @brief Stage a frame in the TX message object and hand it to the FIFO
 */
static uint8_t mcp_send_msg(mcp2518fd_t *dev, const uint8_t *buf, uint8_t len, uint32_t id, uint8_t ext, uint8_t rtr,
                            bool wait_sent)
{
    uint8_t n;
    int i;
    int8_t err;

    (void)wait_sent;

    dev->tx_obj.word[0] = 0;
    dev->tx_obj.word[1] = 0;

    dev->tx_obj.bF.ctrl.RTR = !!rtr;
    if (rtr && len > CAN_DLC_8) {
        len = CAN_DLC_8;
    }
    dev->tx_obj.bF.ctrl.DLC = len;

    dev->tx_obj.bF.ctrl.IDE = !!ext;
    if (ext) {
        dev->tx_obj.bF.id.SID = (id >> 18) & 0x7FF;
        dev->tx_obj.bF.id.EID = id & 0x3FFFF;
    } else {
        dev->tx_obj.bF.id.SID = id;
    }

    dev->tx_obj.bF.ctrl.BRS = true;
    dev->tx_obj.bF.ctrl.FDF = (len > 8);

    n = dlc_to_data_bytes((CAN_DLC)dev->tx_obj.bF.ctrl.DLC);
    for (i = 0; i < n; i++) {
        dev->txd[i] = buf[i];
    }

    err = mcp_transmit_message_queue(dev);
    if (err < 0) {
        switch (err) {
        case -1:
            return CAN_FAILINIT;       // SPI communication error
        case -2:
            return CAN_SENDMSGTIMEOUT; // Timeout waiting for FIFO
        case -4:
            return CAN_FAILINIT;       // SPI write error
        default:
            return CAN_FAILTX;         // DLC mismatch, channel update failure, or anything else
        }
    }

    return CAN_OK;
}

/**
 * @brief Read the oldest frame out of the RX FIFO and latch its header fields
 */
static uint8_t mcp_read_msg_buf_id(mcp2518fd_t *dev, uint8_t *len, uint8_t *buf)
{
    uint8_t n;
    int i;

    mcp_receive_message_get(dev, APP_RX_FIFO, &dev->rx_obj, dev->rxd, MCP2518FD_MAX_DATA_BYTES);

    dev->ext_flg = dev->rx_obj.bF.ctrl.IDE;
    dev->can_id = dev->ext_flg ? (dev->rx_obj.bF.id.EID | (dev->rx_obj.bF.id.SID << 18)) : dev->rx_obj.bF.id.SID;
    dev->rtr = dev->rx_obj.bF.ctrl.RTR;

    n = dlc_to_data_bytes((CAN_DLC)dev->rx_obj.bF.ctrl.DLC);
    if (len) {
        *len = n;
    }

    for (i = 0; i < n; i++) {
        buf[i] = dev->rxd[i];
    }

    return 0;
}

/**
 * @brief Translate a classic CAN bit rate preset into a CAN FD bit rate word
 */
static uint32_t bittime_compat_to_mcp2518fd(uint32_t speedset)
{
    uint32_t r;

    /* Anything this large is already a bit rate word, not a preset index */
    if (speedset > 0x100) {
        return speedset;
    }

    switch (speedset) {
    case CAN_5KBPS:
        r = MCP2518FD_BITRATE(0, 5000UL);
        break;
    case CAN_10KBPS:
        r = MCP2518FD_BITRATE(0, 10000UL);
        break;
    case CAN_20KBPS:
        r = MCP2518FD_BITRATE(0, 20000UL);
        break;
    case CAN_25KBPS:
        r = MCP2518FD_BITRATE(0, 25000UL);
        break;
    case CAN_31K25BPS:
        r = MCP2518FD_BITRATE(0, 31250UL);
        break;
    case CAN_33KBPS:
        r = MCP2518FD_BITRATE(0, 33000UL);
        break;
    case CAN_40KBPS:
        r = MCP2518FD_BITRATE(0, 40000UL);
        break;
    case CAN_50KBPS:
        r = MCP2518FD_BITRATE(0, 50000UL);
        break;
    case CAN_80KBPS:
        r = MCP2518FD_BITRATE(0, 80000UL);
        break;
    case CAN_83K3BPS:
        r = MCP2518FD_BITRATE(0, 83300UL);
        break;
    case CAN_95KBPS:
        r = MCP2518FD_BITRATE(0, 95000UL);
        break;
    case CAN_100KBPS:
        r = MCP2518FD_BITRATE(0, 100000UL);
        break;
    case CAN_125KBPS:
        r = MCP2518FD_BITRATE(0, 125000UL);
        break;
    case CAN_200KBPS:
        r = MCP2518FD_BITRATE(0, 200000UL);
        break;
    case CAN_250KBPS:
        r = MCP2518FD_BITRATE(0, 250000UL);
        break;
    case CAN_666KBPS:
        r = MCP2518FD_BITRATE(0, 666000UL);
        break;
    case CAN_800KBPS:
        r = MCP2518FD_BITRATE(0, 800000UL);
        break;
    case CAN_1000KBPS:
        r = MCP2518FD_BITRATE(0, 1000000UL);
        break;
    case CAN_500KBPS:
    default:
        r = MCP2518FD_BITRATE(0, 500000UL);
        break;
    }

    return r;
}

/**
 * @brief Bring the controller up: reset, RAM, FIFOs, filters, bit timing, mode
 */
static uint8_t mcp_hw_init(mcp2518fd_t *dev, uint32_t speedset, uint8_t clock)
{
    CAN_CONFIG config;
    CAN_TX_FIFO_CONFIG txConfig;
    CAN_RX_FIFO_CONFIG rxConfig;
    REG_CiFLTOBJ fObj;
    REG_CiMASK mObj;
    uint8_t ecccon_original = 0;
    uint8_t ecccon_test = 0;
    uint8_t ecccon_readback = 0;

    mcp_reset(dev);

    /* Enable ECC and initialize RAM, so reads of untouched RAM do not fault */
    mcp_ecc_enable(dev);
    mcp_ram_init(dev, 0xff);

    /* Configure device */
    mcp_configure_object_reset(&config);
    config.IsoCrcEnable = 1;
    config.StoreInTEF = 0;
    mcp_configure(dev, &config);

    /* Setup TX FIFO */
    mcp_tx_channel_configure_object_reset(&txConfig);
    txConfig.FifoSize = 7;
    txConfig.PayLoadSize = CAN_PLSIZE_64;
    txConfig.TxPriority = 1;
    mcp_tx_channel_configure(dev, APP_TX_FIFO, &txConfig);

    /* Setup RX FIFO */
    mcp_rx_channel_configure_object_reset(&rxConfig);
    rxConfig.FifoSize = 15;
    rxConfig.PayLoadSize = CAN_PLSIZE_64;
    mcp_rx_channel_configure(dev, APP_RX_FIFO, &rxConfig);

    /* Setup RX filter and mask: all zero, so nothing is filtered out */
    fObj.word = 0;
    mcp_filter_object_configure(dev, CAN_FILTER0, &fObj.bF);

    mObj.word = 0;
    mcp_filter_mask_configure(dev, CAN_FILTER0, &mObj.bF);

    mcp_filter_to_fifo_link(dev, CAN_FILTER0, APP_RX_FIFO, true);

    /* Setup bit time */
    mcp_bit_time_configure(dev, speedset, CAN_SSP_MODE_AUTO, (CAN_SYSCLK_SPEED)clock);

    /* Setup transmit and receive interrupts */
    mcp_gpio_mode_configure(dev, MCP2518FD_GPIO_MODE_INT, MCP2518FD_GPIO_MODE_INT);
    mcp_rx_channel_event_enable(dev, APP_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
    mcp_module_event_enable(dev, (CAN_MODULE_EVENT)(CAN_TX_EVENT | CAN_RX_EVENT));

    /* Enter the requested operation mode */
    mcp2518fd_switch_mode(dev, dev->mcp_mode);

    /* Verify the SPI link by toggling a bit in ECCCON and reading it back. A
     * plain read cannot tell a disconnected bus apart from a valid 0xFF. */
    mcp_delay_ms(5);

    mcp_read_byte(dev, cREGADDR_ECCCON, &ecccon_original);
    ecccon_test = ecccon_original ^ 0x01;

    mcp_write_byte(dev, cREGADDR_ECCCON, ecccon_test);
    mcp_delay_ms(2);

    mcp_read_byte(dev, cREGADDR_ECCCON, &ecccon_readback);
    mcp_write_byte(dev, cREGADDR_ECCCON, ecccon_original);

    if (ecccon_readback != ecccon_test) {
        return CAN_FAILINIT;
    }

    return 0;
}

// *****************************************************************************
// Section: Public API

esp_err_t mcp2518fd_init(mcp2518fd_t *dev, spi_host_device_t host, gpio_num_t cs_pin)
{
    return mcp2518fd_init_with_clock(dev, host, cs_pin, MCP2518FD_DEFAULT_SPI_CLOCK_HZ);
}

esp_err_t mcp2518fd_init_with_clock(mcp2518fd_t *dev, spi_host_device_t host, gpio_num_t cs_pin, uint32_t spi_clock_hz)
{
    esp_err_t err;

    memset(dev, 0, sizeof(*dev));

    dev->cs_pin = cs_pin;
    dev->mcp_mode = CAN_CLASSIC_MODE;

    /* The transfer buffers are handed straight to the SPI driver, which may use
     * DMA, so they have to live in DMA capable memory rather than in the handle. */
    dev->spi_tx = heap_caps_malloc(SPI_BUF_LEN, MALLOC_CAP_DMA);
    dev->spi_rx = heap_caps_malloc(SPI_BUF_LEN, MALLOC_CAP_DMA);
    if (dev->spi_tx == NULL || dev->spi_rx == NULL) {
        free(dev->spi_tx);
        free(dev->spi_rx);
        dev->spi_tx = NULL;
        dev->spi_rx = NULL;
        ESP_LOGE(TAG, "Could not allocate the SPI transfer buffers");
        return ESP_ERR_NO_MEM;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = spi_clock_hz,
        .mode = 0, // The MCP2518FD supports SPI modes 0 and 3; the driver uses 0
        .spics_io_num = cs_pin,
        .queue_size = 1,
    };

    err = spi_bus_add_device(host, &dev_cfg, &dev->spi);
    if (err != ESP_OK) {
        free(dev->spi_tx);
        free(dev->spi_rx);
        dev->spi_tx = NULL;
        dev->spi_rx = NULL;
        ESP_LOGE(TAG, "Could not add the MCP2518FD to the SPI bus: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MCP2518FD attached to the SPI bus at %u Hz, CS on GPIO %d", (unsigned int)spi_clock_hz,
             (int)cs_pin);

    return ESP_OK;
}

esp_err_t mcp2518fd_deinit(mcp2518fd_t *dev)
{
    esp_err_t err = ESP_OK;

    if (dev->spi) {
        err = spi_bus_remove_device(dev->spi);
        dev->spi = NULL;
    }

    free(dev->spi_tx);
    free(dev->spi_rx);
    dev->spi_tx = NULL;
    dev->spi_rx = NULL;

    return err;
}

uint8_t mcp2518fd_begin(mcp2518fd_t *dev, uint32_t speedset, uint8_t clockset)
{
    /* Translate a classic CAN preset into the bit rate word the chip needs */
    speedset = bittime_compat_to_mcp2518fd(speedset);

    if (mcp_hw_init(dev, speedset, clockset) != 0) {
        return CAN_FAILINIT;
    }

    return CAN_OK;
}

uint8_t mcp2518fd_set_mode(mcp2518fd_t *dev, uint8_t op_mode)
{
    /* Sleep is entered on request but never remembered, so that wake() can put
     * the controller back into the mode it was actually running in. */
    if ((CAN_OPERATION_MODE)op_mode != CAN_SLEEP_MODE) {
        dev->mcp_mode = op_mode;
    }

    return CAN_OK;
}

uint8_t mcp2518fd_switch_mode(mcp2518fd_t *dev, uint8_t op_mode)
{
    mcp2518fd_set_mode(dev, op_mode);

    return mcp_operation_mode_select(dev, (CAN_OPERATION_MODE)dev->mcp_mode);
}

uint8_t mcp2518fd_get_mode(mcp2518fd_t *dev)
{
    return (uint8_t)mcp_operation_mode_get(dev);
}

uint8_t mcp2518fd_init_mask(mcp2518fd_t *dev, uint8_t num, uint8_t ext, uint32_t ul_data)
{
    int8_t err;
    REG_CiMASK mObj;

    /* Filters and masks may only be written in configuration mode */
    mcp_operation_mode_select(dev, CAN_CONFIGURATION_MODE);

    mObj.word = 0;
    mObj.bF.MSID = ul_data;
    mObj.bF.MIDE = ext;
    err = mcp_filter_mask_configure(dev, (CAN_FILTER)num, &mObj.bF);

    mcp_operation_mode_select(dev, (CAN_OPERATION_MODE)dev->mcp_mode);

    return err;
}

uint8_t mcp2518fd_init_filt(mcp2518fd_t *dev, uint8_t num, uint8_t ext, uint32_t ul_data)
{
    int8_t err;
    REG_CiFLTOBJ fObj;

    err = mcp_operation_mode_select(dev, CAN_CONFIGURATION_MODE);

    fObj.word = 0;
    if (!ext) {
        fObj.bF.SID = ul_data;
    } else {
        fObj.bF.EID = ul_data;
    }
    fObj.bF.EXIDE = !!ext;
    mcp_filter_object_configure(dev, (CAN_FILTER)num, &fObj.bF);

    mcp_operation_mode_select(dev, (CAN_OPERATION_MODE)dev->mcp_mode);

    return err;
}

uint8_t mcp2518fd_filter_disable(mcp2518fd_t *dev, CAN_FILTER filter)
{
    uint16_t a;
    REG_CiFLTCON_BYTE fCtrl;

    a = cREGADDR_CiFLTCON + filter;

    if (mcp_read_byte(dev, a, &fCtrl.byte)) {
        return -1;
    }

    fCtrl.bF.Enable = 0;

    if (mcp_write_byte(dev, a, fCtrl.byte)) {
        return -2;
    }

    return 0;
}

uint8_t mcp2518fd_init_filt_mask(mcp2518fd_t *dev, uint8_t num, uint8_t ext, uint32_t f, uint32_t m)
{
    CAN_FILTEROBJ_ID fObj;
    CAN_MASKOBJ_ID mObj;

    /* A filter cannot be rewritten while it is enabled */
    mcp2518fd_filter_disable(dev, (CAN_FILTER)num);

    fObj.SID = ext ? ((f >> 18) & 0x7FF) : (f & 0x7FF);
    fObj.EID = ext ? (f & 0x3FFFF) : 0;
    fObj.SID11 = 0;
    fObj.EXIDE = !!ext;
    mcp_filter_object_configure(dev, (CAN_FILTER)num, &fObj);

    mObj.MSID = ext ? ((m >> 18) & 0x7FF) : (m & 0x7FF);
    mObj.MEID = ext ? (m & 0x3FFFF) : 0;
    mObj.MSID11 = 0;
    /* Match the IDE bit too, so a standard filter never accepts extended frames */
    mObj.MIDE = 1;
    mcp_filter_mask_configure(dev, (CAN_FILTER)num, &mObj);

    mcp_filter_to_fifo_link(dev, (CAN_FILTER)num, APP_RX_FIFO, true);

    return 0;
}

void mcp2518fd_enable_tx_interrupt(mcp2518fd_t *dev, bool enable)
{
    if (enable) {
        mcp_module_event_enable(dev, CAN_TX_EVENT);
        mcp_tx_channel_event_enable(dev, APP_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT);
    }
}

void mcp2518fd_reserve_tx_buffers(mcp2518fd_t *dev, uint8_t n_tx_buf)
{
    dev->n_reserved_tx = (n_tx_buf < 3 ? n_tx_buf : 3 - 1);
}

uint8_t mcp2518fd_get_last_tx_buffer(mcp2518fd_t *dev)
{
    (void)dev;

    return 3 - 1;
}

void mcp2518fd_set_sleep_wakeup(mcp2518fd_t *dev, uint8_t enable)
{
    if (enable) {
        mcp_low_power_mode_enable(dev);
    } else {
        mcp_low_power_mode_disable(dev);
    }
}

uint8_t mcp2518fd_sleep(mcp2518fd_t *dev)
{
    if (mcp2518fd_get_mode(dev) != CAN_SLEEP_MODE) {
        return mcp_operation_mode_select(dev, CAN_SLEEP_MODE);
    }

    return CAN_OK;
}

uint8_t mcp2518fd_wake(mcp2518fd_t *dev)
{
    uint8_t curr_mode = mcp2518fd_get_mode(dev);

    if (curr_mode != dev->mcp_mode) {
        return mcp_operation_mode_select(dev, (CAN_OPERATION_MODE)dev->mcp_mode);
    }

    return CAN_OK;
}

uint8_t mcp2518fd_check_error(mcp2518fd_t *dev, uint8_t *err_ptr)
{
    CAN_ERROR_STATE flags;

    mcp_error_state_get(dev, &flags);

    if (err_ptr) {
        *err_ptr = (uint8_t)flags;
    }

    return (uint8_t)flags;
}

uint8_t mcp2518fd_check_receive(mcp2518fd_t *dev)
{
    CAN_RX_FIFO_STATUS status;

    mcp_rx_channel_status_get(dev, APP_RX_FIFO, &status);

    return (status & CAN_RX_FIFO_NOT_EMPTY) ? CAN_MSGAVAIL : CAN_NOMSG;
}

uint8_t mcp2518fd_read_msg_buf(mcp2518fd_t *dev, uint8_t *len, uint8_t *buf)
{
    return mcp_read_msg_buf_id(dev, len, buf);
}

uint8_t mcp2518fd_read_msg_buf_id(mcp2518fd_t *dev, uint32_t *id, uint8_t *len, uint8_t *buf)
{
    uint8_t r = mcp_read_msg_buf_id(dev, len, buf);

    if (id) {
        *id = dev->can_id;
    }

    return r;
}

uint8_t mcp2518fd_read_msg_buf_id_status(mcp2518fd_t *dev, uint8_t status, uint32_t *id, uint8_t *ext, uint8_t *rtr,
                                         uint8_t *len, uint8_t *buf)
{
    uint8_t r;

    (void)status;

    r = mcp_read_msg_buf_id(dev, len, buf);

    if (id) {
        *id = dev->can_id;
    }
    if (ext) {
        *ext = dev->ext_flg;
    }
    if (rtr) {
        *rtr = dev->rtr;
    }

    return r;
}

uint32_t mcp2518fd_get_can_id(mcp2518fd_t *dev)
{
    return dev->can_id;
}

bool mcp2518fd_is_extended_frame(mcp2518fd_t *dev)
{
    return dev->ext_flg != 0;
}

bool mcp2518fd_is_remote_request(mcp2518fd_t *dev)
{
    return dev->rtr != 0;
}

uint8_t mcp2518fd_send_msg_buf(mcp2518fd_t *dev, uint32_t id, uint8_t ext, uint8_t dlc, const uint8_t *buf)
{
    return mcp_send_msg(dev, buf, dlc, id, ext, 0, true);
}

uint8_t mcp2518fd_send_msg_buf_ex(mcp2518fd_t *dev, uint32_t id, uint8_t ext, uint8_t rtr, uint8_t dlc,
                                  const uint8_t *buf, bool wait_sent)
{
    return mcp_send_msg(dev, buf, dlc, id, ext, rtr, wait_sent);
}

uint8_t mcp2518fd_try_send_msg_buf(mcp2518fd_t *dev, uint32_t id, uint8_t ext, uint8_t rtr, uint8_t dlc,
                                   const uint8_t *buf, uint8_t i_tx_buf)
{
    (void)i_tx_buf;

    return mcp_send_msg(dev, buf, dlc, id, ext, rtr, false);
}

void mcp2518fd_clear_buffer_transmit_if_flags(mcp2518fd_t *dev, uint8_t flags)
{
    (void)flags;

    mcp_tx_channel_event_attempt_clear(dev, APP_TX_FIFO);
}

uint8_t mcp2518fd_read_rx_tx_status(mcp2518fd_t *dev)
{
    CAN_RX_FIFO_EVENT rxFlags = CAN_RX_FIFO_NO_EVENT;

    mcp_rx_channel_event_get(dev, APP_RX_FIFO, &rxFlags);

    return (uint8_t)rxFlags;
}

uint8_t mcp2518fd_check_clear_rx_status(mcp2518fd_t *dev, uint8_t *status)
{
    (void)dev;
    (void)status;

    return 1;
}

uint8_t mcp2518fd_check_clear_tx_status(mcp2518fd_t *dev, uint8_t *status, uint8_t i_tx_buf)
{
    (void)dev;
    (void)status;
    (void)i_tx_buf;

    return 1;
}

bool mcp2518fd_pin_mode(mcp2518fd_t *dev, uint8_t pin, uint8_t mode)
{
    uint16_t a = cREGADDR_IOCON + 3;
    REG_IOCON iocon;

    iocon.word = 0;

    if (mcp_read_byte(dev, a, &iocon.byte[3])) {
        return false;
    }

    if (pin == MCP2518FD_GPIO_PIN_0) {
        iocon.bF.PinMode0 = (GPIO_PIN_MODE)mode;
    } else if (pin == MCP2518FD_GPIO_PIN_1) {
        iocon.bF.PinMode1 = (GPIO_PIN_MODE)mode;
    } else {
        return false;
    }

    if (mcp_write_byte(dev, a, iocon.byte[3])) {
        return false;
    }

    return true;
}

bool mcp2518fd_digital_write(mcp2518fd_t *dev, uint8_t pin, uint8_t mode)
{
    uint16_t a = cREGADDR_IOCON + 1;
    REG_IOCON iocon;

    iocon.word = 0;

    if (mcp_read_byte(dev, a, &iocon.byte[1])) {
        return false;
    }

    switch (pin) {
    case MCP2518FD_GPIO_PIN_0:
        iocon.bF.LAT0 = (GPIO_PIN_STATE)mode;
        break;
    case MCP2518FD_GPIO_PIN_1:
        iocon.bF.LAT1 = (GPIO_PIN_STATE)mode;
        break;
    default:
        return false;
    }

    if (mcp_write_byte(dev, a, iocon.byte[1])) {
        return false;
    }

    return true;
}

uint8_t mcp2518fd_digital_read(mcp2518fd_t *dev, uint8_t pin)
{
    REG_IOCON iocon;

    iocon.word = 0;

    mcp_read_byte(dev, cREGADDR_IOCON + 2, &iocon.byte[2]);

    switch (pin) {
    case MCP2518FD_GPIO_PIN_0:
        return (uint8_t)iocon.bF.GPIO0;
    case MCP2518FD_GPIO_PIN_1:
        return (uint8_t)iocon.bF.GPIO1;
    default:
        return MCP2518FD_GPIO_LOW;
    }
}

uint8_t mcp2518fd_dlc2len(uint8_t dlc)
{
    if (dlc <= CAN_DLC_8) {
        return dlc;
    }

    switch (dlc) {
    case CAN_DLC_12:
        return 12;
    case CAN_DLC_16:
        return 16;
    case CAN_DLC_20:
        return 20;
    case CAN_DLC_24:
        return 24;
    case CAN_DLC_32:
        return 32;
    case CAN_DLC_48:
        return 48;
    case CAN_DLC_64:
    default:
        return 64;
    }
}

uint8_t mcp2518fd_len2dlc(uint8_t len)
{
    if (len <= CAN_DLC_8) {
        return len;
    } else if (len <= 12) {
        return CAN_DLC_12;
    } else if (len <= 16) {
        return CAN_DLC_16;
    } else if (len <= 20) {
        return CAN_DLC_20;
    } else if (len <= 24) {
        return CAN_DLC_24;
    } else if (len <= 32) {
        return CAN_DLC_32;
    } else if (len <= 48) {
        return CAN_DLC_48;
    }

    return CAN_DLC_64;
}
