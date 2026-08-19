# Soldered CAN Bus Breakout MCP2518FD Component

| ![CAN Bus Breakout MCP2518FD](https://soldered.com/cdn/shop/files/333020_featured-photo_d9566c.jpg) |
| :-----------------------------------------------------------------------------------------------------------: |
|                        [CAN Bus Breakout MCP2518FD](https://solde.red/333020)                        |

ESP-IDF driver for the Soldered CAN Bus Breakout board, built around the Microchip MCP2518FD CAN FD controller and driven over SPI. Handles both classic CAN 2.0 and CAN FD, at up to 1 Mbps arbitration and 8 Mbps data rate.

### Repository Contents

- **/src** - source files (.c)
- **/include** - header files (.h)
  - `soldered_mcp2518fd.h` - the public API
  - `mcp2518fd_dfs.h` - register map, bit definitions and object types
- **/examples** - examples for using the library
  - `can20_send` - send frames using CAN 2.0
  - `can20_recv_check` - receive CAN 2.0 frames by polling
  - `can20_recv_int` - receive CAN 2.0 frames using the INT pin
  - `can20_recv_mask_filt` - receive CAN 2.0 frames through a mask and filter
  - `canfd_send` - send frames using CAN FD
  - `canfd_recv_check` - receive CAN FD frames by polling
  - `canfd_recv_int` - receive CAN FD frames using the INT pin
  - `canfd_recv_mask_filt` - receive CAN FD frames through a mask and filter
  - `obdii_pids` - query a vehicle's OBD-II PIDs from the serial console
- **_other_** - idf_component.yml manifest file for ESP Component Registry


### Usage

The SPI bus belongs to your application, not to the driver, so that other devices can share it:

```c
spi_bus_config_t bus_cfg = {
    .mosi_io_num = GPIO_NUM_23,
    .miso_io_num = GPIO_NUM_19,
    .sclk_io_num = GPIO_NUM_18,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};
ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

mcp2518fd_t can;
ESP_ERROR_CHECK(mcp2518fd_init(&can, SPI2_HOST, GPIO_NUM_5));

while (CAN_OK != mcp2518fd_begin(&can, CAN_125KBPS, MCP2518FD_20MHz)) {
    vTaskDelay(pdMS_TO_TICKS(100));
}
```

`mcp2518fd_begin()` sits in a retry loop because the controller needs a moment after power-up before it answers over SPI. `MCP2518FD_20MHz` is the oscillator the Soldered breakout is populated with.

**Sending and receiving:**

```c
uint8_t data[8] = {0, 1, 2, 3, 4, 5, 6, 7};
mcp2518fd_send_msg_buf(&can, 0x01, 0, 8, data);   // id, extended?, length, payload

if (CAN_MSGAVAIL == mcp2518fd_check_receive(&can)) {
    uint8_t len, buf[8];
    mcp2518fd_read_msg_buf(&can, &len, buf);      // read first, then ask for the ID
    uint32_t id = mcp2518fd_get_can_id(&can);
}
```

**CAN FD:** call `mcp2518fd_set_mode(&can, CAN_NORMAL_MODE)` before `mcp2518fd_begin()`, since the driver defaults to classic mode which cannot carry FD frames. Pass a dual bit rate such as `CAN_125K_500K` to `mcp2518fd_begin()`, and convert payload lengths over 8 bytes with `mcp2518fd_len2dlc()`.

**Masks and filters:** `mcp2518fd_init_filt_mask()` sets a filter and its mask together and links it to the RX FIFO. `mcp2518fd_init_mask()` and `mcp2518fd_init_filt()` set them separately. There are 32 of each, numbered 0 to 31.

### Original source

This is a port of the [Soldered CAN Bus Breakout MCP2518 Arduino library](https://github.com/SolderedElectronics/Soldered-CAN-Bus-Breakout-MCP2518-Arduino-Library), whose register level code comes from the Microchip MCP2518FD SDK and whose bit timing calculation comes from [acan2517FD](https://github.com/pierremolinaro/acan2517FD).

### Hardware design

Hardware design, BOM, gerbers, and 3D files for this board are in its [hardware repository](https://github.com/SolderedElectronics/CAN-Transceiver-MCP2518-board-hardware-design).

### Documentation

Access library documentation [here](https://docs.soldered.com/).

### About Soldered

<img src="https://raw.githubusercontent.com/SolderedElectronics/Soldered-Generic-Arduino-Library/dev/extras/Soldered-logo-color.png" alt="soldered-logo" width="500"/>

At Soldered, we design and manufacture a wide selection of electronic products to help you turn your ideas into acts and bring you one step closer to your final project. Our products are intented for makers and crafted in-house by our experienced team in Osijek, Croatia. We believe that sharing is a crucial element for improvement and innovation, and we work hard to stay connected with all our makers regardless of their skill or experience level. Therefore, all our products are open-source. Finally, we always have your back. If you face any problem concerning either your shopping experience or your electronics project, our team will help you deal with it, offering efficient customer service and cost-free technical support anytime. Some of those might be useful for you:

- [Web Store](https://www.soldered.com/shop)
- [Tutorials & Projects](https://soldered.com/learn)
- [Documentation](https://docs.soldered.com)

### Open-source license

Soldered invests vast amounts of time into hardware & software for these products, which are all open-source. Please support future development by buying one of our products.

Check license details in the LICENSE file. Long story short, use these open-source files for any purpose you want to, as long as you apply the same open-source licence to it and disclose the original source. No warranty - all designs in this repository are distributed in the hope that they will be useful, but without any warranty. They are provided "AS IS", therefore without warranty of any kind, either expressed or implied. The entire quality and performance of what you do with the contents of this repository are your responsibility. In no event, Soldered (TAVU) will be liable for your damages, losses, including any general, special, incidental or consequential damage arising out of the use or inability to use the contents of this repository.

## Have fun!

And thank you from your fellow makers at Soldered Electronics.
