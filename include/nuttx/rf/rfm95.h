/****************************************************************************
 * include/nuttx/rf/rfm95.h
 * Character driver for the SPI RFM95 Radio module
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_RF_RFM95_H
#define __INCLUDE_NUTTX_RF_RFM95_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/rf/ioctl.h>
#include <nuttx/spi/spi.h>

/*
 * Register definitions
 */
#define REG_FIFO                       (0x00)
#define REG_OP_MODE                    (0x01)
#define REG_FRF_MSB                    (0x06)
#define REG_FRF_MID                    (0x07)
#define REG_FRF_LSB                    (0x08)
#define REG_PA_CONFIG                  (0x09)
#define REG_LNA                        (0x0c)
#define REG_FIFO_ADDR_PTR              (0x0d)
#define REG_FIFO_TX_BASE_ADDR          (0x0e)
#define REG_FIFO_RX_BASE_ADDR          (0x0f)
#define REG_FIFO_RX_CURRENT_ADDR       (0x10)
#define REG_IRQ_FLAGS                  (0x12)
#define REG_RX_NB_BYTES                (0x13)
#define REG_PKT_SNR_VALUE              (0x19)
#define REG_PKT_RSSI_VALUE             (0x1a)
#define REG_MODEM_CONFIG_1             (0x1d)
#define REG_MODEM_CONFIG_2             (0x1e)
#define REG_PREAMBLE_MSB               (0x20)
#define REG_PREAMBLE_LSB               (0x21)
#define REG_PAYLOAD_LENGTH             (0x22)
#define REG_MODEM_CONFIG_3             (0x26)
#define REG_RSSI_WIDEBAND              (0x2c)
#define REG_DETECTION_OPTIMIZE         (0x31)
#define REG_DETECTION_THRESHOLD        (0x37)
#define REG_SYNC_WORD                  (0x39)
#define REG_DIO_MAPPING_1              (0x40)
#define REG_VERSION                    (0x42)

/*
 * Transceiver modes
 */
#define MODE_LONG_RANGE_MODE           (0x80)
#define MODE_SLEEP                     (0x00)
#define MODE_STDBY                     (0x01)
#define MODE_TX                        (0x03)
#define MODE_RX_CONTINUOUS             (0x05)
#define MODE_RX_SINGLE                 (0x06)

/*
 * PA configuration
 */
#define PA_BOOST                       (0x80)

/*
 * IRQ masks
 */
#define IRQ_TX_DONE_MASK               (0x08)
#define IRQ_PAYLOAD_CRC_ERROR_MASK     (0x20)
#define IRQ_RX_DONE_MASK               (0x40)

#define PA_OUTPUT_RFO_PIN              (0)
#define PA_OUTPUT_PA_BOOST_PIN         (1)

#define TIMEOUT_RESET                  (100)

/* IOCTL DEFINITION */
#define RFM95_IOCTL_INIT 0

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int rfm95_read_reg(FAR struct spi_dev_s *spi, int reg);
void rfm95_write_reg(FAR struct spi_dev_s *spi, int reg, int val);
static void rfm95_reset();
static void rfm95_init(FAR struct file *filep);
void rfm95_send_packet(FAR struct spi_dev_s *spi, const uint8_t *buf, int size);
static inline void rfm95_configspi(FAR struct spi_dev_s *spi);

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: rfm95_register
 *
 * Description:
 *   Register the rfm95 character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/radio0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *   spidev  - Number of the spi device (used to drive the Latch Enable pin).
 *
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rfm95_register(FAR const char *devpath,
                       FAR struct spi_dev_s *spi,
                       int spidev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_RF_RFM95_H */
