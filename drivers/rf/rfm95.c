/****************************************************************************
 * drivers/rf/rfm95.c
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

/* Custom SPI RFM95 Driver */

#include <nuttx/config.h>

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/rf/ioctl.h>
#include <nuttx/rf/attenuator.h>
#include <nuttx/rf/rfm95.h>
#include <arch/board/board.h>
#include "esp32_gpio.h"


#if defined(CONFIG_SPI) && defined(CONFIG_RF_RFM95)

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/rf/ioctl.h>
#include <nuttx/spi/spi.h>
#include "rfm95_register.h"


#ifndef CONFIG_RF_RFM95_SPI_CS_PIN
/* UART2_RDX for Sony Spresense */
#  define CONFIG_RF_RFM95_SPI_CS_PIN 68
#endif /* CONFIG_RF_RFM95_SPI_CS_PIN */

#ifndef CONFIG_RF_RFM95_SPI_FREQUENCY
// We can push it to 9MHz, and maybe faster
#  define CONFIG_RF_RFM95_SPI_FREQUENCY 9000000
#endif /* CONFIG_RFM95_SPI_FREQUENCY */

#ifndef CONFIG_RF_RFM95_RESET_PIN
/* UART2_TDX for Sony Spresense */
#define CONFIG_RF_RFM95_RESET_PIN 67
#endif

#ifndef CONFIG_RF_RFM95_TX_FREQ
/* Transmission frequency for LoRa 
*  Default: 868Mhz */
#define CONFIG_RF_RFM95_TX_FREQ 868000000
#endif

#ifndef CONFIG_RF_RFM95_TX_POWER
#define CONFIG_RF_RFM95_TX_POWER 17
#endif

#ifndef CONFIG_RF_RFM95_SYNC_WORD
/* Transmission frequency for LoRa 
*  0x00 for none */
#define CONFIG_RF_RFM95_SYNC_WORD 0
#endif

#  define RFM95_SPI_MODE (SPIDEV_MODE0) /* SPI Mode 0: CPOL=0,CPHA=0 */


struct rfm95_dev_s {
  FAR struct spi_dev_s *spi;    /* Saved SPI driver instance */
  int spidev;
};
static struct rfm95_dev_s rfm_dev;

static char recv_buffer[256];  /* Buffer for SPI response */
static int recv_buffer_len = 0;  /* Length of SPI response */

void rfm95_configspi(void) {
  _info("\n");
  SPI_LOCK(rfm_dev.spi, true);

  /* Set SPI Mode (Polarity and Phase) and Transfer Size (8 bits) */

  SPI_SETMODE(rfm_dev.spi, RFM95_SPI_MODE);
  SPI_SETBITS(rfm_dev.spi, 8);

  /* Set SPI Hardware Features and Frequency */

  SPI_HWFEATURES(rfm_dev.spi, 0);
  SPI_SETFREQUENCY(rfm_dev.spi, CONFIG_RF_RFM95_SPI_FREQUENCY);

  SPI_LOCK(rfm_dev.spi, false);
}

int rfm95_read_reg(int reg) {
  uint8_t out[2] = { reg, 0xff };
  uint8_t in[2];

  SPI_LOCK(rfm_dev.spi, true);

  /* Enable CS pin */
  esp32_gpiowrite(CONFIG_RF_RFM95_SPI_CS_PIN, 0);
  SPI_SELECT(rfm_dev.spi, rfm_dev.spidev, true);


  /* Transmit buffer to SPI device and receive the response */
  SPI_EXCHANGE(rfm_dev.spi, out, in, 2);
  recv_buffer_len = 2;


  /* Deassert CS */
  esp32_gpiowrite(CONFIG_RF_RFM95_SPI_CS_PIN, 1);
  SPI_SELECT(rfm_dev.spi, rfm_dev.spidev, false);

  SPI_LOCK(rfm_dev.spi, false);

  return in[1];
}

void rfm95_write_reg(int reg, int val) {
  uint8_t out[2] = { 0x80 | reg, val };
  uint8_t in[2];

  SPI_LOCK(rfm_dev.spi, true);

  esp32_gpiowrite(CONFIG_RF_RFM95_SPI_CS_PIN, 0);
  SPI_SELECT(rfm_dev.spi, rfm_dev.spidev, true);

  /* Transmit buffer to SPI device */
  SPI_EXCHANGE(rfm_dev.spi, out, in, 2);


  esp32_gpiowrite(CONFIG_RF_RFM95_SPI_CS_PIN, 1);
  SPI_SELECT(rfm_dev.spi, rfm_dev.spidev, false);


  SPI_LOCK(rfm_dev.spi, false);
}


static int __implicit;
static long __frequency;

/**
 * Perform physical reset on the Lora chip
 */
void rfm95_reset(void) {
  esp32_gpiowrite(CONFIG_RF_RFM95_RESET_PIN, 0);
  up_mdelay(1);
  esp32_gpiowrite(CONFIG_RF_RFM95_RESET_PIN, 1);
  up_mdelay(10);
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void rfm95_explicit_header_mode(void) {
  __implicit = 0;
  rfm95_write_reg(REG_MODEM_CONFIG_1, rfm95_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void rfm95_implicit_header_mode(int size) {
  __implicit = 1;
  rfm95_write_reg(REG_MODEM_CONFIG_1, rfm95_read_reg(REG_MODEM_CONFIG_1) | 0x01);
  rfm95_write_reg(REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void rfm95_idle(void) {
  rfm95_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void rfm95_sleep(void) { 
  rfm95_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void rfm95_receive(void) {
  rfm95_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void rfm95_set_tx_power(int level) {
  // RF9x module uses PA_BOOST pin
  if (level < 2) level = 2;
  else if (level > 17) level = 17;
  rfm95_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void rfm95_set_frequency(long frequency) {
  __frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  rfm95_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
  rfm95_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
  rfm95_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void rfm95_set_spreading_factor(int sf) {
  if (sf < 6) sf = 6;
  else if (sf > 12) sf = 12;

  if (sf == 6) {
    rfm95_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
    rfm95_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    rfm95_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
    rfm95_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
  }

  rfm95_write_reg(REG_MODEM_CONFIG_2, (rfm95_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Bandwidth in Hz (up to 500000)
 */
void rfm95_set_bandwidth(long sbw) {
  int bw;

  if (sbw <= 7.8E3) bw = 0;
  else if (sbw <= 10.4E3) bw = 1;
  else if (sbw <= 15.6E3) bw = 2;
  else if (sbw <= 20.8E3) bw = 3;
  else if (sbw <= 31.25E3) bw = 4;
  else if (sbw <= 41.7E3) bw = 5;
  else if (sbw <= 62.5E3) bw = 6;
  else if (sbw <= 125E3) bw = 7;
  else if (sbw <= 250E3) bw = 8;
  else bw = 9;
  rfm95_write_reg(REG_MODEM_CONFIG_1, (rfm95_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

/**
 * Set coding rate 
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */ 
void rfm95_set_coding_rate(int denominator) {
  if (denominator < 5) denominator = 5;
  else if (denominator > 8) denominator = 8;

  int cr = denominator - 4;
  rfm95_write_reg(REG_MODEM_CONFIG_1, (rfm95_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void rfm95_set_preamble_length(long length) {
  rfm95_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  rfm95_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void rfm95_set_sync_word(int sw) {
  rfm95_write_reg(REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
void rfm95_enable_crc(void) {
  rfm95_write_reg(REG_MODEM_CONFIG_2, rfm95_read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
void rfm95_disable_crc(void) {
  rfm95_write_reg(REG_MODEM_CONFIG_2, rfm95_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

/**
 * Perform hardware initialization.
 */
int rfm95_init(void) {
  /*
  * Configure MCU hardware to communicate with the radio chip
  */
  esp32_configgpio(CONFIG_RF_RFM95_RESET_PIN, OUTPUT);
  esp32_configgpio(CONFIG_RF_RFM95_SPI_CS_PIN, OUTPUT);


  rfm95_configspi();

  /*
  * Perform hardware reset.
  */
  rfm95_reset();

  /*
  * Check version.
  */
  uint8_t version;
  uint8_t i = 0;
  while(i++ < TIMEOUT_RESET) {
    version = rfm95_read_reg(REG_VERSION);
    if(version == 0x12) break;
    up_mdelay(2);
  }
  assert(i <= TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1

  /*
  * Default configuration.
  */
  rfm95_sleep();
  rfm95_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
  rfm95_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
  rfm95_write_reg(REG_LNA, rfm95_read_reg(REG_LNA) | 0x03);
  rfm95_write_reg(REG_MODEM_CONFIG_3, 0x04);

  rfm95_idle();

  rfm95_set_frequency(CONFIG_RF_RFM95_TX_FREQ);
  rfm95_set_tx_power(CONFIG_RF_RFM95_TX_POWER);
  rfm95_set_spreading_factor(CONFIG_RF_RFM95_SPREADING_FACTOR);
  rfm95_set_bandwidth(CONFIG_RF_RFM95_BANDWIDTH);
  #ifdef CONFIG_RF_RFM95_ENABLE_CRC
    rfm95_enable_crc();
  #endif
  //rfm95_implicit_header_mode(11);

  return 1;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void rfm95_send_packet(const uint8_t *buf, int size) {
  /*
  * Transfer data to radio.
  */
  rfm95_idle();
  rfm95_write_reg(REG_FIFO_ADDR_PTR, 0);

  for(int i=0; i<size; i++) 
    rfm95_write_reg(REG_FIFO, *buf++);
  
  rfm95_write_reg(REG_PAYLOAD_LENGTH, size);
  
  /*
  * Start transmission and wait for conclusion.
  */
  rfm95_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  while((rfm95_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
    up_mdelay(2);

  rfm95_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int rfm95_receive_packet(uint8_t *buf, int size) {
  int len = 0;

  rfm95_sleep();
  rfm95_receive();

  /*
  * Check interrupts.
  */
  int irq;
  do {
    irq = rfm95_read_reg(REG_IRQ_FLAGS);
    rfm95_write_reg(REG_IRQ_FLAGS, irq);

    //_info("irq: %x\n", irq);
    up_mdelay(100);
  } while((irq & IRQ_RX_DONE_MASK) == 0);

  if(irq & IRQ_PAYLOAD_CRC_ERROR_MASK) {
    _info("CRC error");
    return 0;
  }

  /*
  * Find packet size.
  */
  if (__implicit) len = rfm95_read_reg(REG_PAYLOAD_LENGTH);
  else len = rfm95_read_reg(REG_RX_NB_BYTES);

  /*
  * Transfer data from radio.
  */
  rfm95_idle();   
  rfm95_write_reg(REG_FIFO_ADDR_PTR, rfm95_read_reg(REG_FIFO_RX_CURRENT_ADDR));
  if(len > size) len = size;
  for(int i=0; i<len; i++) 
    *buf++ = rfm95_read_reg(REG_FIFO);

  return len;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int rfm95_received(void) {
  if(rfm95_read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) return 1;
  return 0;
}

/**
 * Return last packet's RSSI.
 */
int rfm95_packet_rssi(void) {
  return (rfm95_read_reg(REG_PKT_RSSI_VALUE) - (__frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float rfm95_packet_snr(void) {
  return ((int8_t)rfm95_read_reg(REG_PKT_SNR_VALUE)) * 0.25;
}






/* Character device related operations */

/**
 * Responds to an open operation on the associated character device and configures the SPI bus.
*/
static int rfm95_open(FAR struct file *filep) {
  _info("\n");
  DEBUGASSERT(filep != NULL);

  /* Debug info */
  _info("SPI reset pin: %d\n", CONFIG_RF_RFM95_RESET_PIN);
  _info("SPI CS pin: %d\n", CONFIG_RF_RFM95_SPI_CS_PIN);
  _info("SPI freq: %d\n", CONFIG_RF_RFM95_SPI_FREQUENCY);
  _info("TX frequency: %d\n", CONFIG_RF_RFM95_TX_FREQ);
  _info("TX power: %d\n", CONFIG_RF_RFM95_TX_POWER);
  _info("Sync word: %d\n", CONFIG_RF_RFM95_SYNC_WORD);

  /* Get the SPI interface */

  FAR struct inode *inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  FAR struct rfm95_dev_s *priv = inode->i_private;
  DEBUGASSERT(priv != NULL);

  return OK;
}

static int rfm95_close(FAR struct file *filep) {
  _info("\n");
  DEBUGASSERT(filep != NULL);
  return OK;
}

/**
 * Sends a packet down the LoRa link. The packet is an array of bytes.
*/
static ssize_t rfm95_write(FAR struct file *filep, FAR const char *buffer, size_t buflen) {
  _info("buflen=%u\n", buflen);
  DEBUGASSERT(buflen <= sizeof(recv_buffer));
  DEBUGASSERT(buffer != NULL);
  DEBUGASSERT(filep  != NULL);

  rfm95_send_packet(buffer, buflen);

  return buflen;
}

/**
 * Waits for a packet and writes it in buffer, this call is blocking.
*/
static ssize_t rfm95_read(FAR struct file *filep, FAR char *buffer, size_t buflen) {
  _info("buflen=%u\n", buflen);
  DEBUGASSERT(buflen <= sizeof(recv_buffer));
  DEBUGASSERT(buffer != NULL);
  DEBUGASSERT(filep  != NULL);

  int len = rfm95_receive_packet(buffer, buflen);
  return len;
}

static int rfm95_ioctl(FAR struct file *filep, int cmd, unsigned long arg) {
  _info("cmd=0x%x, arg=0x%lx\n", cmd, arg);
  DEBUGASSERT(filep != NULL);

  int ret = OK;

  switch (cmd)
    {
      case RFM95_IOCTL_INIT:
        rfm95_init();
        break;

      case RFM95_IOCTL_RECEIVER:
        rfm95_sleep();
        rfm95_receive();
        break;

      default:
        sninfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}


static const struct file_operations g_rfm95_fops = {
  rfm95_open,
  rfm95_close,
  rfm95_read,
  rfm95_write,
  NULL,  /* Seek not implemented */
  rfm95_ioctl,
  NULL   /* Poll not implemented */
};


/**
 * Register the RFM95 character device as 'devpath' during NuttX startup.
 * 
 * @param devpath The device path
 * @param spi Pointer to SPI device connected to the module
 * @param spidev Device number bound to the SPI controller; unique per controller.
 *
*/
int rfm95_register(FAR const char *devpath, FAR struct spi_dev_s *spi, int spidev) {
  _info("devpath=%s, spidev=%d\n", devpath, spidev);
  FAR struct rfm95_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(spi != NULL);

  /* Initialize the device structure */

  priv = (FAR struct rfm95_dev_s *)
      kmm_malloc(sizeof(struct rfm95_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi    = spi;
  priv->spidev = spidev;

  rfm_dev = (struct rfm95_dev_s){
    .spi = spi,
    .spidev = spidev
  };

  /* Clear the LE pin */

  SPI_SELECT(priv->spi, priv->spidev, false);
  esp32_gpiowrite(CONFIG_RF_RFM95_SPI_CS_PIN, 1);
  /* Register the character driver */

  ret = register_driver(devpath, &g_rfm95_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif
