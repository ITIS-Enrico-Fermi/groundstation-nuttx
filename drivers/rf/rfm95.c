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


#if 1 || defined(CONFIG_SPI) && defined(CONFIG_RF_RFM95)

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

/* Sends a reset signal down the RST GPIO pin */
static void rfm95_reset() {
  esp32_gpiowrite(CONFIG_RF_RFM95_RESET_PIN, 0);
  up_mdelay(1);
  esp32_gpiowrite(CONFIG_RF_RFM95_RESET_PIN, 1);
  up_mdelay(10);
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
 * Enable appending/verifying packet CRC.
 */
void rfm95_enable_crc() {
   rfm95_write_reg(REG_MODEM_CONFIG_2, rfm95_read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void rfm95_sleep() {
  _info("Set sleep");
  rfm95_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void rfm95_idle() {
  rfm95_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

static inline void rfm95_configspi() {
  _info("\n");
  SPI_LOCK(rfm_dev.spi, true);

  /* Set SPI Mode (Polarity and Phase) and Transfer Size (8 bits) */

  SPI_SETMODE(rfm_dev.spi, RFM95_SPI_MODE);
  SPI_SETBITS(rfm_dev.spi, 8);

  /* Set SPI Hardware Features and Frequency */

  SPI_HWFEATURES(rfm_dev.spi, 0);
  SPI_SETFREQUENCY(rfm_dev.spi, CONFIG_RF_RFM95_SPI_FREQUENCY);

  SPI_LOCK(rfm_dev.spi, false);

  /* Configure reset pin */
  esp32_configgpio(CONFIG_RF_RFM95_RESET_PIN, OUTPUT);

  /* Configure cs spi pin */
  esp32_configgpio(CONFIG_RF_RFM95_SPI_CS_PIN, OUTPUT);
}

static void rfm95_init(FAR struct file *filep) {
  DEBUGASSERT(filep  != NULL);
  
  /* Get the SPI interface */
  FAR struct inode *inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);

  rfm95_reset();

  rfm95_configspi();

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
  DEBUGASSERT(i < TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1
  _info("Version: %d\n", version);

  /*
  * Default configuration.
  */
  rfm95_sleep();
  rfm95_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
  rfm95_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
  rfm95_write_reg(REG_LNA, rfm95_read_reg(REG_LNA) | 0x03); //LNA boost
  rfm95_write_reg(REG_MODEM_CONFIG_3, 0x04);                //auto AGC

  rfm95_set_tx_power(CONFIG_RF_RFM95_TX_POWER);
  
  //set sync mode

  rfm95_idle();

  rfm95_set_frequency(CONFIG_RF_RFM95_TX_FREQ);
  rfm95_set_spreading_factor(9);
  rfm95_set_bandwidth(500e3);

  if(true) //TODO: parametrize CRC enable
    rfm95_enable_crc();

}

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
  int irq_reg = rfm95_read_reg(REG_IRQ_FLAGS);
  while((irq_reg & IRQ_TX_DONE_MASK) == 0)
  {
    syslog(LOG_INFO, "waiting TX_DONE. Reg value: %d", irq_reg);
    up_mdelay(100);
  }
  
  rfm95_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
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

static ssize_t rfm95_read(FAR struct file *filep, FAR char *buffer, size_t buflen) {
  return -ENOSYS;
}

static int rfm95_ioctl(FAR struct file *filep, int cmd, unsigned long arg) {
  _info("cmd=0x%x, arg=0x%lx\n", cmd, arg);
  DEBUGASSERT(filep != NULL);

  int ret = OK;

  switch (cmd)
    {
      case RFM95_IOCTL_INIT:
        rfm95_init(filep);
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
