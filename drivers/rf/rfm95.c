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

/****************************************************************************
 * Included Files
 ****************************************************************************/
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SPI) && defined(CONFIG_RF_RFM95)

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

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rfm95_dev_s
{
  FAR struct spi_dev_s *spi;    /* Saved SPI driver instance */
  int spidev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int rfm95_open(FAR struct file *filep);
static int rfm95_close(FAR struct file *filep);
static ssize_t rfm95_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t rfm95_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int rfm95_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_rfm95_fops =
{
  rfm95_open,
  rfm95_close,
  rfm95_read,
  rfm95_write,
  NULL,  /* Seek not implemented */
  rfm95_ioctl,
  NULL   /* Poll not implemented */
};

static char recv_buffer[256];  /* Buffer for SPI response */

static int recv_buffer_len = 0;  /* Length of SPI response */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Low level functions to expose the underlying SPI bus to rfm95_logic.c */

int rfm95_read_reg(FAR struct spi_dev_s *spi, int reg) {
   uint8_t out[2] = { reg, 0xff };
   uint8_t in[2];

   /* Transmit buffer to SPI device and receive the response */
   SPI_EXCHANGE(spi, out, in, 2);
   recv_buffer_len = 2;
   return in[1];
}

void rfm95_write_reg(FAR struct spi_dev_s *spi, int reg, int val) {
   uint8_t out[2] = { 0x80 | reg, val };
   uint8_t in[2];

   /* Transmit buffer to SPI device */
   SPI_EXCHANGE(spi, out, in, 2);
}

/* Sends a reset signal down the RST GPIO pin */
static void rfm95_reset() {
  esp32_gpiowrite(CONFIG_RF_RFM95_RESET_PIN, 0);
  up_mdelay(1);
  esp32_gpiowrite(CONFIG_RF_RFM95_RESET_PIN, 1);
  up_mdelay(10);
}

static void rfm95_init(FAR struct file *filep) {
  
  DEBUGASSERT(filep  != NULL);
  
  /* Get the SPI interface */
  FAR struct inode *inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  FAR struct rfm95_dev_s *priv = inode->i_private;
  DEBUGASSERT(priv != NULL);

  /* Configure reset pin */
  esp32_configgpio(CONFIG_RF_RFM95_RESET_PIN, OUTPUT);
  rfm95_reset();
  /* Configure cs spi pin */
  esp32_configgpio(CONFIG_RF_RFM95_SPI_CS_PIN, OUTPUT);

  /* Lock the SPI bus */
  DEBUGASSERT(priv->spi != NULL);
  SPI_LOCK(priv->spi, true);
  /* Enable CS pin */
  esp32_gpiowrite(CONFIG_RF_RFM95_SPI_CS_PIN, 0);
  SPI_SELECT(priv->spi, priv->spidev, true);

  rfm95_configspi(priv->spi);

  /*
  * Check version.
  */
  uint8_t version;
  uint8_t i = 0;
  while(i++ < TIMEOUT_RESET) {
    version = rfm95_read_reg(priv->spi, REG_VERSION);
    if(version == 0x12) break;
    up_mdelay(2);
  }
  DEBUGASSERT(i < TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1
  _info("Version: %d\n", version);

  /*
  * Default configuration.
  */
  rfm95_write_reg(priv->spi, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP); // sleep mode
  rfm95_write_reg(priv->spi, REG_FIFO_RX_BASE_ADDR, 0);
  rfm95_write_reg(priv->spi, REG_FIFO_TX_BASE_ADDR, 0);
  rfm95_write_reg(priv->spi, REG_LNA, rfm95_read_reg(priv->spi, REG_LNA) | 0x03);
  rfm95_write_reg(priv->spi, REG_MODEM_CONFIG_3, 0x04);
  uint8_t level = CONFIG_RF_RFM95_TX_POWER;
  if (level < 2) level = 2;
  else if (level > 17) level = 17;
  rfm95_write_reg(priv->spi, REG_PA_CONFIG, PA_BOOST | (level - 2)); // set tx power

  /* Setup transmission frequency */
  int frequency = CONFIG_RF_RFM95_TX_FREQ;
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  rfm95_write_reg(priv->spi, REG_FRF_MSB, (uint8_t)(frf >> 16));
  rfm95_write_reg(priv->spi, REG_FRF_MID, (uint8_t)(frf >> 8));
  rfm95_write_reg(priv->spi, REG_FRF_LSB, (uint8_t)(frf >> 0));

  /* Set sync word 
  * 0x00 = none
  */
  int syncw = CONFIG_RF_RFM95_SYNC_WORD;
  if (syncw != 0x00)
  {
    rfm95_write_reg(priv->spi, REG_SYNC_WORD, syncw);
  }

  /* Add the rest of the config.. */

  rfm95_write_reg(priv->spi, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY); // idle mode

  esp32_gpiowrite(CONFIG_RF_RFM95_SPI_CS_PIN, 1);
  SPI_SELECT(priv->spi, priv->spidev, false);
  SPI_LOCK(priv->spi, false);
}

void rfm95_send_packet(FAR struct spi_dev_s *spi, const uint8_t *buf, int size) {
   /*
    * Transfer data to radio.
    */
   rfm95_write_reg(spi, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY); // idle mode
   rfm95_write_reg(spi, REG_FIFO_ADDR_PTR, 0);

   for(int i=0; i<size; i++) 
      rfm95_write_reg(spi, REG_FIFO, *buf++);
   
   rfm95_write_reg(spi, REG_PAYLOAD_LENGTH, size);
   
   /*
    * Start transmission and wait for conclusion.
    */
   rfm95_write_reg(spi, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
   while((rfm95_read_reg(spi, REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
   {
      up_mdelay(100);
   }
   

   rfm95_write_reg(spi, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

/****************************************************************************
 * Name: rfm95_configspi
 *
 * Description:
 *   Configure the SPI instance
 *
 ****************************************************************************/

static inline void rfm95_configspi(FAR struct spi_dev_s *spi)
{
  _info("\n");
  DEBUGASSERT(spi != NULL);

  /* Set SPI Mode (Polarity and Phase) and Transfer Size (8 bits) */

  SPI_SETMODE(spi, RFM95_SPI_MODE);
  SPI_SETBITS(spi, 8);

  /* Set SPI Hardware Features and Frequency */

  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_RF_RFM95_SPI_FREQUENCY);
}

/****************************************************************************
 * Name: rfm95_open
 *
 * Description:
 *   This function is called whenever the device is opened.
 *
 ****************************************************************************/

static int rfm95_open(FAR struct file *filep)
{
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

  SPI_LOCK(priv->spi, true);
  rfm95_configspi(priv->spi);
  SPI_LOCK(priv->spi, false); // closing lock is important :)

  return OK;
}

/****************************************************************************
 * Name: rfm95_close
 *
 * Description:
 *   This function is called whenever the device is closed.
 *
 ****************************************************************************/

static int rfm95_close(FAR struct file *filep)
{
  _info("\n");
  DEBUGASSERT(filep != NULL);
  return OK;
}

/****************************************************************************
 * Name: rfm95_write
 *
 * Description:
 *   Transmit a message.
 ****************************************************************************/

static ssize_t rfm95_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t buflen)
{
  _info("buflen=%u\n", buflen);
  DEBUGASSERT(buflen <= sizeof(recv_buffer));
  DEBUGASSERT(buffer != NULL);
  DEBUGASSERT(filep  != NULL);

  /* Get the SPI interface */

  FAR struct inode *inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  FAR struct rfm95_dev_s *priv = inode->i_private;
  DEBUGASSERT(priv != NULL);

  /* Lock the SPI bus */
  
  DEBUGASSERT(priv->spi != NULL);
  SPI_LOCK(priv->spi, true);

  /* Assert CS pin of the module */
  esp32_gpiowrite(CONFIG_RF_RFM95_SPI_CS_PIN, 0);
  SPI_SELECT(priv->spi, priv->spidev, true);

  rfm95_send_packet(priv->spi, (uint8_t)buffer, buflen);

  /* Deassert CS pin of the module */
  SPI_SELECT(priv->spi, priv->spidev, false);
  esp32_gpiowrite(CONFIG_RF_RFM95_SPI_CS_PIN, 1);
  /* Unlock the SPI bus */

  SPI_LOCK(priv->spi, false);

  return buflen;
}

/****************************************************************************
 * Name: rfm95_read
 *
 * Description:
 *   Return the data received from the device.
 ****************************************************************************/

static ssize_t rfm95_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  _info("buflen=%u\n", buflen);
  DEBUGASSERT(filep  != NULL);
  DEBUGASSERT(buffer != NULL);

  /* Copy the SPI response to the buffer */

  DEBUGASSERT(recv_buffer_len >= 0);
  DEBUGASSERT(recv_buffer_len <= buflen);
  memcpy(buffer, recv_buffer, recv_buffer_len);

  /* Return the number of bytes read */

  return recv_buffer_len;
}

/****************************************************************************
 * Name: rfm95_ioctl
 *
 * Description:
 *   Execute ioctl commands for the device.
 ****************************************************************************/

static int rfm95_ioctl(FAR struct file *filep,
                           int cmd,
                           unsigned long arg)
{
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rfm95_register
 *
 * Description:
 *   Register the rfm95 character device as 'devpath' during NuttX startup.
 *
 ****************************************************************************/

int rfm95_register(FAR const char *devpath,
                       FAR struct spi_dev_s *spi,
                       int spidev)
{
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
