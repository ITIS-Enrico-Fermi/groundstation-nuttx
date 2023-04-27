#ifndef __INCLUDE_NUTTX_RF_RFM95_H
#define __INCLUDE_NUTTX_RF_RFM95_H

/* IOCTL codes */

/* Initialize RFM9x module */
#define RFM95_IOCTL_INIT 0

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/**
 * Register the rfm95 character device as 'devpath'.
 * 
 * @param devpath The full path to the driver to register. E.g., "/dev/radio0"
 * @param spi An instance of the SPI interface to use to communicate with
 * @param spidev Number of the spi device (used to drive the Latch Enable pin).
 *
 * @return Zero (OK) on success; a negative errno value on failure.
 *
*/
int rfm95_register(FAR const char *devpath, FAR struct spi_dev_s *spi, int spidev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_RF_RFM95_H */
