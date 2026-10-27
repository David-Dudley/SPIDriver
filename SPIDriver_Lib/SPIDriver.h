/* ****************************************************************************************
 *
 * SPI Driver Interface definitions
 *
 * ****************************************************************************************/

#include <endian.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

// Show the Opened SPI Channel settings
char *SPIdumpstat(const char *name, int fd);

// Open the SPI channel
int openSPI(const char *device, const uint32_t mode, const uint8_t bits, const uint32_t speed);

// Close the SPI channel
void closeSPI(const int fd);

// Do a SPI transaction using 8-bit values
int doSPITransaction8(const int fd, const uint32_t len, const uint8_t *xmt, uint8_t *rcv);

// Do a SPI transaction using 16-bit values
int doSPITransaction16(const int fd, const uint32_t len, const uint16_t *xmt, uint16_t *rcv);
