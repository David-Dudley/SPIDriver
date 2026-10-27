/* *************************************************************************
 *
 * This set of functions does the actual communications with a SPI device
 *
 * ************************************************************************* */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <arpa/inet.h>

#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>

#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "SPIDriver.h"

// Verbosity Debug Flag
//
// Bit 1 = show 16 bit transactions
// Bit 2 = show 8 bit transactions

extern int SPIDebug;

// ---------------------------------------------------------------------------------------------------------------
//
// This next set of routines is for working with the SPI channel
//
// ---------------------------------------------------------------------------------------------------------------

//
// This routine will print out the current settings
//
// Pass name of the interface, and the file descriptor that needs to be already open
//
//
static char msg[132];

char *SPIdumpstat(const char *name, int fd)
{
	uint8_t		lsb, bits;
	uint32_t	mode, speed;

	if (ioctl(fd, SPI_IOC_RD_MODE32, &mode) < 0) {
        perror("Can't read the SPI rd_mode");
	}
	if (ioctl(fd, SPI_IOC_RD_LSB_FIRST, &lsb) < 0) {
        perror("Can't read the SPI RD_LSB first");
	}
	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
		perror("Can't get the SPI bits_per_word");
	}
	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
		perror("Can't get the SPI max_speed_hz");
	}

	snprintf(msg, sizeof(msg),"SPI Interface settings\nDevice %s:\nSPI Mode 0x%x\n%d bits\n%sper word\n%u Hz max\n", name, mode, bits, lsb ? "(lsb first) " : "", speed);
	return msg;
}


// This routine will OPEN the spi channel so I can communicate through it,
// and set the commucations parameters for thedevice
//
// pass
// Name of the device to open,
// SPI Mode to use
// Number of bits per transaction
// speed
//
int openSPI(const char *device, const uint32_t mode, const uint8_t bits, const uint32_t speed)
{
	int fd;
	uint32_t result = 0;
	int ret = 0;

	fd = open(device, O_RDWR | O_NONBLOCK);
	if (fd < 0)
	{
		perror("open SPI device");
		return(fd);
	}

	// Now, lets set my transfer parameters
	/*
	 * spi mode
	 */
	/* make a request to assign 'mode' */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
    {
		perror("can't set spi mode");
        return(-1);
    }

	/* Read what mode the device actually is in */
	ret = ioctl(fd, SPI_IOC_RD_MODE, &result);
	if (ret == -1)
    {
		perror("Can't get spi mode");
        return(-1);
    }
	/* Drivers can reject some mode bits without returning an error.
	 * Read the current value to identify what mode it is in, and if it
	 * differs from the requested mode, warn the user.
	 */
	if (result != mode)
		printf("WARNING device does not support requested mode 0x%x <> 0x%x\n", mode, result);

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
    {
		perror("can't set bits per word");
        return(-1);
    }

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
    {
		perror("can't get bits per word");
        return(-1);
    }

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
    {
		perror("can't set max speed hz");
        return(-1);
    }

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
    {
        perror("can't get max speed hz");
        return(-1);
    }

	// Show my net settings
	if(SPIDebug&1)
		SPIdumpstat(device, fd);

	return(fd);
}

//
// This routine will close the SPI channel
//
// Right now, it's simple, but I want to make it a separate call in case I need to add somehting
void closeSPI(const int fd)
{
	close(fd);
}

// This routine will send a message to the SPI chip, and receive the corresponding data.
// Note that it expects the length to send/receive, and buffers allocated to store that much data
// Also note the returned value
//
// On entry:
//          fd - File handle to this SPI
//          len - length to transmit
//          xmt - Pointer to transmit buffer
//          rcv - Pointer to Rcv buffer
//
//  This routine sends a BYTE message out and receives a BYTE in.
//  Conversion to other formats must be handled by the caller.

int doSPITransaction8(const int fd, const uint32_t len, const uint8_t *xmt, uint8_t *rcv)
{
	struct spi_ioc_transfer tr = {0};
	int		status;
	char msg[80];

	// Log the message going out if in verbose mode
	if(SPIDebug & 2)
	{
		printf("Writing( 8) < ");
		for(uint8_t i=0; i<len ; i++)
			printf("%02x, ", xmt[i]);
		printf(" >\n");
	}

	// Do the operation
	tr.tx_buf = (__u64 ) xmt;		// Pointer to xmit buffer
	tr.rx_buf =  (__u64) rcv;		// pointer to receive buffer
	tr.len = len;						// Number of BYTES to transfer
	tr.delay_usecs = 0;				// Delay in uSecs after transfer (settling???)
	tr.cs_change = 0;					// Deselect device before next transfer if set
	tr.word_delay_usecs = 0;			// Delay between words in a transfer (optional)

	// Do the transaction
	status = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (status < 0)
	{
		sprintf(msg, "doSPITransaction8 failed Handle=%d",fd);
		perror(msg);
	}

	// Log the message received if in verbose mode
	if(SPIDebug & 2)
	{
		printf("Reading( 8) < ");
           for(uint8_t i=0; i<len ; i++)
			printf("%02x, ", rcv[i]);
        printf(" >\n");
	}

	return(status);
}

/* =====================================================================================
 * Send a WORD message out the SPI port.
 * On entry, the file descriptor is passed,
 * as well as an array pointer of what 16-bit values to send,
 *  and a buffer to receive the 16-bit response.
 *
 * This routine expects the message to be passed in HOST order, and will produce a local
 * copy of the message that is in net order.
 *
 * Finally, the data received will be translated from NET order into HOST order before return
 *
 * Parameters:
 * fd	-	file handle
 * len	-	length of message in BYTES
 * xmt	-	Pointer to message to transmit
 * rcv	-	Pointer of where to put response
 */

int doSPITransaction16(const int fd, const uint32_t len, const uint16_t *xmt, uint16_t *rcv)
{
	struct spi_ioc_transfer tr = {0};

	uint16_t *tempMsg;
	int status = -1;

	// Clear the receive buffer
	memset(rcv,0,len);

	// Allocate a temporary transmit message
	tempMsg = malloc(len);
	memset(tempMsg,0,len);

	// If memory allocation was successful
	if(tempMsg)
	{
		// I only transmit 16 bit values out, but they have to be in net order
		for(uint8_t i = 0; i<len/2; i++)
		{
			tempMsg[i] = htons(xmt[i]);
		}

		// Now, send out the message and get the response
		// Do the operation
		tr.tx_buf = (__u64 ) tempMsg;		// Pointer to xmit buffer
		tr.rx_buf =  (__u64) rcv;			// pointer to receive buffer
		tr.len = len;						// Number of BYTES to transfer
		tr.delay_usecs = 0;					// Delay in uSecs after transfer (settling???)
		tr.cs_change = 0;					// Deselect device before next transfer if set
		tr.word_delay_usecs = 0;			// Delay between words in a transfer (optional)

		// Do the transaction
		status = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
		if(status < 0)
			perror("doSPITransaction16 Failed!");

		//So, now, convert the receive data into HOST order
		for(uint8_t i = 0; i<len/2; i++)
		{
			rcv[i] = ntohs(rcv[i]);
		}

		// Release the temp buffer
		free(tempMsg);
	}
	else
		perror("Couldn't allocate temp buffer");

	return(status);
}
