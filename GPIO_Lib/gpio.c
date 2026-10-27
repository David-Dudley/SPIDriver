/* *******************************************************************************************************************
 *
 * Code to run the GPIO on the Beagleboard.
 * This provides the ChipSelect capability
 *
 * *******************************************************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <error.h>

#include <linux/types.h>
#include "gpio.h"
#include "epicsTimer.h"

/* ********************************************************************************************************************
 *
 * The following routines work with the GPIO Bits on the BeagleBone to allow it to generate the Chip Select signals for the
 * selection of which SPI peripherial I'm going to work with
 *
 * ********************************************************************************************************************/


// GPIO Chip Data Structures
struct gpioData
{
	char *filename;
	int file;
}
gpioData[7] =
{
	// Here are the chip select definitions
	{
		// P9_12 - SEL0
		"/sys/class/leds/SEL0/brightness",
		0,
	},
	{
		// P8_26 - SEL1
		"/sys/class/leds/SEL1/brightness",
		0,
	},
	{
		// P8_15 - SEL2
		"/sys/class/leds/SEL2/brightness",
		0,
	},
	{
		// P9_15 - SEL3
		"/sys/class/leds/SEL3/brightness",
		0,
	},
	{
		// P8_7 - +24V Power On
		"/sys/class/leds/Control_+24V/brightness",
		0,
	},
	{
		// P8_9 - +5V Power
		"/sys/class/leds/Control_+5V/brightness",
		0,
	},
	{
		// P8_11 - +3.3V Power
		"/sys/class/leds/Control_+3.3V/brightness",
		0,
	},
};


//
// This first routine will open all the GPIO channels and initialize them
//
// This will give me a indexable array of I/O bits to reference
// Note that currently, all these bits are defined to be outputs.
int initGPIO(const char *consumer)
{
	char msg[80];

	printf("Working with %d pins\n", (int) (sizeof(gpioData) / sizeof(struct gpioData) ) );
	for(int i=0; i<(sizeof(gpioData) / sizeof(struct gpioData) ); i++ )
	{
		printf("initGPIO - opening bit \"%s\"\n",gpioData[i].filename);
		// Attempt to attach to this GPIO chip
		gpioData[i].file = open(gpioData[i].filename, O_WRONLY | O_SYNC );
		if(gpioData[i].file <= 0)
		{
			snprintf(msg, sizeof(msg), "Couldn't open %s\n",gpioData[i].filename);
			perror(msg);
			return(-1);
		}
	}
	return(0);
}

//
// This routine closes all the open GPIO channels
//
void closeGPIO()
{
	for(int i=0; i<(sizeof(gpioData) / sizeof(struct gpioData) ); i++ )
	{
		// If this chip was successfully opened, close it.
		if(gpioData[i].file)
			close(gpioData[i].file);
	}
}

int GPIOOutput(uint8_t index, uint8_t value)
{
	char *function;
	int ret=-1;

	function = ( (value != 0) ? "1" : "0" );
	if(index < (sizeof(gpioData) / sizeof(struct gpioData) ) )
		ret = write(gpioData[index].file,function,1 );
	else
		printf("Index of %d is out of range\n",index);

	return(ret);
}

//
// Select a specific chip select ID
//
int chipSelect(uint8_t select)
{
	int ret = -1;

	if( (ret = GPIOOutput( 0, (select & 1) ) < 0) )
		return(ret);
	if( (ret = GPIOOutput( 1, (select & 2) ) < 0) )
		return(ret);
	if( (ret = GPIOOutput( 2, (select & 4) ) < 0) )
		return(ret);
	if( (ret = GPIOOutput( 3, (select & 8) ) < 0) )
		return(ret);

	return(0);
}
