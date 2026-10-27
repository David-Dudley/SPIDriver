/* *****************************************************************************
 *
 * This is a general purpose driver developed to allow interfacing with
 * a SPI interface on a host card.
 *
 * Currently it is defined for the BeagleBoard Black and AI64 devices.
 *
 *
 * *****************************************************************************/

#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <asm/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <ctype.h>
#include <getopt.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <error.h>
#include <endian.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <arpa/inet.h>

#include <drvSup.h>
#include <dbAccess.h>
#include <dbScan.h>
#include <initHooks.h>
#include <dbStaticLib.h>
#include <devSup.h>
#include <recSup.h>
#include <recGbl.h>
#include <errlog.h>
#include <alarm.h>
#include <callback.h>
#include <epicsMutex.h>

#include <aiRecord.h>
#include <aoRecord.h>
#include <biRecord.h>
#include <boRecord.h>
#include <mbbiRecord.h>
#include <mbboRecord.h>
#include <mbbiDirectRecord.h>
#include <mbboDirectRecord.h>
#include <iocsh.h>
#include <epicsExport.h>

#include "gpio.h"
#include "SPIDriver.h"

#include "printReportBuffer.h"

#define MAX_STR 255
#define SPI_BUF_LEN 1024


// Global variables
int SPIDebug=0;
epicsExportAddress(int, SPIDebug);

// Heres the definition of the chain that contains all the servers that have been defined.
static struct deviceServer {
  struct deviceServer *next;
  char *name;
  epicsMutexId plock;
  int handle;
  uint8_t mode;
  uint8_t bits;
  uint32_t speed;
} *deviceServers = NULL;


// Private data that the access routines use
struct privateData 
{
  dbCommon  *pv;            // Pointer back to the record definition
//  ELLNODE node;
  char *    device;         // Which device this is (name)
  struct    deviceServer *server; // Link to actual device data
  char *    driverName;     // Name of the driver I'm going to use
  EPICSTHREADFUNC driver;   //What Driver to use
  epicsThreadId driverThread; // Thread to do the function
  CALLBACK  cb;             // Callback to execute when compete
  uint8_t   tpro;           // Show debug data on this transaction
  uint8_t   chipSelect;     // ChipSelect to use
  uint8_t   indexsupplied;  // There was an index included in this command
  uint16_t  idx;           // Where to access raw data in xmt/rcv
  int16_t   rval;           // Raw Value
  uint8_t dataType;         // type of data (0=words, 1=bytes, 2=special (allocate words))
  uint8_t len;              // Length of data
  union                     // Data Transmit Buffer
  {
    uint16_t  *words;
    uint8_t   *bytes;
  } xmt;
  union                     // Data Receive Buffer
  {
    uint16_t  *words;
    uint8_t   *bytes;
  } rcv;
  //epicsMutexId lock;
};


/* *********************************************************************************
 * 
 * Add this channel into the driver
 * 
 * On entry:
 * name   - Name for the device
 * device - File link for access
 * mode   - What SPI mode to use
 * bits   - How many bits for each character to transmit
 * speed  - How fast
 * ********************************************************************************/
static int addSPIDevice( const char *name, const char *device, const uint8_t mode, const uint8_t bits, const uint32_t speed)
{
  struct deviceServer *newServer;

  int fd = 0;

//  printf("Opening %s -> %s\n",name, device);
  fd = openSPI(device, mode, bits, speed);
  if (fd < 0)
  {
    perror("open SPI device");
    return(fd);
  }
  else
  {
    // Add this server onto the open servers list now
		
	// Allocate memory for the structure
	newServer = (struct deviceServer *) calloc(1, sizeof(struct deviceServer) );
		
    // extend the list with the new one at the first entry
	newServer->next = deviceServers;
    deviceServers = newServer;
    newServer->name = strdup(name);
    newServer->plock = epicsMutexMustCreate();
	newServer->handle = fd;
  }
  return(0);
}

static struct deviceServer *findDevice(char *name)
{
  struct deviceServer *server;
	
  for(server = deviceServers; server != NULL; server=server->next)
  {
    if( (strlen(name) == strlen(server->name) ) && !strcmp(name, server->name) )
    {
      return(server);
    }
  }
  return(NULL);
}
	
void closeSPIDevices()
{
  struct deviceServer *server;

  server = deviceServers;
  while(server)
  {
//    epicsMutexDestroy(server->plock);
    closeSPI(server->handle);
    server = server->next;
  }
}


/*
 * Execution of this routine is delayed until after the IOC has reached the initHookAfterInitDrvSup state
 * 
 * From that state on, I can track what state the IOC is moving into, which will allow me to provide additional
 * support when particular states occur.
 * 
 * Currently, I wait till "initHookAfterInterruptAccept" state, at which time it's safe to start the driver,
 * as all the database stuff has been initialized by then.
 *
 * States in this routine are being documented directly from the API document, so there is some details 
 * on what is in progress at the moment. 
 */
static const char statsDef[] = 
"record(aai, \"$(P):DriverStats\")\n"
"{\n"
"	field(DESC, \"Statistics\")\n"
"	field(DTYP, \"Stats\")\n"
"	field(SCAN, \"Passive\")\n"
"	field(FTVL, \"DOUBLE\")\n"
"	field(SCAN, \"1 second\")\n"
"	field(NELM, 20)\n"
"#  field(INP, \"stats\")\n"
"}\n";


/*
 * This routine will take my -in-memory- record definition, write it to a temporary
 * file, and then load that file into the database.
 */

// Set this to use 'tmpfile' to create a temprary file, or don't set to use tempname and create an actual file in /tmp
//#define USE_TMPFILE
void loadMyDatabaseRecords(void)
{
  // Keep these out of the routine till I figure out what I want.
	char *pptr = NULL;
	char *subsPtr = NULL;
	FILE *tempFile = NULL;
	const char substring[] = "P=";

  return;
#ifndef USE_TMPFILE
	char nameTemplate[] = "SPI_IO";
	char *tempName;
#endif

	/* Generate a temporary file, and write the database definition of the diagnostics record into it. */
#ifdef USE_TMPFILE
	tempFile = tmpfile();
#else
	tempName = tempnam(NULL, nameTemplate);
//	printf("Opening %s\n",tempName);
	tempFile=fopen(tempName, "w+");
#endif
	if(tempFile) {
		size_t i;

		i=fputs(statsDef, tempFile);
#ifdef USE_TMPFILE
		// rewind the file to the start
		rewind(tempFile);
#else
		// If I'm using a file to do this, lets just close the file at the moment.
		fclose(tempFile);
#endif
		// Not quite ready yet, I need to fill out my "substitutions" buffer
		pptr=getenv("IOCNAME");
		i=strlen(substring)+strlen(pptr);
		subsPtr = (char *) calloc(sizeof(char), i+1);
		strncpy(subsPtr, substring, strlen(substring) );
		strncat(subsPtr, pptr, strlen(pptr)+1);

#ifdef USE_TMPFILE
//		printf("Loading new database\n");
		// use dbReadDatabaseFP to load the file
		dbReadDatabaseFP(&pdbbase, tempFile, NULL, subsPtr);
#else
//		printf("Loading my temp file %s\n",tempName);
		// use dbReadDatabase to load the file
		dbReadDatabase(&pdbbase, tempName, NULL, subsPtr);
		free(tempName);
#endif
//		printf("Done\n");
		free(subsPtr);
//		printf("Closing file\n");
#ifdef USE_TMPFILE
		// Closing the file will delete it as well.
		fclose(tempFile);
#else
#endif
	}
}
#ifdef NOT_NOW
static void scannerThread(void *parm);
void startScannerThread()
{
  epicsThreadCreate("scannerThread", epicsThreadPriorityHigh, epicsThreadGetStackSize(epicsThreadStackBig), &scannerThread, 0);
}
#endif

static void initStateRoutine(initHookState state)
{
//	printf("initState called at state %s\n",initHookName(state));
	switch(state)
	{
	case initHookAtIocBuild:			// Initial case.  called before the IOC has begun it's initialization processing.  Not much happening yet'
		break;
	case initHookAtBeginning:			// At this point, the log is showing that iocInit is starting, and sigHub has been set to be ignored
		loadMyDatabaseRecords();        // Load my specific database records that I might want to manage for this driver.  These are specifically allocated to this driver.
		break;
	case initHookAfterCallbackInit:		// Show the version message, initialize the task watchdog, and initialize the callback functions.
		break;
	case initHookAfterCaLinkInit:		// Initialize the CaLink routines, but don't start anything yet
		break;
	case initHookAfterInitDrvSup:		// Driver initialization routines are called. Realize that up to this point, there still hasn't been any database work.
		break;
	case initHookAfterInitRecSup:		// Initialize each Record Support routine.  This still hasn't linked to the database, as that comes later
		break;
	case initHookAfterInitDevSup:		// Initialize the Device support routines, indicating this is the initial call. This is separate from the Record
										// and Driver support routines, but still, realize that no Database work has started yet.
      initGPIO("ioc");                       // Startup the GPIO lines
		break;
	case initHookAfterInitDatabase:		// Start Database initialization now.  Three passes are made over the database.
										// 1. Initialize RSET, RDES, MLOK, MLIS, PACTA, and DSET for all records. Call each record support's init_record(first pass)
										// 2. Convert each PV_LINK into a DB_LINK or a CA_LINK.  Call any extended device support's add-record routines.
										// 3. Call each record support's init-record for a second pass.
										// Next register an epicsAtExit routine to shutdown the database.  Then create the lock sets.  Finally initialize the db debugging support
		break;
	case initHookAfterFinishDevSup:		// Second pass at Device Support, indicating this is the final call.
		break;
		/* Initial setup for all the SCAN threads (but they're not operational yet)
		 * Initialize access security as well
		 * Initialize process notification
		 */
	case initHookAfterScanInit:
		break;
	case initHookAfterInitialProcess:	// Here's where we start some basic work.  Initially process all records that have PINI set
		break;
	case initHookAfterCaServerInit:		// Initially start the server going, but don't allow tasks to announce on the network yet
		break;
	case initHookAfterIocBuilt:			// The IOC is fully initialized, but is still quiet. iocBuild command completes here
		break;
	case initHookAtIocRun:				// Coming out of iocBuilt, and moving to active state.  Just announce getting started
		break;
	case initHookAfterDatabaseRunning:	// Start up the database operation and scanning now.  Set InterruptAccept to TRUE
		break;
/*
 * Execution of this routine is delayed until after the IOC has achieved a particular state.
 * By doing this, the startup of the IOC may be improved, and when this routine actually executes,
 * the database system is already started, and processing can continue without errors.
 */
	/* Show this after things are started, if this is the first time reaching this level */
	case initHookAfterInterruptAccept:	// OK, I'm now at the right state to be able to start things up.
		break;
	case initHookAfterCaServerRunning:	// Start up allthe channel access server tasks
		break;
	case initHookAtEnd:					// If the first time reacing this point, show the state I'm in
		break;
	case initHookAfterIocRunning:		// At this point, the iocInit process is complete, so log a completion message, and announce the startup function is complete.
//        (void)startScannerThread();
		break;
	case initHookAtIocPause:
		break;
	case initHookAfterCaServerPaused:
		break;
	case initHookAfterDatabasePaused:
		break;
	case initHookAfterIocPaused:
		break;
	case initHookAtShutdown:             // Start of iocShutdown commands
		break;
	case initHookAfterCloseLinks:
		break;
	case initHookAfterStopScan:          // triggered only by unittest code.   testIocShutdownOk()
		break;
	case initHookAfterStopCallback:      // triggered only by unittest code.   testIocShutdownOk()
		break;
	case initHookAfterStopLinks:
		break;
	case initHookBeforeFree:             // triggered only by unittest code.   testIocShutdownOk()
		break;
	case initHookAfterShutdown:          // End of iocShutdown commands
		closeSPIDevices();               // Shutdown SPI
        closeGPIO();                     // and GPIO as well
		break;

	default:
		break;
	}
}

static long report(int)
{
	return(0);
}

// Initial routine
static long initRoutine()
{
	initHookRegister(&initStateRoutine);

	return 0;
}

/* ******************************************************************************************
 *
 * Heres where the magic starts to happen
 *
 * ******************************************************************************************/

/* Each of these drivers is a thread that operates when requested. */

/* ============================================================================================
 *
 * Here is the driver that outputs a value to the actual GPIO bit defined
 *
 * ============================================================================================ */
static void GPIOWrite( void *ptr)
{
  struct privateData *priv = ptr;
  do
  {
    // Wait till someone requests service
    epicsThreadSuspendSelf();

    if(priv->tpro == 1)
      printf("GPIO[%d] = %d\n",priv->idx, priv->rval);

    if( GPIOOutput(priv->idx, priv->rval) < 0)
    {
      perror(" Inside __FILE__ Doing GPIOWrite Failed");
    }

  } while(1);
}

/* ============================================================================================
 *
 * This driver is specific to the ADS114 A/D converter chip analog input channels.
 *
 * It will perform the following sequence of control
 * 1. Lock the SPI Channel
 * 2. Output the byte string that is defined in the record
 * 3. Output a 'START' command to start the conversion
 * 4. Check if I have DRDY valid
 * 5. If not ready, allow remainder to scan
 * 6. Once the device is ready (NOP returns a 0 status when done) output a RDATA command
 * 7. Get the result of the conversion from bytes 1 and 2 in the response (MSB first)
 * 8. Send out a 'STOP' command to stop conversion.
 * 9. Set state back to starting value (0 I'd guess)
 * 10.Return result
 *
 * ============================================================================================ */

static uint8_t startCmd[] = {0x8};
static uint8_t getDeviceStatus[] = { 0x21, 0, 0 };
static uint8_t readCmd[] = { 0x12, 0, 0};
static uint8_t stopCmd[] = {0xa};
static uint8_t readDRDY[] = {0,0};

static void ADS114Read_Driver(void *ptr)
{
  struct privateData *priv = ptr;
  int status = -1;
  uint8_t rcv[]={0,0,0,0,0,0,0,0,0,0};

  do
  {
    // Wait till someone requests service
    epicsThreadSuspendSelf();

    // Wait till I can lock the resource
    epicsMutexMustLock(priv->server->plock)


    // Now, step through the sequence

    // First, lets see if the chip is ready to accept another command
    do
    {
      // Log the message going out if in verbose mode
      if(priv->tpro == 1)
      {
        printf("\nADS114Read_Driver:\nWriting(8) 0x%1x> ",priv->chipSelect);
        for(uint32_t i=0; i<sizeof(getDeviceStatus); i++)
          printf("%02x ", getDeviceStatus[i]);
        printf("\n");
      }
      // Select the chip and do the transaction
      chipSelect(priv->chipSelect);

      status = doSPITransaction8(priv->server->handle, sizeof(getDeviceStatus), getDeviceStatus, rcv );
      if(status < 0)
      {
        printf(" doSPITransaction8 to %s[%d] failed (1)\n",priv->pv->name, priv->server->handle);
        exit(1);
      }
      if(rcv[2] & 0x40)
        printf("Device not ready, Status=%02x ",rcv[2]);
    }
    while(rcv[2] & 0x40);

    // Output the initial setup command and start the conversion

    // Log the message going out if in verbose mode
    if(priv->tpro == 1)
    {
      printf("\nADS114Read_Driver:\nWriting(8) 0x%1x> ",priv->chipSelect);
      for(uint32_t i=0; i<priv->len; i++)
        printf("%02x ", priv->xmt.bytes[i]);
      printf("\n");
    }
    // Select the chip and do the transaction
    chipSelect(priv->chipSelect);

    // Note that the command string should not have a START command appended to it.
    // This output is just to set the input path, mux, PGA, ...
    status = doSPITransaction8(priv->server->handle, priv->len*sizeof(uint8_t), priv->xmt.bytes, priv->rcv.bytes );
    if(status < 0)
    {
        printf(" doSPITransaction8 to %s[%d] failed (2.1)\n",priv->pv->name, priv->server->handle);
        exit(1);
    }

    // Now, send the start command
    status = doSPITransaction8(priv->server->handle, sizeof(startCmd), startCmd, priv->rcv.bytes );
    if(status < 0)
    {
        printf(" doSPITransaction8 to %s[%d] failed (2.2)\n",priv->pv->name, priv->server->handle);
        exit(1);
    }

    if(priv->tpro == 1)
      printf("\nADS114Read_Driver: Checking for DRDY: ");

    // Wait till the conversion is complete
    // check the DRDY flag
    do
    {
      chipSelect(0);

      status = doSPITransaction8(priv->server->handle, sizeof(readDRDY), readDRDY, rcv);
      if(status < 0)
      {
        printf(" doSPITransaction8 to %s[%d] failed (3)\n",priv->pv->name, priv->server->handle);
        exit(1);
      }

      chipSelect(15);
      // If it's not clear, I'm not ready yet
      if(rcv[0])
      {
        // Still busy, break out of the sequence
        if(priv->tpro == 1)
          printf(". ");
      }
    }
    while(rcv[0]);

    // Once the conversion is complete, read the data, get the result, and then stop the conversion
    if(priv->tpro == 1)
    {
      printf("\nADS114Read_Driver:\nPerforming RDATA(8) 0x%1x< ",priv->chipSelect);
      for(uint8_t i=0; i<sizeof(readCmd) ; i++)
        printf("%02x ", readCmd[i]);
      printf(">\n");
    }
    chipSelect(priv->chipSelect);
    status = doSPITransaction8(priv->server->handle, sizeof(readCmd), readCmd, priv->rcv.bytes );
    if(status < 0)
    {
        printf(" doSPITransaction8 to %s[%d] failed (4)\n",priv->pv->name, priv->server->handle);
        exit(1);
    }

    if(priv->tpro == 1)
    {
      printf(" =< " );
      for(uint8_t i=0; i<sizeof(readCmd) ; i++)
        printf("%02x ", priv->rcv.bytes[i]);
      printf(" >\n");
    }

    priv->rval=(priv->rcv.bytes[1] << 8 ) | priv->rcv.bytes[2];
    if(priv->tpro == 1)
      printf("Returning value of %d\n", priv->rval);

    // Now stop the converter
    chipSelect(priv->chipSelect);
    status = doSPITransaction8(priv->server->handle, sizeof(stopCmd), stopCmd, priv->rcv.bytes );
    if(status < 0)
    {
        printf(" doSPITransaction8 to %s[%d] failed (5)\n",priv->pv->name, priv->server->handle);
        exit(1);
    }

    // Now that I'm done, unlock the channel so someone else can use it.
    chipSelect(15);
    // Release the resource
    epicsMutexUnlock(priv->server->plock);

    if(priv->tpro == 1)
      printf("Unlocked\n");

    // Show I'm done
    callbackRequest(&priv->cb);

  } while(TRUE);
}


/* ============================================================================================
 *
 * Here is the driver that outputs a string of bytes (and recieves a response simultaneously),
 * and then returns a selected byte from the response.
 *
 * ============================================================================================ */

static void Read8_Driver(void *ptr)
{
  struct privateData *priv = ptr;
  int status;

  do
  {
    // Sleep till requested to run
    epicsThreadSuspendSelf();

    // lock the resource
    epicsMutexMustLock(priv->server->plock);

    // Log the message going out if in verbose mode
    if(priv->tpro == 1)
    {
      printf("\nRead8_Driver\nWriting(8) 0x%1x < ",priv->chipSelect);
      for(uint32_t i=0; i<priv->len; i++)
        printf("%02x ", priv->xmt.bytes[i]);
      printf(" >\n");
    }

    // Select the chip and do the transaction
    chipSelect(priv->chipSelect);

    status = doSPITransaction8(priv->server->handle, priv->len*sizeof(uint8_t), priv->xmt.bytes, priv->rcv.bytes );
    if(status < 0)
    {
        printf(" doSPITransaction8 to %s[%d] failed (6)\n",priv->pv->name, priv->server->handle);
        exit(1);
    }

    // Log the message I got back if in verbose mode
    if(priv->tpro == 1)
    {
      printf("Reading(8) 0x%1x < ",priv->chipSelect);
      for(uint8_t i=0; i<priv->len ; i++)
        printf("%02x ", priv->rcv.bytes[i]);
      printf(" >\n");
    }

    // Get data to return
    if(priv->tpro == 1)
      printf("Returning value of %x\n",priv->rcv.bytes[priv->idx]);
    priv->rval = priv->rcv.bytes[priv->idx];

    if(priv->tpro == 1)
      printf("Unselecting chip\n");
    chipSelect(15);

    // Release the resource
    epicsMutexUnlock(priv->server->plock);

    // Show I'm done
    callbackRequest(&priv->cb);
  }
  while(TRUE);
}

/* ============================================================================================
 *
 * This driver outputs a string of bytes (and recieves a response simultaneously),
 * and then returns a selected WORD from the response.
 *
 * ============================================================================================ */

static void Read8_16_Driver(void *ptr)
{
  struct privateData *priv = ptr;
  int status;

  do
  {
    // Wait till service requested
    epicsThreadSuspendSelf();
    // get the resource
    epicsMutexMustLock(priv->server->plock);

    // Log the message going out if in verbose mode
    if(priv->tpro == 1)
    {
      printf("\nRead8.16 Driver\nWriting(8.16) 0x%1x < ",priv->chipSelect);
      for(uint32_t i=0; i<priv->len; i++)
        printf("%02x ", priv->xmt.bytes[i]);
      printf(" >\n");
    }

    // Select the chip and do the transaction
    chipSelect(priv->chipSelect);

    // Output the bytes, but receive the words
    status = doSPITransaction8(priv->server->handle, priv->len*sizeof(uint8_t), priv->xmt.bytes, priv->rcv.bytes );
    if(status < 0)
    {
        printf(" doSPITransaction8.16 to %s[%d] failed (1)\n",priv->pv->name, priv->server->handle);
        exit(1);
    }

    // Log the message I got back if in verbose mode
    if(priv->tpro == 1)
    {
      printf("Reading(8.16) 0x%1x < ",priv->chipSelect);
      for(uint8_t i=0; i<priv->len ; i++)
        printf("%02x ", priv->rcv.bytes[i]);
      printf(" >\n");
    }

    // If supposed to put data back, get it
    if(priv->indexsupplied)
    {
      priv->rval = priv->rcv.bytes[priv->idx] << 8 | priv->rcv.bytes[priv->idx+1];
      if(priv->tpro == 1)
        printf("Returning value of [%d] %x\n",priv->idx, priv->rval);
    }

    if(priv->tpro == 1)
      printf("Unselecting chip\n");
    chipSelect(15);

    // Release the resource
    epicsMutexUnlock(priv->server->plock);

    // Show I'm done
    callbackRequest(&priv->cb);
  }
  while(TRUE);
}

/* ============================================================================================
 *
 * OK, in the case of the AMC7823, reading the analog channels also returns the channel ID in the most
 * significant 3 bits of the response.  Good for figuring out how to get it working, but causes all
 * sorts of problems trying to mask off 12 bits in EPICS.
 *
 * This routine first outputs a string of 16 bit words (and recieves a response simultaneously),
 * and then gets the word you request and returns it, after masking it to 12 bits.
 *
 * ============================================================================================ */

static void Read12_Driver(void *ptr)
{
  int status;

  struct privateData *priv = ptr;
  do
  {
    // Wait till service requested
    epicsThreadSuspendSelf();
    // Release the resource
    epicsMutexMustLock(priv->server->plock);
    // Select the chip and do the transaction
    chipSelect(priv->chipSelect);

    // Log the message going out if in verbose mode
    if(priv->tpro == 1)
    {
      printf("Read_12 Driver\nWriting(12) > ");
      for(uint32_t i=0; i<priv->len; i++)
        printf("%04x ", priv->xmt.words[i]);
      printf("\n");
    }

    status = doSPITransaction16(priv->server->handle, priv->len*sizeof(uint16_t), priv->xmt.words, priv->rcv.words );
    if(status < 0)
    {
        printf(" doSPITransaction12 to %s[%d] failed (1)\n",priv->pv->name, priv->server->handle);
        exit(1);
    }

    // Log the message going out if in verbose mode
    if(priv->tpro == 1)
    {
      printf("Reading(12) < ");
      for(uint8_t i=0; i<priv->len ; i++)
        printf("%04x ", priv->rcv.words[i]);
      printf("\n");
    }

    // If supposed to put data back, get it
    if(priv->idx)
      priv->rval = priv->rcv.words[priv->idx] & 0xfff;

    chipSelect(15);

    // Release the resource
    epicsMutexUnlock(priv->server->plock);

    // Show I'm done
    callbackRequest(&priv->cb);
  }
  while(TRUE);
}


/* ============================================================================================
 *
 * Next routine.  This driver outputs a string of 16 bit words (and recieves a response simultaneously),
 * and then returns a selected word from the response.
 *
 * ============================================================================================ */

static void Read16_Driver(void *ptr)
{
  int status;
  struct privateData *priv = ptr;
  do
  {
    // Wait till requested
    epicsThreadSuspendSelf();
    // Grab the resource
    epicsMutexMustLock(priv->server->plock);

    // Select the chip and do the transaction
    chipSelect(priv->chipSelect);

    // Log the message going out if in verbose mode
    if(priv->tpro == 1)
    {
      printf("Read16 Driver\nWriting(16) > ");
      for(uint32_t i=0; i<priv->len; i++)
        printf("%04x ", priv->xmt.words[i]);
      printf("\n");
    }

    status = doSPITransaction16(priv->server->handle, priv->len*sizeof(uint16_t), priv->xmt.words, priv->rcv.words );
    if(status < 0)
    {
        printf(" doSPITransaction16 to %s[%d] failed (1)\n",priv->pv->name, priv->server->handle);
        exit(1);
    }

    // Log the message going out if in verbose mode
    if(priv->tpro == 1)
    {
      printf("Reading(16) < ");
      for(uint8_t i=0; i<priv->len ; i++)
        printf("%04x ", priv->rcv.words[i]);
      printf("\n");
    }

    // If supposed to put data back, get it
    if(priv->indexsupplied)
      priv->rval = priv->rcv.words[priv->idx];


    chipSelect(15);
    // Release the resource
    epicsMutexUnlock(priv->server->plock);

    // Show I'm done
    callbackRequest(&priv->cb);
  }
  while(TRUE);
}

/* ============================================================================================
 *
 * Next routine.  This driver modifies a byte in the output string using the value you requested, and
 * the index of where to place it.  It then outputs the string as 8 bit bytes and returns.
 *
 * ============================================================================================ */

static void Write8_Driver(void *ptr)
{
  struct privateData *priv = ptr;
  do
  {
    // Wait till I'm needed
    epicsThreadSuspendSelf();

    // Get the resource
    epicsMutexMustLock(priv->server->plock);

    if(priv->tpro == 1)
      printf("\nWrite 8\n");
    // If I'm supposed to put data in the record, put it here
    if(priv->indexsupplied)
      priv->xmt.bytes[priv->idx] = priv->rval;

    // Log the message going out if in verbose mode
    if(priv->tpro == 1)
    {
      printf("\nWriting(8) 0x%01x> ",priv->chipSelect);
      for(uint32_t i=0; i<priv->len; i++)
        printf("%02x ", priv->xmt.bytes[i]);
      printf("\n");
    }

    // Select the chip and do the transaction
    chipSelect(priv->chipSelect);
    doSPITransaction8(priv->server->handle, priv->len, priv->xmt.bytes, priv->rcv.bytes );

    // Log the message I got back if in verbose mode
    if(priv->tpro == 1)
    {
      printf("Got back(8) 0x%1x < ",priv->chipSelect);
      for(uint8_t i=0; i<priv->len ; i++)
        printf("%02x ", priv->rcv.bytes[i]);
      printf(" >\n");
    }

    // Get data to return
    if(priv->tpro == 1)
      printf("Returning value of %x\n",priv->rcv.bytes[priv->idx]);
    priv->rval = priv->rcv.bytes[priv->idx];

    chipSelect(15);
    // Release the resource
    epicsMutexUnlock(priv->server->plock);

    // Show I'm done
    callbackRequest(&priv->cb);
  }
  while(TRUE);
}

/* ============================================================================================
 *
 * Next routine.  This driver modifies a 16 bit word in the output string using the value you requested, and
 * the index of where to place it.  It then outputs the string as 16 bit words and returns.
 *
 * ============================================================================================ */

static void Write16_Driver(void *ptr)
{
  struct privateData *priv = ptr;
  do
  {
    // Wait till requested
    epicsThreadSuspendSelf();
    // Get the resource
    epicsMutexMustLock(priv->server->plock);

    // If I'm supposed to put data in the record, put it here
    if(priv->indexsupplied)
      priv->xmt.words[priv->idx] = priv->rval;

    // Log the message going out if in verbose mode
    if(priv->tpro == 1)
    {
      printf("Writing(16) > ");
      for(uint32_t i=0; i<priv->len; i++)
        printf("%04x ", priv->xmt.words[i]);
      printf("\n");
    }

    // Select the chip and do the transaction
    chipSelect(priv->chipSelect);
    doSPITransaction16(priv->server->handle, priv->len*sizeof(uint16_t), priv->xmt.words, priv->rcv.words );

    // Log the message going out if in verbose mode
    if(priv->tpro == 1)
    {
      printf("Got back(16) < ");
      for(uint8_t i=0; i<priv->len ; i++)
        printf("%04x ", priv->rcv.words[i]);
      printf("\n");
    }

    // If supposed to put data back, get it
    if(priv->indexsupplied)
      priv->rval = priv->rcv.words[priv->idx];

    chipSelect(15);
    // Release the resource
    epicsMutexUnlock(priv->server->plock);

    // Show I'm done
    callbackRequest(&priv->cb);
  }
  while(TRUE);
}



// List of defined device Names
typedef struct deviceDriverEntry
{
	char *driverName;
	EPICSTHREADFUNC driver;
    uint8_t type;     // 0=words, 1 = bytes, 2 = special (reserve words)
} deviceDriverEntry;

static deviceDriverEntry deviceDrivers[] =
{
  {"GPIOWrite", GPIOWrite, 1},
  {"ADS114READ", ADS114Read_Driver,1 },
  {"READ8", Read8_Driver,1 },
  {"READ8.16", Read8_16_Driver,2},
  {"READ12", Read12_Driver,0 },
  {"READ16", Read16_Driver,0 },
  {"WRITE8", Write8_Driver,1},
  {"WRITE16", Write16_Driver,0},
  {NULL, NULL}
};



// Actually starting to get down into the meat of this thing now.

/*
 * OK, this is where some of the magic starts to happen.
 *
 * This is where you define device definitions, which are defined as:
 *
 * "@deviceName, -c chipselect -i index functionName hex-data,hex-data,..."
 * in this case, 
 * deviceName = Device to use
 * chipSelect = which chip select to generate for this access.
 * index      = where in the record any data will be stored/accessed
 * function   = What am I doing.  Right now, I think all I need is "read16" and "write16" to read and write unsigned 16-bit registers
 * "hex-data,..." The rest of the items on this list is the actual HEX data to be sent to the device
 *
 * for instance,
 *  parseLink("device -c 0 -i 1 read16 0x0a0a0, 0x0")
 * This tells you to communicate with SPI channel <device>
 *  using chip-select <0>
 *  index for value is <1> - Note that this entry is optional.  You may just want to send a static message out
 *  function to use for the communication is <read16>
 *  and the command to send out for the read is <0xa0a0, 0x0000>
 *
 * Note in most cases, the actual data will be a single byte or word string.
 * Remember that SPI is full duplex, so to get data back, you have to add a pad word
 *
 * Currently the defined records are going to be:
 * GPIOWrite - output a bit to the actual GPIO
 * ADS114Read - Read a variable from the ADS114
 * read16 - Send out the data and expect to get a 16 bit response in the proper location
 * write16 - move the private rval into the proper indexed location, and send out the data.
 * read8    - Send out a string of 8-bit bytes, and expect to get an 8-bit response in the proper location
 * write8   - Move the private rval into the proper index location, and send out the data
 */
	
static int parseLink(const char *linkLine, struct privateData *priv)
{
  uint8_t i;
  char *p, *p1;											// Pointer to data link.

  // Initialize the private structure, to make sure there are no undefined items.
  priv->device=NULL;
  priv->server=NULL;
  priv->driver=NULL;
  priv->chipSelect=15;
  priv->indexsupplied=0;
  priv->idx=0;
  priv->rval=0;
  priv->dataType=0;
  priv->len=0;
  priv->xmt.words=NULL;
  priv->rcv.words=NULL;

//  printf("linkLine = \"%s\"\n",linkLine);
  // Find where the device name ends (it may end with a simple space, or a comma)
  p1=p=strpbrk(linkLine, " ,");
  priv->device = strndup(linkLine, p-linkLine);

  // Link to the device
  priv->server = findDevice(priv->device);

  // now, p points to the byte after the device name.

  while(p && *p)
  {
    // Move to next token in the string
    while ((*p ==',' || *p == ' ') && *p )  // Skip whitespace
      p++;

    // If this is one of my options, handle it.
    if(*p == '-' && (*(p+1) == 'c' || *(p+1) == 'C' || *(p+1) == 'i' || *(p+1) == 'I') )
    {
      p++;
      switch(*p++)
      {
        // Get the Chip Select to use
        case 'c':
        case 'C':
          priv->chipSelect = strtol(p, &p, 0);                // Read a integer from the data stream and skip over it.
          break;

        // Get the index of where the DATA goes
        case 'i':
        case 'I':
          priv->indexsupplied = 1;
          priv->idx = strtol(p, &p, 0);
          break;
      }
    }
    else
    {
      // First valid string in this line should be the function to use ('READ16', 'write16', 'READ8', 'write8'... case doesn't matter)
      p1=strpbrk(p, " ,");
      if(!p1)
        p1=p+strlen(p);
      priv->driverName=strndup(p, p1-p);
      // So, now p2 contains the name of the function to use.
      // See if I can find the right entry in the table
      for(i=0; i<(sizeof(deviceDrivers)/sizeof(deviceDriverEntry)); i++)
      {
        if( (strlen(priv->driverName) == strlen(deviceDrivers[i].driverName) ) && (!strcasecmp(priv->driverName, deviceDrivers[i].driverName) ) )
        {
          // found it
          priv->driver=deviceDrivers[i].driver;
          priv->dataType = deviceDrivers[i].type;
          break;
        }
        else
        {
          if(deviceDrivers[i].driverName == NULL || deviceDrivers[i].driver == NULL)
          {
            printf("Ran off end of valid table at index %d\n",i);
            break;
          }
        }
      }

      if(!priv->driver)
        printf("Couldn't find driver \"%s\"\n",priv->driverName);

      break;
    }
  }
  // So... P1 points to break character now.
  p=p1;
  // Pass over whitespace
  while ((*p ==',' || *p == ' ') && *p )  // Skip whitespace
    p++;

  // OK, now if the character I'm pointing to is NOT a option character, its the list of bytes to send to the device
  if(*p != '-' && *p)
  {
    // Start pulling in the bytes to send to the device.  They can be separated by whitespace or a comma.
    // first, lets allocate a buffer (both BYTE and WORD) the remaining size of the line.  I'll resize it later
    if(priv->dataType == 0)
      priv->xmt.words = (uint16_t *) callocMustSucceed(sizeof(uint16_t), strlen(p), "Couldn't allocate WORD buffer");
    else
      priv->xmt.bytes = (uint8_t *)  callocMustSucceed(sizeof(uint8_t), strlen(p), "Couldn't allocate BYTE buffer");

    // Now, start pulling in words(bytes) until I either run to the end of the line, or I come across something that isn't valid
    for(priv->len=0; *p; priv->len++)
    {
      // Lets assume I'm pointing at the start of a valid number
      if(priv->dataType == 0)
        priv->xmt.words[priv->len] = strtol(p, &p, 0);
      else
        priv->xmt.bytes[priv->len] = strtol(p, &p, 0);

      //OK, valid conditions at this point are either end-of-string, a comma, or whitespace
      while ((*p ==',' || *p == ' ') && *p )  // Skip whitespace
        p++;

      // Now, I'm pointing to the character -after- the whitespace
      // If it's an 'option' string (-c or -i) skip out of this (what are the options doing at tne end???)
      if(*p == '-' && (*(p+1) == 'c' || *(p+1) == 'C' || *(p+1) == 'i' || *(p+1) == 'I') )
        break;  // break out of the loop
    }
  }

  // OK, I've got all the data to transmit, lets resize the buffer down to what is needed.
  if(priv->len)
  {
    if(priv->dataType == 0)
      priv->xmt.words = (uint16_t *) realloc(priv->xmt.words, sizeof(uint16_t)*priv->len);
    else
      priv->xmt.bytes = (uint8_t *) realloc(priv->xmt.bytes, sizeof(uint8_t)*priv->len);

    // Finally, allocate a buffer for the received data
    if(priv->dataType == 0)
    {
      priv->rcv.words = (uint16_t *) callocMustSucceed(sizeof(uint16_t), priv->len, "Rcv Buffer Allocation Failed");
    }
    else
    {
      if(priv->dataType == 1)
      {
        priv->rcv.bytes = (uint8_t *) callocMustSucceed(sizeof(uint8_t), priv->len, "Rcv Buffer Allocation Failed");
      }
      else
      {
        if(priv->dataType == 2)
        {
          priv->rcv.bytes = (uint8_t *) callocMustSucceed(sizeof(uint8_t), 2*priv->len, "Rcv Buffer Allocation Failed");
        }
      }
    }
  }
  // At this point, I should have the transmit buffer filled (with bytes or words...),
  //  the length I'm tranmitting set
  // and the receive buffer allocated

  return 0;
}


/* =========================================================================================
 *
 * Start of the actual db routines (as opposed to the support bits)
 *
 * ========================================================================================= */



// Lets start with the callback function here
static void aiCallback(CALLBACK *cb)
{
  aiRecord* prec;             // Pointer to this specific record
  struct privateData* priv;   // Private Data pointer for this
  rset* prset;

  callbackGetUser(prec,cb);
  prset=(rset*)prec->rset;
  priv=prec->dpvt;

  if(prec->tpro)
    printf("Inside Callback\n");

  dbScanLock((dbCommon*)prec);    // Lock this record
  prec->rval=priv->rval;          // update the rval
  (*prset->process)((dbCommon *) prec);   // Now, lets process this record
  dbScanUnlock((dbCommon *)prec); // all done.
}

// Initialize the AI record to get it prepared to use.
long init_AI(aiRecord *prec)
{
  struct privateData* priv;
  unsigned long start;
  char msg[80];

  /* do all the initialization stuff I need for an AI. */
  // Allocate a private data structure for this record and link it to the record
  prec->dpvt=priv=callocMustSucceed(1, sizeof(struct privateData), "init_AI Out of memory");

  // Add a pointer back to the record
  priv->pv = (dbCommon *) prec;

  // Now, update all the link information for this I/O point
  parseLink(prec->inp.value.instio.string,  priv);

  // If this is a constant, set it up
  recGblInitConstantLink(&prec->inp,DBF_ULONG,&start);

  // Setup for background completion
  switch(prec->inp.type)
  {
    case (INST_IO):
      callbackSetCallback(aiCallback, &priv->cb);
      callbackSetPriority(priorityLow, &priv->cb);
      callbackSetUser(prec, &priv->cb);
      break;

    default:
      snprintf(msg, sizeof(msg),"DevAiTestAsyn(int_record) Illegal Input Field(%d)\n",prec->inp.type);
      recGblRecordError(S_db_badField, (void *) prec, msg);
      return(S_db_badField);
      break;
  }

  // Start the function up now
  priv->driverThread = epicsThreadCreate(prec->name, epicsThreadPriorityHigh, epicsThreadGetStackSize(epicsThreadStackBig), priv->driver, priv);

  return 0;
}

long read_AI(aiRecord *prec)
{
  struct privateData* priv=prec->dpvt;

  if(!priv)
  {
    (void)recGblSetSevr(prec, COMM_ALARM, INVALID_ALARM);
    return 0;
  }


  if( ! prec->pact )
  {
    /* start async operation */
    priv->tpro=prec->tpro;
    epicsThreadResume(priv->driverThread);
    prec->pact=TRUE;
    return 0;
  }
  else
  {
    /* complete operation */
    prec->pact=FALSE;
  }
  return 0;
}

struct
{
  long num;
  DEVSUPFUN  report;
  DEVSUPFUN  init;
  DEVSUPFUN  init_record;
  DEVSUPFUN  get_ioint_info;
  DEVSUPFUN  read_ai;
  DEVSUPFUN  special_linconv;
} SPIdigAI = {
  6, /* space for 6 functions */
  NULL,
  NULL,
  init_AI,
  NULL,
  read_AI,
  NULL
};
epicsExportAddress(dset,SPIdigAI);




// Lets start with the callback function here
static void aoCallback(CALLBACK *cb)
{
  aoRecord* prec;             // Pointer to this specific record
  struct privateData* priv;   // Private Data pointer for this
  rset* prset;

  callbackGetUser(prec,cb);
  prset=(rset*)prec->rset;
  priv=prec->dpvt;

  if(prec->tpro)
    printf("Inside Callback\n");

  dbScanLock((dbCommon*)prec);    // Lock this record
  prec->rval=priv->rval;          // update the rval
  (*prset->process)((dbCommon *) prec);   // Now, lets process this record
  dbScanUnlock((dbCommon *)prec); // all done.
}

// Initialize the AO record to get it prepared to use.
static long init_AO(aoRecord *prec)
{
  struct privateData* priv;
  unsigned long start;
  char msg[80];

  /* do all the initialization stuff I need for an AO.
   *
   */

  // Allocate a private data structure for this record and link it to the record
  prec->dpvt = priv = callocMustSucceed(1, sizeof(struct privateData), "init_AO Out of memory");

  // Add a pointer back to the record
  priv->pv = (dbCommon *) prec;

  // Now, update all the link information for this I/O point
  parseLink(prec->out.value.instio.string,  priv);

  // If this is a constant, set it up
  recGblInitConstantLink(&prec->out,DBF_ULONG,&start);

  // Setup for background completion
  switch(prec->out.type)
  {
    case (INST_IO):
      callbackSetCallback(aoCallback, &priv->cb);
      callbackSetPriority(priorityLow, &priv->cb);
      callbackSetUser(prec, &priv->cb);
      break;

    default:
      snprintf(msg, sizeof(msg),"DevAoTestAsyn(int_record) Illegal Input Field(%d)\n",prec->out.type);
      recGblRecordError(S_db_badField, (void *) prec, msg);
      return(S_db_badField);
      break;
  }

  // Start the driver function up now
  priv->driverThread = epicsThreadCreate(prec->name, epicsThreadPriorityHigh, epicsThreadGetStackSize(epicsThreadStackBig), priv->driver, priv);

  return 0;
}



// Handle an AO record
//
static long write_AO(aoRecord *prec)
{
  struct privateData* priv=prec->dpvt;

  if(!priv)
  {
    (void)recGblSetSevr(prec, COMM_ALARM, INVALID_ALARM);
    return 0;
  }
  if( !prec->pact)
  {
    /* Start the async output operations */
    priv->tpro=prec->tpro;
	priv->rval = prec->rval;
    prec->pact=TRUE;
    epicsThreadResume(priv->driverThread);
    return 0;
  }
  else
  {
    /* Complete the operations */
    prec->pact=FALSE;
  }
  return(0);
}

struct {
  long num;
  DEVSUPFUN  report;
  DEVSUPFUN  init;
  DEVSUPFUN  init_record;
  DEVSUPFUN  get_ioint_info;
  DEVSUPFUN  read_ai;
  DEVSUPFUN  special_linconv;
} SPIdigAO = {
  6, /* space for 6 functions */
  NULL,
  NULL,
  init_AO,
  NULL,
  write_AO,
  NULL
};
epicsExportAddress(dset,SPIdigAO);






// Lets start with the callback function here
static void mbbiDirectCallback(CALLBACK* cb)
{
  mbbiDirectRecord* prec;
  struct privateData* priv;
  rset* prset;

  callbackGetUser(prec,cb);
  prset=(rset*)prec->rset;
  priv=prec->dpvt;

  dbScanLock((dbCommon*)prec);
  prec->rval=priv->rval;
  (*prset->process)((dbCommon *) prec);
  dbScanUnlock((dbCommon *) prec);
}

// Initialize the MBBI record to get it prepared to use.
static long init_MBBIDirect(mbbiDirectRecord *prec)
{
  struct privateData* priv;
  unsigned long start;

  /* do all the initialization stuff I need for an AI. */
  // Allocate a private data structure for this record and link it to the record
  prec->dpvt=priv=callocMustSucceed(1, sizeof(struct privateData), "init_mbbiDirect Out of memory");

  // Add a pointer back to the record
  priv->pv = (dbCommon *) prec;

  // Now, update all the link information for this I/O point
  parseLink(prec->inp.value.instio.string,  priv);

  // If this is a constant, set it up
  recGblInitConstantLink(&prec->inp,DBF_ULONG,&start);

  // Setup for background completion
  switch(prec->inp.type)
  {
    case (INST_IO):
      callbackSetCallback(mbbiDirectCallback, &priv->cb);
      callbackSetPriority(priorityLow, &priv->cb);
      callbackSetUser(prec, &priv->cb);
      break;

    default:
      recGblRecordError(S_db_badField, (void *) prec, "DevmbbiDirectiTestAsyn(int_record) Illegal Input Field");
      return(S_db_badField);
      break;
  }

  // Start the function up now
  priv->driverThread = epicsThreadCreate(prec->name, epicsThreadPriorityHigh, epicsThreadGetStackSize(epicsThreadStackBig), priv->driver, priv);
  return 0;
}

static long read_MBBIDirect(mbbiDirectRecord *prec)
{
  struct privateData* priv=prec->dpvt;

  if(!priv)
  {
    (void)recGblSetSevr(prec, COMM_ALARM, INVALID_ALARM);
    return 0;
  }

  priv->tpro=prec->tpro;

  if( ! prec->pact )
  {
    /* start async operation */
    prec->pact=TRUE;
    callbackSetUser(prec,&priv->cb);
    epicsThreadResume(priv->driverThread);
  }
  else
  {
    /* complete operation */
    prec->pact=FALSE;
  }
  return 0;
}

struct {
  long num;
  DEVSUPFUN  report;
  DEVSUPFUN  init;
  DEVSUPFUN  init_record;
  DEVSUPFUN  get_ioint_info;
  DEVSUPFUN  read_mbbiDirect;
  DEVSUPFUN  special_linconv;
} SPIdigMBBIDirect = {
  6, /* space for 6 functions */
  NULL,
  NULL,
  init_MBBIDirect,
  NULL,
  read_MBBIDirect,
  NULL
};
epicsExportAddress(dset,SPIdigMBBIDirect);


/* **********************************************************************************************
 * Code for MBBODirect
 * *********************************************************************************************/

// Lets start with the callback function here
static void mbboDirectCallback(CALLBACK *cb)
{
  mbboDirectRecord *prec;             // Pointer to this specific record
  struct privateData *priv;   // Private Data pointer for this
  rset* prset;

  callbackGetUser(prec,cb);
  prset=(rset*)prec->rset;
  priv=prec->dpvt;

  if(prec->tpro)
    printf("Inside Callback\n");

  dbScanLock((dbCommon*)prec);    // Lock this record
  prec->rval=priv->rval;          // update the rval
  (*prset->process)((dbCommon *) prec);   // Now, lets process this record
  dbScanUnlock((dbCommon *)prec); // all done.
}


// Initialize the MBBO record to get it prepared to use.
static long init_MBBODirect(mbboDirectRecord *prec)
{
  struct privateData* priv;
  unsigned long start;
  char msg[80];

  /* do all the initialization stuff I need for an MBBODirect.
   *
   */

  // Allocate a private data structure for this record and link it to the record
  prec->dpvt = priv = callocMustSucceed(1, sizeof(struct privateData), "init_MBBODirect Out of memory");

  // Add a pointer back to the record
  priv->pv = (dbCommon *) prec;

  // Now, update all the link information for this I/O point
  parseLink(prec->out.value.instio.string,  priv);

  // If this is a constant, set it up
  recGblInitConstantLink(&prec->out,DBF_ULONG,&start);

  // Setup for background completion
  switch(prec->out.type)
  {
    case (INST_IO):
      callbackSetCallback(mbboDirectCallback, &priv->cb);
      callbackSetPriority(priorityLow, &priv->cb);
      callbackSetUser(prec, &priv->cb);
      break;

    default:
      snprintf(msg, sizeof(msg),"DevMbboDirectTestAsyn(int_record) Illegal Input Field(%d)\n",prec->out.type);
      recGblRecordError(S_db_badField, (void *) prec, msg);
      return(S_db_badField);
      break;
  }

  // Start the function up now
  priv->driverThread = epicsThreadCreate(prec->name, epicsThreadPriorityHigh, epicsThreadGetStackSize(epicsThreadStackBig), priv->driver, priv);

  return 0;
}


// Handle a MBBO record
//
static long write_MBBODirect(mbboDirectRecord *prec)
{
  struct privateData* priv=prec->dpvt;

  if(!priv)
  {
    (void)recGblSetSevr(prec, COMM_ALARM, INVALID_ALARM);
    return 0;
  }
  if(!prec->pact)
  {
    /* Start the async output operations */
    priv->tpro=prec->tpro;
    priv->rval = prec->rval;
    prec->pact=TRUE;
    epicsThreadResume(priv->driverThread);
    return(0);
  }
  else
  {
    /* Complete the operations */
    prec->pact=FALSE;
  }
  return 0;
}


struct {
  long num;
  DEVSUPFUN  report;
  DEVSUPFUN  init;
  DEVSUPFUN  init_record;
  DEVSUPFUN  get_ioint_info;
  DEVSUPFUN  read_ai;
  DEVSUPFUN  special_linconv;
} SPIdigMBBODirect = {
  6, /* space for 6 functions */
  NULL,
  NULL,
  init_MBBODirect,
  NULL,
  write_MBBODirect,
  NULL
};
epicsExportAddress(dset,SPIdigMBBODirect);



#if NOTYET
// Initialize the BI record to get it prepared to use.
static long init_BI(biRecord *prec)
{
  struct privateData* priv;
  unsigned long start;

  priv=callocMustSucceed(1, sizeof(struct privateData), "init_BI Out of memory");
  if(!priv)
  {
    recGblRecordError(S_db_noMemory, (void*)prec, "devBI failed to allocate private Data structure");
    return S_db_noMemory;
  }

  prec->dpvt=priv;
  parseLink(prec->inp.value.instio.string, priv);

  recGblInitConstantLink(&prec->inp,DBF_ULONG,&start);

  return 0;
}

static long read_BI(biRecord *prec)
{
  struct privateData* priv=prec->dpvt;

  int err;
  int len;
  char bfr[64];

  if(!priv)
  {
    (void)recGblSetSevr(prec, COMM_ALARM, INVALID_ALARM);
    return 0;
  }

  len = sprintf(bfr,"  %s %d \r\n",priv->deviceType, priv->index);

  // Lock comm channel
  epicsMutexMustLock(priv->hid->plock);

  // Send out the command through LIBHID
  err=hid_write(priv->hid->hidHandle, (const unsigned char *) bfr, len );
  if (err >= 0)
  {
    // Now, read the response
    bfr[0]='\0';
    err=hid_read_timeout(priv->hid->hidHandle, (unsigned char *)bfr, sizeof(bfr), 1000);
    // printf("bi[%d] %d Bytes \"%s\"\n", priv->index, bfr[1], bfr+2);
    if(err >= 0)
    {
      priv->value = strtol(bfr+2, NULL, 0);

//      printf("Result was %2x\n",priv->value);

      prec->rval = priv->value;
    }
  }

  // Done, make available for someone else.
  epicsMutexUnlock(priv->hid->plock);

  return 0;
}


struct {
  long num;
  DEVSUPFUN  report;
  DEVSUPFUN  init;
  DEVSUPFUN  init_record;
  DEVSUPFUN  get_ioint_info;
  DEVSUPFUN  read_ai;
  DEVSUPFUN  special_linconv;
} USBdigBI = {
  6, /* space for 6 functions */
  NULL,
  NULL,
  init_BI,
  NULL,
  read_BI,
  NULL
};
epicsExportAddress(dset,USBdigBI);


#endif


// Initialize the BO record to get it prepared to use.
static long init_BO(boRecord *prec)
{
  struct privateData* priv;
  unsigned long start;

  /* do all the initialization stuff I need for an MBBODirect.
   *
   */


  // Allocate a private data structure for this record and link it to the record
  prec->dpvt = priv = callocMustSucceed(1, sizeof(struct privateData), "init_BO Out of memory");

  // Add a pointer back to the record
  priv->pv =  (dbCommon *) prec;

  // Now, update all the link information for this I/O point
  parseLink(prec->out.value.instio.string,  priv);

  // If this is a constant, set it up
  recGblInitConstantLink(&prec->out,DBF_ULONG,&start);

  // Start the function up now
  priv->driverThread = epicsThreadCreate(prec->name, epicsThreadPriorityHigh, epicsThreadGetStackSize(epicsThreadStackBig), priv->driver, priv);
  return 0;
}

static long write_BO(boRecord *prec)
{
  struct privateData* priv=prec->dpvt;
  if(!priv)
  {
    (void)recGblSetSevr(prec, COMM_ALARM, INVALID_ALARM);
    return 0;
  }
  priv->tpro=prec->tpro;
  priv->rval = prec->rval;
  epicsThreadResume(priv->driverThread);

  return 0;
}

struct {
  long num;
  DEVSUPFUN  report;
  DEVSUPFUN  init;
  DEVSUPFUN  init_record;
  DEVSUPFUN  get_ioint_info;
  DEVSUPFUN  read_ai;
  DEVSUPFUN  special_linconv;
} SPIdigBO = {
  6, /* space for 6 functions */
  NULL,
  NULL,
  init_BO,
  NULL,
  write_BO,
  NULL
};
epicsExportAddress(dset,SPIdigBO);



/* EPICS driver support entry table */
// Doing this, cause I want to generate a report routine
typedef struct
{
	long number;
	long (* report) (int);
	long (* initDriver) ();
} drvType;

drvType drvSPI =
{
	2,
	report,
	initRoutine,
};

epicsExportAddress(drvet, drvSPI);


// This table should go at the end, as it references earlier functions
//

// Start of the command functions

// First function to define allows you to open a SPI device for use
// call consists of "addSPIDevice <name> <filename> <mode> <bits> <speed>"
// <name> is what the device is going to be called
// <filename> is the complete /dev/??? filename for this SPI channel
// <mode> is the mode to use on this SPI device (0,1,2,3)
// <bits> is the number of bits to transmit
// <speed> is the speed of the channel in Hz.

// void addSPIDevice(const char *name, const char *device, const uint8_t mode, const uint8_t bits, const uint32_t speed)
static const iocshArg addSPIDeviceArg0 = {"name", iocshArgString };
static const iocshArg addSPIDeviceArg1 = {"device", iocshArgString };
static const iocshArg addSPIDeviceArg2 = {"mode", iocshArgInt };
static const iocshArg addSPIDeviceArg3 = {"bits", iocshArgInt };
static const iocshArg addSPIDeviceArg4 = {"speed", iocshArgInt};
static const iocshArg *const addSPIDeviceArgs[5] = { &addSPIDeviceArg0, &addSPIDeviceArg1, &addSPIDeviceArg2,&addSPIDeviceArg3, &addSPIDeviceArg4 };
static const iocshFuncDef addSPIDeviceDef = {"addSPIDevice", 5, addSPIDeviceArgs };
static void addSPIDeviceCall(const iocshArgBuf * args)
{
	addSPIDevice(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}



// This table should go at the end, as it references earlier functions
//
// Commands to Register
void drvSPIRegister() {
	initRoutine();
	iocshRegister(&addSPIDeviceDef, addSPIDeviceCall);

}

epicsExportRegistrar(drvSPIRegister);

