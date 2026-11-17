
/** *****************************************************************************
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
#include <epicsTime.h>
#include <ellLib.h>

#include "gpio.h"
#include "SPIDriver.h"

#include "printReportBuffer.h"

#define MAX_STR 255
#define SPI_BUF_LEN 1024


// Global variables
uint8_t  cs_doNotDisturb;   // This chip is busy.  Don't bother it
uint32_t verbosityMask = 0;

// Heres the definition of the chain that contains all the servers that have been defined.
static struct deviceServer {
  struct deviceServer *next;
  char *name;
  int handle;
  uint8_t mode;
  uint8_t bits;
  uint32_t speed;
} *deviceServers = NULL;


/// Private data that the access routines use
struct privateData 
{
  ELLNODE   node;
  dbCommon  *pv;            // Pointer back to the record definition
  char *    device;         // Which device this is (name)
  struct    deviceServer *server; // Link to actual device data
  char *    driverName;     // Name of the driver I'm going to use
  uint8_t   act;            // This record is active
  CALLBACK  cb;             // Callback to execute when compete
  IOSCANPVT scan;           // update routine for this record
  uint8_t   tpro;           // Show debug data on this transaction
  uint8_t   chipSelect;     // ChipSelect to use
  uint8_t   altChipSelect;  // Alternate Chip Select (specific for ADS114 right now)
  uint8_t   indexsupplied;  // There was an index included in this command
  uint16_t  idx;            // Where to access raw data in xmt/rcv
  uint8_t   nmfr;           // Not My First Rodeo.  Handled the record before
  uint16_t  lastVal;        // Last value updated in the record
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
};

/// Here are the ancfhors for the I/O chains
ELLLIST gpioWriteChain;     /// Chain for GPIO stuff
ELLLIST read8Chain;         /// chain of 8-bit input devices
ELLLIST read12Chain;        /// chain of 12-bit input devices
ELLLIST read816Chain;       /// chain of 8.16-bit input devices
ELLLIST read16Chain;        /// chain of 16-bit input devices
ELLLIST ads114Chain;        /// chain of ADS114 input devices
ELLLIST write8Chain;        /// chain of 8-bit output devices
ELLLIST write816Chain;      /// Chain of 8.16-bit output devices
ELLLIST write16Chain;       /// chain of 16-bit output devices

/** *********************************************************************************
 * 
 * \brief Add this channel into the driver
 * 
 * On entry:
 * @param name   - Name for the device
 * @param device - File link for access
 * @param mode   - What SPI mode to use
 * @param bits   - How many bits for each character to transmit
 * @param speed  - How fast
 * ********************************************************************************/
static int addSPIDevice( const char *name, const char *device, const uint8_t mode, const uint8_t bits, const uint32_t speed)
{
  struct deviceServer *newServer;

  int fd = 0;

//  printf("Opening %s -> %s\n",name, device);
#define SPI_AVAIL
#ifdef SPI_AVAIL
  fd = openSPI(device, mode, bits, speed);
#endif
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
	newServer->handle = fd;
  }
  return(0);
}


/// Find a specific device server
///
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


/// Close the SPI devices
///
void closeSPIDevices()
{
  struct deviceServer *server;

  server = deviceServers;
  while(server)
  {
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

static void scannerThread(void *parm);
void startScannerThread()
{
  epicsThreadCreate("scannerThread", epicsThreadPriorityLow, epicsThreadGetStackSize(epicsThreadStackBig), &scannerThread, 0);
}

/** ****************************************************************************************************************************
 *
 * This routine runs during the initial state machine.
 * it allows operations during specific states to be executed
 * This is particularly useful during startup and shutdown when ou want
 * to add a specific function during a specific state.
 * For instance, during state initHookAtBeginning state, I load a set of local database records to provide statistics
 * and during state  initHookAterIocRuning I start the scanner thread to scan all the PV blocks
 *
 * @param state - specific state that EPICS in in at the moment.
 * *****************************************************************************************************************************/

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
        (void)startScannerThread();
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

// List of defined device Names
typedef struct deviceDriverEntry
{
	char *driverName;
    ELLLIST *thisPVType;
    uint8_t type;     // 0=words, 1 = bytes, 2 = special (reserve words)
} deviceDriverEntry;

static deviceDriverEntry deviceDrivers[] =
{
  {"GPIOWrite", &gpioWriteChain, 1},
  {"ADS114READ", &ads114Chain, 1 },
  {"READ8", &read8Chain, 1 },
  {"READ8.16", &read816Chain, 2},
  {"READ12", &read12Chain, 0 },
  {"READ16", &read16Chain, 0 },
  {"WRITE8", &write8Chain, 1},
  {"WRITE8.16",&write816Chain, 2},
  {"WRITE16", &write16Chain, 0},
  {NULL, NULL}
};

static uint8_t ADS114ReadStatus[] = {0x21, 0, 0};
static uint8_t ADS114readCmd[] = { 0x12, 0, 0};
static uint8_t ADS114stopCmd[] = {0xa};
static uint8_t ADS114readDRDY[] = {0,0};

/** **********************************************************************************
 * \brief IO Scanner Thread
 * This routine runs as a thread that is started upon completion of iocInit
 * Its purpose is to run through the records in the IO Chain lists, and execute operations when required.
 * This runs down the lists as fast as it can, so it executes all the reads and writes continuously
 * as listed in the chain.
 *
 * Now, Writes are handled before reads as they may effect the reads
 * */
void scannerThread(void *parm)
{
  ELLNODE *chain;
  ELLNODE *ads114Temp = ellFirst(&ads114Chain);
  uint8_t ads114Flag = 0;
  uint8_t counter = 0;
  struct timespec sleeper = {0, 100000000};

  do
  {
    nanosleep(&sleeper, NULL);

    if(verbosityMask & 1<<31)
      printf("Start of loop\n");

    // OK, I'm going to put all the routines inline here, and each chain will be executed completely (except for ADS114)
    // before moving to the next.  All outputs are up first, as they might affect the read operations later.

    /** ============================================================================================
     *
     * GPIO Output Chain
     * Currently handles output requests.
     *
     * ============================================================================================ */
    for(chain=ellFirst(&gpioWriteChain); chain!=NULL; chain=ellNext(chain))
    {
      struct privateData *priv = (struct privateData *)chain;

      if(priv->act != 0)
      {
        if(priv->tpro)
          printf("Working on active GPIOWrite record, \"%s\" value=%01x\n",priv->pv->name, priv->rval);

        if( GPIOOutput(priv->idx, priv->rval) < 0)
        {
          perror(" Inside __FILE__ Doing GPIOWrite Failed");
        }
        priv->act = 0;
      }
    }

    /* ============================================================================================
     *
     * Write8 chain
     * Writes are handled as callbacks, so that data can be returned to the caller.  They do not happen
     * based on Software Interrupts, so only callbacks are needed.
     *
     * ============================================================================================ */
    for(chain=ellFirst(&write8Chain); chain != 0; chain=ellNext(chain))
    {
      struct privateData  *priv = (struct privateData *)chain;
      if(priv->act !=0)
      {
        if(priv->tpro)
          printf("Working on active write8 record, \"%s\" value=%02x\n",priv->pv->name, priv->rval);

        // If I'm supposed to put data in the record, put it here
        if(priv->indexsupplied)
          priv->xmt.bytes[priv->idx] = priv->rval;

        // Select the chip and do the transaction
        chipSelect(priv->chipSelect);
        doSPITransaction8(priv->server->handle, priv->len, priv->xmt.bytes, priv->rcv.bytes );

        // Get data to return
        priv->rval = priv->rcv.bytes[priv->idx];

        chipSelect(15);

        // Show I'm done
        callbackRequest(&priv->cb);
        priv->act=0;
      }
    }

    /* ============================================================================================
     *
     * Write8.16 chain
     * This  routine handles the special occurance where I'm sending an 8-bit command, followed by 16-bit data (i.e. DAC8562).
     * Writes are handled as callbacks, so that data can be returned to the caller.  They do not happen
     * based on Software Interrupts, so only callbacks are needed.
     *
     * ============================================================================================ */
    for(chain=ellFirst(&write816Chain); chain != 0; chain=ellNext(chain))
    {
      struct privateData  *priv = (struct privateData *)chain;
      if(priv->act !=0)
      {
        if(priv->tpro)
          printf("Working on active write8.16 record, \"%s\" value=%04x\n",priv->pv->name, priv->rval);

        // If I'm supposed to put data in the record, put it here
        if(priv->indexsupplied)
        {
          priv->xmt.bytes[priv->idx+2] = priv->rval&255;
          priv->xmt.bytes[priv->idx+1] = priv->rval>>8;
        }

        // now, I have to swap the bytes of the first word, since I need to send starting with byte 1
        priv->xmt.bytes[1] = priv->xmt.bytes[0];

        // Log the message going out if in verbose mode
        if(priv->tpro != 0)
        {
          printf("Write8.16 Driver\nWriting(8) > ");
          for(uint32_t i=1; i<priv->len+1; i++)
            printf("%02x ", priv->xmt.bytes[i]);
          printf("\n");
        }

        // Select the chip and do the transaction
        chipSelect(priv->chipSelect);

        // OK, this is special.  I need to transmit from the byte pointer, but I'm getting a word back.
        doSPITransaction8(priv->server->handle, priv->len, priv->xmt.bytes+1, priv->rcv.bytes+1 );

        // Log the message going out if in verbose mode
        if(priv->tpro != 0)
        {
          printf("Received(8) > ");
          for(uint32_t i=1; i<priv->len+1; i++)
            printf("%02x ", priv->rcv.bytes[i]);
          printf("\n");
        }


        // Get data to return
        priv->rval = priv->rcv.bytes[priv->idx+2] | (priv->rcv.bytes[priv->idx+1] << 8);

        chipSelect(15);

        // Show I'm done
        callbackRequest(&priv->cb);
        priv->act=0;
      }
    }

    /* ============================================================================================
     *
     * Write16 chain
     * Writes are handled as callbacks, so that data can be returned to the caller.  They do not happen
     * based on Software Interrupts, so only callbacks are needed.
     *
     * ============================================================================================ */
    for(chain=ellFirst(&write16Chain); chain != 0; chain=ellNext(chain))
    {
      struct privateData  *priv = (struct privateData *)chain;
      if(priv->act != 0)
      {
        if(priv->tpro)
          printf("Working on active write16 record, \"%s\" value=%04x\n",priv->pv->name, priv->rval);

        // If I'm supposed to put data in the record, put it here
        if(priv->indexsupplied)
          priv->xmt.words[priv->idx] = priv->rval;

        // Select the chip and do the transaction
        chipSelect(priv->chipSelect);
        doSPITransaction16(priv->server->handle, priv->len*sizeof(uint16_t), priv->xmt.words, priv->rcv.words );


        // If supposed to put data back, get it
        if(priv->indexsupplied)
          priv->rval = priv->rcv.words[priv->idx];

        chipSelect(15);

        // Show I'm done
        callbackRequest(&priv->cb);
        priv->act=0;
      }
    }

    // All doue with the output routines, next lets do the routine to read8

    /* ============================================================================================
     *
     * Read8 handler
     * Here is the driver that outputs a string of bytes (and recieves a response simultaneously),
     * and then returns a selected byte from the response.
     *
     * These are handled as interrupt inputs, so processing is pretty quick.  The driver will repeatedly
     * go through the chain at full speed, and if the value returned changes, it will update the PV.
     *
     * ============================================================================================ */
    for(chain=ellFirst(&read8Chain); chain != 0; chain=ellNext(chain))
    {
      struct privateData  *priv = (struct privateData *)chain;
      int status;

      if(priv->chipSelect != cs_doNotDisturb)
      {
        if(priv->act!=0)
        {
          // Log the message going out if in verbose mode
          if(priv->pv->tpro != 0)
          {
            printf("\nRead8_Driver\nWriting(8)[0x%1x] < ",priv->chipSelect);
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

          // output idle chip select
          chipSelect(15);

          // Log the message I got back if in verbose mode
          if(priv->pv->tpro != 0)
          {
            printf("Reading(8)[0x%1x] < ",priv->chipSelect);
            for(uint8_t i=0; i<priv->len ; i++)
              printf("%02x ", priv->rcv.bytes[i]);
            printf(" >\n");
          }

          // Get data to return
          priv->rval = priv->rcv.bytes[priv->idx];

          // If the value changed, notify someone.
          if((priv->rval != priv->lastVal) || !priv->nmfr)
          {

            if((verbosityMask & 1<<0) & ellCount(&read8Chain))
              printf("Returning value of %x\n",priv->rcv.bytes[priv->idx]);

            // Show I'm done
            scanIoImmediate(priv->scan, priorityHigh);
            scanIoImmediate(priv->scan, priorityMedium);
            scanIoImmediate(priv->scan, priorityLow);
            priv->pv->tpro = 0;

            // update the last value that I sent
            priv->lastVal = priv->rval;
            priv->nmfr = 1;
          }
          //        priv->act=0;
        }
      }
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

    for(chain=ellFirst(&read12Chain); chain != 0; chain=ellNext(chain))
    {
      struct privateData  *priv = (struct privateData *)chain;
      int status;

      if(priv->chipSelect != cs_doNotDisturb)
      {
        if(priv->act!=0)
        {
          // Log the message going out if in verbose mode
          if(priv->tpro != 0)
          {
            printf("\nRead12_Driver\nWriting(12)[0x%1x] < ",priv->chipSelect);
            for(uint32_t i=0; i<priv->len; i++)
              printf("%04x ", priv->xmt.words[i]);
            printf(" >\n");
          }

          // Select the chip and do the transaction
          chipSelect(priv->chipSelect);

          status = doSPITransaction16(priv->server->handle, priv->len*sizeof(uint16_t), priv->xmt.words, priv->rcv.words );
          if(status < 0)
          {
            printf(" doSPITransaction12 to %s[%d] failed (6)\n",priv->pv->name, priv->server->handle);
            exit(1);
          }

          // output idle chip select
          chipSelect(15);

          // Log the message I got back if in verbose mode
          if(priv->tpro != 0)
          {
            printf("Reading(12)[0x%1x] < ",priv->chipSelect);
            for(uint8_t i=0; i<priv->len ; i++)
              printf("%04x ", priv->rcv.words[i]);
            printf(" >\n");
          }

          // Get data to return
          priv->rval = priv->rcv.words[priv->idx] & 0xfff;

          // If the value has changed, notify someone.
          if((priv->rval != priv->lastVal) || !priv->nmfr)
          {

            if((verbosityMask & 1<<1) && ellCount(&read12Chain))
              printf("Returning value of %x\n",priv->rcv.words[priv->idx]);

            // Show I'm done
            scanIoImmediate(priv->scan, priorityHigh);
            scanIoImmediate(priv->scan, priorityMedium);
            scanIoImmediate(priv->scan, priorityLow);
            priv->pv->tpro = 0;

            // Update the last value that I sent
            priv->lastVal = priv->rval;
            priv->nmfr = 1;
          }
          //        priv->act=0;
        }
      }
    }


    /* ============================================================================================
     *
     * This driver outputs a string of bytes (and recieves a response simultaneously),
     * and then returns a selected WORD from the response.
     *
     * ============================================================================================ */
    for(chain=ellFirst(&read816Chain); chain != 0; chain=ellNext(chain))
    {
      struct privateData  *priv = (struct privateData *)chain;
      int status;

      if(priv->chipSelect != cs_doNotDisturb)
      {
        if(priv->act != 0)
        {
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

          // If the value has changed, notify someone.
          if((priv->rval != priv->lastVal) || !priv->nmfr)
          {
            if((verbosityMask & 1<<2) && ellCount(&read816Chain))
              printf("Returning value of %x\n",priv->rcv.words[priv->idx]);

            // Show I'm done
            scanIoImmediate(priv->scan, priorityHigh);
            scanIoImmediate(priv->scan, priorityMedium);
            scanIoImmediate(priv->scan, priorityLow);
            priv->pv->tpro = 0;
            priv->nmfr = 1;

            // Update the last value that I sent
            priv->lastVal = priv->rval;
          }
          //        priv->act=0;
        }
      }
    }

    /* ============================================================================================
     * Read16
     * Here is the driver that outputs a string of words (and recieves a response simultaneously),
     * and then returns a selected word from the response.
     *
     * Again,
     * These are handled as interrupt inputs, so processing is pretty quick.  The driver will repeatedly
     * go through the chain at full speed, and if the value returned changes, it will update the PV.
     *
     * ============================================================================================ */
    for(chain=ellFirst(&read16Chain); chain != 0; chain=ellNext(chain))
    {
      struct privateData  *priv = (struct privateData *)chain;
      int status;

      if(priv->chipSelect != cs_doNotDisturb)
      {
        if(priv->act!=0)
        {
          // Log the message going out if in verbose mode
          if(priv->tpro != 0)
          {
            printf("\nRead16_Driver\nWriting(16)[0x%1x] < ",priv->chipSelect);
            for(uint32_t i=0; i<priv->len; i++)
              printf("%04x ", priv->xmt.words[i]);
            printf(" >\n");
          }

          // Select the chip and do the transaction
          chipSelect(priv->chipSelect);

          status = doSPITransaction16(priv->server->handle, priv->len*sizeof(uint16_t), priv->xmt.words, priv->rcv.words );
          if(status < 0)
          {
            printf(" doSPITransaction16 to %s[%d] failed (6)\n",priv->pv->name, priv->server->handle);
            exit(1);
          }

          // output idle chip select
          chipSelect(15);

          // Log the message I got back if in verbose mode
          if(priv->tpro != 0)
          {
            printf("Reading(16)[0x%1x] < ",priv->chipSelect);
            for(uint8_t i=0; i<priv->len ; i++)
              printf("%04x ", priv->rcv.words[i]);
            printf(" >\n");
          }

          // Get data to return
          priv->rval = priv->rcv.words[priv->idx];

          // If the value has changed, notify someone.
          if((priv->rval != priv->lastVal) || !priv->nmfr)
          {

            if((verbosityMask & 1<<3) && ellCount(&read16Chain))
              printf("Returning value of %x\n",priv->rcv.words[priv->idx]);

            // Show I'm done
            scanIoImmediate(priv->scan, priorityHigh);
            scanIoImmediate(priv->scan, priorityMedium);
            scanIoImmediate(priv->scan, priorityLow);
            priv->pv->tpro = 0;
            priv->nmfr = 1;

            // Update the last value that I sent
            priv->lastVal = priv->rval;
            priv->nmfr = 1;
          }
          //        priv->act=0;
        }
      }
    }



    /* ============================================================================================
     *
     * This driver is specific to the ADS114 A/D converter chip analog input channels,
     * This chip is somewhat slow, so of course, its more complicated.
     *
     * It will perform the following sequence of control
     * 1. If the ADS114Temp pointer is NULL, grab the first record of the ADS114 list and store it.
     * 2. If not null (meaning a record is in process), skip to step 6
     * 3. Output the byte string that is defined in the record I'm pointing to
     * 4. Output a 'START' command to start the conversion
     * 5. Skip to step 12 at the moment
     * 6. Check if I have DRDY valid. If not, skip to step 12 at the moment
     * 7. Once the device is ready (NOP returns a 0 status when done) output a RDATA command
     * 8. Get the result of the conversion from bytes 1 and 2 in the response (MSB first)
     * 9. Send out a 'STOP' command to stop conversion.
     * 10. Store the result
     * 11. Move to the next record, and loop back to step 1.
     * 12. continue on
     *
     * ============================================================================================ */

    // of course, this is different from the others.
    // Make sure there's something to do
    if(ellCount(&ads114Chain))
    {
      struct privateData  *priv = (struct privateData *)ads114Temp;

      uint8_t rcv[]={0,0,0,0,0,0,0,0,0,0};

      if((verbosityMask & 1<<4) && ellCount(&ads114Chain))
        printf("Working on  %s\n",priv->pv->name);

      // See if this record is disabled
      if(priv->pv->disa != priv->pv->disv)
      {
        if((verbosityMask & 1<<4) && ellCount(&ads114Chain))
          printf("Initial Entry on  %s\n",priv->pv->name);

        // Tell everyone else to leave this chip alone for the moment
        cs_doNotDisturb = chipSelect(priv->chipSelect);

        // Maybe I can go now
        chipSelect(priv->chipSelect);
        doSPITransaction8(priv->server->handle, sizeof(ADS114ReadStatus), ADS114ReadStatus, rcv );
        chipSelect(15);

        // See if the chip is out of Power On Reset
        if(!(rcv[2] & 0x80))
        {
          if((verbosityMask & 1<<4) && ellCount(&ads114Chain))
            printf("Out of POR  %s\n",priv->pv->name);

          // 1. See if I'm already busy
          if(!ads114Flag)
          {
            if((verbosityMask & 1<<4) && ellCount(&ads114Chain))
              printf("Initial Processing on %s\n",priv->pv->name);

            counter=0;
            // 3. Output the initial setup command
            // Log the message going out if in verbose mode
            if(priv->tpro != 0)
            {
              printf("\nADS114Read_Driver:\n Writing(8) cs=%1x <",priv->chipSelect);
              for(uint32_t i=0; i<priv->len; i++)
              {
                printf("%02x ", priv->xmt.bytes[i]);
              }
              printf(">\n");
            }
            // Note that the command string should have a START command appended to it.
            chipSelect(priv->chipSelect);
            doSPITransaction8(priv->server->handle, priv->len*sizeof(uint8_t), priv->xmt.bytes, priv->rcv.bytes );
            ads114Flag = 1;
          }
          else
          {
            if((verbosityMask & 1<<4) && ellCount(&ads114Chain))
              printf("Waiting for DRDY  %s\n",priv->pv->name);
            // 6. Wait till the conversion is complete
            // check the DRDY flag
            chipSelect(0);
            doSPITransaction8(priv->server->handle, sizeof(ADS114readDRDY), ADS114readDRDY, rcv);
            chipSelect(15);
            // If it's clear, I'm ready to get the data
            if(!rcv[0])
            {
              // 7. Once the conversion is complete, read the data, get the result, and then stop the conversion
              // 8. Get the data
              chipSelect(priv->chipSelect);
              doSPITransaction8(priv->server->handle, sizeof(ADS114readCmd), ADS114readCmd, priv->rcv.bytes );

              // 10. Store the result
              priv->rval=(priv->rcv.bytes[1] << 8 ) | priv->rcv.bytes[2];

              // 9. Now stop the converter
              chipSelect(priv->chipSelect);
              doSPITransaction8(priv->server->handle, sizeof(ADS114stopCmd), ADS114stopCmd, priv->rcv.bytes );
              // Now that I'm done, release the chip select
              chipSelect(15);

              // If the value has changed, notify someone.
              if((priv->rval != priv->lastVal) || !priv->nmfr)
              {

                // Show I'm done
                scanIoImmediate(priv->scan, priorityHigh);
                scanIoImmediate(priv->scan, priorityMedium);
                scanIoImmediate(priv->scan, priorityLow);
                priv->pv->tpro = 0;

                // Update the last value that I sent
                priv->lastVal = priv->rval;
                priv->nmfr = 1;
              }

              // Move down the list to the next record on the chain
              ads114Temp = ellNext(ads114Temp);
              if(ads114Temp == NULL)
              {
                ads114Temp = ellFirst(&ads114Chain);
              }

              ads114Flag = 0;
              cs_doNotDisturb = 15;
            }
          }
        }
        else
        {
          // Move down the list to the next record on the chain
          ads114Temp = ellNext(ads114Temp);
          if(ads114Temp == NULL)
          {
            ads114Temp = ellFirst(&ads114Chain);
          }

          ads114Flag = 0;
          cs_doNotDisturb = 15;

        }
        cs_doNotDisturb = 15;
      }
      else
      {
          // Reset the sequence, and wait for POR to go off
          ads114Flag = 0;
          cs_doNotDisturb = 15;
      }
    }

    // OK, done with everything, loop around for next pass.
   } while(1);
}



// Actually starting to get down into the meat of this thing now.

/* **********************************************************************************************************
 * \brief OK, this is where some of the magic starts to happen.
 *
 * This is where you define device definitions, which are defined as:
 *
 * "@deviceName, -c chipselect -i index functionName hex-data,hex-data,..."
 * in this case, 
 * @param deviceName = Device to use
 * @param chipSelect = which chip select to generate for this access.
 * @param altChipSelect = Alternate Chip Select for devices that need 2 chip selects.
 * @param index      = where in the record any data will be stored/accessed
 * @param function   = What am I doing.  Right now, I think all I need is "read16" and "write16" to read and write unsigned 16-bit registers
 * @param "hex-data,..." The rest of the items on this list is the actual HEX data to be sent to the device
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
  priv->chipSelect=15;
  priv->altChipSelect=15;
  priv->indexsupplied=0;
  priv->act=0;
  priv->idx=0;
  priv->rval=0;
  priv->dataType=0;
  priv->len=0;
  priv->xmt.words=NULL;
  priv->rcv.words=NULL;

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
    if(*p == '-' && (*(p+1) == 'c' || *(p+1) == 'C' || (*p+1) == 'd' || *(p+1) == 'D' || *(p+1) == 'i' || *(p+1) == 'I') )
    {
      p++;
      switch(*p++)
      {
        // Get the Chip Select to use
        case 'c':
        case 'C':
          priv->chipSelect = strtol(p, &p, 0);                // Read a integer from the data stream and skip over it.
          break;

        case 'd':
        case 'D':
          priv->altChipSelect = strtol(p, &p, 0);             // Read a integer from the data stream and skip over it.
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
        // now, put this definition in the right list
        if( (strlen(priv->driverName) == strlen(deviceDrivers[i].driverName) ) && (!strcasecmp(priv->driverName, deviceDrivers[i].driverName) ) )
        {
          // found it
          ellAdd(deviceDrivers[i].thisPVType, &priv->node);
          priv->dataType = deviceDrivers[i].type;
          break;
        }
        else
        {
          if(deviceDrivers[i].driverName == NULL || deviceDrivers[i].thisPVType == NULL)
          {
            printf("Ran off end of valid table at index %d\n",i);
            break;
          }
        }
      }
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
    if(priv->dataType == 1)
      priv->xmt.bytes = (uint8_t *)  callocMustSucceed(sizeof(uint8_t), strlen(p), "Couldn't allocate BYTE buffer");
    else
      priv->xmt.words = (uint16_t *) callocMustSucceed(sizeof(uint16_t), strlen(p), "Couldn't allocate WORD buffer");

    // Now, start pulling in words(bytes) until I either run to the end of the line, or I come across something that isn't valid
    // Note that if this is a byte sized transrer, I allocate a byte size buffer, otherwise it is a word size buffer.
    for(priv->len=0; *p; priv->len++)
    {
      // Lets assume I'm pointing at the start of a valid number
      if(priv->dataType == 1)
        priv->xmt.bytes[priv->len] = strtol(p, &p, 0);    // Data is byte sized
      else
        priv->xmt.words[priv->len] = strtol(p, &p, 0);    // data is not -pure- byte sized

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
  // Note that if this is an 8-bit transmission and reception, I allocate a byte size buffer.
  // Anything else gets word size buffers.
  if(priv->len)
  {
    if(priv->dataType == 1)
      priv->xmt.bytes = (uint8_t *) realloc(priv->xmt.bytes, sizeof(uint8_t)*priv->len);
    else
      priv->xmt.words = (uint16_t *) realloc(priv->xmt.words, sizeof(uint16_t)*priv->len);

    // Finally, allocate a buffer for the received data
    if(priv->dataType == 1)
      priv->rcv.bytes = (uint8_t *) callocMustSucceed(sizeof(uint8_t), priv->len, "Rcv Buffer Allocation Failed");
    else
      priv->rcv.words = (uint16_t *) callocMustSucceed(sizeof(uint16_t), priv->len, "Rcv Buffer Allocation Failed");
  }
  // At this point, I should have the transmit buffer filled (with bytes or words...),
  //  the length I'm tranmitting set
  // and the receive buffer allocated

  return 0;
}

/** ****************************************************************************************
 *
 * Common GetIoIntrInfo routine
 *
 * *****************************************************************************************/
static long getIoIntrInfo(int dir, dbCommon *prec, IOSCANPVT *io)
{
  struct privateData *priv=prec->dpvt;
  if(priv)
  {
      *io = priv->scan;
  }
  return 0;
}


/* =========================================================================================
 *
 * Start of the actual db routines (as opposed to the support bits)
 *
 * ========================================================================================= */

// Initialize the AI record to get it prepared to use.
long init_AI(aiRecord *prec)
{
  struct privateData* priv;
  unsigned long start;

  /* do all the initialization stuff I need for an AI. */
  // Allocate a private data structure for this record and link it to the record
  prec->dpvt=priv=callocMustSucceed(1, sizeof(struct privateData), "init_AI Out of memory");

  // Add a pointer back to the record
  priv->pv = (dbCommon *) prec;

  // Now, update all the link information for this I/O point
  parseLink(prec->inp.value.instio.string,  priv);

  // If this is a constant, set it up
  recGblInitConstantLink(&prec->inp,DBF_ULONG,&start);

  scanIoInit(&priv->scan);
  priv->tpro = prec->tpro;
  priv->act=1;

  return 0;
}

long read_AI(aiRecord *prec)
{
  struct privateData* priv=prec->dpvt;


  if(!priv)
  {
    priv->tpro = prec->tpro;
    (void)recGblSetSevr(prec, COMM_ALARM, INVALID_ALARM);
    return 0;
  }

  prec->rval = priv->rval;
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
  getIoIntrInfo,
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
    priv->act=1;
    prec->pact=TRUE;
    return 0;
  }
  else
  {
    /* Complete the operations */
    prec->pact=FALSE;
    priv->act=0;
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





// Initialize the MBBI record to get it prepared to use.
static long init_MBBIDirect(mbbiDirectRecord *prec)
{
  struct privateData* priv;
  unsigned long start;

  /* do all the initialization stuff I need for an MBBIDirect. */
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
      scanIoInit(&priv->scan);
      priv->tpro = prec->tpro;
      priv->act=1;
      break;

    default:
      recGblRecordError(S_db_badField, (void *) prec, "DevmbbiDirectiTestAsyn(int_record) Illegal Input Field");
      return(S_db_badField);
      break;
  }

  return 0;
}

static long read_MBBIDirect(mbbiDirectRecord *prec)
{
  struct privateData* priv=prec->dpvt;

  if(!priv)
  {
    priv->tpro = prec->tpro;
    (void)recGblSetSevr(prec, COMM_ALARM, INVALID_ALARM);
    return 0;
  }

  priv->tpro=prec->tpro;
  prec->rval = priv->rval;
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
  getIoIntrInfo,
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
    priv->act = 1;
    prec->pact=TRUE;
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
  priv->act=1;

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

// Setting Verbosity Mask
void setVerbosityMask(int level)
{
	errlogPrintf("Setting verbosityMask: %08u, level: %u\n", verbosityMask, level);
	verbosityMask = level;
}
// Setting Verbosity Mask
static const iocshArg VerbosityMaskArg0 = {"Mask", iocshArgInt };
static const iocshArg *const VerbosityMaskArgs[1] = { &VerbosityMaskArg0 };
static const iocshFuncDef setVerbosityMaskDef = {"VerbosityMask", 1, VerbosityMaskArgs };
static void VerbosityMaskCall(const iocshArgBuf * args)
{
	setVerbosityMask(args[0].ival);
}

void setVerbosityBit(int bit)
{
	uint32_t newVerbosity;

	if(bit < 32)
	{
		errlogPrintf("verbosityMask:%08x inverting bit: %u", verbosityMask, bit);
		newVerbosity = verbosityMask ^ ((uint32_t)1 << bit);
		errlogPrintf("-> %08x\n", newVerbosity);
		verbosityMask = newVerbosity;
	}
	else
		errlogPrintf("Bit out of range %d > 31\n",bit);
}

// Setting Verbosity Mask Bit
static const iocshArg VerbosityBitArg0 = {"Bit", iocshArgInt };
static const iocshArg *const VerbosityBitArgs[1] = { &VerbosityBitArg0 };
static const iocshFuncDef setVerbosityBitDef = {"VerbosityBit", 1, VerbosityBitArgs };
static void VerbosityBitCall(const iocshArgBuf * args)
{
	setVerbosityBit(args[0].ival);
}


// This table should go at the end, as it references earlier functions
//
// Commands to Register
void drvSPIRegister()
{
  initRoutine();
  iocshRegister(&setVerbosityMaskDef, VerbosityMaskCall);
  iocshRegister(&setVerbosityBitDef, VerbosityBitCall);
  iocshRegister(&addSPIDeviceDef, addSPIDeviceCall);
}

epicsExportRegistrar(drvSPIRegister);

