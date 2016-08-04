#include <board.h>
#include <board_memories.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <pit/pit.h>
#include <aic/aic.h>
#include <tc/tc.h>
#include <utility/led.h>
#include <utility/trace.h>
#include <stdio.h>

//------------------------------------------------------------------------------
//         Local definitions
//------------------------------------------------------------------------------
/// Size of DP in bytes -- 32K x 32 bits
#define DPSIZE              0x20000

/// Size of DP in words -- 32K
#define NDPWORDS            0x8000

/// Size of SDRAM in bytes -- 64M - 0x10,0000 - 0x8000
/// First 0x8000 bytes occuppied by user app, i.e. this program
/// Last 0x100000 bytes occupied by u-boot handling the basics
#define SDSIZE              0x03ef8000

/// Size of SDRAM in words
#define NSDWORDS            0x0fbe0000

/// Number of data banks in Dual-port
#define NBANKS              16
#define NWORDSPERBANK       0x400     //1K 32-bit words per bank
#define NBYTESPERBANK       0x1000    //4096 = 32K * 32 / 4

/// Masks to retrive info
#define NWORDSMASK          0x7ff00000
#define BANKIDMASK          0x0000000f

//------------------------------------------------------------------------------
//         Local variables
//------------------------------------------------------------------------------
/// Global flag about the current stage of running: 0 = beamOn, 1 = beamOff
volatile unsigned int stage = 0;

/// Total number of words in this spill
volatile unsigned int nWordsTotal = 0;

/// Pointer to a long int or unsigned int
typedef unsigned long*  lPTR;    // int and long on ARM are both 32-bit, learnt sth new

/// Addresses
// start and end address of DP and SD
const lPTR dpStartAddr = (lPTR)0x50000000;
const lPTR dpEndAddr   = (lPTR)0x50010000;   //for now the upper half is not used in FPGA
const lPTR sdStartAddr = (lPTR)0x20008000;
const lPTR sdEndAddr   = (lPTR)0x23f00000;

// Address of first and last word of each DP bank
lPTR dpBankStartAddr[NBANKS];    // the starting point of DP memory bank, where header is saved
lPTR dpBankLastAddr[NBANKS];     // the last word of DP memory bank -- where eventID is saved

// Address of current read address from SD
lPTR currentSDAddr = 0;//sdStartAddr;

//------------------------------------------------------------------------------
/// initialize
//------------------------------------------------------------------------------
void init(void)
{
    TRACE_DEBUG("Entering init function. \n\r");

    //Start/end address of each DP memory bank
    dpBankStartAddr[0] = dpStartAddr;
    *(dpBankStartAddr[0]) = 0;
    for(int i = 1; i < NBANKS; ++i)
    {
        dpBankStartAddr[i] = dpBankStartAddr[i-1] + NWORDSPERBANK;
        dpBankLastAddr[i] = dpBankStartAddr[i] + NWORDSPERBANK - 3;

        //clear all the header positions
        *(dpBankStartAddr[i]) = 0;
    }

    //set running stage to be ready for beam
    stage = 0;

    //set the number of words in SDRAM to 0
    nWordsTotal = 0;
}

//------------------------------------------------------------------------------
/// transfer from DP to SDRam during beam on time
//------------------------------------------------------------------------------
void beamOnTransfer(void)
{
    TRACE_DEBUG("Entering beamOnTransfer function. \n\r");

    //Initialize address, stage register, and DP headers
    init();

    //Start looping indefinitely
    lPTR sdAddr = sdStartAddr;
    unsigned int currentDPBank = 0;
    while(stage == 0)
    {
        //Read all the bank headers until the next finished bank
        unsigned int header = *(dpBankStartAddr[currentDPBank]);
        while(header == 0)
        {
            currentDPBank = (currentDPBank + 1) & BANKIDMASK;
            header = *(dpBankStartAddr[currentDPBank]);
        }

        //extract nWords from header
        lPTR dpAddr = dpBankStartAddr[currentDPBank] + 1;
        unsigned int nWords = (header & NWORDSMASK) >> 20;
        //if(nWords > NWORDSPERBANK)        TRACE_ERROR("Number of words in event %d exceeded bank size.", eventID);
        //if(sdAddr + nWords*4 > sdEndAddr) TRACE_ERROR("SDRAM overflow.");
        TRACE_DEBUG("- Bank %d has %d words: \n\r", currentDPBank, nWords);

        //extract eventID info
        unsigned int eventID = *(dpBankLastAddr[currentDPBank]);
        TRACE_DEBUG("- EventID in this bank is: %8X\n\r", eventID);
        //unsigned int bankID = eventID & BANKIDMASK;
        //if(bankID != currentDPBank)       TRACE_ERROR("BankID does not match on FPGA side.");

        //move the header to SDRAM
        *sdAddr = header;
        ++sdAddr;

        //move the content to SDRAM
        unsigned int i = nWords - 1;
        for(; i != 0; --i)
        {
            *sdAddr = *dpAddr + 1;   // this is for testing only
            ++sdAddr; ++dpAddr;

            TRACE_DEBUG("-- Read one word from DP to SD.\n\r");
        }

        //move the eventID word to SDRAM
        *sdAddr = eventID;
        ++sdAddr;

        //Reset the event header and move to next bank
        *(dpBankStartAddr[currentDPBank]) = 0;
        currentDPBank = (currentDPBank + 1) & BANKIDMASK;

#if (TRACE_LEVEL >= TRACE_LEVEL_DEBUG)
        TRACE_DEBUG("- Stage %d: finished reading bank %d, eventID = %08X, has %d words, %d words in SDRAM now.\n\r", stage, currentDPBank, eventID, nWords, sdAddr - sdStartAddr);
        unsigned int n = sdAddr - sdStartAddr;
        for(i = 0; i < n; ++i) TRACE_DEBUG("-- %d: %08X = %08X\n\r", i, sdStartAddr+i, *(sdStartAddr+i));
#endif
    }

    nWordsTotal = sdAddr - sdStartAddr;  //Total number of words
    TRACE_DEBUG("Exiting beamOnTransfer, stage = %d", stage);
}

//------------------------------------------------------------------------------
/// transfer from SDRam to DP during beam off time
//------------------------------------------------------------------------------
const Pin pinPC11 = {1 << 11, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP};       //Dual-port interrupt
void beamOffTransfer(void)
{
    TRACE_DEBUG("Entering beamOffTransfer function, stage = %d\n\r", stage);

    //Acknowledge interrupt from PC11
    unsigned int dp_isr = PIO_GetISR(&pinPC11);
    TRACE_DEBUG("- Receive and Acknowledge the interrupt %d.", dp_isr);
    if(!dp_isr) return;

    //If it's the first entry in this spill, set stage to 1 to stop beam off reading
    if(stage == 0)
    {
        TRACE_DEBUG("- First time entering beamOffTransfer in this spill, stage = %d, set it to 1\n\r", stage);

        stage = 1;
        currentSDAddr = sdStartAddr;  //initialize SD read address
    }
    TRACE_DEBUG("- Currently the SD RD pointer is at %08X", currentSDAddr);

    //Write as much data as possible to the DP memory, save the last word for word count
    unsigned int nWords = NDPWORDS - 1;
    if(nWords > nWordsTotal) nWords = nWordsTotal;
    TRACE_DEBUG("- Currently SDRAM has %d words, will transfer %d words from to DPRAM.\n\r");

    //Write nWords to the first word at DP
    lPTR dpAddr = dpStartAddr;
    *dpAddr = nWords; ++dpAddr;

    //Transfer nWords words from SD to DP
    unsigned int i = nWords;
    for(; i != 0; --i)
    {
        *dpAddr = *currentSDAddr;
        ++dpAddr; ++currentSDAddr;

        TRACE_DEBUG("-- Read one word from SD to DP\n\r");
    }

    //Subtract the nWords from nWordsTotal, and reset running stage to 0
    nWordsTotal = nWordsTotal - nWords;
    if(nWordsTotal == 0) stage = 0;

    TRACE_DEBUG("- %d words left in SDRAM, stage code is set to %d\n\r", nWordsTotal, stage);
    TRACE_DEBUG("Leaving beamOffTransfer\n\r");
}

//------------------------------------------------------------------------------
//         Utility functions to initialize Dualport SRAM -- mostly by Terry
//------------------------------------------------------------------------------
// Don't know if it's necessary, but apparently every pin definition is defined
// outside as a global const, just follow the convention here
const Pin pinCE4 = {1 << 8, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_PERIPH_A, PIO_DEFAULT};    //chip select 4
const Pin pinCE5 = {1 << 9, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_PERIPH_A, PIO_DEFAULT};    //chip select 5 -- semaphore mode

void ConfigureDPRam()
{
    // Configure PIO pins for DP control
    PIO_Configure(&pinCE4, 1);
    PIO_Configure(&pinCE5, 1);
    PIO_Configure(&pinPC11, 1);  // Note this is PC11 instead of PC13 as specified on the datasheet

    // For detailed explaination of each setting bits, refer to datasheet 19.14.1 - 19.14.4
    // Note SMC_CTRL corresponds to SMC Mode Register
    // Configure EBI selection
    AT91C_BASE_CCFG->CCFG_EBICSA |= (AT91C_EBI_SUPPLY);

    // Configure SMC for CS4
    AT91C_BASE_SMC->SMC_SETUP4 = 0x00000000;
    AT91C_BASE_SMC->SMC_PULSE4 = 0x03020202;  // NCS_RD=0x03, NRD=0x02, NCS_WR=Ox02, NWE=0x02
    AT91C_BASE_SMC->SMC_CYCLE4 = 0x00050002;  // NRDCYCLE=005, NWECYCLE=002
    AT91C_BASE_SMC->SMC_CTRL4  = (AT91C_SMC_READMODE   |
                                  AT91C_SMC_WRITEMODE  |
                                  AT91C_SMC_NWAITM_NWAIT_DISABLE |
                                  ((0x1 << 16) & AT91C_SMC_TDF)  |
                                  AT91C_SMC_DBW_WIDTH_THIRTY_TWO_BITS);

    // Configure interrupt
    PIO_InitializeInterrupts(AT91C_AIC_PRIOR_LOWEST);
    PIO_ConfigureIt(&pinPC11, (void (*)(const Pin *)) beamOffTransfer);
    PIO_EnableIt(&pinPC11);
}

//------------------------------------------------------------------------------
/// Application entry point.
//------------------------------------------------------------------------------
int main(void)
{
    // DBGU output configuration
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    TRACE_INFO("-- SeaQuest VME TDC Embedded Project %s --\n\r", SOFTPACK_VERSION);
    TRACE_INFO("-- %s\n\r", BOARD_NAME);
    TRACE_INFO("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    // Configuration
    BOARD_ConfigureSdram(32);
    ConfigureDPRam();

    // Main loop
    stage = 0;
    while(1)
    {
        //Enters beam on transfer
        if(stage == 0) beamOnTransfer();

        //when beam off, interrupt mode will take over
    }
}
