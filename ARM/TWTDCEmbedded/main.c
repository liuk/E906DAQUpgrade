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

/// Size of the temp buffer
#define BUFSIZE             512

/// Size of the temp buffer in bytes
#define BUFSIZEB            2048

/// Size of SDRAM in bytes -- 64M - 0x10,0000 - 0x8000
/// First 0x8000 bytes occuppied by user app, i.e. this program
/// Last 0x100000 bytes occupied by u-boot handling the basics
#define SDSIZE              0x03cf8000

/// Size of SDRAM in words
#define NSDWORDS            0x00f3e000

/// Number of data banks in Dual-port
#define NBANKS              16
#define NWORDSPERBANK       0x400     //1K 32-bit words per bank
#define NBYTESPERBANK       0x1000    //4096 = 32K * 32 / 4

/// EventID register position
#define EVENTIDPOS          0x100

/// Masks to retrive info
#define NWORDSMASK          0x7ff00000
#define BANKIDMASK          0x0000000f

/// System states
#define BOS                 0x0       //beam is on, keep moving DP to SD
#define EOS                 0x1       //beam is off, trasnfer from SD to DP
#define WAIT                0x2       //transfer back is done, wait for last IRQ
#define READY               0x3       //transfer is done, ready for next spill
#define ES_SDOVERFLOW       0xf1      //this error happens when SDRAM overflow
#define ES_INCOMPLETE       0xf2      //this error happens when off-beam transfer is incomplete

/// Commands, upper 16-bits are fixed at 0xe906
#define RESETCMD            0xe906000f   //This command triggers the hardware RESET
#define BOSCMD              0xe9060000   //This command is issued on BOS, force the system to go to READY
#define EOSCMD              0xe9060001   //This command is issued on EOS, starts off beam transfer
#define TRANSFERCMD         0xe9060002   //This command is issued on flush event, starts one block transfer
#define LASTEVTCMD          0xe9060003   //This command is issued on last flush, change state to wait

/// Error codes
#define EC_INCOMPLETE       0xe906ae01   //Off-beam transfer is somehown incomplete
#define EC_SDOVERFLOW       0xe906ae02   //SD-RAM overflow --- too many words
#define EC_UNKNOWNCMD_ON    0xe906be01   //other unknow state for command on beam transfer
#define EC_UNKNOWNCMD_OFF   0xe906be02   //other unknow state for command off beam transfer
#define EC_UNKNOWNCMD_BOS   0xe906be03   //other unknow state for command BOS
#define EC_UNKNOWNCMD_EOS   0xe906be04   //other unknow state for command EOS
#define EC_UNKNOWNCMD_LST   0xe906be05   //other unknow state for command LASTEVT
#define EC_UNKNOWNCMD_OTR   0xe906be99   //other unknow state for other commands

//------------------------------------------------------------------------------
//         Local variables
//------------------------------------------------------------------------------
/// Pointer to a long int or unsigned int
typedef unsigned long* lPTR;    // int and long on ARM are both 32-bit, learnt sth new

/// Global flag about the current state of running: 0 = BOS, 1 = EOS, 2 = Ready for next spill
/// Note the definition of BOS/EOS/READY is not the same as standard E906 definition
volatile unsigned int state = BOS;

/// Total number of words in this spill
volatile unsigned int nWordsTotal = 0;

/// Header position index
unsigned int headerPos = 0;

/// Size of the block transfer
unsigned int blkSize = 1800;

// Address of current write/read address from SD
volatile lPTR currentSDAddr = 0;

// Current DP bank ID
volatile unsigned int currentDPBankID = 0;

/// interrupt port for dual-port mem
const Pin pinPC11 = {1 << 11, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP};       //Dual-port interrupt

/// Addresses
// start and end address of DP and SD
const lPTR dpStartAddr  = (lPTR)0x50000000;
const lPTR dpEndAddr    = (lPTR)0x50010000;   //for now the upper half is not used in FPGA
const lPTR sdStartAddr  = (lPTR)0x20208000;   //this program is copied to 0x20200000 taking 0x8000 bytes
const lPTR sdEndAddr    = (lPTR)0x23f00000;   //u-boot is copied to 0x23f00000

// Dual-port configuration/operation address
const lPTR dpErrorAddr  = (lPTR)0x5001ffe8;   //register for run control parameters -- 0x7ffa
const lPTR dpStateAddr  = (lPTR)0x5001ffec;   //register for run control parameters -- 0x7ffb
const lPTR dpIRQRevAddr = (lPTR)0x5001fff8;   //register to receive interrupt       -- 0x7ffe
const lPTR dpIRQSndAddr = (lPTR)0x5001fffe;   //register to send interrupt          -- 0x7fff

// Address of first and last word of each DP bank
lPTR dpBankStartAddr[NBANKS];    // the starting point of DP memory bank, where header is saved
lPTR dpBankEventIDAddr[NBANKS];  // the last word of DP memory bank -- where eventID is saved
lPTR dpBankHeaderAddr[NBANKS];   // configured at start time where header of event is stored

// Buffer for block transfer
unsigned int buffer[BUFSIZE];

//------------------------------------------------------------------------------
/// initialize
//------------------------------------------------------------------------------
void init(void)
{
#if (BeamOnDBG > 0)
    printf("Entering init function. \n\r");
#endif

    //Reset all the headers to 0
    for(unsigned int i = NBANKS-1; i != 0; --i) *(dpBankHeaderAddr[i]) = 0;

    //Read interrupt bit to clear previous interrupt state
    volatile unsigned int dummy = *dpIRQRevAddr;

    //set the number of words in SDRAM to 0
    nWordsTotal = 0;

#if (BeamOnDBG > 0)
    printf("Leaving init function. \n\r");
#endif
}

//------------------------------------------------------------------------------
/// This is a manual reset, force reload from RomBOOT --- by TEK
//------------------------------------------------------------------------------
unsigned short myReset()
{
    //To avoid infinit reset
    init();

    //RSTC_RCR = AT91C_RSTC_PROCRST;
    *AT91C_RSTC_RMR = (0xA5 << 24) | (0x4 << 8) | AT91C_RSTC_URSTIEN | AT91C_RSTC_PROCRST;
    *AT91C_SHDWC_SHCR = AT91C_SHDWC_SHDW;
    AT91C_BASE_PMC->PMC_SCDR= 0xFFFFFFFF;
    AT91C_BASE_RSTC->RSTC_RCR = 0xA5000005; // Controller+Periph
    asm("b .\n");

    return 0;
}

//------------------------------------------------------------------------------
/// transfer from DP to SDRam during beam on time
//------------------------------------------------------------------------------
void beamOnTransfer(void)
{
#if (BeamOnDBG > 0)
    printf("Entering beamOnTransfer function. \n\r");
#endif

    //set number of words
#if (ScalarMode == 0)
    unsigned int nWords = 257;
#else
    unsigned int nWords = 48;
#endif

#if (BeamOnDBG > 0)
    if(nWords > NWORDSPERBANK)             printf("Number of words in bank %u exceeded bank size.", currentDPBankID);
    if(currentSDAddr + nWords > sdEndAddr) printf("SDRAM overflow.");
    printf("- Bank %u has %u words: \n\r", currentDPBankID, nWords);
#endif

    //Protection against SDRAM overflow
    if(currentSDAddr > sdEndAddr)
    {
        //printf("- ERROR: SDRAM overflow!!!!\n\r");
        state = ES_SDOVERFLOW;
        *dpErrorAddr = EC_SDOVERFLOW;
        return;
    }

    //move the content to SDRAM, without zero suppression, the last word is eventID
    __aeabi_memcpy(currentSDAddr, dpBankStartAddr[currentDPBankID], nWords << 2);
    currentSDAddr = currentSDAddr + nWords;

    //Now move the event ID, if needed, and Update the number of words in SD
#if (ScalarMode == 0)
    nWordsTotal = nWordsTotal + nWords;
#else
    *currentSDAddr++ = *(dpBankEventIDAddr[currentDPBankID]);
    nWordsTotal = nWordsTotal + nWords + 1;  //Total number of words = nWords in this event + 1 for eventID
#endif

#if (BeamOnDBG > 0)
    unsigned int bankID = *(dpBankEventIDAddr[currentDPBankID]) & BANKIDMASK;
    printf("- EventID in this bank is: %8X, supposed to be in bank %u.\n\r", *(dpBankEventIDAddr[currentDPBankID]), bankID);
    //if(bankID != currentDPBank) TRACE_ERROR("BankID does not match on FPGA side.\n\r");
#endif

    //Reset the event header
    *(dpBankHeaderAddr[currentDPBankID]) = 0;

#if (BeamOnDBG > 0)
    printf("- State %u: finished reading bank %u, eventID = %08X, has %u words, %u words in SDRAM now.\n\r", state, currentDPBankID, *(dpBankEventIDAddr[currentDPBankID]), nWords, nWordsTotal);
    printf("- IRQ state: level = %u, content = %u\n\r", PIO_Get(&pinPC11), *dpIRQRevAddr);
#if (BeamOnDBG > 1)
    unsigned int n = sdAddr - sdStartAddr;
    for(unsigned int i = 0; i < n; ++i) printf("-- %u: %08X = %08X\n\r", i, sdStartAddr+i, *(sdStartAddr+i));
#endif
#endif

#if (BeamOnDBG > 0)
    printf("Exiting beamOnTransfer, state = %u\n\r", state);
#endif
}

//------------------------------------------------------------------------------
/// transfer from SDRam to DP during beam off time
//------------------------------------------------------------------------------
void beamOffTransfer(void)
{
#if (BeamOffDBG > 0)
    printf("Entering beamOffTransfer function, state = %u\n\r", state);
#endif

#if (BeamOffDBG > 0)
    printf("- Currently the SD pointer is at %08X\n\r", currentSDAddr);
#endif

    //Check if it's already completed in previous transfer, if so, write the header and set state to WAIT
    if(nWordsTotal == 0)
    {
        *dpStartAddr = 0;
        state = WAIT;
        *dpStateAddr = WAIT;
        return;
    }

    //Write as much data as possible to the DP memory
    unsigned int nWords = blkSize;
    if(nWords > nWordsTotal) nWords = nWordsTotal;
#if (BeamOffDBG > 0)
    printf("- Currently SDRAM has %u words, will transfer %u words from SD to DPRAM.\n\r", nWordsTotal, nWords);
#endif

    //Transfer nWords words from SD to DP
    lPTR dpAddr = dpStartAddr + 1;
    unsigned int bufPtr = 0;
    unsigned int transferCount = 1;
    for(unsigned int i = nWords; i != 0; --i)
    {
        unsigned int word = *currentSDAddr++;
#if (ScalarMode == 0)
        if(word != 0) buffer[bufPtr++] = word;
#else
        buffer[bufPtr++] = word;
#endif

        if(bufPtr == BUFSIZE)
        {
            __aeabi_memcpy(dpAddr, buffer, BUFSIZEB);
            dpAddr = dpAddr + BUFSIZE;

            transferCount = transferCount + BUFSIZE;
            bufPtr = 0;
        }
    }

    if(bufPtr != 0)  //make the last transfer
    {
        __aeabi_memcpy(dpAddr, buffer, bufPtr << 2);
        transferCount = transferCount + bufPtr;
    }

    //Write nWords to the first word at DP
    *dpStartAddr = transferCount;

#if (BeamOffDBG > 1)
    printf("- Currently the SD RD pointer is at %08X\n\r", currentSDAddr);
    for(unsigned int i = 0; i <= nWords; ++i) printf("-!- %u: %08X = %08X\n\r", i, dpStartAddr+i, *(dpStartAddr+i));
#endif

    //Subtract the nWords from nWordsTotal, and reset running state to WAIT if it's all done at next entry
    nWordsTotal = nWordsTotal - nWords;

#if (BeamOffDBG > 0)
    printf("- %u words left in SDRAM, state code is set to %u\n\r", nWordsTotal, state);
    printf("Leaving beamOffTransfer\n\r");
#endif
}

void CentralDispatch(void)
{
#if (CDPDebug > 10)
    printf("Entering CentralDispatch function, state = %u\n\r", state);
#endif

    //Acknowledge interrupt from PC11
    unsigned int dp_isr = PIO_GetISR(&pinPC11);
    unsigned int dp_lev = PIO_Get(&pinPC11);
    volatile unsigned int cmd = *dpIRQRevAddr;
#if (CDPDebug > 10)
    printf("- Receive and Acknowledge the interrupt %08X, level = %08X \n\r", cmd, dp_lev);
#endif
    if(dp_lev == 1) return; //only trigger on positive edge
    if(cmd == RESETCMD) myReset();

    if((cmd & 0xffff0000) != 0xe9060000)   //normal event transfer, this is sent from FPGA
    {
        currentDPBankID = cmd & 0xf;
        if(state == BOS)
        {
            beamOnTransfer();
        }
        else if(state == READY)
        {
            state = BOS;
            *dpStateAddr = BOS;
            currentSDAddr = sdStartAddr;
            beamOnTransfer();
        }
        else
        {
#if (CDPDebug > 0)
            *dpErrorAddr = EC_UNKNOWNCMD_ON;
            printf("- ERROR: state 0x%x is not expected for a on-beam transfer command. \n\r", state);
#endif
            return;
        }
    }
    else if(cmd == TRANSFERCMD)    //flush event transfer, this is sent from VME
    {
#if (CDPDebug > 9)
        printf("- Received one flush, state = %u, nWordsTotal = %u\n\r", state, nWordsTotal);
#endif
        if(state == EOS)
        {
            beamOffTransfer();
        }
        else
        {
#if (CDPDebug > 0)
            *dpErrorAddr = EC_UNKNOWNCMD_OFF;
            printf("- ERROR: state 0x%x is not expected for a off-beam transfer command. \n\r", state);
#endif
            return;
        }
    }
    else if(cmd == BOSCMD)
    {
        if(state == EOS || state == WAIT)
        {
#if (CDPDebug > 0)
            *dpErrorAddr = EC_INCOMPLETE;
            printf("- ERROR: Off beam transfer not completed for previous spill, force state from 0x%x to READY \n\r", state);
#endif

            init();
            state = READY;
            *dpStateAddr = READY;
        }
        else if(state == READY)
        {
            //printf("- INFO: Received BOS, transits to beam on state \n\r");
            return;
        }
        else
        {
#if (CDPDebug > 0)
            *dpErrorAddr = EC_UNKNOWNCMD_BOS;
            printf("- ERROR: state 0x%x is not expected for a BOS command. \n\r", state);
#endif
            return;
        }
    }
    else if(cmd == EOSCMD)
    {
        if(state == BOS || state == ES_SDOVERFLOW)
        {
            //printf("- INFO: Received EOS, transits to beam off state, will transfer %u words \n\r", nWordsTotal);
            state = EOS;
            *dpStateAddr = EOS;
            currentSDAddr = sdStartAddr;
            beamOffTransfer();
        }
        else
        {
#if (CDPDebug > 0)
            *dpErrorAddr = EC_UNKNOWNCMD_EOS;
            printf("- ERROR: state 0x%x is not expected for a EOS command. \n\r", state);
#endif
            return;
        }
    }
    else if(cmd == LASTEVTCMD)
    {
        if(nWordsTotal != 0)
        {
#if (CDPDebug > 0)
            *dpErrorAddr = EC_INCOMPLETE;
            printf("- ERROR: off-beam transfer is not complete! \n\r");
#endif
        }

        //move to READY
        if(state == EOS)
        {
            //printf("- INFO: Received last flush, change back to READY, %d words left \n\r", nWordsTotal);

            init();
            state = READY;
            *dpStateAddr = READY;
        }
        else
        {
#if (CDPDebug > 0)
            *dpErrorAddr = EC_UNKNOWNCMD_LST;
            printf("- ERROR: state 0x%x is not expected for a LASTEVT command. \n\r", state);
#endif
            return;
        }
    }
    else if((cmd & 0xe9068000) == 0xe9068000)
    {
        blkSize = cmd & 0x7fff;
        //printf("- INFO: Set the flush event block size to %d\n\r", blkSize);
    }
    else
    {
#if (CDPDebug > 0)
        *dpErrorAddr = EC_UNKNOWNCMD_OTR;
        printf("- ERROR: No condition satisfied, cmd = 0x%08x, state = 0x%1x !!!!!\n\r", cmd, state);
#endif
        return;
    }
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
    AT91C_BASE_SMC->SMC_SETUP4 = 0x00010001;
    AT91C_BASE_SMC->SMC_PULSE4 = 0x03010301;  // NCS_RD=0x03, NRD=0x02, NCS_WR=Ox02, NWE=0x02
    AT91C_BASE_SMC->SMC_CYCLE4 = 0x00030003;  // NRDCYCLE=005, NWECYCLE=002
    AT91C_BASE_SMC->SMC_CTRL4  = (AT91C_SMC_READMODE   |
                                  AT91C_SMC_WRITEMODE  |
                                  AT91C_SMC_NWAITM_NWAIT_DISABLE |
                                  ((0x1 << 16) & AT91C_SMC_TDF)  |
                                  AT91C_SMC_DBW_WIDTH_THIRTY_TWO_BITS);

    // Configure interrupt
    PIO_InitializeInterrupts(AT91C_AIC_PRIOR_LOWEST);
    PIO_ConfigureIt(&pinPC11, (void (*)(const Pin *)) CentralDispatch);
    PIO_EnableIt(&pinPC11);

    //Start/end address of each DP memory bank
#if (ScalarMode == 0)
    headerPos = 0;
#else
    headerPos = 47;
#endif
    blkSize = 1800;
    printf("- INIT: Initializing DP ram, header pos = 0x%x, blkSize = %d\n\r", headerPos, blkSize);

    for(unsigned int i = 0; i < NBANKS; ++i)
    {
        if(i == 0)
        {
            dpBankStartAddr[i] = dpStartAddr;
        }
        else
        {
            dpBankStartAddr[i] = dpBankStartAddr[i-1] + NWORDSPERBANK;
        }
        dpBankHeaderAddr[i] = dpBankStartAddr[i] + headerPos;
        dpBankEventIDAddr[i] = dpBankStartAddr[i] + EVENTIDPOS;
    }
}

void ConfigureLED()
{
    LED_Configure(0);
    LED_Configure(1);
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

    // Configuration
    ConfigureLED();
    BOARD_ConfigureSdram(32);
    ConfigureDPRam();

    //Write everything in DP to 0
    __aeabi_memset(dpStartAddr, NDPWORDS << 2, 0);
    printf("- INFO: Compiled: %s %s. \n\r", __DATE__, __TIME__);

    // Set to be ready for beam
    init();
    state = READY;

    // Main loop
    while(1);
}
