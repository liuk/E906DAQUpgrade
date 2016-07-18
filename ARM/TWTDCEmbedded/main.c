//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

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
/// Delay for pushbutton debouncing (in milliseconds).
#define DEBOUNCE_TIME       500

/// PIT period value in µseconds.
#define PIT_PERIOD          1000

//------------------------------------------------------------------------------
//         Local variables
//------------------------------------------------------------------------------
/// Pushbutton #1 pin instance.
const Pin pinPB1 = PIN_PUSHBUTTON_1;

/// Pushbutton #2 pin instance.
const Pin pinPB2 = PIN_PUSHBUTTON_2;

/// Indicates the current state (on or off) for each LED.
unsigned char pLedStates[2] = {1, 1};

/// Global timestamp in milliseconds since start of application.
volatile unsigned int timestamp = 0;

typedef unsigned short* sPTR;
typedef unsigned long*  lPTR;    // int and long on ARM are both 32-bit, learnt sth new

//------------------------------------------------------------------------------
/// Simple function to read 32Kx32 bits from address 0x50000000
//------------------------------------------------------------------------------
void DPRead(void)
{
    unsigned int i;
    unsigned int nWords = 32*1024;   // DP is 32Kx32 bits
    lPTR i_dpaddr = (lPTR)0x50000000;
    for(i = nWords; i != 0; i--) 
    {
        // readout the data and check consistency
        unsigned int readout = *i_dpaddr;
        if(i % 1000 == 0)
        {
            printf(" -- %d DPRam address %08X: %08X \n\r", nWords - i, i_dpaddr, readout);
        }
        
        //increment addr pointers by 4 bytes
        ++i_dpaddr;
    }
}

//------------------------------------------------------------------------------
/// Handler for PIT interrupt. Increments the timestamp counter.
//------------------------------------------------------------------------------
void ISR_Pit(void)
{
    unsigned int status;

    // Read the PIT status register
    status = PIT_GetStatus() & AT91C_PITC_PITS;
    if(status != 0) // 1 indicates the Periodic Interval timer reached PIV since the last read of PIT_PIVR
    {
        // Read the PIVR to acknowledge interrupt and get number of ticks
        // Returns the number of occurrences of periodic intervals since the last read of PIT_PIVR
        // Right shift by 20 bits to get milliseconds
        timestamp += (PIT_GetPIVR() >> 20);
    }
}

//------------------------------------------------------------------------------
/// Configure the periodic interval timer to generate an interrupt every
/// millisecond.
//------------------------------------------------------------------------------
void ConfigurePit(void)
{
    // Initialize the PIT to the desired frequency
    PIT_Init(PIT_PERIOD, BOARD_MCK / 1000000);

    // Configure interrupt on PIT
    AIC_DisableIT(AT91C_ID_SYS);
    AIC_ConfigureIT(AT91C_ID_SYS, AT91C_AIC_PRIOR_LOWEST, ISR_Pit);
    AIC_EnableIT(AT91C_ID_SYS);
    PIT_EnableIT();

    // Enable the pit -- seems redundent
    PIT_Enable();
}

//------------------------------------------------------------------------------
/// Interrupt handler for pushbutton #1. Starts or stops LED #1.
//------------------------------------------------------------------------------
void ISR_Bp1(void)
{
    static unsigned int lastPress = 0;

    // Check if the button has been pressed
    if(!PIO_Get(&pinPB1)) 
    {
        // Simple debounce method: limit push frequency to 1/DEBOUNCE_TIME
        // (i.e. at least DEBOUNCE_TIME ms between each push)
        if((timestamp - lastPress) > DEBOUNCE_TIME) 
        {
            lastPress = timestamp;

            printf("Instructed to read DP by push button 1:\n\r");
            DPRead();
        }
    }
}

//------------------------------------------------------------------------------
/// Interrupt handler for pushbutton #2. Starts or stops LED #2 and TC0.
//------------------------------------------------------------------------------
void ISR_Bp2(void)
{
    static unsigned int lastPress = 0;
    
    // Check if the button has been pressed
    if(!PIO_Get(&pinPB2)) 
    {
        // Simple debounce method: limit push frequency to 1/DEBOUNCE_TIME
        // (i.e. at least DEBOUNCE_TIME ms between each push)
        if((timestamp - lastPress) > DEBOUNCE_TIME) 
        {
            lastPress = timestamp;

            // Disable LED#2 and TC0 if there were enabled
            if(pLedStates[1])
            {
                pLedStates[1] = 0;
                LED_Clear(1);
                AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS; 
            }   
            else    // Enable LED#2 and TC0 if there were disabled 
            {             
                pLedStates[1] = 1;
                LED_Set(1);
                AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
            }
        }
    }
}

//------------------------------------------------------------------------------
/// Configures the pushbuttons to generate interrupts when pressed.
//------------------------------------------------------------------------------
void ConfigureButtons(void)
{
    // Configure pios
    PIO_Configure(&pinPB1, 1);
    PIO_Configure(&pinPB2, 1);

    // Initialize interrupts
    PIO_InitializeInterrupts(AT91C_AIC_PRIOR_LOWEST);
    PIO_ConfigureIt(&pinPB1, (void (*)(const Pin *)) ISR_Bp1);
    PIO_ConfigureIt(&pinPB2, (void (*)(const Pin *)) ISR_Bp2);
    PIO_EnableIt(&pinPB1);
    PIO_EnableIt(&pinPB2);
}

//------------------------------------------------------------------------------
/// Configures LEDs #1 and #2 (cleared by default).
//------------------------------------------------------------------------------
void ConfigureLeds(void)
{
    LED_Configure(0);
    LED_Configure(1);
}

//------------------------------------------------------------------------------
/// Interrupt handler for TC0 interrupt. Toggles the state of LED #2.
//------------------------------------------------------------------------------
void ISR_Tc0(void)
{
    volatile unsigned int dummy;

    // Clear status bit to acknowledge interrupt
    dummy = AT91C_BASE_TC0->TC_SR;

    // Toggle LED state
    LED_Toggle(1);
}

//------------------------------------------------------------------------------
/// Configure Timer Counter 0 to generate an interrupt every 250ms.
//------------------------------------------------------------------------------
void ConfigureTc(void)
{
    unsigned int div;
    unsigned int tcclks;

    // Enable peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC0;

    // Configure TC for a 4Hz frequency and trigger on RC compare
    TC_FindMckDivisor(4, BOARD_MCK, &div, &tcclks);
    TC_Configure(AT91C_BASE_TC0, tcclks | AT91C_TC_CPCTRG);
    AT91C_BASE_TC0->TC_RC = (BOARD_MCK / div) / 4; // timerFreq / desiredFreq

    // Configure and enable interrupt on RC compare
    AIC_ConfigureIT(AT91C_ID_TC0, AT91C_AIC_PRIOR_LOWEST, ISR_Tc0);
    AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS;
    AIC_EnableIT(AT91C_ID_TC0);

    // Start the counter if LED is enabled.
    if(pLedStates[1]) TC_Start(AT91C_BASE_TC0);
}

//------------------------------------------------------------------------------
/// Waits for the given number of milliseconds (using the timestamp generated
/// by the PIT).
/// \param delay  Delay to wait for, in milliseconds.
//------------------------------------------------------------------------------
void Wait(unsigned long delay)
{
    volatile unsigned int start = timestamp;
    unsigned int elapsed;
    do 
    {
        elapsed = timestamp;
        elapsed -= start;
    }
    while(elapsed < delay);
}

//------------------------------------------------------------------------------
//         Utility functions to initialize Dualport SRAM -- mostly by Terry
//------------------------------------------------------------------------------

// Don't know if it's necessary, but apparently every pin definition is defined 
// outside as a global const, just follow the convention here
const Pin pinCE4 = {1 << 8, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_PERIPH_A, PIO_DEFAULT};    //chip select 4
const Pin pinCE5 = {1 << 9, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_PERIPH_A, PIO_DEFAULT};    //chip select 5 -- semaphore mode
const Pin pinInt = {1 << 13, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_PERIPH_A, PIO_DEFAULT};   //Dual-port interrupt
const Pin pinBsy = {1 << 15, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_PERIPH_A, PIO_DEFAULT};   //Dual-port busy -- should not be needed

// Interrupt handler for DP - read and display everything
void ISR_DPInt()
{
	//Acknowledge the DP interrupt
	unsigned char dp_isr = PIO_GetISR(&pinInt);

    printf("Instructed to read DP by PC13\n\r");
    DPRead();


}

// Configures the Dual-port RAM on CompactFlash controller -- this is done be TEK
void ConfigureDPRam()
{
    // Configure PIO pins for DP control
    PIO_Configure(&pinCE4, 1);
    PIO_Configure(&pinCE5, 1);
    PIO_Configure(&pinInt, 1);

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

    // Configure interrupt -- one alternative way might be through AIC using AT91C_ID_FIQ
    PIO_ConfigureIt(&pinInt, (void (*)(const Pin *)) ISR_DPInt);
    PIO_EnableIt(&pinInt);
}


//------------------------------------------------------------------------------
/// Application entry point. 
//------------------------------------------------------------------------------
int main(void)
{
    // DBGU output configuration
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- SeaQuest VME TDC Embedded Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    // Configuration
    ConfigurePit();
    ConfigureTc();
    ConfigureButtons();   //note all the PIO intl has been initialized in this call
    ConfigureLeds();
    BOARD_ConfigureSdram(32);
    ConfigureDPRam();
    
    // Base addresses of DPRAM and SDRAM
    lPTR dpAddr = (lPTR)0x50000000;
    lPTR sdAddr = (lPTR)0x21000000;   //just to be safe so that we don't overwrite u-boot
    
    // Initialize DP to a bunch or dummy values
    printf("Initialize DP to a bunch or dummy values\n\r");
    unsigned int i;
    unsigned int nWords = 32*1024;   // DP is 32Kx32 bits
    lPTR i_dpaddr = dpAddr;
    for(i = nWords; i != 0; i--) 
    {
    	// write with some dummy data
    	unsigned int data = 0xDEAD0000 + nWords - i;
        *i_dpaddr = data;

        // readout the data and check consistency
        unsigned int readout = *i_dpaddr;
        if(i % 1000 == 0)
        {
            printf(" -- %d DPRam address %08X: input = %08X, readout = %08X \n\r", nWords - i, i_dpaddr, data, readout);
        }
        
        //increment addr pointers by 4 bytes
        ++i_dpaddr;
    }
    
    // Main loop
    while(1) 
    {
        // Wait for LED to be active
        while(!pLedStates[0]);
        
        // Toggle LED state if active
        if(pLedStates[0]) LED_Toggle(0);

        /*
        // Read from DP, increment by 1 and write to SDRAM
        printf("Reading from DP and writing to SDRAM \n\r");
        lPTR fAddr = dpAddr;
        lPTR tAddr = sdAddr;
        for(i = nWords; i != 0; i--) 
        {
            *tAddr = *fAddr + 1;

            if(i % 1000 == 0)
            {
            	printf(" -- %d DP: %08X = %08X, SD: %08X = %08X \n\r", nWords - i, fAddr, *fAddr, tAddr, *tAddr);
            }
            ++fAddr; ++tAddr;
        }
        
        // Read from SDRAM, increment by 1 and write back to DP
        printf("Reading from SDRAM and writing to DP\n\r");
        fAddr = sdAddr;
        tAddr = dpAddr;
        for(i = nWords; i != 0; i--) 
        {
            *tAddr = *fAddr + 1;

            if(i % 1000 == 0)
            {
            	printf(" -- %d DP: %08X = %08X, SD: %08X = %08X \n\r", nWords - i, tAddr, *tAddr, fAddr, *fAddr);
            }
            ++fAddr; ++tAddr;
        }*/

        // Wait for 10s
        printf("One cycle finished. \n\r");
        Wait(10000);
    }
}
