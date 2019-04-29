/* DriverLib Includes */
#include "debug.h"
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>

#include <stdbool.h>
#include <string.h>

#include "touch.h"

int main(void)
{
    volatile uint32_t ii;

    /* Halting the Watchdog */

	WDT_A_holdTimer();
	
	
    /* Setting DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    /////////////////
    // UART
    /////////////////
    initUartDebug();


    /////////////////
    // Outputs
    /////////////////


		/////////////////
		// Inputs
		/////////////////
		initTouch();
		
    /* Enabling SRAM Bank Retention */
    SysCtl_enableSRAMBankRetention(SYSCTL_SRAM_BANK1);
    //Interrupt_enableSleepOnIsrExit();
    /* Enabling MASTER interrupts */
    Interrupt_enableMaster();
		
		pollTouch();

    /* Going to LPM3 */
    while (1)
    {
        PCM_gotoLPM0();
    }
}
