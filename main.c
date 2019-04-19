/* DriverLib Includes */
#include "debug.h"
#include "driverlib.h"
#include "touch.h"

/* Standard Includes */
#include <stdint.h>

#include <stdbool.h>
#include <string.h>

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
    
    /* Enabling MASTER interrupts */
    Interrupt_enableMaster();

    /* Going to LPM3 */
    while (1)
    {
        PCM_gotoLPM0();
    }
}
