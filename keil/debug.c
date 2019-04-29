/* DriverLib Includes */
#include "driverlib.h"
#include "helpers.h"

/* Standard Includes */
#include <stdint.h>

#include <stdbool.h>
#include <string.h>
#include "debug.h"

const eUSCI_UART_Config uartConfig_9600 =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                      // BRDIV = 13
        2,                                       // UCxBRF = 0
        0,                                      // UCxBRS = 37
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_MSB_FIRST,                  // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};


void initUartDebug(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                                               GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A2_BASE, &uartConfig_9600);
    UART_enableModule(EUSCI_A2_BASE);
    Interrupt_enableInterrupt(INT_EUSCIA2);
}

void printDebug(char* message) {
		uint16_t i = 0;
		while (message[i] != '\0') {
			while(!(UCA2IFG&UCTXIFG));
			UART_transmitData(EUSCI_A2_BASE, reverse(message[i]));
			i++;
		}
}

void EUSCIA2_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    UART_clearInterruptFlag(EUSCI_A2_BASE, status);
}