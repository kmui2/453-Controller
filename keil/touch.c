#include "debug.h"
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "cmd.h"

#include "i2c_driver.h"

/* Slave Address for I2C Slave */
// There is one preset I2C address of 0x12. This is not changeable
#define SLAVE_ADDRESS 0x12

#define DETECTION_STATUS_REG 4

#define LED_PORT GPIO_PORT_P4
#define LED0_PIN GPIO_PIN2
#define LED1_PIN GPIO_PIN0
#define LED2_PIN GPIO_PIN1
#define LED3_PIN GPIO_PIN3


void initTouch(void)
{
    initI2C();

    GPIO_setAsOutputPin(LED_PORT, LED0_PIN);
    GPIO_setAsOutputPin(LED_PORT, LED1_PIN);
    GPIO_setAsOutputPin(LED_PORT, LED2_PIN);
    GPIO_setAsOutputPin(LED_PORT, LED3_PIN);

    GPIO_setOutputLowOnPin(LED_PORT, LED0_PIN);
    GPIO_setOutputLowOnPin(LED_PORT, LED1_PIN);
    GPIO_setOutputLowOnPin(LED_PORT, LED2_PIN);
    GPIO_setOutputLowOnPin(LED_PORT, LED3_PIN);
}

volatile bool stopPolling = false;
uint8_t detection_status = 0;
bool success = true;

void pollTouch(void)
{
		char zero = 0;
		char one = 1;
		char two = 2;
		char three = 3;
		char four = 4;
		char last_pressed = zero;
				int i = 0;
    success = writeI2C(SLAVE_ADDRESS, DETECTION_STATUS_REG, &detection_status, 0);

    while (!stopPolling)
    {
		char message[100];
        //__delay_cycles(300000); // ~100ms pause between transmissions
        success = readI2C(SLAVE_ADDRESS, DETECTION_STATUS_REG, &detection_status, 1);

        Timer_A_clearInterruptFlag(TIMER_A0_BASE);

        GPIO_setOutputLowOnPin(LED_PORT, LED0_PIN);
        GPIO_setOutputLowOnPin(LED_PORT, LED1_PIN);
        GPIO_setOutputLowOnPin(LED_PORT, LED2_PIN);
        GPIO_setOutputLowOnPin(LED_PORT, LED3_PIN);
			
				for (i = 0; i < 3000; i++);

        if (detection_status == 0x1)
        {
            GPIO_setOutputHighOnPin(LED_PORT, LED0_PIN);
						if (last_pressed != one) {
							printDebug(&one);
							last_pressed = one;
						};
        } else if (detection_status == 0x7)
        {
            GPIO_setOutputHighOnPin(LED_PORT, LED1_PIN);
						if (last_pressed != two) {
							printDebug(&two);
							last_pressed = two;
						}
        } else if (detection_status == 0xe)
        {
            GPIO_setOutputHighOnPin(LED_PORT, LED3_PIN);
						if (last_pressed != three) {
							printDebug(&three);
							last_pressed = three;
						}
        } else if (detection_status == 0x8)
        {
            GPIO_setOutputHighOnPin(LED_PORT, LED2_PIN);
						if (last_pressed != four) {
							printDebug(&four);
							last_pressed = four;
						}
        } else {
						if (last_pressed != zero) {
							printDebug(&zero);
							last_pressed = zero;
						}
				}
    }
}
