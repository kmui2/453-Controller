#include "debug.h"
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>

#include <stdbool.h>
#include <string.h>

#include "i2c_driver.h"

/* Slave Address for I2C Slave */
// There is one preset I2C address of 0x12. This is not changeable
#define SLAVE_ADDRESS 0x12

#define DETECTION_STATUS_REG 4

// #define LED_PORT GPIO_PORT_P4
// #define LED0_PIN GPIO_PIN0
// #define LED1_PIN GPIO_PIN1
// #define LED2_PIN GPIO_PIN2
// #define LED3_PIN GPIO_PIN3


void initTouch(void)
{
    initI2C();
	
		/*
    GPIO_setAsOutputPin(LED_PORT, LED0_PIN);
    GPIO_setAsOutputPin(LED_PORT, LED1_PIN);
    GPIO_setAsOutputPin(LED_PORT, LED2_PIN);
    GPIO_setAsOutputPin(LED_PORT, LED3_PIN);

    GPIO_setOutputLowOnPin(LED_PORT, LED0_PIN);
    GPIO_setOutputLowOnPin(LED_PORT, LED1_PIN);
    GPIO_setOutputLowOnPin(LED_PORT, LED2_PIN);
    GPIO_setOutputLowOnPin(LED_PORT, LED3_PIN);
		*/
}

volatile bool stopPolling = false;
uint8_t detection_status = 0;
bool success = true;

void pollTouch(void)
{
	
        success = writeI2C(SLAVE_ADDRESS, DETECTION_STATUS_REG, &detection_status, 0);
	
    while (!stopPolling)
    {
        /*switch (RXData)
        {
        case 0:
            GPIO_setOutputHighOnPin(LED_PORT, LED0_PIN);
            GPIO_setOutputLowOnPin(LED_PORT, LED1_PIN);
            GPIO_setOutputLowOnPin(LED_PORT, LED2_PIN);
            GPIO_setOutputLowOnPin(LED_PORT, LED3_PIN);
            break;
        case 1:

            GPIO_setOutputLowOnPin(LED_PORT, LED0_PIN);
            GPIO_setOutputHighOnPin(LED_PORT, LED1_PIN);
            GPIO_setOutputLowOnPin(LED_PORT, LED2_PIN);
            GPIO_setOutputLowOnPin(LED_PORT, LED3_PIN);
            break;
        case 2:

            GPIO_setOutputLowOnPin(LED_PORT, LED0_PIN);
            GPIO_setOutputLowOnPin(LED_PORT, LED1_PIN);
            GPIO_setOutputHighOnPin(LED_PORT, LED2_PIN);
            GPIO_setOutputLowOnPin(LED_PORT, LED3_PIN);
            break;
        case 4:

            GPIO_setOutputLowOnPin(LED_PORT, LED0_PIN);
            GPIO_setOutputLowOnPin(LED_PORT, LED1_PIN);
            GPIO_setOutputHighOnPin(LED_PORT, LED2_PIN);
            GPIO_setOutputLowOnPin(LED_PORT, LED3_PIN);
            break;
        default:

            GPIO_setOutputLowOnPin(LED_PORT, LED0_PIN);
            GPIO_setOutputLowOnPin(LED_PORT, LED1_PIN);
            GPIO_setOutputLowOnPin(LED_PORT, LED2_PIN);
            GPIO_setOutputLowOnPin(LED_PORT, LED3_PIN);
        }
				*/
        success = readI2C(SLAVE_ADDRESS, DETECTION_STATUS_REG, &detection_status, 1);
    }
}
