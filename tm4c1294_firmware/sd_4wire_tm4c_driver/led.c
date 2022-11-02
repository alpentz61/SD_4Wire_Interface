/*
 * led.c:
 *
 * Implements a demo of turning the LED on using only direct register access.
 * We could easily use the library for this, but this is a first step of direct
 * register access in the TM4C.
 *
 *  Created on: Jul 6, 2019
 *      Author: Andrew
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/tm4c1294kcpdt.h"

void led_init(void)
{
    // Enable the clock to Port N
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R | 0x00001000;


    // Wait until the peripheral clock is running
    while (!SYSCTL_RCGCGPIO_R)
    {
    }

    //Set the direction of Port 1 to output (all others inputs)
    GPIO_PORTN_DIR_R = 0x00000002;

    // Set AFSEL to GPIO (alternate function selection).
    // Note: Not needed for setting a pin as an GPIO (this is the default setting)

    //Set drive strengths:
    //Disable 2ma drive for pin 1, and enable 8ma drive instead
    //GPIO_PORTN_DR2R_R = GPIO_PORTN_DR2R_R & 0xfffffffd;
    //GPIO_PORTN_DR8R_R = GPIO_PORTN_DR8R_R | 0x00000002;

    //Unlock the port so we can do protected writes and set pin 1 for commit.
    GPIO_PORTN_LOCK_R = 0x4C4F434B;
    GPIO_PORTN_CR_R = 0x00000002;

    //Set pull up, pull down, open drain, slew (not needed)

    //Enable digital functionality for pin 1
    GPIO_PORTN_DEN_R = GPIO_PORTN_DEN_R | 0x00000002;

    //Relock now that we are done
    GPIO_PORTN_LOCK_R = 0;
}

void led_on(void)
{
    GPIO_PORTN_DATA_R |= (1 << 1);
}

void led_off(void)
{
    GPIO_PORTN_DATA_R &= ~(1<<1);
}



