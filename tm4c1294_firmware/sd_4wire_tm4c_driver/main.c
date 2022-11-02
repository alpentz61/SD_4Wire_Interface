#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"

#include "delay.h"
#include "led.h"
#include "sd.h"

//=============================================================================
// Constants

//=============================================================================
// Macros

//=============================================================================
// Private Definitions

//=============================================================================
// External Declarations

//=============================================================================
// Public Definitions
uint32_t MAIN_sys_clock;

volatile uint32_t sys_timer = 0;
volatile uint16_t resp_timer = 0;
volatile uint16_t timer_1 = 0;

//=============================================================================
// Private Function Prototypes

//=============================================================================
// Main Function

int main(void)
{
    uint32_t stat = STAT_SUCCESS;

    int led_half_period = 1000;

    //Set clock frequency to 120 MHz:
    MAIN_sys_clock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ |
                       SYSCTL_OSC_MAIN |
                       SYSCTL_USE_PLL |
                       SYSCTL_CFG_VCO_480,
                       120000000);

    // Configure SysTick for a 100Hz interrupt.
    SysTickPeriodSet(MAIN_sys_clock / 100);
    SysTickEnable();
    SysTickIntEnable();

    // Initialize the LED
    led_init();

    // Initialize the peripheral hardware necessary to communicate with the SD card
    SD_init();

    // Initialize the card.
   stat =  SD_card_init();

    // After completing the initialization, we will either blink the LED slowly on success
    // or quickly if failure occurred
    if (STAT_SUCCESS != stat)
        led_half_period = 100;

    while(1)
    {

        DELAY_MS(led_half_period);

        led_on();

        DELAY_MS(led_half_period);

        led_off();
    }
}

//=============================================================================
// Private Function Definitions

void SysTickHandler(void)
{
    if (sys_timer > 0)
    {
        sys_timer--;
    }

    if (timer_1 > 0)
    {
        timer_1--;
    }

    if (resp_timer > 0)
    {
        resp_timer--;
    }
}
