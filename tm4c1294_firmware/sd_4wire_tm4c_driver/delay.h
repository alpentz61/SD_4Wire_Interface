/*
 * delay.h
 *
 * A single header file which provides more convenient macros for dealing
 * with the system delay loop functions.
 *
 *  Created on: Jul 28, 2019
 *      Author: Andrew
 */
#ifndef DEBUG_DELAY_H_
#define DEBUG_DELAY_H_

#include "driverlib/sysctl.h"

extern uint32_t MAIN_sys_clock;

#define DELAY_S(delay_s)    SysCtlDelay(delay_s * (MAIN_sys_clock / 3))
#define DELAY_MS(delay_ms)  SysCtlDelay(delay_ms * (MAIN_sys_clock / 3000))
#define DELAY_US(delay_us)  SysCtlDelay(delay_us * (MAIN_sys_clock / 3000000))

#endif /* DEBUG_DELAY_H_ */
