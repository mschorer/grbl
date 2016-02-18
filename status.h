/*
  status.h - does a status feedback by LED blink code
  Part of Grbl v0.9

  Copyright (c) 2014 ms@ms-ite.de  

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef status_h
#define status_h

// Define system header files and standard libraries used by Grbl
#include <inttypes.h>    
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Define Grbl configuration and shared header files
#include "config.h"
#include "cpu_map.h"
#include "gcode.h"

//
// Indexes into the array of colors
//
#define RED                     0
#define GREEN                   1
#define BLUE                    2

//
// Ratio for percent of full on that should be "true" white.
//
#define RED_WHITE_BALANCE        0.497f
#define GREEN_WHITE_BALANCE      0.6f
#define BLUE_WHITE_BALANCE       1.0f

//
// GPIO, Timer, Peripheral, and Pin assignments for the colors
//
#define RED_GPIO_PERIPH         SYSCTL_PERIPH_GPIOF
#define RED_TIMER_PERIPH        SYSCTL_PERIPH_TIMER0
#define BLUE_GPIO_PERIPH        SYSCTL_PERIPH_GPIOF
#define BLUE_TIMER_PERIPH       SYSCTL_PERIPH_TIMER1
#define GREEN_GPIO_PERIPH       SYSCTL_PERIPH_GPIOF
#define GREEN_TIMER_PERIPH      SYSCTL_PERIPH_TIMER1


#define RED_GPIO_BASE           GPIO_PORTF_AHB_BASE
#define RED_TIMER_BASE          TIMER0_BASE
#define BLUE_GPIO_BASE          GPIO_PORTF_AHB_BASE
#define BLUE_TIMER_BASE         TIMER1_BASE
#define GREEN_GPIO_BASE         GPIO_PORTF_AHB_BASE
#define GREEN_TIMER_BASE        TIMER1_BASE

#define RED_GPIO_PIN            GPIO_PIN_1
#define BLUE_GPIO_PIN           GPIO_PIN_2
#define GREEN_GPIO_PIN          GPIO_PIN_3


#define RED_GPIO_PIN_CFG        GPIO_PF1_T0CCP1
#define BLUE_GPIO_PIN_CFG       GPIO_PF2_T1CCP0
#define GREEN_GPIO_PIN_CFG      GPIO_PF3_T1CCP1

#define RED_TIMER_CFG           TIMER_CFG_B_PWM
#define BLUE_TIMER_CFG          TIMER_CFG_A_PWM
#define GREEN_TIMER_CFG         TIMER_CFG_B_PWM

#define RED_TIMER               TIMER_B
#define BLUE_TIMER              TIMER_A
#define GREEN_TIMER             TIMER_B

// Initialize the serial protocol
void status_init();

// Executes an internal status command, defined as a string starting with a '$'
void status_set( uint8_t status);

#endif
