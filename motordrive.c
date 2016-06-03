/*
 * This file is part of the yurt_motordrive project.
 *
 * Copyright (C) 2010 Vaibhav Kapoor <vaibhav.kapoor06@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/stm32/usart.h>

// #include "ring.h"
#include "usart.h"
#include "systick.h"


int main(void)
{
	usbacm_init();
	systick_setup();
	
	while (1)
		usbacm_poll();
}
void sys_tick_handler(void)
{
    static int counter = 0;
    static float fcounter = 0.0;
    static double dcounter = 0.0;
    static u32 temp32 = 0;
    
    temp32++;
    /*
        * We call this handler every 1ms so we are updating the PID control loop.
        * every 100ms / 100Hz.
        */
    if (temp32 == 100)
    {
        counter++;
        fcounter += 0.01;
        dcounter += 0.01;
        temp32 = 0;
    }

}
