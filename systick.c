#include "systick.h"

void systick_setup(void)
{
	/* 168MHz / 8 => 21000000 counts per second. */
	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);

	/* 21000000/21000 = 1000 overflows per second - every 1ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(21000*3);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}