#ifndef __MOTOR_H__
#define __MOTOR_H__
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/exti.h>

#define system_freq 	168000000 	//168MHz, System Clock frequency
#define pwm_freq		14000	//14KHz frequency of the PWM wave, integer value
#define prescale 		1	//prescale value, used to be able to have a bigger range of values for the pwm wave periods, see below.
#define pwm_period_ARR 	system_freq/(pwm_freq*(prescale+1))	//(1312)this defines de pwm wave period based on the frequency of the clock
#define SERVO_OFFSET	19000

struct motor_t {
	enum driver_t {IR2104, POLOLU, POLOLU2, SERVO} driver; /*Controls how the output PWMH/PWML are outputted*/
	enum {CW, CCW} direction;/*Clockwise:CW, Counter-Clockwise:CCW*/
	uint16_t sp;	/*Set-point for actutators*/	
	uint32_t timer; /*Timer Peripheral*/
	uint32_t gpioport;
	uint32_t gpio_h, gpio_l;
	enum tim_oc_id oc_high; /*Timer output*/
	enum tim_oc_id oc_low; /*Timer output*/
	uint8_t safety;
	// uint32_t pwm_freq; /*PWM Frequency used*/
};
void motor_init(
  struct motor_t *m, 
  uint32_t timer,
  enum tim_oc_id oc_high, 
  enum tim_oc_id oc_low,
  enum driver_t driver,
  uint32_t gpioport,
  uint16_t gpio_h,
  uint16_t gpio_l,
  uint8_t safety);
void motor_set(struct motor_t *m);
void setup_periodic_timer(uint32_t timer, unsigned int freq_in_hz);
// void timer_setup(uint32_t timer, enum tim_oc_id oc_high, enum tim_oc_id oc_low);
void timer3_setup(void);
void timer4_setup(void);



#endif //__MOTOR_H__