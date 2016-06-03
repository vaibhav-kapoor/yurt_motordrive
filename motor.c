#include <libopencm3/stm32/f4/gpio.h>

#include "motor.h"

/*! \details This function initializes the data in a PID structure.
 *
 */

void motor_init(
  struct motor_t *m, 
  uint32_t timer,
  enum tim_oc_id oc_high, 
  enum tim_oc_id oc_low,
  enum driver_t driver,
  uint32_t gpioport,
  uint16_t gpio_h,
  uint16_t gpio_l,
  uint8_t safety)
{
  m->driver = driver;
  m->timer=timer;
  m->direction = CW;
  m->oc_high = oc_high;
  m->oc_low = oc_low;
  m->gpioport = gpioport;
  m->gpio_h = gpio_h;
  m->gpio_l = gpio_l;
  m->safety = safety;

  m->sp=0;
}

void motor_set(struct motor_t *m)
{
  uint32_t gpioport;
  uint16_t gpio_h,gpio_l;  

  /*Specify safety levels*/
  switch (m->safety)
  {
    /*Zero means no safety limit*/
    case 0:
      break;
    case 1:
      if (m->sp > 32767)
        m->sp = 32767;
      break;
    case 2:
      if (m->sp > 16383)
        m->sp = 16383;
      break;
    case 3:
      if (m->sp > 8191)
        m->sp = 8191;
      break;
    case 4:
      if (m->sp > 4095)
        m->sp = 4095;
      break;
  }

  switch(m->driver)
  {
    case POLOLU:
      if(m->direction == CW)
      {
        timer_set_oc_value(m->timer, m->oc_high, m->sp);
        gpio_set(m->gpioport, m->gpio_h);
      }
      else 
      {
        timer_set_oc_value(m->timer, m->oc_high, m->sp);
        // gpio_clear(GPIOC, GPIO3);
        gpio_clear(m->gpioport, m->gpio_h);
      }
      break;

    case POLOLU2:
      timer_set_oc_value(m->timer, m->oc_high, m->sp);
      if(m->direction == CW)
      {
        gpio_set(m->gpioport, m->gpio_h);
        gpio_clear(m->gpioport, m->gpio_l);
      }
      else
      {
        gpio_set(m->gpioport, m->gpio_l);
        gpio_clear(m->gpioport, m->gpio_h);
      }
      break;
    case SERVO:
      timer_set_oc_value(m->timer, m->oc_high, m->sp);
      break;

  }
  
}

void timer4_setup()
{
  
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
  /*
   * Set TIM3 channel output pins to
   * 'output alternate function push-pull'.
   */
  gpio_mode_setup(GPIOB, GPIO_MODE_AF,
                GPIO_PUPD_NONE, GPIO6 | GPIO7);
  /*TIM3 CH3, CH4 as alternate functions*/
  gpio_set_af(GPIOB, GPIO_AF2, GPIO6 | GPIO7);
  
  gpio_mode_setup(GPIOB, GPIO_MODE_AF,
                GPIO_PUPD_NONE, GPIO8 | GPIO9);
  /*TIM3 CH3, CH4 as alternate functions*/
  gpio_set_af(GPIOB, GPIO_AF2, GPIO8 | GPIO9);  

  /* Enable TIM3 clock. */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);
  setup_periodic_timer(TIM4, 15000);
  // timer_enable_counter(TIM3);
  // /* Reset TIM1 peripheral. */
  timer_reset(TIM4);
  // /* Timer global mode:
  //  * - No divider
  //  * - Alignment, center with mode 3* see manual for details
  //  * - Direction up
  //  */
  timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
  
  // /* Set prescaler value for TIM1. */
  // timer_set_prescaler(TIM3, prescale);
  timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM4, TIM_OC3, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM1);  
  
  // /* Reenable outputs. */
  timer_enable_oc_output(TIM4, TIM_OC1);
  timer_enable_oc_output(TIM4, TIM_OC2);
  timer_enable_oc_output(TIM4, TIM_OC3);
  timer_enable_oc_output(TIM4, TIM_OC4);  
  // timer_enable_break_main_output(TIM3);
  // /* ARR reload enable. */
  // timer_enable_preload(TIM3); 
  // /* Pnce the configuration is done and the lines are enabled again, */ 
  // /* we reload the TIM3 */

  // /* Set the initial capture compare value for OC1. */
  // //this will be changed later to: initial_duty_cycle*pwm_period_ARR
  timer_set_oc_value(TIM4, TIM_OC1, 0);
  timer_set_oc_value(TIM4, TIM_OC2, 0);
  timer_set_oc_value(TIM4, TIM_OC3, 0);
  timer_set_oc_value(TIM4, TIM_OC4, 0);  
  // /* Sets the Period for TIM1 PWM, named ARR. Set the defines in motordrive.c */
  // timer_set_period(TIM3,pwm_period_ARR);
  // /* Sets TIM1 Continuous mode. */
  // // timer_continuous_mode(TIM3);
  timer_enable_counter(TIM4);
  
  /* Enable capture/compare interrupt. */
  timer_enable_irq(TIM4, TIM_SR_UIF);
  
  nvic_enable_irq(NVIC_TIM4_IRQ);

}

void timer3_setup(void)
{
  // gpio_set(GPIOD, GPIO12);
  /*Enable AF clock*/
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
  /*
   * Set TIM3 channel output pins to
   * 'output alternate function push-pull'.
   */
  gpio_mode_setup(GPIOB, GPIO_MODE_AF,
                GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO4 | GPIO5);
  /*TIM3 CH3, CH4 as alternate functions*/
  gpio_set_af(GPIOB, GPIO_AF2, GPIO0 | GPIO1 | GPIO4 | GPIO5);
  
  /* Enable TIM3 clock. */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);
  // setup_periodic_timer(TIM3, 15000);
  // PWM Freq: 168 000 000 Hz / 10000 = 1 MHz
  timer_reset(TIM3);
  timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
  timer_enable_break_main_output(TIM3);
  timer_set_prescaler(TIM3, 1);
  timer_set_period(TIM3, 51000);
  // /* Timer global mode:
  //  * - No divider
  //  * - Alignment, center with mode 3* see manual for details
  //  * - Direction up
  //  */
  // timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
  
  timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
  timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);  
  // /* Reenable outputs. */
  timer_enable_oc_output(TIM3, TIM_OC1);
  timer_enable_oc_output(TIM3, TIM_OC2);
  timer_enable_oc_output(TIM3, TIM_OC3);
  timer_enable_oc_output(TIM3, TIM_OC4);  
  // timer_enable_break_main_output(TIM3);
  // /* ARR reload enable. */
  // timer_enable_preload(TIM3); 
  // /* Pnce the configuration is done and the lines are enabled again, */ 
  // /* we reload the TIM3 */

  // /* Set the initial capture compare value for OC1. */
  // //this will be changed later to: initial_duty_cycle*pwm_period_ARR
  timer_set_oc_value(TIM3, TIM_OC1, 0);
  timer_set_oc_value(TIM3, TIM_OC2, 0);
  timer_set_oc_value(TIM3, TIM_OC3, 0);
  timer_set_oc_value(TIM3, TIM_OC4, 0);
  /* Sets the Period for TIM1 PWM, named ARR. Set the defines in motordrive.c */
  // timer_set_period(TIM3,pwm_period_ARR);
  // /* Sets TIM1 Continuous mode. */
  timer_enable_counter(TIM3);
  
  /* Enable capture/compare interrupt. */
  timer_enable_irq(TIM3, TIM_SR_UIF);
  //DO NOT ENABLE 2 timer IRQs!
  
  nvic_enable_irq(NVIC_TIM3_IRQ);

}


void tim3_isr (void)//TIM3 CC interruption
{
  timer_clear_flag(TIM3, TIM_SR_UIF);
}


void tim4_isr (void)//TIM3 CC interruption
{
  timer_clear_flag(TIM4, TIM_SR_UIF);
}

void tim2_isr (void)//TIM3 CC interruption
{
  timer_clear_flag(TIM2, TIM_SR_UIF);
  printf("Blah\r\n");
}

void setup_periodic_timer(uint32_t timer, unsigned int freq_in_hz)
{
    unsigned int prescaler = 1;
    while (system_freq / prescaler / freq_in_hz > 0xffff)
        prescaler *= 2;
    timer_reset(timer);
    timer_set_prescaler(timer, prescaler-1);
    // printf("%u\r\n", system_freq / prescaler / freq_in_hz);
    timer_set_period(timer, system_freq / prescaler / freq_in_hz);
    timer_direction_down(timer);
    timer_enable_preload(timer);
}
