#ifndef HC_PWM_H
#define HC_PWM_H

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_pwm.h"
#include "app_error.h"
#include "Naptime_III.h"

void led_pwm_init(void);
void LED_ON_duty(uint8_t RED_duty,uint8_t GREEN_duty,uint8_t BLUE_duty);
void led_off(void);
void pwm_uint(void);

#endif
