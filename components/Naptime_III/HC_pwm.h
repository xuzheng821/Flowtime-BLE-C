#ifndef HC_PWM_H
#define HC_PWM_H

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "app_pwm.h"
#include "nrf_delay.h"
#include "Naptime_III.h"

void led_pwm_init(void);
void PWM_uint(void);
void LED_RED(uint8_t fre,uint8_t PWM);
void LED_BLUE(uint8_t fre,uint8_t PWM);
void LED_GREEN(uint8_t PWM);
void LED_OFF(void);
void PWM1_fre_changge(uint8_t fre);
void PWM3_fre_changge(uint8_t fre);

#endif
