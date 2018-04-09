#ifndef HC_LED_H__
#define HC_LED_H__

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "HC_pwm.h"
#include "HC_timer.h"

typedef enum
{
	  BSP_INDICATE_IDLE,
    BSP_INDICATE_POWER_ON, 
    BSP_INDICATE_POWER_OFF,
	  BSP_INDICATE_CONNECTED,
	  BSP_INDICATE_WITH_WHITELIST,
	  BSP_INDICATE_WITHOUT_WHITELIST,
	  BSP_INDICATE_Battery_LOW_ADV,
	  BSP_INDICATE_Battery_LOW_CONN,
	  BSP_INDICATE_Battery_CHARGING,
	  BSP_INDICATE_Battery_CHARGEOVER,
	  BSP_INDICATE_FACTORY_LED_TEST,
} led_indication_t;

void LED_timeout_restart(void);
uint32_t bsp_led_indication(led_indication_t indicate);
void leds_state_update(void);

#endif 
