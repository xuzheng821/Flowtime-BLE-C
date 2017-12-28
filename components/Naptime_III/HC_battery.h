#ifndef HC_BATTERY_H__
#define HC_BATTERY_H__

#include <stdint.h>
#include <string.h>
#include "ble_bas.h"
#include "ble_gatt.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_saadc.h"
#include "nrf_drv_saadc.h"
#include "softdevice_handler.h"
#include "HC_led.h"
#include "Naptime_III.h"
#include "nrf_delay.h"
#include "HC_key.h"
#include "HC_timer.h"
#include "SEGGER_RTT_Conf.h"
#include "SEGGER_RTT.h"

void saadc_callback(nrf_drv_saadc_evt_t const * p_event);																	 
void saadc_init(void);
void battery_level_update(void);
void battery_level_meas_timeout_handler(void * p_context);
void battery_vol_send_timer_init(void);
void battery_measure_timer_start(void);
void battery_measure_timer_stop(void);
void Power_Check(void);
void ble_battory_serv_init(void);
void charging_check(void);
uint8_t connected_power_check(void);

#endif
