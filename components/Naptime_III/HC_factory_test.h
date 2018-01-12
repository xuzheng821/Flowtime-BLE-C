#ifndef HC_FACTORY_TEST_H_
#define HC_FACTORY_TEST_H_

#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "SEGGER_RTT_Conf.h"
#include "SEGGER_RTT.h"
#include "HC_uart.h"
#include "HC_led.h"
#include "ble_com.h"
#include "protocol_analysis.h"
#include "HC_ads129x_driver.h"
#include "app_error.h"
#include "Naptime_III.h"

void bootup_check(void);
void led_test(void);
void App_Nap_data_Analysis(uint8_t *pdata);

#endif

