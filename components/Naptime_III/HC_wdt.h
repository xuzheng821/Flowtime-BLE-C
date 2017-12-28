#ifndef HC_WDT_H__
#define HC_WDT_H__

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "nrf_drv_wdt.h"
#include "app_util_platform.h"
#include "SEGGER_RTT_Conf.h"
#include "SEGGER_RTT.h"
#include "HC_timer.h"

void wdt_Init(void);

#endif 
