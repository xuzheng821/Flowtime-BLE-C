#ifndef _HC_UART_H
#define _HC_UART_H

#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "app_uart.h"
#include "Naptime_III.h"
#include "nordic_common.h"
#include "ble_hci.h"
#include "softdevice_handler.h"
#include "app_util_platform.h"
#include "SEGGER_RTT_Conf.h"
#include "SEGGER_RTT.h"
#include "protocol_analysis.h"

void uart_data_Analysis(uint8_t *pdata);
void Uart_init(void);

#endif
