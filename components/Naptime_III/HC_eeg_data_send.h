#ifndef HC_EEG_SEND_H
#define HC_EEG_SEND_H

#include <stdint.h>
#include <stdint.h>
#include "ble_eeg.h"
#include "ble_bas.h"
#include "ble_com.h"
#include "ble_gatt.h"
#include "app_error.h"
#include "app_timer.h"
#include "softdevice_handler.h"
#include "HC_ads129x_driver.h"
#include "sdk_errors.h"
#include "SEGGER_RTT_Conf.h"
#include "SEGGER_RTT.h"

void ble_send_data(void);
void ble_send_more_data(void);
void ble_state_send(uint8_t pdata);

#endif



