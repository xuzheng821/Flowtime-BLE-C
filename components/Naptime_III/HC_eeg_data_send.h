#ifndef HC_EEG_SEND_H
#define HC_EEG_SEND_H

#include <stdint.h>
#include <stdint.h>
#include "ble_eeg.h"
#include "ble_gatt.h"
#include "app_error.h"
#include "app_timer.h"
#include "softdevice_handler.h"

uint32_t ble_send_data(uint8_t *pdata, uint8_t Send_Len);
void ble_send_more_data(uint8_t *pdata, uint8_t Send_Len);
void ble_state_send(uint8_t pdata);

#endif



