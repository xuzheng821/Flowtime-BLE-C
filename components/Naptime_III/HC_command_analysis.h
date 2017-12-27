#ifndef HC_COMMAND_ANALYSIS_H__
#define HC_COMMAND_ANALYSIS_H__

#include <stdint.h>
#include "app_error.h"
#include "softdevice_handler.h"
#include "ble_hci.h"
#include "ble_conn.h"
#include "HC_communicate_protocol.h"
#include "app_timer.h"


void ble_Com_ID_Analysis(uint8_t * p_data, uint16_t length);
void ble_Com_Shakehands_Analysis(uint8_t * p_data, uint16_t length);
void Handshake_agreement_first(uint8_t * p_data);
void Handshake_agreement_Second(void);
void Handshake_agreement_third(uint8_t * p_data);
static void Pair_confirm_timeout_handler(void * p_context);


#endif
