#ifndef HC_DATA_FLASH_H__
#define HC_DATA_FLASH_H__

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_log.h"
#include "ble.h"
#include "pstorage.h"
#include "SEGGER_RTT_Conf.h"
#include "SEGGER_RTT.h"
#include "app_error.h"
#include "ble_com.h"
#include "protocol_analysis.h"

void flash_callback(pstorage_handle_t * handle,uint8_t op_code,uint32_t result,uint8_t * p_data, uint32_t data_len);
void flash_init(void);
void Read_User_ID(void);
void Story_User_ID(void);
void Story_Device_ID(void);
void Story_SN(void);
void Read_device_id_sn(void);
void delete_User_id(void);

#endif
