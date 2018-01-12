/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA抯 Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

#include "ble_eeg.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "sdk_common.h"
#include "HC_ads129x_driver.h"

extern bool Global_connected_state;
 
#define BLE_UUID_EEG_EEG_DATA_CHARACTERISTIC   0xFF31                      /**< The UUID of the RX Characteristic. */
#define BLE_UUID_EEG_ELE_STATE_CHARACTERISTIC  0xFF32

#define EEG_BASE_UUID  {{0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xCD, 0xAB, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} /**< Used vendor specific UUID. */

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_EEG       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_EEG_t * p_EEG, ble_evt_t * p_ble_evt)
{
    p_EEG->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_EEG       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_EEG_t * p_EEG, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_EEG->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_EEG       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_EEG_t * p_EEG, ble_evt_t * p_ble_evt)
{
		ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    if ((p_evt_write->handle == p_EEG->eeg_handles.cccd_handle)
        &&(p_evt_write->len == 2) && Global_connected_state )   //握手成功才能notify数据 
				{
						if (ble_srv_is_notification_enabled(p_evt_write->data))
						{
								p_EEG->is_eeg_notification_enabled = true;

						}
						else
						{
								p_EEG->is_eeg_notification_enabled = false;
						}
				}
				
    if ((p_evt_write->handle == p_EEG->ele_state_handles.cccd_handle)
        &&(p_evt_write->len == 2) && Global_connected_state)    //握手成功才能notify数据 
				{
						if (ble_srv_is_notification_enabled(p_evt_write->data))
						{
								p_EEG->is_state_notification_enabled = true;
						}
						else
						{
								p_EEG->is_state_notification_enabled = false;
						}
				}
}


void ble_eeg_on_ble_evt(ble_EEG_t * p_EEG, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_EEG, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_EEG, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_EEG, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static uint32_t eeg_ele_state_char_add(ble_EEG_t            * p_EEG,
                                      const ble_EEG_init_t * p_EEG_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_EEG->uuid_type;
    ble_uuid.uuid = BLE_UUID_EEG_ELE_STATE_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));
		
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;

    return sd_ble_gatts_characteristic_add(p_EEG->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_EEG->ele_state_handles);
}



/**@brief Function for adding the Heart Rate Measurement characteristic.
 *
 * @param[in]   p_EEG        Heart Rate Service structure.
 * @param[in]   p_EEG_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t eeg_data_char_add(ble_EEG_t            * p_EEG,
                                                const ble_EEG_init_t * p_EEG_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_EEG->uuid_type;
    ble_uuid.uuid = BLE_UUID_EEG_EEG_DATA_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));
		
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;

    return sd_ble_gatts_characteristic_add(p_EEG->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_EEG->eeg_handles);
}

uint32_t ble_EEG_init(ble_EEG_t * p_EEG, const ble_EEG_init_t * p_EEG_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t EEG_base_uuid = EEG_BASE_UUID;
	
    // Initialize service structure
    p_EEG->evt_handler                 = p_EEG_init->evt_handler;
    p_EEG->conn_handle                 = BLE_CONN_HANDLE_INVALID;
	  p_EEG->is_eeg_notification_enabled = false;
	  p_EEG->is_state_notification_enabled = false;
    p_EEG->last_state = 0x24;
	
    err_code = sd_ble_uuid_vs_add(&EEG_base_uuid, &p_EEG->uuid_type);
    VERIFY_SUCCESS(err_code);
		
	  ble_uuid.type = p_EEG->uuid_type;
		ble_uuid.uuid = BLE_UUID_EEG_SERVICE;
	
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_EEG->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add ele state characteristic
    err_code = eeg_ele_state_char_add(p_EEG, p_EEG_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
				
		// Add eeg data characteristic
    err_code = eeg_data_char_add(p_EEG, p_EEG_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
    return NRF_SUCCESS;

}

uint32_t ble_EEG_DATA_send(ble_EEG_t * p_EEG, uint8_t * p_string, uint16_t length)
{
	  uint32_t err_code = NRF_SUCCESS;
    ble_gatts_hvx_params_t hvx_params;
    VERIFY_PARAM_NOT_NULL(p_EEG);

	  // Send value if connected and notifying.
    if ((p_EEG->conn_handle != BLE_CONN_HANDLE_INVALID) && p_EEG->is_eeg_notification_enabled)
    {
       memset(&hvx_params, 0, sizeof(hvx_params));
       hvx_params.handle = p_EEG->eeg_handles.value_handle;
       hvx_params.p_data = p_string;
       hvx_params.p_len  = &length;
       hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
       return sd_ble_gatts_hvx(p_EEG->conn_handle, &hvx_params);
		}
    return err_code;
}

uint32_t ble_EEG_ELE_STATE_send(ble_EEG_t *p_EEG, uint8_t state, uint16_t length)
{
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
    VERIFY_PARAM_NOT_NULL(p_EEG);
	
    if (p_EEG == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    if (state != p_EEG->last_state)
    {
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = &state;

        // Update database.
        err_code = sd_ble_gatts_value_set(p_EEG->conn_handle,
                                          p_EEG->ele_state_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            // Save new battery value.
            p_EEG->last_state = state;
        }
        else
        {
            return err_code;
        }

        // Send value if connected and notifying.
        if ((p_EEG->conn_handle != BLE_CONN_HANDLE_INVALID) && p_EEG->is_state_notification_enabled)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_EEG->ele_state_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            return sd_ble_gatts_hvx(p_EEG->conn_handle, &hvx_params);
				}
		}
    return err_code;
}
