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

/**@file
 *
 * @defgroup ble_sdk_srv_com Nordic UART Service
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Nordic UART Service implementation.
 *
 * @details The Nordic UART Service is a simple GATT-based service with Up and Down characteristics.
 *          Data received from the peer is passed to the application, and the data received
 *          from the application of this service is sent to the peer as Handle Value
 *          Notifications. This module demonstrates how to implement a custom GATT-based
 *          service and characteristics using the SoftDevice. The service
 *          is used by the application to send and receive ASCII text strings to and from the
 *          peer.
 *
 * @note The application must propagate SoftDevice events to the Nordic UART Service module
 *       by calling the ble_com_on_ble_evt() function from the ble_stack_handler callback.
 */

#ifndef BLE_CON_H__
#define BLE_CON_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_UUID_CON_SERVICE 0xFF10                      /**< The UUID of the Nordic UART Service. */
#define BLE_CON_MAX_DATA_LEN (GATT_MTU_SIZE_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/* Forward declaration of the ble_com_t type. */
typedef struct ble_conn_s ble_conn_t;

/**@brief Nordic UART Service event handler type. */
typedef void (*ble_conn_data_handler_t) (ble_conn_t * p_conn, uint8_t * p_data, uint16_t length);

/**@brief Nordic UART Service initialization structure.
 *
 * @details This structure conntains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_conn_init
 *          function.
 */
typedef struct
{
    ble_conn_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_conn_init_t;

/**@brief Nordic UART Service structure.
 *
 * @details This structure conntains status information related to the service.
 */
struct ble_conn_s
{
    uint8_t                  uuid_type;               /**< UUID type for Nordic UART Service Base UUID. */
    uint16_t                 service_handle;          /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t State_up_handles;              /**< Handles related to the Up characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t Shakehands_handles;              /**< Handles related to the Up characteristic (as provided by the SoftDevice). */
	  ble_gatts_char_handles_t ID_down_handles;              /**< Handles related to the Down characteristic (as provided by the SoftDevice). */
    uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the Down characteristic.*/
    ble_conn_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
};

/**@brief Function for initializing the Nordic UART Service.
 *
 * @param[out] p_conn      Nordic UART Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_conn_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_conn or p_conn_init is NULL.
 */
uint32_t ble_conn_init(ble_conn_t * p_conn, const ble_conn_init_t * p_conn_init);

/**@brief Function for handling the Nordic UART Service's BLE events.
 *
 * @details The Nordic UART Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the Nordic UART Service event handler of the
 * application if necessary.
 *
 * @param[in] p_con       Nordic UART Service structure.
 * @param[in] p_ble_evt   Event received from the SoftDevice.
 */
void ble_conn_on_ble_evt(ble_conn_t * p_conn, ble_evt_t * p_ble_evt);

/**@brief Function for sending a string to the peer.
 *
 * @details This function sends the input string as an Down characteristic notification to the
 *          peer.
 *
 * @param[in] p_con       Pointer to the Nordic UART Service structure.
 * @param[in] p_string    String to be sent.
 * @param[in] length      Length of the string.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_Shakehands_string_send(ble_conn_t * p_conn, uint8_t * p_string, uint16_t length);
uint32_t ble_State_string_send(ble_conn_t * p_conn, uint8_t * p_string, uint16_t length);


#endif // BLE_con_H__

/** @} */
