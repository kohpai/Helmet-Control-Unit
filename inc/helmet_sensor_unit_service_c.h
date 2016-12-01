/*
 * Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

/**@file
 *
 * @defgroup ble_sdk_srv_hsus_c   Nordic UART Service Client
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Nordic UART Service Client module.
 *
 * @details  This module contains the APIs and types exposed by the Nordic UART Service Client
 *           module. These APIs and types can be used by the application to perform discovery of
 *           the Nordic UART Service at the peer and interact with it.
 *
 * @note     The application must propagate BLE stack events to this module by calling
 *           ble_hsus_c_on_ble_evt().
 *
 */


#ifndef BLE_HSUS_C_H__
#define BLE_HSUS_C_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_gatt.h"
#include "ble_db_discovery.h"

#define BLE_UUID_HSU_BASE_UUID              {{ 0x63, 0xC3, 0xD4, 0x1D, \
                                               0x74, 0xCD, 0x42, 0x5B, \
                                               0x9B, 0xD6, 0x89, 0x10, \
                                               0x96, 0xA3, 0xC1, 0xEA }}

#define BLE_UUID_HSU_SERVICE_UUID           0x0001
#define BLE_UUID_ERR_CHARACTERISTIC_UUID    0x0101
#define BLE_UUID_ACC_CHARACTERISTIC_UUID    0x0102
#define BLE_UUID_GYRO_CHARACTERISTIC_UUID   0x0103
#define BLE_UUID_MAG_CHARACTERISTIC_UUID    0x0104
#define BLE_UUID_HRM_CHARACTERISTIC_UUID    0x0105

#define BLE_HSUS_MAX_DATA_LEN           (GATT_MTU_SIZE_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */


/**@brief HSUS Client event type. */
typedef enum
{
    BLE_HSUS_C_EVT_DISCOVERY_COMPLETE = 1,
    BLE_HSUS_C_EVT_DISCONNECTED,
    BLE_HSUS_C_EVT_READ_RSP,
} ble_hsus_c_evt_type_t;


/**@brief Handles on the connected peer device needed to interact with it.
*/
typedef struct {
    uint16_t                err_handle;
    uint16_t                acc_handle;
    uint16_t                gyro_handle;
    uint16_t                mag_handle;
    uint16_t                hrm_handle;
} ble_hsus_c_handles_t;


/**@brief Structure containing the HSUS event data received from the peer. */
typedef struct {
    ble_hsus_c_evt_type_t   evt_type;
    uint16_t                conn_handle;
    uint8_t                 *p_data;
    uint8_t                 data_len;
    ble_hsus_c_handles_t    handles;     /**< Handles on which the Nordic Uart service characteristics was discovered on the peer device. This will be filled if the evt_type is @ref BLE_HSUS_C_EVT_DISCOVERY_COMPLETE.*/
} ble_hsus_c_evt_t;


// Forward declaration of the ble_hsus_t type.
typedef struct ble_hsus_c_s ble_hsus_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module to receive events.
 */
typedef void (* ble_hsus_c_evt_handler_t)(ble_hsus_c_t * p_ble_hsus_c, const ble_hsus_c_evt_t * p_evt);


/**@brief HSUS Client structure.
 */
struct ble_hsus_c_s
{
    uint8_t                     uuid_type;          /**< UUID type. */
    uint16_t                    conn_handle;        /**< Handle of the current connection. Set with @ref ble_hsus_c_handles_assign when connected. */
    ble_hsus_c_handles_t        handles;            /**< Handles on the connected peer device needed to interact with it. */
    ble_hsus_c_evt_handler_t    evt_handler;        /**< Application event handler to be called when there is an event related to the HSUS. */
};

/**@brief HSUS Client initialization structure.
 */
typedef struct {
    ble_hsus_c_evt_handler_t evt_handler;
} ble_hsus_c_init_t;


/**@brief     Function for initializing the Nordic UART client module.
 *
 * @details   This function registers with the Database Discovery module
 *            for the HSUS. Doing so will make the Database Discovery
 *            module look for the presence of a HSUS instance at the peer when a
 *            discovery is started.
 *
 * @param[in] p_ble_hsus_c      Pointer to the HSUS client structure.
 * @param[in] p_ble_hsus_c_init Pointer to the HSUS initialization structure containing the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS If the module was initialized successfully. Otherwise, an error
 *                        code is returned. This function
 *                        propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_hsus_c_init(ble_hsus_c_t * p_ble_hsus_c, ble_hsus_c_init_t * p_ble_hsus_c_init);


/**@brief Function for handling events from the database discovery module.
 *
 * @details This function will handle an event from the database discovery module, and determine
 *          if it relates to the discovery of HSUS at the peer. If so, it will
 *          call the application's event handler indicating that HSUS has been
 *          discovered at the peer. It also populates the event with the service related
 *          information before providing it to the application.
 *
 * @param[in] p_ble_hsus_c Pointer to the HSUS client structure.
 * @param[in] p_evt       Pointer to the event received from the database discovery module.
 */
 void ble_hsus_c_on_db_disc_evt(ble_hsus_c_t * p_ble_hsus_c, ble_db_discovery_evt_t * p_evt);


/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function handles the BLE events received from the SoftDevice. If a BLE
 *            event is relevant to the HSUS module, it is used to update
 *            internal variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_hsus_c Pointer to the HSUS client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event.
 */
void ble_hsus_c_on_ble_evt(ble_hsus_c_t * p_ble_hsus_c, const ble_evt_t * p_ble_evt);

/**@brief Function for sending a string to the server.
 *
 * @details This function writes the TX characteristic of the server.
 *
 * @param[in] p_ble_hsus_c Pointer to the HSUS client structure.
 * @param[in] p_string    String to be sent.
 * @param[in] length      Length of the string.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
// uint32_t ble_hsus_c_string_send(ble_hsus_c_t * p_ble_hsus_c, uint8_t * p_string, uint16_t length);


/**@brief Function for assigning handles to a this instance of hsus_c.
 *
 * @details Call this function when a link has been established with a peer to
 *          associate this link to this instance of the module. This makes it
 *          possible to handle several link and associate each link to a particular
 *          instance of this module. The connection handle and attribute handles will be
 *          provided from the discovery event @ref BLE_HSUS_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in] p_ble_hsus_c    Pointer to the HSUS client structure instance to associate with these
 *                           handles.
 * @param[in] conn_handle    Connection handle to associated with the given HSUS Instance.
 * @param[in] p_peer_handles Attribute handles on the HSUS server that you want this HSUS client to
 *                           interact with.
 *
 * @retval    NRF_SUCCESS    If the operation was successful.
 * @retval    NRF_ERROR_NULL If a p_hsus was a NULL pointer.
 */
uint32_t ble_hsus_c_handles_assign(ble_hsus_c_t * p_ble_hsus_c, const uint16_t conn_handle, const ble_hsus_c_handles_t * p_peer_handles);


#endif // BLE_HSUS_C_H__

/** @} */
