#include <stdlib.h> // definition of NULL

#include "ble.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "sdk_common.h"
#include "helmet_sensor_unit_service_c.h"


/** @TODO: add error handler */
void ble_hsus_c_on_db_disc_evt(
        ble_hsus_c_t            *p_ble_hsus_c,
        ble_db_discovery_evt_t  *p_evt)
{
    uint8_t count;
    ble_hsus_c_evt_t hsus_c_evt;
    memset(&hsus_c_evt,0,sizeof(ble_hsus_c_evt_t));

    ble_gatt_db_char_t *p_chars = p_evt->params.discovered_db.charateristics;

    // Check if the HSUS was discovered.
    switch (p_evt->evt_type) {
        case BLE_DB_DISCOVERY_ERROR:
            break;

        case BLE_DB_DISCOVERY_SRV_NOT_FOUND:
            break;

        case BLE_DB_DISCOVERY_COMPLETE:
            if (p_evt->params.discovered_db.srv_uuid.uuid ==
                    BLE_UUID_HSU_SERVICE_UUID &&
                    p_evt->params.discovered_db.srv_uuid.type ==
                    p_ble_hsus_c->uuid_type) {

                for (count = 0;
                        count < p_evt->params.discovered_db.char_count;
                        ++count) {
                    switch (p_chars[count].characteristic.uuid.uuid) {
                        case BLE_UUID_ERR_CHARACTERISTIC_UUID:
                            hsus_c_evt.handles.err_handle =
                                p_chars[count].characteristic.handle_value;
                            break;

                        case BLE_UUID_ACC_CHARACTERISTIC_UUID:
                            hsus_c_evt.handles.acc_handle =
                                p_chars[count].characteristic.handle_value;
                            break;

                        case BLE_UUID_GYRO_CHARACTERISTIC_UUID:
                            hsus_c_evt.handles.gyro_handle =
                                p_chars[count].characteristic.handle_value;
                            break;

                        case BLE_UUID_MAG_CHARACTERISTIC_UUID:
                            hsus_c_evt.handles.mag_handle =
                                p_chars[count].characteristic.handle_value;
                            break;

                        case BLE_UUID_HRM_CHARACTERISTIC_UUID:
                            hsus_c_evt.handles.hrm_handle =
                                p_chars[count].characteristic.handle_value;
                            break;

                        default:
                            break;
                    }
                }

                if (p_ble_hsus_c->evt_handler != NULL) {
                    hsus_c_evt.conn_handle = p_evt->conn_handle;
                    hsus_c_evt.evt_type    = BLE_HSUS_C_EVT_DISCOVERY_COMPLETE;
                    p_ble_hsus_c->evt_handler(p_ble_hsus_c, &hsus_c_evt);
                }
            }

            break;

        case BLE_DB_DISCOVERY_AVAILABLE:
            break;
    }
}

uint32_t ble_hsus_c_init(
        ble_hsus_c_t        *p_ble_hsus_c,
        ble_hsus_c_init_t   *p_ble_hsus_c_init)
{
    uint32_t      err_code;
    ble_uuid_t    hsus_uuid;
    ble_uuid128_t hsu_base_uuid = BLE_UUID_HSU_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_ble_hsus_c);
    VERIFY_PARAM_NOT_NULL(p_ble_hsus_c_init);

    err_code = sd_ble_uuid_vs_add(&hsu_base_uuid, &p_ble_hsus_c->uuid_type);
    VERIFY_SUCCESS(err_code);

    hsus_uuid.type = p_ble_hsus_c->uuid_type;
    hsus_uuid.uuid = BLE_UUID_HSU_SERVICE_UUID;

    p_ble_hsus_c->conn_handle           = BLE_CONN_HANDLE_INVALID;
    p_ble_hsus_c->evt_handler           = p_ble_hsus_c_init->evt_handler;
    p_ble_hsus_c->handles.err_handle    = BLE_GATT_HANDLE_INVALID;
    p_ble_hsus_c->handles.acc_handle    = BLE_GATT_HANDLE_INVALID;
    p_ble_hsus_c->handles.gyro_handle   = BLE_GATT_HANDLE_INVALID;
    p_ble_hsus_c->handles.mag_handle    = BLE_GATT_HANDLE_INVALID;
    p_ble_hsus_c->handles.hrm_handle    = BLE_GATT_HANDLE_INVALID;

    return ble_db_discovery_evt_register(&hsus_uuid);
}

void ble_hsus_c_on_ble_evt(
        ble_hsus_c_t    *p_ble_hsus_c,
        const ble_evt_t *p_ble_evt)
{
    if ((p_ble_hsus_c == NULL) || (p_ble_evt == NULL))
        return;

    if ( (p_ble_hsus_c->conn_handle != BLE_CONN_HANDLE_INVALID) &&
            (p_ble_hsus_c->conn_handle != p_ble_evt->evt.gap_evt.conn_handle))
        return;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GATTC_EVT_READ_RSP:
            if (p_ble_evt->evt.gap_evt.conn_handle == p_ble_hsus_c->conn_handle
                    && p_ble_hsus_c->evt_handler != NULL) {
                ble_hsus_c_evt_t hsus_c_evt;

                hsus_c_evt.data_len =
                    p_ble_evt->evt.gattc_evt.params.read_rsp.len;
                hsus_c_evt.p_data =
                    (uint8_t *)p_ble_evt->evt.gattc_evt.params.read_rsp.data;
                hsus_c_evt.handles.acc_handle =
                    p_ble_evt->evt.gattc_evt.params.read_rsp.handle;
                hsus_c_evt.evt_type = BLE_HSUS_C_EVT_READ_RSP;

                p_ble_hsus_c->evt_handler(p_ble_hsus_c, &hsus_c_evt);
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            if (p_ble_evt->evt.gap_evt.conn_handle == p_ble_hsus_c->conn_handle
                    && p_ble_hsus_c->evt_handler != NULL) {
                ble_hsus_c_evt_t hsus_c_evt;

                hsus_c_evt.evt_type = BLE_HSUS_C_EVT_DISCONNECTED;

                p_ble_hsus_c->conn_handle = BLE_CONN_HANDLE_INVALID;
                p_ble_hsus_c->evt_handler(p_ble_hsus_c, &hsus_c_evt);
            }
            break;
  /** BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP = BLE_GATTC_EVT_BASE */
  /** BLE_GATTC_EVT_REL_DISC_RSP */
  /** BLE_GATTC_EVT_CHAR_DISC_RSP */
  /** BLE_GATTC_EVT_DESC_DISC_RSP */
  /** BLE_GATTC_EVT_ATTR_INFO_DISC_RSP */
  /** BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP */
  /** BLE_GATTC_EVT_WRITE_RSP */
  /** BLE_GATTC_EVT_HVX */
  /** BLE_GATTC_EVT_TIMEOUT */
    }
}

uint32_t ble_hsus_c_handles_assign(ble_hsus_c_t * p_ble_hsus,
                                  const uint16_t conn_handle,
                                  const ble_hsus_c_handles_t * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_hsus);

    p_ble_hsus->conn_handle = conn_handle;

    if (p_peer_handles != NULL) {
        p_ble_hsus->handles.err_handle  = p_peer_handles->err_handle;
        p_ble_hsus->handles.acc_handle  = p_peer_handles->acc_handle;
        p_ble_hsus->handles.gyro_handle = p_peer_handles->gyro_handle;
        p_ble_hsus->handles.mag_handle  = p_peer_handles->mag_handle;
        p_ble_hsus->handles.hrm_handle  = p_peer_handles->hrm_handle;
    }

    return NRF_SUCCESS;
}

