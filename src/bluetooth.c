#include <stdio.h>

#include "ble_db_discovery.h"
#include "ble_hci.h"
#include "softdevice_handler.h"
#include "boards.h"
#include "helmet_sensor_unit_service_c.h"
#include "bluetooth.h"
#include "error_event.h"

#define DEBUG 0

#define CENTRAL_LINK_COUNT      1
#define PERIPHERAL_LINK_COUNT   0

#define SCAN_INTERVAL           0x00A0 // times 0.625 ms
#define SCAN_WINDOW             0x0050 // times 0.625 ms
#define SCAN_ACTIVE             1
#define SCAN_SELECTIVE          0
#define SCAN_TIMEOUT            0x0000

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS)
#define SLAVE_LATENCY           0
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS)

#define DEV_CODE            { 0x53, 0x48, 0x41, 0x44, 0x44, 0x52 ,0x00, 0x00 }
#define TEAM_CODE           0x0059

#define HSU_STATUS          0
#define HCU_STATUS          1
#define ACC_STATUS          2
#define GYRO_STATUS         3
#define MAG_STATUS          4
#define HRM_STATUS          5

enum internal_state_e {
    FOUND,
    CONNECTED,
    DISCOVERED,
    DISCONNECTED
};

enum reading_state_e {
    READING_ERR,
    READING_ACC,
    READING_GYR,
    READING_MAG,
    READING_HRM
};

static const ble_gap_conn_params_t m_connection_param = {
  (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
  (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
  (uint16_t)SLAVE_LATENCY,            // Slave latency
  (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
};

static const ble_gap_scan_params_t m_scan_params = {
  .active      = SCAN_ACTIVE,
  .selective   = SCAN_SELECTIVE,
  .p_whitelist = NULL,
  .interval    = SCAN_INTERVAL,
  .window      = SCAN_WINDOW,
  .timeout     = SCAN_TIMEOUT
};

static enum internal_state_e    ble_state           = DISCONNECTED;
static enum reading_state_e     rd_state            = READING_ERR;
static ble_db_discovery_t       m_ble_db_discovery;
static ble_hsus_c_t             m_ble_hsus_c;
static ble_gap_addr_t           m_hsu_addr;
static uint8_t                  num_of_conn         = 0;
static uint8_t                  pending             = 0;
static uint16_t                 evt_conn_handle     = 0;
static uint16_t                 err_evt             = 0;
static int16_t                  acc_val[3]          = {0};
static int16_t                  gyr_val[3]          = {0};
static int16_t                  mag_val[3]          = {0};
static int16_t                  hrm_val             = 0;

static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_hsus_c_on_db_disc_evt(&m_ble_hsus_c, p_evt);
}

static uint8_t db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init(db_disc_handler);
    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
    const ble_gap_evt_adv_report_t *p_adv_report =
        &p_gap_evt->params.adv_report;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_ADV_REPORT: {
            uint16_t team =
                (((uint16_t)p_gap_evt->params.adv_report.data[6]) << 8);

            team |= p_gap_evt->params.adv_report.data[5];

            if (p_gap_evt->params.adv_report.scan_rsp == 0 &&
                    p_gap_evt->params.adv_report.dlen == 0x0f &&
                    team == TEAM_CODE) {
                uint8_t count;
                uint8_t confirm_code[] = DEV_CODE;

                for (   count = 7;
                        count < p_gap_evt->params.adv_report.dlen;
                        ++count)
                    if (p_gap_evt->params.adv_report.data[count] !=
                            confirm_code[count - 7])
                        break;

                if (count < p_gap_evt->params.adv_report.dlen) {
#if DEBUG == 1
                    printf("Wrong ID code\n");
#endif
                    break;
                }
#if DEBUG == 1
                printf("Confirmed ID code\n");
#endif
                m_hsu_addr = p_adv_report->peer_addr;
                pending = 0;
                ble_state = FOUND;
            }

            break;
        }
        case BLE_GAP_EVT_CONNECTED:
            evt_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            pending = 0;
            ble_state = CONNECTED;
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN) {
#if DEBUG == 1
                printf("Scan timed out.\r\n");
#endif
            }
            else if (p_gap_evt->params.timeout.src ==
                    BLE_GAP_TIMEOUT_SRC_CONN) {
#if DEBUG == 1
                printf("Connection Request timed out.\r\n");
#endif
            }

            pending = 0;

            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code =
                sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
                        BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);

            if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                printf("Cannot reply the secured request\n");
#endif
            }

            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                    &p_gap_evt->params.conn_param_update_request.conn_params);

            if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                printf("Cannot update connection parameters\n");
#endif
            }

            break;

        default:
            break;
    }
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_hsus_c_on_ble_evt(&m_ble_hsus_c, p_ble_evt);
}

static uint8_t ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    if (err_code != NRF_SUCCESS)
        return 1;

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);

    if (err_code != NRF_SUCCESS)
        return 2;

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);

    if (err_code != NRF_SUCCESS)
        return 3;

    return 0;
}

static void arr_to_xyz(uint8_t *src, int16_t *dst, uint8_t len)
{
    uint8_t count;

    for (count = 0; count < len; count += 2)
        dst[count / 2] = ((((uint16_t)src[count]) << 8) | src[count + 1]);
}

static void arr_to_err(uint8_t *src, uint16_t *dst, uint8_t len)
{
    uint8_t count;

    for (count = 0; count < len; count += 2)
        dst[count / 2] = ((((uint16_t)src[count]) << 8) | src[count + 1]);
}

static void ble_hsus_c_evt_handler(
        ble_hsus_c_t            *p_ble_hsus_c,
        const ble_hsus_c_evt_t  *p_ble_hsus_evt)
{
    uint32_t err_code;

    switch (p_ble_hsus_evt->evt_type) {
        case BLE_HSUS_C_EVT_DISCOVERY_COMPLETE: {
#if DEBUG == 1
            printf("Discovery done\n");
#endif
            err_code    = ble_hsus_c_handles_assign(
                    p_ble_hsus_c,
                    p_ble_hsus_evt->conn_handle,
                    &p_ble_hsus_evt->handles);

            if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                printf("Cannot assign characteristic handles\n");
#endif
                set_sys_error(1);
                break;
            } else {
                pending = 0;
                ble_state   = DISCOVERED;
            }

            break;
        }
        case BLE_HSUS_C_EVT_READ_RSP: {
            uint16_t char_handle = p_ble_hsus_evt->handles.acc_handle;

            if (char_handle == p_ble_hsus_c->handles.err_handle) {
                arr_to_err(
                        p_ble_hsus_evt->p_data,
                        &err_evt,
                        p_ble_hsus_evt->data_len);
#if DEBUG == 1
                printf("Read ERR\n");
#endif
                pending = 0;
            } else if (char_handle == p_ble_hsus_c->handles.acc_handle) {
                arr_to_xyz(
                        p_ble_hsus_evt->p_data,
                        acc_val,
                        p_ble_hsus_evt->data_len);
#if DEBUG == 1
                printf("Read ACC\n");
#endif
                pending = 0;
            } else if (char_handle == p_ble_hsus_c->handles.gyro_handle) {
                arr_to_xyz(
                        p_ble_hsus_evt->p_data,
                        gyr_val,
                        p_ble_hsus_evt->data_len);
#if DEBUG == 1
                printf("Read GYRO\n");
#endif
                pending = 0;
            } else if (char_handle == p_ble_hsus_c->handles.mag_handle) {
                arr_to_xyz(
                        p_ble_hsus_evt->p_data,
                        mag_val,
                        p_ble_hsus_evt->data_len);
#if DEBUG == 1
                printf("Read MAG\n");
#endif
                pending = 0;
            } else if (char_handle == p_ble_hsus_c->handles.hrm_handle) {
                arr_to_xyz(
                        p_ble_hsus_evt->p_data,
                        &hrm_val,
                        p_ble_hsus_evt->data_len);
#if DEBUG == 1
                printf("Read HRM\n");
#endif
                set_sys_error(0);
                pending = 0;
            }

            break;
        }

        case BLE_HSUS_C_EVT_DISCONNECTED:
#if DEBUG == 1
            printf("Disconnected\r\n");
#endif
            pending = 0;
            ble_state = DISCONNECTED;

            break;
    }

/**     if (!isDone)
  *         return;
  *
  *     err_code = sd_ble_gap_disconnect(p_ble_hsus_c->conn_handle,
  *             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  *
  *     if (err_code != NRF_SUCCESS) {
  * #if DEBUG == 1
  *         printf("Cannot start disconnecting\n");
  * #endif
  *     } */
}

static uint8_t hsus_c_init(void)
{
    uint32_t            err_code;
    ble_hsus_c_init_t   hsus_c_init_t;

    hsus_c_init_t.evt_handler = ble_hsus_c_evt_handler;

    err_code = ble_hsus_c_init(&m_ble_hsus_c, &hsus_c_init_t);

    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}

uint8_t init_bluetooth(void)
{
    uint8_t ret;

    ret = db_discovery_init();

    if (ret != 0)
        return 1;

    ret = ble_stack_init();

    if (ret != 0)
        return 2;

    ret = hsus_c_init();

    if (ret != 0)
        return 3;

    return 0;
}

static uint8_t scan_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_scan_start(&m_scan_params);

    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}

static uint8_t read_chars(
        enum reading_state_e *rd,
        ble_hsus_c_t *hsus_c,
        uint8_t *pending)
{
    uint32_t err_code;
    uint8_t ret = 0;

    if (*pending)
        goto RETURN;

    switch (*rd) {
        case READING_ERR:
            err_code = sd_ble_gattc_read(
                    hsus_c->conn_handle,
                    hsus_c->handles.err_handle,
                    0);

            if (err_code != NRF_SUCCESS) {
                set_sys_error(1);
                ret = 1;
            } else {
                *pending = 1;
                *rd = READING_ACC;
            }

            break;
        case READING_ACC:
            err_code = sd_ble_gattc_read(
                    hsus_c->conn_handle,
                    hsus_c->handles.acc_handle,
                    0);

            if (err_code != NRF_SUCCESS) {
                set_sys_error(1);
                ret = 2;
            } else {
                *pending = 1;
                *rd = READING_GYR;
            }

            break;
        case READING_GYR:
            err_code = sd_ble_gattc_read(
                    hsus_c->conn_handle,
                    hsus_c->handles.gyro_handle,
                    0);

            if (err_code != NRF_SUCCESS) {
                set_sys_error(1);
                ret = 3;
            } else {
                *pending = 1;
                *rd = READING_MAG;
            }

            break;
        case READING_MAG:
            err_code = sd_ble_gattc_read(
                    hsus_c->conn_handle,
                    hsus_c->handles.mag_handle,
                    0);

            if (err_code != NRF_SUCCESS) {
                set_sys_error(1);
                ret = 4;
            } else {
                *pending = 1;
                *rd = READING_HRM;
            }

            break;
        case READING_HRM:
            err_code = sd_ble_gattc_read(
                    hsus_c->conn_handle,
                    hsus_c->handles.hrm_handle,
                    0);

            if (err_code != NRF_SUCCESS) {
                set_sys_error(1);
                ret = 5;
            } else {
                *pending = 1;
                *rd = READING_ERR;
            }

            break;
    }

RETURN:
    return ret;
}

void timer_tick(void)
{
    uint8_t ret;
    uint32_t err_code;

    switch (ble_state) {
        case FOUND:
#if DEBUG == 1
            printf("timer_tick: found\n");
#endif
            if (pending)
                break;

            err_code = sd_ble_gap_connect(
                    &m_hsu_addr,
                    &m_scan_params,
                    &m_connection_param);

            if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                printf("Attempt to connect failed\n");
#endif
                set_sys_error(1);
            }
            else {
#if DEBUG == 1
                printf("Connecting => "
                        "%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                        m_hsu_addr.addr[0],
                        m_hsu_addr.addr[1],
                        m_hsu_addr.addr[2],
                        m_hsu_addr.addr[3],
                        m_hsu_addr.addr[4],
                        m_hsu_addr.addr[5]);
#endif
                set_sys_error(0);
                pending = 1;
            }
            break;
        case CONNECTED:
#if DEBUG == 1
            printf("timer_tick: connected\n");
#endif
            ++num_of_conn;

            if (num_of_conn > 3) {
                num_of_conn = 0;
                pending     = 0;
                ble_state   = DISCONNECTED;

                break;
            }

            if (pending)
                break;
#if DEBUG == 1
            printf("Connected => "
                    "%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                    m_hsu_addr.addr[0],
                    m_hsu_addr.addr[1],
                    m_hsu_addr.addr[2],
                    m_hsu_addr.addr[3],
                    m_hsu_addr.addr[4],
                    m_hsu_addr.addr[5]);
#endif
            err_code = ble_db_discovery_start(&m_ble_db_discovery,
                    evt_conn_handle);

            if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                printf("Cannot start discovery services\n");
#endif
                set_sys_error(1);
            }
            else {
#if DEBUG == 1
                printf("Start discovery ...\n");
#endif
                set_sys_error(0);
                pending = 1;
            }

            break;
        case DISCOVERED:
#if DEBUG == 1
            printf("timer_tick: discovered\n");
#endif
            if (pending)
                break;

            ret = read_chars(&rd_state, &m_ble_hsus_c, &pending);

            if (ret != 0) {
#if DEBUG == 1
                printf("cannot read: %u\n", ret);
#endif
            }

            break;
        case DISCONNECTED:
#if DEBUG == 1
            printf("timer_tick: disconnected\n");
#endif
            if (pending)
                break;

            ret = scan_start();

            if (ret != 0) {
                set_sys_error(1);
            } else {
                set_sys_error(0);
                pending = 1;
            }

            break;
        default:
#if DEBUG == 1
            printf("timer_tick: default\n");
#endif
            break;
    }
}

void get_err_evt_status(uint16_t *ee)
{
    (*ee) = err_evt;
}

void get_hrm_val(int16_t *hrm)
{
    (*hrm) = hrm_val;
}

void get_acc_val(int16_t *x, int16_t *y, int16_t *z)
{
    (*x) = acc_val[0];
    (*y) = acc_val[1];
    (*z) = acc_val[2];
}

void get_gyr_val(int16_t *x, int16_t *y, int16_t *z)
{
    (*x) = gyr_val[0];
    (*y) = gyr_val[1];
    (*z) = gyr_val[2];
}

void get_mag_val(int16_t *x, int16_t *y, int16_t *z)
{
    (*x) = mag_val[0];
    (*y) = mag_val[1];
    (*z) = mag_val[2];
}
