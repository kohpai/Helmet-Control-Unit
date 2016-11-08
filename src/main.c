#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf_delay.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_trace.h"
#include "app_util.h"
#include "app_error.h"
#include "app_uart.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "boards.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "softdevice_handler.h"
#include "ble_advdata.h"
#include "helmet_sensor_unit_service_c.h"

#define CENTRAL_LINK_COUNT      1                               /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT   0                               /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define UART_TX_BUF_SIZE        256                             /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        1                               /**< UART RX buffer size. */

#define HSU_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN      /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_TIMER_PRESCALER     0                               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 3                               /**< Size of timer operation queues. */

#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE             1                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000                          /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS) /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS) /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                               /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS) /**< Determines supervision time-out in units of 10 millisecond. */

#define UUID16_SIZE             2                               /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                               /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                              /**< Size of 128 bit UUID */

#define DEV_CODE            { 0x53, 0x48, 0x41, 0x44, 0x44, 0x52 ,0x00, 0x00 }
#define TEAM_CODE           0x0059

#define HSU_STATUS          0
#define HCU_STATUS          1
#define ACC_STATUS          2
#define GYRO_STATUS         3
#define MAG_STATUS          4
#define HRM_STATUS          5

#define DEBUG 0

enum internal_state_e {
    FOUND,
    CONNECTED,
    DISCOVERED,
    DISCONNECTED
};

enum unit_status_e {
    SYSTEM_CLEAR = 0,
    SYSTEM_ERROR,
    UART_ERROR,
    TIMER_ERROR,
    DB_DISC_ERROR,
    BLE_STACK_ERROR,
    HSUS_ERROR,
    SCAN_ERROR,
    SCAN_TIMEOUT_ERROR,
    CONN_REQ_TIMEOUT_ERROR
};

static ble_hsus_c_t             m_ble_hsus_c;                   /**< Instance of HSUS service. Must be passed to all NUS_C API calls. */
static ble_db_discovery_t       m_ble_db_discovery;             /**< Instance of database discovery module. Must be passed to all db_discovert API calls */
static          enum internal_state_e   internal_state      = DISCONNECTED;
static volatile bool                    timer_read_tick     = false;
static          uint16_t                sensor_values[10]   = {0};
static          enum unit_status_e      unit_status[2]      = {0};
static          uint8_t                 overview            = (1 << ACC_STATUS)
    | (1 << GYRO_STATUS)
    | (1 << MAG_STATUS)
    | (1 << HRM_STATUS) | 0;

APP_TIMER_DEF(ble_read_timer_id);

/**
 * @brief Connection parameters requested for connection.
 */
static const ble_gap_conn_params_t m_connection_param =
{
  (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
  (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
  (uint16_t)SLAVE_LATENCY,            // Slave latency
  (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
};

/**
 * @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t m_scan_params =
{
  .active      = SCAN_ACTIVE,
  .selective   = SCAN_SELECTIVE,
  .p_whitelist = NULL,
  .interval    = SCAN_INTERVAL,
  .window      = SCAN_WINDOW,
  .timeout     = SCAN_TIMEOUT
};

static uint8_t  app_timers_init(void);
static uint8_t  power_manage(void);
static uint8_t  uart_init(void);
static uint8_t  db_discovery_init(void);
static uint8_t  ble_stack_init(void);
static uint8_t  hsus_c_init(void);
static uint8_t  scan_start(void);
#if DEBUG == 0
static void sensor_resp(uint16_t *sv, uint8_t start);
#endif

int main(void)
{
    uint8_t ret;
    uint8_t conn_cnt = 0;

    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);


    ret = uart_init();

    if (ret != 0) {
        LEDS_ON(BSP_LED_2_MASK);
        overview &= (~(1 << HCU_STATUS));
        unit_status[HCU_STATUS] = UART_ERROR;

        goto EXIT_PROGRAM;
    }

    ret = app_timers_init();

    if (ret != 0) {
#if DEBUG == 1
        printf("Cannot initialize timers\n");
#endif
        overview &= (~(1 << HCU_STATUS));
        unit_status[HCU_STATUS] = TIMER_ERROR;
        goto EXIT_PROGRAM;
    }

    ret = db_discovery_init();

    if (ret != 0) {
#if DEBUG == 1
        printf("Cannot initialize DB Discovery module\n");
#endif
        overview &= (~(1 << HCU_STATUS));
        unit_status[HCU_STATUS] = DB_DISC_ERROR;
        goto EXIT_PROGRAM;
    }

    ret = ble_stack_init();

    if (ret != 0) {
#if DEBUG == 1
        printf("Cannot initialize BLE stack, Error: %u.\n", ret);
#endif
        overview &= (~(1 << HCU_STATUS));
        unit_status[HCU_STATUS] = BLE_STACK_ERROR;
        goto EXIT_PROGRAM;
    }

    ret = hsus_c_init();

    if (ret != 0) {
#if DEBUG == 1
        printf("Cannot initialize HSUS client\n");
#endif
        overview &= (~(1 << HCU_STATUS));
        unit_status[HCU_STATUS] = HSUS_ERROR;
        goto EXIT_PROGRAM;
    }

    overview |= 1 << HCU_STATUS;
    unit_status[HCU_STATUS] = SYSTEM_CLEAR;

    // Start scanning for peripherals and initiate connection
    // with devices that advertise NUS UUID.
    while (true) {
        uint8_t ch;

        if (timer_read_tick) {
            timer_read_tick = false;

            switch (internal_state) {
                case FOUND:
                    break;
                case CONNECTED:
#if DEBUG == 1
                    printf("main connected\n");
#endif
                    overview |= 1 << HSU_STATUS;
                    unit_status[HSU_STATUS] = SYSTEM_CLEAR;

                    if ((++conn_cnt) > 1) {
                        conn_cnt = 0;
                        internal_state = DISCONNECTED;
                    }

                    break;
                case DISCOVERED:
#if DEBUG == 1
                    printf("main discovered\n");
#endif
                    break;
                case DISCONNECTED:
                    ret = scan_start();

                    if (ret != 0) {
                        overview &= (~(1 << HCU_STATUS));
                        unit_status[HCU_STATUS] = SCAN_ERROR;
                    }
                    else {
                        overview |= 1 << HCU_STATUS;
                        unit_status[HCU_STATUS] = SYSTEM_CLEAR;
                    }

                    break;
            }
        }

        if (app_uart_get(&ch) == NRF_SUCCESS) {
            switch (ch) {
                case 'O':
#if DEBUG == 1
                    printf("%02x\n", overview);
#else
                    while(app_uart_put(overview) != NRF_SUCCESS);
#endif
                    break;
                case 'S':
#if DEBUG == 1
                    printf("%02x\n", unit_status[HSU_STATUS]);
#else
                    while(app_uart_put(unit_status[HSU_STATUS]) != NRF_SUCCESS);
#endif
                    break;
                case 'C':
#if DEBUG == 1
                    printf("%02x\n", unit_status[HCU_STATUS]);
#else
                    while(app_uart_put(unit_status[HCU_STATUS]) != NRF_SUCCESS);
#endif
                    break;
                case 'A':
#if DEBUG == 1
                    printf( "%04x%04x%04x\n",
                            sensor_values[0],
                            sensor_values[1],
                            sensor_values[2]);
#else
                    sensor_resp(sensor_values, 0);
#endif
                    break;
                case 'G':
#if DEBUG == 1
                    printf( "%04x%04x%04x\n",
                            sensor_values[3],
                            sensor_values[4],
                            sensor_values[5]);
#else
                    sensor_resp(sensor_values, 3);
#endif
                    break;
                case 'M':
#if DEBUG == 1
                    printf( "%04x%04x%04x\n",
                            sensor_values[6],
                            sensor_values[7],
                            sensor_values[8]);
#else
                    sensor_resp(sensor_values, 6);
#endif
                    break;
                case 'H':
#if DEBUG == 1
                    printf( "%04x\n", sensor_values[9]);
#else
                    while(app_uart_put((uint8_t)(sensor_values[9] >> 8)) !=
                            NRF_SUCCESS);
                    while(app_uart_put((uint8_t)(sensor_values[9])) !=
                            NRF_SUCCESS);
#endif
                    break;
                default:
#if DEBUG == 1
                    printf("invalid requested data\n");
#else
                    printf("A");
#endif
                    break;
            }
        }

        ret = power_manage();

        if (ret != 0) {
#if DEBUG == 1
            printf("Cannot go to sleep\n");
#endif
        }
    }

EXIT_PROGRAM:
#if DEBUG == 1
    printf("someone call exit\n");
#endif

    if (unit_status[HCU_STATUS] == UART_ERROR)
        while (true);

    while (true) {
        uint8_t ch;

        if (app_uart_get(&ch) == NRF_SUCCESS) {
            switch (ch) {
                case 'O':
#if DEBUG == 1
                    printf("%02x\n", overview);
#else
                    while(app_uart_put(overview) != NRF_SUCCESS);
#endif
                    break;
                case 'C':
#if DEBUG == 1
                    printf("%02x\n", unit_status[HCU_STATUS]);
#else
                    while(app_uart_put(unit_status[HCU_STATUS]) != NRF_SUCCESS);
#endif
                    break;
                default:
#if DEBUG == 1
                    printf("invalid requested data\n");
#endif
                    break;
            }
        }
    }

    return 0;
}

#if DEBUG == 0
static void sensor_resp(uint16_t *sv, uint8_t start)
{
    uint8_t cr = (uint8_t)(sv[start] >> 8);
    while(app_uart_put(cr) != NRF_SUCCESS);

    cr = (uint8_t)(sv[start]);
    while(app_uart_put(cr) != NRF_SUCCESS);

    cr = (uint8_t)(sv[start + 1] >> 8);
    while(app_uart_put(cr) != NRF_SUCCESS);

    cr = (uint8_t)(sv[start + 1]);
    while(app_uart_put(cr) != NRF_SUCCESS);

    cr = (uint8_t)(sv[start + 2] >> 8);
    while(app_uart_put(cr) != NRF_SUCCESS);

    cr = (uint8_t)(sv[start + 2]);
    while(app_uart_put(cr) != NRF_SUCCESS);
}
#endif

// Timeout handler for the repeated timer
static void ble_read_int_handler(void * p_context)
{
    // Do something
    timer_read_tick = true;
}

/**
 * @brief HSUS uuid
 */
static uint8_t app_timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers
    err_code = app_timer_create(&ble_read_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                ble_read_int_handler);

    if (err_code != NRF_SUCCESS)
        return 1;

    err_code = app_timer_start(
            ble_read_timer_id,
            APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER),
            NULL);

    return 0;
}

/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function to start scanning.
 */
static uint8_t scan_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_scan_start(&m_scan_params);

    if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
        printf("SCAN Error: %lu\n", err_code); // NRF_ERROR_INVALID_STATE Most of the time
#endif
        return 1;
    }

    return 0;
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_hsus_c_on_db_disc_evt(&m_ble_hsus_c, p_evt);
}

void uart_callback(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type) {
        case APP_UART_COMMUNICATION_ERROR:
            LEDS_ON(BSP_LED_4_MASK);
            break;

        case APP_UART_FIFO_ERROR:
            LEDS_ON(BSP_LED_3_MASK);
            break;

        case APP_UART_DATA_READY:
        case APP_UART_TX_EMPTY:
        case APP_UART_DATA:
            break;
    }
}

/**@brief Callback handling NUS Client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS Client Handle. This identifies the NUS client
 * @param[in]   p_ble_nus_evt Pointer to the NUS Client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_hsus_c_evt_handler(
        ble_hsus_c_t            *p_ble_hsus_c,
        const ble_hsus_c_evt_t  *p_ble_hsus_evt)
{
    uint32_t    err_code;

    switch (p_ble_hsus_evt->evt_type) {
        case BLE_HSUS_C_EVT_DISCOVERY_COMPLETE: {
            err_code = ble_hsus_c_handles_assign(
                    p_ble_hsus_c,
                    p_ble_hsus_evt->conn_handle,
                    &p_ble_hsus_evt->handles);

            if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                printf("Cannot assign characteristic handles\n");
#endif
                overview &= (~(1 << HCU_STATUS));
                unit_status[HCU_STATUS] = SYSTEM_ERROR;
                break;
            }
#if DEBUG == 1
            printf("Discovery done\n");
#endif
            internal_state = DISCOVERED;

            err_code = sd_ble_gattc_read(
                    p_ble_hsus_c->conn_handle,
                    p_ble_hsus_c->handles.acc_handle,
                    0);

            if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                printf("Cannot read ACC value\n");
#endif
                overview &= (~(1 << HCU_STATUS));
                unit_status[HCU_STATUS] = SYSTEM_ERROR;

                err_code = sd_ble_gap_disconnect(p_ble_hsus_c->conn_handle,
                        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

                if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                    printf("Cannot start disconnecting\n");
#endif
                }
            }
            else {
                overview |= (1 << HCU_STATUS);
                unit_status[HCU_STATUS] = SYSTEM_CLEAR;
            }

            break;
        }

        case BLE_HSUS_C_EVT_READ_RSP: {
            uint16_t    char_handle     = p_ble_hsus_evt->handles.acc_handle;
            bool        isDone          = false;
            uint8_t     start           = 0;

            if (char_handle == p_ble_hsus_c->handles.acc_handle) {
#if DEBUG == 1
                printf("Read ACC\n");
#endif
                err_code = sd_ble_gattc_read(
                        p_ble_hsus_c->conn_handle,
                        p_ble_hsus_c->handles.gyro_handle,
                        0);

                if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                    printf("Cannot read GYRO value, Error: %lu\n", err_code);
#endif
                    overview &= (~(1 << HCU_STATUS));
                    unit_status[HCU_STATUS] = SYSTEM_ERROR;
                    isDone = true;

                    goto DISCONNECT;
                }
                else {
                    overview |= (1 << HCU_STATUS);
                    unit_status[HCU_STATUS] = SYSTEM_CLEAR;
                }

                start = 0;
            }
            else if (char_handle == p_ble_hsus_c->handles.gyro_handle) {
#if DEBUG == 1
                printf("Read GYRO\n");
#endif
                err_code = sd_ble_gattc_read(
                        p_ble_hsus_c->conn_handle,
                        p_ble_hsus_c->handles.mag_handle,
                        0);

                if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                    printf("Cannot read MAG value\n");
#endif
                    overview &= (~(1 << HCU_STATUS));
                    unit_status[HCU_STATUS] = SYSTEM_ERROR;
                    isDone = true;

                    goto DISCONNECT;
                }
                else {
                    overview |= (1 << HCU_STATUS);
                    unit_status[HCU_STATUS] = SYSTEM_CLEAR;
                }

                start = 3;
            }
            else if (char_handle == p_ble_hsus_c->handles.mag_handle) {
#if DEBUG == 1
                printf("Read MAG\n");
#endif
                err_code = sd_ble_gattc_read(
                        p_ble_hsus_c->conn_handle,
                        p_ble_hsus_c->handles.hrm_handle,
                        0);

                if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                    printf("Cannot read HRM value\n");
#endif
                    overview &= (~(1 << HCU_STATUS));
                    unit_status[HCU_STATUS] = SYSTEM_ERROR;
                    isDone = true;

                    goto DISCONNECT;
                }
                else {
                    overview |= (1 << HCU_STATUS);
                    unit_status[HCU_STATUS] = SYSTEM_CLEAR;
                }

                start = 6;
            }
            else if (char_handle == p_ble_hsus_c->handles.hrm_handle) {
#if DEBUG == 1
                printf("Read HRM\n");
#endif
                overview &= (~(1 << HCU_STATUS));
                unit_status[HCU_STATUS] = SYSTEM_ERROR;
                isDone = true;
                start = 9;
            }
#if DEBUG == 1
            printf("\tData: ");

            for (   uint8_t count = 0;
                    count < p_ble_hsus_evt->data_len;
                    count += 2) {
                printf( "%04x",
                        ((((uint16_t)p_ble_hsus_evt->p_data[count]) << 8) |
                         p_ble_hsus_evt->p_data[count + 1]));
                sensor_values[start + (count/2)] =
                    ((((uint16_t)p_ble_hsus_evt->p_data[count]) << 8) |
                     p_ble_hsus_evt->p_data[count + 1]);
            }

            printf("\n");
#else
            for (   uint8_t count = 0;
                    count < p_ble_hsus_evt->data_len;
                    count += 2) {
                sensor_values[start + (count/2)] =
                    ((((uint16_t)p_ble_hsus_evt->p_data[count]) << 8) |
                     p_ble_hsus_evt->p_data[count + 1]);
            }
#endif

            if (!isDone)
                break;
DISCONNECT:
            err_code = sd_ble_gap_disconnect(p_ble_hsus_c->conn_handle,
                    BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

            if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                printf("Cannot start disconnecting\n");
#endif
                overview &= (~(1 << HCU_STATUS));
                unit_status[HCU_STATUS] = SYSTEM_ERROR;
            }
            else {
                overview |= (1 << HCU_STATUS);
                unit_status[HCU_STATUS] = SYSTEM_CLEAR;
            }

            break;
        }

        case BLE_HSUS_C_EVT_DISCONNECTED:
#if DEBUG == 1
            printf("Disconnected\r\n");
#endif
            internal_state = DISCONNECTED;

            break;
    }
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
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

                for (count = 7; count < p_gap_evt->params.adv_report.dlen; ++count)
                    if (p_gap_evt->params.adv_report.data[count] != confirm_code[count - 7])
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
                internal_state = FOUND;
                err_code = sd_ble_gap_connect(
                        &p_adv_report->peer_addr,
                        &m_scan_params,
                        &m_connection_param);

                if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                    printf("Attempt to connect failed\n");
#endif
                    overview &= (~(1 << HCU_STATUS));
                    unit_status[HCU_STATUS] = SYSTEM_ERROR;
                }
                else {
#if DEBUG == 1
                    printf("Connecting => "
                            "%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                            p_adv_report->peer_addr.addr[0],
                            p_adv_report->peer_addr.addr[1],
                            p_adv_report->peer_addr.addr[2],
                            p_adv_report->peer_addr.addr[3],
                            p_adv_report->peer_addr.addr[4],
                            p_adv_report->peer_addr.addr[5]);
#endif
                    overview |= (1 << HCU_STATUS);
                    unit_status[HCU_STATUS] = SYSTEM_CLEAR;
                }
            }

            break;
        }

        case BLE_GAP_EVT_CONNECTED:
#if DEBUG == 1
            printf("Connected => "
                    "%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                    p_adv_report->peer_addr.addr[0],
                    p_adv_report->peer_addr.addr[1],
                    p_adv_report->peer_addr.addr[2],
                    p_adv_report->peer_addr.addr[3],
                    p_adv_report->peer_addr.addr[4],
                    p_adv_report->peer_addr.addr[5]);
#endif
            internal_state = CONNECTED;
            err_code = ble_db_discovery_start(&m_ble_db_discovery,
                    p_ble_evt->evt.gap_evt.conn_handle);

            if (err_code != NRF_SUCCESS) {
#if DEBUG == 1
                printf("Cannot start discovery services\n");
#endif
                overview &= (~(1 << HCU_STATUS));
                unit_status[HCU_STATUS] = SYSTEM_ERROR;
            }
            else {
#if DEBUG == 1
                printf("Start discovery ...\n");
#endif
                overview |= (1 << HCU_STATUS);
                unit_status[HCU_STATUS] = SYSTEM_CLEAR;
            }

            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN) {
#if DEBUG == 1
                printf("Scan timed out.\r\n");
#endif
                overview &= (~(1 << HSU_STATUS));
                unit_status[HSU_STATUS] = SCAN_TIMEOUT_ERROR;
            }
            else if (p_gap_evt->params.timeout.src ==
                    BLE_GAP_TIMEOUT_SRC_CONN) {
#if DEBUG == 1
                printf("Connection Request timed out.\r\n");
#endif
                overview &= (~(1 << HSU_STATUS));
                unit_status[HSU_STATUS] = CONN_REQ_TIMEOUT_ERROR;
            }
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

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *          been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_hsus_c_on_ble_evt(&m_ble_hsus_c, p_ble_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
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

/**@brief Function for initializing the UART.
 */
static uint8_t uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_ENABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud9600
      };

    APP_UART_FIFO_INIT(&comm_params,
                        UART_RX_BUF_SIZE,
                        UART_TX_BUF_SIZE,
                        uart_callback,
                        APP_IRQ_PRIORITY_LOW,
                        err_code);

    /** APP_ERROR_CHECK(err_code); */
    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}

/**@brief Function for initializing the NUS Client.
 */
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

/** @brief Function for initializing the Database Discovery Module.
 */
static uint8_t db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init(db_disc_handler);
    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}

/** @brief Function for the Power manager.
 */
static uint8_t power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}

