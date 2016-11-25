#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf_delay.h"
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
#include "ble_advdata.h"
#include "helmet_sensor_unit_service_c.h"
#include "bluetooth.h"
#include "error_event.h"

#define UART_TX_BUF_SIZE        256                             /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        1                               /**< UART RX buffer size. */

#define APP_TIMER_PRESCALER     0                               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 3                               /**< Size of timer operation queues. */

#define UUID16_SIZE             2                               /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                               /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                              /**< Size of 128 bit UUID */

#define HSU_MASK                0x1FFF

#define DEBUG 0

static volatile bool timer_read_tick     = false;

APP_TIMER_DEF(ble_read_timer_id);

static uint8_t  init_timer(void);
static uint8_t  power_manage(void);
static uint8_t  init_uart(void);
#if DEBUG == 0
static void mpu_resp(int16_t x, int16_t y, int16_t z);
static void hrm_resp(int16_t hrm);
static void overview_resp(uint16_t overview);
#endif

int main(void)
{
    uint8_t ret;

    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);


    ret = init_uart();

    if (ret != 0) {
        LEDS_ON(BSP_LED_2_MASK);

        goto EXIT_PROGRAM;
    }

    ret = init_timer();

    if (ret != 0) {
#if DEBUG == 1
        printf("Cannot initialize timers\n");
#endif
        set_tim_error(1);
    }

    ret = init_bluetooth();

    if (ret != 0) {
#if DEBUG == 1
        printf("Cannot initialize BLE\n");
#endif
        set_ble_error(1);
    }

#if DEBUG == 1
        printf("Done Initialization!!!\n");
#endif

    while (true) {
        uint8_t ch;

        if (timer_read_tick) {
            timer_read_tick = false;
            timer_tick();
        }

        if (app_uart_get(&ch) == NRF_SUCCESS) {
            switch (ch) {
                uint16_t err_evt;
                int16_t x, y, z, hrm;

                case 'O':
                    get_err_evt_status(&err_evt);

                    err_evt &= HSU_MASK;
                    err_evt += ((uint16_t) get_err_status()) << 8;
#if DEBUG == 1
                    printf("%04x\n", err_evt);
#else
                    overview_resp(err_evt);
#endif
                    break;
                case 'A':
                    get_acc_val(&x, &y, &z);
#if DEBUG == 1
                    printf( "%04x%04x%04x\n", x, y, z);
#else
                    mpu_resp(x, y, z);
#endif
                    break;
                case 'G':
                    get_gyr_val(&x, &y, &z);
#if DEBUG == 1
                    printf( "%04x%04x%04x\n", x, y, z);
#else
                    mpu_resp(x, y, z);
#endif
                    break;
                case 'M':
                    get_mag_val(&x, &y, &z);
#if DEBUG == 1
                    printf( "%04x%04x%04x\n", x, y, z);
#else
                    mpu_resp(x, y, z);
#endif
                    break;
                case 'H':
                    get_hrm_val(&hrm);
#if DEBUG == 1
                    printf( "%04x\n", hrm);
#else
                    hrm_resp(hrm);
#endif
                    break;
                default:
#if DEBUG == 1
                    printf("invalid requested data\n");
#endif
                    break;
            }
        }

        ret = power_manage();

#if DEBUG == 1
        if (ret != 0)
            printf("Cannot go to sleep\n");
#endif
    }

EXIT_PROGRAM:
    while (1)
        ret = power_manage();

    return 0;
}

#if DEBUG == 0
static void mpu_resp(int16_t x, int16_t y, int16_t z)
{
    uint8_t cr = (uint8_t) (x >> 8);
    while(app_uart_put(cr) != NRF_SUCCESS);

    cr = (uint8_t) x;
    while(app_uart_put(cr) != NRF_SUCCESS);

    cr = (uint8_t) (y >> 8);
    while(app_uart_put(cr) != NRF_SUCCESS);

    cr = (uint8_t) y;
    while(app_uart_put(cr) != NRF_SUCCESS);

    cr = (uint8_t) (z >> 8);
    while(app_uart_put(cr) != NRF_SUCCESS);

    cr = (uint8_t) z;
    while(app_uart_put(cr) != NRF_SUCCESS);
}

static void hrm_resp(int16_t hrm)
{
    uint8_t cr = (uint8_t)(hrm >> 8);
    while(app_uart_put(cr) != NRF_SUCCESS);

    cr = (uint8_t) hrm;
    while(app_uart_put(cr) != NRF_SUCCESS);
}

static void overview_resp(uint16_t overview)
{
    uint8_t cr = (uint8_t)(overview >> 8);
    while(app_uart_put(cr) != NRF_SUCCESS);

    cr = (uint8_t) overview;
    while(app_uart_put(cr) != NRF_SUCCESS);
}
#endif

static void ble_read_int_handler(void * p_context)
{
    // Do something
    timer_read_tick = true;
#if DEBUG == 1
    /** printf("tick\n"); */
#endif
}

static uint8_t init_timer(void)
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

    if (err_code != NRF_SUCCESS)
        return 2;

    return 0;
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
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

/**@brief Function for initializing the UART.
 */
static uint8_t init_uart(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
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

/** @brief Function for the Power manager.
 */
static uint8_t power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}

