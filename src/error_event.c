#include "error_event.h"

#define TIM_ERR_BIT     7
#define BLE_ERR_BIT     6
#define SYS_ERR_BIT     5

static uint8_t evt_err = 0xE0;

void set_tim_error(uint8_t set)
{
    if (set)
        evt_err &= ~(((uint16_t)(1)) << TIM_ERR_BIT);
    else
        evt_err |= ((uint16_t)(1)) << TIM_ERR_BIT;

    set_ble_error(1);
}

void set_ble_error(uint8_t set)
{
    if (set)
        evt_err &= ~(((uint16_t)(1)) << BLE_ERR_BIT);
    else
        evt_err |= ((uint16_t)(1)) << BLE_ERR_BIT;
}

void set_sys_error(uint8_t set)
{
    if (set)
        evt_err &= ~(((uint16_t)(1)) << SYS_ERR_BIT);
    else
        evt_err |= ((uint16_t)(1)) << SYS_ERR_BIT;
}

uint8_t get_err_status(void)
{
    return evt_err;
}
