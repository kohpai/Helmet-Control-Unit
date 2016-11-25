#ifndef ERROR_EVENT_H
#define ERROR_EVENT_H

#include <stdint.h>

void set_tim_error(uint8_t set);
void set_ble_error(uint8_t set);
void set_sys_error(uint8_t set);

uint8_t get_err_status(void);
#endif /* ifndef ERROR_EVENT_H */
