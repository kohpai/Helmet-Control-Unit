#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>

uint8_t init_bluetooth(void);
void    timer_tick(void);

void    get_err_evt_status(uint16_t *err_evt);

void    get_hrm_val(int16_t *hrm);
void    get_acc_val(int16_t *x, int16_t *y, int16_t *z);
void    get_gyr_val(int16_t *x, int16_t *y, int16_t *z);
void    get_mag_val(int16_t *x, int16_t *y, int16_t *z);
#endif /* ifndef BLUETOOTH_H */
