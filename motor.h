#ifndef MOTOR_H__
#define MOTOR_H__

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "nrf_uart.h"
#include "nrf_drv_pwm.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_log_ctrl.h"
#include "ble_nus.h"
#include "nrf_delay.h"
#include "util.h"

void motor_handler(nrf_drv_pwm_evt_type_t event_type);
void set_up_motor_pwm(nrf_drv_pwm_t *pwm_module, nrf_pwm_sequence_t const *led_sequence, void (*handler)(nrf_drv_pwm_evt_type_t event_type));

#endif // MOTOR_H
