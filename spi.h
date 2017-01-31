#ifndef SPI_H__
#define SPI_H__

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "util.h"
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
#include "nrf_drv_spi.h"

void spi_init(void);

#endif // SPI_H__
