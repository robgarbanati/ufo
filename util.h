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


#define CTR_APP_ERROR_CHECK(ERR_CODE, error_message)  		\
    do {                                                    \
        const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
        if (LOCAL_ERR_CODE != NRF_SUCCESS) {                \
			NRF_LOG_ERROR("file: " __FILE__ ". error: ");	\
			nrf_log_string(error_message, strlen(error_message));	\
            APP_ERROR_HANDLER(LOCAL_ERR_CODE);              \
        }                                                   \
    } while (0)

void nrf_log_string(char *string, uint16_t length);
