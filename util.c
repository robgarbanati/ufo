#include "util.h"

void nrf_log_string(char *string, uint16_t length) {
	for (uint32_t i = 0; i < length; i++) {
        NRF_LOG_RAW_INFO("%c", string[i]);
    }
	NRF_LOG_RAW_INFO("\r\n");
}
