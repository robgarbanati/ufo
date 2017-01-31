#include "spi.h"

//**********************************************************************************************************************//
// Private Variables
//**********************************************************************************************************************//

static const nrf_drv_spi_t drv_spi_parameters = {
    .p_registers  = (void *)NRF_SPIM0,
    .irq          = SPI0_IRQ,
    .drv_inst_idx = 0,
    .use_easy_dma = 1
};

static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
#define TEST_STRING "Nordic"
//static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_tx_buf[] = { 0x8F, 0x00 };           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

#undef NRF_LOG_MODULE_NAME
#define NRF_LOG_MODULE_NAME "SPI"


//**********************************************************************************************************************//
// Private Functions
//**********************************************************************************************************************//

void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.\r\n");
    if (m_rx_buf[0] != 0) {
        NRF_LOG_INFO("Received: \r\n");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

//**********************************************************************************************************************//
// Public Functions
//**********************************************************************************************************************//

void spi_init(void) {
	spi_xfer_done = false;
	
	NRF_LOG_INFO("Started\r\n");
	
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin   = 18;
    spi_config.miso_pin = 7;
    spi_config.mosi_pin = 8;
    spi_config.sck_pin  = 9;
    APP_ERROR_CHECK(nrf_drv_spi_init(&drv_spi_parameters, &spi_config, spi_event_handler));

	APP_ERROR_CHECK(nrf_drv_spi_transfer(&drv_spi_parameters, m_tx_buf, m_length, m_rx_buf, m_length));

	while (!spi_xfer_done) {
		__WFE();
	}
}