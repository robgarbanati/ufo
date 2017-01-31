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

static void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.\r\n");
    if (m_rx_buf[0] != 0) {
        NRF_LOG_INFO("Received: \r\n");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
		NRF_LOG_INFO("LSM6DS3 chip ID = 0x%x\r\n", m_rx_buf[1]);
    }
}

static void lsm_id_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.\r\n");
    if (m_rx_buf[0] != 0) {
        NRF_LOG_INFO("Received: \r\n");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
		NRF_LOG_INFO("LSM6DS3 chip ID = 0x%x\r\n", m_rx_buf[1]);
    }
}

static void lis_id_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.\r\n");
    if (m_rx_buf[0] != 0) {
        NRF_LOG_INFO("Received: \r\n");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
		NRF_LOG_INFO("LIS3DSL chip ID = 0x%x\r\n", m_rx_buf[1]);
    }
}

//**********************************************************************************************************************//
// Public Functions
//**********************************************************************************************************************//

#define LSM_SS_PIN		18
#define LIS_SS_PIN		10
#define SPI_MISO_PIN	7
#define SPI_MOSI_PIN	8
#define SPI_SCK_PIN		9

void spi_init(void) {
	uint16_t err_code;
	spi_xfer_done = false;
	
	NRF_LOG_INFO("Started\r\n");
	
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin   = LSM_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    CTR_APP_ERROR_CHECK(nrf_drv_spi_init(&drv_spi_parameters, &spi_config, lsm_id_handler), "nrf_drv_spi_init failed!");

	CTR_APP_ERROR_CHECK(nrf_drv_spi_transfer(&drv_spi_parameters, m_tx_buf, m_length, m_rx_buf, m_length), "nrf_drv_spi_transfer failed!");

	while (!spi_xfer_done) {
		__WFE();
	}
	
	spi_config.ss_pin   = LIS_SS_PIN;
	nrf_drv_spi_uninit(&drv_spi_parameters);
	CTR_APP_ERROR_CHECK(nrf_drv_spi_init(&drv_spi_parameters, &spi_config, lis_id_handler), "nrf_drv_spi_init failed!");

	CTR_APP_ERROR_CHECK(nrf_drv_spi_transfer(&drv_spi_parameters, m_tx_buf, m_length, m_rx_buf, m_length), "nrf_drv_spi_transfer failed!");

	while (!spi_xfer_done) {
		__WFE();
	}
}