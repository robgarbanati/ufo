/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(SPI)
#define ENABLED_SPI_COUNT (SPI0_ENABLED+SPI1_ENABLED+SPI2_ENABLED)
#if ENABLED_SPI_COUNT

#include "nrf_drv_spi.h"
#include "nrf_drv_common.h"
#include "nrf_gpio.h"
#include "nrf_assert.h"
#include "app_util_platform.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define NRF_LOG_MODULE_NAME "SPI"

#if SPI_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       SPI_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  SPI_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR SPI_CONFIG_DEBUG_COLOR
#else //SPI_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif //SPI_CONFIG_LOG_ENABLED
#include "nrf_log.h"

#ifndef SPIM_PRESENT
    // Make sure SPIx_USE_EASY_DMA is 0 for nRF51 (if a common
    // "nrf_drv_config.h" file is provided for nRF51 and nRF52).
    #undef  SPI0_USE_EASY_DMA
    #define SPI0_USE_EASY_DMA 0
    #undef  SPI1_USE_EASY_DMA
    #define SPI1_USE_EASY_DMA 0
    #undef  SPI2_USE_EASY_DMA
    #define SPI2_USE_EASY_DMA 0
#endif

#ifndef SPI0_USE_EASY_DMA
#define SPI0_USE_EASY_DMA 0
#endif

#ifndef SPI1_USE_EASY_DMA
#define SPI1_USE_EASY_DMA 0
#endif

#ifndef SPI2_USE_EASY_DMA
#define SPI2_USE_EASY_DMA 0
#endif

// This set of macros makes it possible to exclude parts of code when one type
// of supported peripherals is not used.
#if ((NRF_MODULE_ENABLED(SPI0) && SPI0_USE_EASY_DMA) || \
     (NRF_MODULE_ENABLED(SPI1) && SPI1_USE_EASY_DMA) || \
     (NRF_MODULE_ENABLED(SPI2) && SPI2_USE_EASY_DMA))
    #define SPIM_IN_USE
#endif
#if ((NRF_MODULE_ENABLED(SPI0) && !SPI0_USE_EASY_DMA) || \
     (NRF_MODULE_ENABLED(SPI1) && !SPI1_USE_EASY_DMA) || \
     (NRF_MODULE_ENABLED(SPI2) && !SPI2_USE_EASY_DMA))
    #define SPI_IN_USE
#endif
#if defined(SPIM_IN_USE) && defined(SPI_IN_USE)
    // SPIM and SPI combined
    #define CODE_FOR_SPIM(code) if (p_instance->use_easy_dma) { code }
    #define CODE_FOR_SPI(code)  else { code }
#elif defined(SPIM_IN_USE) && !defined(SPI_IN_USE)
    // SPIM only
    #define CODE_FOR_SPIM(code) { code }
    #define CODE_FOR_SPI(code)
#elif !defined(SPIM_IN_USE) && defined(SPI_IN_USE)
    // SPI only
    #define CODE_FOR_SPIM(code)
    #define CODE_FOR_SPI(code)  { code }
#else
    #error "Wrong configuration."
#endif

#ifdef SPIM_IN_USE
#define END_INT_MASK     NRF_SPIM_INT_END_MASK
#endif

// Control block - driver instance local data.
typedef struct
{
    nrf_drv_spi_handler_t handler;
    nrf_drv_spi_evt_t     evt;  // Keep the struct that is ready for event handler. Less memcpy.
    nrf_drv_state_t       state;
    volatile bool         transfer_in_progress;

    // [no need for 'volatile' attribute for the following members, as they
    //  are not concurrently used in IRQ handlers and main line code]
    uint8_t         ss_pin;
    uint8_t         orc;
    uint8_t         bytes_transferred;

    bool tx_done : 1;
    bool rx_done : 1;
} spi_control_block_t;
static spi_control_block_t spi_control_block_array[ENABLED_SPI_COUNT];

#if NRF_MODULE_ENABLED(PERIPHERAL_RESOURCE_SHARING)
    #define IRQ_HANDLER_NAME(n) irq_handler_for_instance_##n
    #define IRQ_HANDLER(n)      static void IRQ_HANDLER_NAME(n)(void)

    #if NRF_MODULE_ENABLED(SPI0)
        IRQ_HANDLER(0);
    #endif
    #if NRF_MODULE_ENABLED(SPI1)
        IRQ_HANDLER(1);
    #endif
    #if NRF_MODULE_ENABLED(SPI2)
        IRQ_HANDLER(2);
    #endif
    static nrf_drv_irq_handler_t const m_irq_handlers[ENABLED_SPI_COUNT] = {
    #if NRF_MODULE_ENABLED(SPI0)
        IRQ_HANDLER_NAME(0),
    #endif
    #if NRF_MODULE_ENABLED(SPI1)
        IRQ_HANDLER_NAME(1),
    #endif
    #if NRF_MODULE_ENABLED(SPI2)
        IRQ_HANDLER_NAME(2),
    #endif
    };
#else
    #define IRQ_HANDLER(n) void SPI##n##_IRQ_HANDLER(void)
#endif // NRF_MODULE_ENABLED(PERIPHERAL_RESOURCE_SHARING)


ret_code_t nrf_drv_spi_init(nrf_drv_spi_t const * const p_instance,
                            nrf_drv_spi_config_t const * p_config,
                            nrf_drv_spi_handler_t handler)
{
    uint32_t mosi_pin;
    uint32_t miso_pin;
	
	// Check sane parameters.
	ASSERT(p_instance);
	ASSERT(p_config);
	
	// Create a pointer to the appropriate index of spi_control_block_array.
    spi_control_block_t * spi_control_block  = &spi_control_block_array[p_instance->drv_inst_idx];
    ret_code_t err_code;
    
	// Is our SPI control block properly uninitialized?
    if (spi_control_block->state != NRF_DRV_STATE_UNINITIALIZED) {
		// Exit.
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_WARNING("Function: %s, error code: %s.\r\n", (uint32_t)__func__, (uint32_t)ERR_TO_STR(err_code));
        return err_code;
    }

    spi_control_block->handler = handler;

    // Configure pins used by the peripheral:
    // - SCK - output with initial value corresponding with the SPI mode used:
    //   0 - for modes 0 and 1 (CPOL = 0), 1 - for modes 2 and 3 (CPOL = 1);
    //   according to the reference manual guidelines this pin and its input
    //   buffer must always be connected for the SPI to work.
    if (p_config->mode <= NRF_DRV_SPI_MODE_1) {
        nrf_gpio_pin_clear(p_config->sck_pin);
    } else {
        nrf_gpio_pin_set(p_config->sck_pin);
    }
	// Set characteristics for sck pin.
    NRF_GPIO->PIN_CNF[p_config->sck_pin] =
        (GPIO_PIN_CNF_DIR_Output        << GPIO_PIN_CNF_DIR_Pos)
      | (GPIO_PIN_CNF_INPUT_Connect     << GPIO_PIN_CNF_INPUT_Pos)
//	  | (GPIO_PIN_CNF_INPUT_Disconnect     << GPIO_PIN_CNF_INPUT_Pos)
      | (GPIO_PIN_CNF_PULL_Disabled     << GPIO_PIN_CNF_PULL_Pos)
      | (GPIO_PIN_CNF_DRIVE_S0S1        << GPIO_PIN_CNF_DRIVE_Pos)
      | (GPIO_PIN_CNF_SENSE_Disabled    << GPIO_PIN_CNF_SENSE_Pos);
    
	// - MOSI (optional) - output with initial value 0,
    if (p_config->mosi_pin != NRF_DRV_SPI_PIN_NOT_USED) {
        mosi_pin = p_config->mosi_pin;
        nrf_gpio_pin_clear(mosi_pin);
        nrf_gpio_cfg_output(mosi_pin);
    } else {
        mosi_pin = NRF_SPI_PIN_NOT_CONNECTED;
    }
	
    // - MISO (optional) - input,
    if (p_config->miso_pin != NRF_DRV_SPI_PIN_NOT_USED) {
        miso_pin = p_config->miso_pin;
        nrf_gpio_cfg_input(miso_pin, NRF_GPIO_PIN_NOPULL);
    } else {
        miso_pin = NRF_SPI_PIN_NOT_CONNECTED;
    }
	
    // - Slave Select (optional) - output with initial value 1 (inactive).
    if (p_config->ss_pin != NRF_DRV_SPI_PIN_NOT_USED) {
        nrf_gpio_pin_set(p_config->ss_pin);
        nrf_gpio_cfg_output(p_config->ss_pin);
    }
    spi_control_block_array[p_instance->drv_inst_idx].ss_pin = p_config->ss_pin;
	
	// Point to correct spi register and configure it.
	NRF_SPIM_Type * p_spim = p_instance->p_registers;
	nrf_spim_pins_set(p_spim, p_config->sck_pin, mosi_pin, miso_pin);
	nrf_spim_frequency_set( p_spim, (nrf_spim_frequency_t)p_config->frequency);
	nrf_spim_configure( p_spim, (nrf_spim_mode_t)p_config->mode, (nrf_spim_bit_order_t)p_config->bit_order);
	nrf_spim_orc_set(p_spim, p_config->orc); // (overrun character)
	
	// If there is an interrupt handler, set up interrupts to be called once the spi transfer completes. TODO not sure how this tells NVIC when to trigger interrupt though.
	if (spi_control_block->handler) {
		nrf_spim_int_enable(p_spim, NRF_SPIM_INT_END_MASK);
	}
	// Enable spi master!
	nrf_spim_enable(p_spim);
	// Enable the spi interrupt with the NVIC. Not sure how it connects to the options we set up a few lines above though.
    if (spi_control_block->handler) {
        nrf_drv_common_irq_enable(p_instance->irq, p_config->irq_priority);
    }

    spi_control_block->transfer_in_progress = false;
    spi_control_block->state = NRF_DRV_STATE_INITIALIZED;

    NRF_LOG_INFO("Init\r\n");
    
    err_code = NRF_SUCCESS;
    NRF_LOG_INFO("Function: %s, error code: %s.\r\n", (uint32_t)__func__, (uint32_t)ERR_TO_STR(err_code));
    return err_code;
}

void nrf_drv_spi_uninit(nrf_drv_spi_t const * const p_instance)
{
    spi_control_block_t * p_cb = &spi_control_block_array[p_instance->drv_inst_idx];
    ASSERT(p_cb->state != NRF_DRV_STATE_UNINITIALIZED);

    if (p_cb->handler)
    {
        nrf_drv_common_irq_disable(p_instance->irq);
    }

    #define DISABLE_ALL  0xFFFFFFFF

	NRF_SPIM_Type * p_spim = p_instance->p_registers;
	if (p_cb->handler)
	{
		nrf_spim_int_disable(p_spim, DISABLE_ALL);
		if (p_cb->transfer_in_progress)
		{
			// Ensure that SPI is not performing any transfer.
			nrf_spim_task_trigger(p_spim, NRF_SPIM_TASK_STOP);
			while (!nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_STOPPED)) {}
			p_cb->transfer_in_progress = false;
		}
	}
	nrf_spim_disable(p_spim);
    #undef DISABLE_ALL

    p_cb->state = NRF_DRV_STATE_UNINITIALIZED;
}

// Load tx and rx params into an nrf_drv_spi_xfer_desc_t.
// Log some stuff.
// Run nrf_drv_spi_xfer.
ret_code_t nrf_drv_spi_transfer(nrf_drv_spi_t const * const drv_spi_parameters,
                                uint8_t const * p_tx_buffer,
                                uint8_t         tx_buffer_length,
                                uint8_t       * p_rx_buffer,
                                uint8_t         rx_buffer_length)
{
    nrf_drv_spi_xfer_desc_t xfer_desc;
    xfer_desc.p_tx_buffer = p_tx_buffer;
    xfer_desc.p_rx_buffer = p_rx_buffer;
    xfer_desc.tx_length   = tx_buffer_length;
    xfer_desc.rx_length   = rx_buffer_length;

    NRF_LOG_INFO("Transfer tx_len:%d, rx_len:%d.\r\n", tx_buffer_length, rx_buffer_length);
    NRF_LOG_DEBUG("Tx data:\r\n");
    NRF_LOG_HEXDUMP_DEBUG((uint8_t *)p_tx_buffer, tx_buffer_length * sizeof(p_tx_buffer));
	
	// Set transfer_in_progress, set CS low, and start transfer task.
    return nrf_drv_spi_xfer(drv_spi_parameters, &xfer_desc, 0);
}

static void finish_transfer(spi_control_block_t * p_cb) {
    // If Slave Select signal is used, this is the time to deactivate it.
    if (p_cb->ss_pin != NRF_DRV_SPI_PIN_NOT_USED) {
        nrf_gpio_pin_set(p_cb->ss_pin);
    }

    // By clearing this flag before calling the handler we allow subsequent
    // transfers to be started directly from the handler function.
    p_cb->transfer_in_progress = false;
    p_cb->evt.type = NRF_DRV_SPI_EVENT_DONE;
    NRF_LOG_INFO("Transfer rx_len:%d.\r\n", p_cb->evt.data.done.rx_length);
    NRF_LOG_DEBUG("Rx data:\r\n");
    NRF_LOG_HEXDUMP_DEBUG((uint8_t *)p_cb->evt.data.done.p_rx_buffer, 
                            p_cb->evt.data.done.rx_length * sizeof(p_cb->evt.data.done.p_rx_buffer));
    p_cb->handler(&p_cb->evt);
}

__STATIC_INLINE void spim_int_enable(NRF_SPIM_Type * p_spim, bool enable) {
    if (!enable) {
        nrf_spim_int_disable(p_spim, END_INT_MASK);
    } else {
        nrf_spim_int_enable(p_spim, END_INT_MASK);
    }
}

__STATIC_INLINE void spim_list_enable_handle(NRF_SPIM_Type * p_spim, uint32_t flags) {
    if (NRF_DRV_SPI_FLAG_TX_POSTINC & flags) {
        nrf_spim_tx_list_enable(p_spim);
    } else {
        nrf_spim_tx_list_disable(p_spim);
    }

    if (NRF_DRV_SPI_FLAG_RX_POSTINC & flags) {
        nrf_spim_rx_list_enable(p_spim);
    } else {
        nrf_spim_rx_list_disable(p_spim);
    }
}

// Point spi master buffers and transfer counts to spi_messages parameters.
// Tell the control register to start the spi master task.
// Enable spi interrupts. Exit without blocking unless there is no user-written handler.
static ret_code_t spim_xfer(NRF_SPIM_Type                * spi_control_register,
                           spi_control_block_t           * spi_control_block,
                           nrf_drv_spi_xfer_desc_t const * spi_messages,
                           uint32_t                        flags)
{
    ret_code_t err_code;
    // EasyDMA requires that transfer buffers are placed in Data RAM region;
    // signal error if they are not.
    if ((spi_messages->p_tx_buffer != NULL && !nrf_drv_is_in_RAM(spi_messages->p_tx_buffer)) ||
        (spi_messages->p_rx_buffer != NULL && !nrf_drv_is_in_RAM(spi_messages->p_rx_buffer))) {
        spi_control_block->transfer_in_progress = false;
        err_code = NRF_ERROR_INVALID_ADDR;
        NRF_LOG_WARNING("Function: %s, error code: %s.\r\n", (uint32_t)__func__, (uint32_t)ERR_TO_STR(err_code));
        return err_code;
    }

	// Point spi master buffers and transfer counts to spi_messages parameters.
    nrf_spim_tx_buffer_set(spi_control_register, spi_messages->p_tx_buffer, spi_messages->tx_length);
    nrf_spim_rx_buffer_set(spi_control_register, spi_messages->p_rx_buffer, spi_messages->rx_length);

	// Make sure transfer done IRQ doesn't go off erroneously.
    nrf_spim_event_clear(spi_control_register, NRF_SPIM_EVENT_END);

    // Enable tx list and rx list if the flags say to do so. If there are several tx and rx bytes to transfer,
	// then increment the pointer to those bytes after each transfer if this is enabled, I think.
	spim_list_enable_handle(spi_control_register, flags);

	// Tell the control register to start the spi master task (which I think means transfer up to max(rx_count, tx_count)), UNLESS flags say set up transfer but don't start it.
    if (!(flags & NRF_DRV_SPI_FLAG_HOLD_XFER)) {
        nrf_spim_task_trigger(spi_control_register, NRF_SPIM_TASK_START);
    }

	// Enable spi interrupts, unless there is no user-written event handler.
    if (!spi_control_block->handler) {
		// Block until transfer is done, then raise CS high.
        while (!nrf_spim_event_check(spi_control_register, NRF_SPIM_EVENT_END)){}
        if (spi_control_block->ss_pin != NRF_DRV_SPI_PIN_NOT_USED) {
            nrf_gpio_pin_set(spi_control_block->ss_pin);
        }
    } else {
		spim_int_enable(spi_control_register, !(flags & NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER));
	}
	
	
    err_code = NRF_SUCCESS;
    NRF_LOG_INFO("Function: %s, error code: %s.\r\n", (uint32_t)__func__, (uint32_t)ERR_TO_STR(err_code));
    return err_code;
}

// Using appropriate spi_control_block...
// 		Mark that there is a transfer in progress.
// 		Set CS low.
// 		Run spim_xfer.
ret_code_t nrf_drv_spi_xfer(nrf_drv_spi_t     const * const drv_spi_parameters,
                            nrf_drv_spi_xfer_desc_t const * spi_messages,
                            uint32_t                        flags)
{
	// Grab appropriate spi_control_block (e.g. control block for SPI0)
    spi_control_block_t * spi_control_block  = &spi_control_block_array[drv_spi_parameters->drv_inst_idx];
	
	// Check valid parameters passed in.
    ASSERT(spi_control_block->state != NRF_DRV_STATE_UNINITIALIZED);
    ASSERT(spi_messages->p_tx_buffer != NULL || spi_messages->tx_length == 0);
    ASSERT(spi_messages->p_rx_buffer != NULL || spi_messages->rx_length == 0);

	
    ret_code_t err_code = NRF_SUCCESS;

	// Mark that there is a transfer in progress. (Unless there is already one in progress, in which case warn and exit)
    if (spi_control_block->transfer_in_progress) {
        err_code = NRF_ERROR_BUSY;
        NRF_LOG_WARNING("Function: %s, error code: %s.\r\n", (uint32_t)__func__, (uint32_t)ERR_TO_STR(err_code));
        return err_code;
    } else {
        if (spi_control_block->handler && !(flags & (NRF_DRV_SPI_FLAG_REPEATED_XFER | NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER))) {
            spi_control_block->transfer_in_progress = true;
        }
    }

	// TODO what does evt.data.done get used for?
    spi_control_block->evt.data.done = *spi_messages;
    spi_control_block->tx_done = false;
    spi_control_block->rx_done = false;

	// Set CS low.
    if (spi_control_block->ss_pin != NRF_DRV_SPI_PIN_NOT_USED) {
        nrf_gpio_pin_clear(spi_control_block->ss_pin);
    }
	
	// Tell the control register to start the spi master task.
    return spim_xfer(drv_spi_parameters->p_registers, spi_control_block,  spi_messages, flags);
}

static void irq_handler_spim(NRF_SPIM_Type * p_spim, spi_control_block_t * p_cb)
{
    ASSERT(p_cb->handler);

    if (nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_END))
    {
        nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);
        NRF_LOG_DEBUG("SPIM: Event: NRF_SPIM_EVENT_END.\r\n");
        finish_transfer(p_cb);
    }
}

uint32_t nrf_drv_spi_start_task_get(nrf_drv_spi_t const * p_instance)
{
    NRF_SPIM_Type * p_spim = (NRF_SPIM_Type *)p_instance->p_registers;
    return nrf_spim_task_address_get(p_spim, NRF_SPIM_TASK_START);
}

uint32_t nrf_drv_spi_end_event_get(nrf_drv_spi_t const * p_instance)
{
    NRF_SPIM_Type * p_spim = (NRF_SPIM_Type *)p_instance->p_registers;
    return nrf_spim_event_address_get(p_spim, NRF_SPIM_EVENT_END);
}

#if NRF_MODULE_ENABLED(SPI0)
IRQ_HANDLER(0)
{
    spi_control_block_t * p_cb  = &spi_control_block_array[SPI0_INSTANCE_INDEX];
    #if SPI0_USE_EASY_DMA
        irq_handler_spim(NRF_SPIM0, p_cb);
    #else
        irq_handler_spi(NRF_SPI0, p_cb);
    #endif
}
#endif // NRF_MODULE_ENABLED(SPI0)

#if NRF_MODULE_ENABLED(SPI1)
IRQ_HANDLER(1)
{
    spi_control_block_t * p_cb  = &spi_control_block_array[SPI1_INSTANCE_INDEX];
    #if SPI1_USE_EASY_DMA
        irq_handler_spim(NRF_SPIM1, p_cb);
    #else
        irq_handler_spi(NRF_SPI1, p_cb);
    #endif
}
#endif // NRF_MODULE_ENABLED(SPI1)

#if NRF_MODULE_ENABLED(SPI2)
IRQ_HANDLER(2)
{
    spi_control_block_t * p_cb  = &spi_control_block_array[SPI2_INSTANCE_INDEX];
    #if SPI2_USE_EASY_DMA
        irq_handler_spim(NRF_SPIM2, p_cb);
    #else
        irq_handler_spi(NRF_SPI2, p_cb);
    #endif
}
#endif // NRF_MODULE_ENABLED(SPI2)
#endif // ENABLED_SPI_COUNT
#endif // NRF_MODULE_ENABLED(SPI)
