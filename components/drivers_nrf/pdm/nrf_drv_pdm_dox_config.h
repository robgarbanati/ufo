/**
 *
 * @defgroup nrf_drv_pdm_config PDM peripheral driver configuration
 * @{
 * @ingroup nrf_drv_pdm
 */
/** @brief Enable PDM driver
 *
 *  Set to 1 to activate.
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PDM_ENABLED

/** @brief Mode
 *
 *  Following options are available:
 * - 0 - Stereo
 * - 1 - Mono
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PDM_CONFIG_MODE


/** @brief Edge
 *
 *  Following options are available:
 * - 0 - Left falling
 * - 1 - Left rising
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PDM_CONFIG_EDGE


/** @brief Clock frequency
 *
 *  Following options are available:
 * - 134217728 - 1000k
 * - 138412032 - 1032k (default)
 * - 142606336 - 1067k
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PDM_CONFIG_CLOCK_FREQ


/** @brief Interrupt priority
 *
 * Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
 *
 *  Following options are available:
 * - 0 - 0 (highest)
 * - 1 - 1
 * - 2 - 2
 * - 3 - 3
 * - 4 - 4 (except nRF51 family)
 * - 5 - 5 (except nRF51 family)
 * - 6 - 6 (except nRF51 family)
 * - 7 - 7 (except nRF51 family)
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PDM_CONFIG_IRQ_PRIORITY


/** @brief Enables logging in the module.
 *
 *  Set to 1 to activate.
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PDM_CONFIG_LOG_ENABLED

/** @brief Default Severity level
 *
 *  Following options are available:
 * - 0 - Off
 * - 1 - Error
 * - 2 - Warning
 * - 3 - Info
 * - 4 - Debug
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PDM_CONFIG_LOG_LEVEL


/** @brief ANSI escape code prefix.
 *
 *  Following options are available:
 * - 0 - Default
 * - 1 - Black
 * - 2 - Red
 * - 3 - Green
 * - 4 - Yellow
 * - 5 - Blue
 * - 6 - Magenta
 * - 7 - Cyan
 * - 8 - White
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PDM_CONFIG_INFO_COLOR


/** @brief ANSI escape code prefix.
 *
 *  Following options are available:
 * - 0 - Default
 * - 1 - Black
 * - 2 - Red
 * - 3 - Green
 * - 4 - Yellow
 * - 5 - Blue
 * - 6 - Magenta
 * - 7 - Cyan
 * - 8 - White
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PDM_CONFIG_DEBUG_COLOR




/** @} */
