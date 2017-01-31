#include "motor.h"

#undef NRF_LOG_MODULE_NAME
#define NRF_LOG_MODULE_NAME "MOTOR"

#define M1_PWM_A	24
#define M1_PWM_B	23
#define M2_PWM_A	11
#define M2_PWM_B	12
#define M3_PWM_A	14
#define M3_PWM_B	13

//**********************************************************************************************************************//
// Simple sequence for motor_handler to manipulate.
//**********************************************************************************************************************//
static uint16_t const              motor_handler_pwm_sequence_top  = 10000;
static uint16_t const              motor_handler_pwm_sequence_step = 200;
static uint8_t                     motor_handler_pwm_sequence_phase;
//static nrf_pwm_values_individual_t motor_handler_pwm_sequence_values;
static nrf_pwm_values_individual_t motor_handler_pwm_sequence_values;;
nrf_pwm_sequence_t const    motor_handler_pwm_sequence =
{
    .values.p_individual = &motor_handler_pwm_sequence_values,
    .length              = NRF_PWM_VALUES_LENGTH(motor_handler_pwm_sequence_values),
    .repeats             = 1,
    .end_delay           = 0
};

//**********************************************************************************************************************//
// Turn off motors when there is a critical error
//**********************************************************************************************************************//
static uint16_t const              motor_error_sequence_top  = 10000;
static uint16_t const              motor_error_sequence_step = 200;
static uint8_t                     motor_error_sequence_phase;
static nrf_pwm_values_individual_t motor_error_sequence_values = {motor_error_sequence_top, motor_error_sequence_top, motor_error_sequence_top, 0};
nrf_pwm_sequence_t const    motor_error_sequence =
{
    .values.p_individual = &motor_error_sequence_values,
    .length              = NRF_PWM_VALUES_LENGTH(motor_error_sequence_values),
    .repeats             = 1,
    .end_delay           = 0
};


//**********************************************************************************************************************//
// Private Functions
//**********************************************************************************************************************//


//**********************************************************************************************************************//
// Public Functions
//**********************************************************************************************************************//

void motor_handler(nrf_drv_pwm_evt_type_t event_type) {
    if (event_type == NRF_DRV_PWM_EVT_FINISHED) {
        uint8_t channel    = motor_handler_pwm_sequence_phase >> 1;
        bool    down       = motor_handler_pwm_sequence_phase & 1;
        bool    next_phase = false;

        uint16_t * p_channels = (uint16_t *)&motor_handler_pwm_sequence_values;
        uint16_t   value      = p_channels[channel];
        if (down) {
            value -= motor_handler_pwm_sequence_step;
            if (value == 0) {
                next_phase = true;
            }
        } else {
            value += motor_handler_pwm_sequence_step;
            if (value >= motor_handler_pwm_sequence_top) {
                next_phase = true;
            }
        }
        p_channels[channel] = value;

        if (next_phase) {
            if (++motor_handler_pwm_sequence_phase >= 2 * 3) {
                motor_handler_pwm_sequence_phase = 0;
            }
		}
	}
}

void set_up_motor_pwm(nrf_drv_pwm_t *pwm_module, nrf_pwm_sequence_t const *pwm_sequence, void (*handler)(nrf_drv_pwm_evt_type_t event_type)) {
	NRF_LOG_INFO("Setting up motor pwm\r\n");

	// Configure motor pwm pins as outputs.
	nrf_gpio_cfg_output(M1_PWM_A);
	nrf_gpio_cfg_output(M2_PWM_A);
	nrf_gpio_cfg_output(M3_PWM_A);
//	nrf_gpio_cfg_output(M2_PWM_B);

    /*
     * This demo plays back a sequence with different values for individual
     * channels (LED 1 - LED 4). Only four values are used (one per channel).
     * Every time the values are loaded into the compare registers, they are
     * updated in the provided event handler. The values are updated in such
     * a way that increase and decrease of the light intensity can be observed
     * continuously on succeeding channels (one second per channel).
     */

	// TODO paramaterize this.
    uint32_t                   err_code;
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            M1_PWM_A, // channel 0
            M2_PWM_A, // channel 1
            M3_PWM_A, // channel 2
            NRF_DRV_PWM_PIN_NOT_USED  // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = motor_handler_pwm_sequence_top,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };

	nrf_drv_pwm_uninit(pwm_module);
    err_code = nrf_drv_pwm_init(pwm_module, &config0, handler);
    CTR_APP_ERROR_CHECK(err_code, "nrf_drv_pwm_init failed!");

    motor_handler_pwm_sequence_values.channel_0 = 0;
    motor_handler_pwm_sequence_values.channel_1 = 0;
    motor_handler_pwm_sequence_values.channel_2 = 0;
    motor_handler_pwm_sequence_values.channel_3 = 0;
    motor_handler_pwm_sequence_phase                = 0;

    nrf_drv_pwm_simple_playback(pwm_module, pwm_sequence, 1, NRF_DRV_PWM_FLAG_LOOP);
}
