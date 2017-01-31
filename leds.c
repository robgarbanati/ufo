#include "leds.h"

// This is for tracking PWM instances being used, so we can unintialize only
// the relevant ones when switching from one demo to another.
#define USED_PWM(idx) (1UL << idx)
static uint8_t m_used = 0;

#define FUN_SEQUENCE_DUTY 			0.1
#define FUN_SEQUENCE_OFF_TIME 		1 - FUN_SEQUENCE_DUTY
#define FUN_SEQUENCE_PULSE_WIDTH 	fun_led_sequence_top - FUN_SEQUENCE_OFF_TIME*fun_led_sequence_top


//**********************************************************************************************************************//
// Simple sequence for led_handler to manipulate.
//**********************************************************************************************************************//
static uint16_t const              led_handler_pwm_sequence_top  = 10000;
static uint16_t const              led_handler_pwm_sequence_step = 200;
static uint8_t                     led_handler_pwm_sequence_phase;
//static nrf_pwm_values_individual_t led_handler_pwm_sequence_values;
static nrf_pwm_values_individual_t led_handler_pwm_sequence_values;;
nrf_pwm_sequence_t const    led_handler_pwm_sequence =
{
    .values.p_individual = &led_handler_pwm_sequence_values,
    .length              = NRF_PWM_VALUES_LENGTH(led_handler_pwm_sequence_values),
    .repeats             = 1,
    .end_delay           = 0
};


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
// Fun sequence which takes no processor resources. TODO this stops after a while - why?
//**********************************************************************************************************************//
static uint16_t const              fun_led_sequence_top  = 10000;
static uint16_t const              fun_led_sequence_step = 200;
static uint8_t                     fun_led_sequence_phase;
//static nrf_pwm_values_individual_t fun_led_sequence_values;
static nrf_pwm_values_individual_t fun_led_sequence_values[] = 
{
	{FUN_SEQUENCE_PULSE_WIDTH, fun_led_sequence_top, fun_led_sequence_top, 0},
	{fun_led_sequence_top, FUN_SEQUENCE_PULSE_WIDTH, fun_led_sequence_top, 0},
	{fun_led_sequence_top, fun_led_sequence_top, FUN_SEQUENCE_PULSE_WIDTH, 0},
	{FUN_SEQUENCE_PULSE_WIDTH, FUN_SEQUENCE_PULSE_WIDTH, FUN_SEQUENCE_PULSE_WIDTH, 0}
};
nrf_pwm_sequence_t const    fun_led_sequence =
{
    .values.p_individual = fun_led_sequence_values,
    .length              = NRF_PWM_VALUES_LENGTH(fun_led_sequence_values),
    .repeats             = 10,
    .end_delay           = 0
};

//**********************************************************************************************************************//
// Blink red led to indicate critical error
//**********************************************************************************************************************//
static uint16_t const              error_led_sequence_top  = 10000;
static uint16_t const              error_led_sequence_step = 200;
static uint8_t                     error_led_sequence_phase;
//static nrf_pwm_values_individual_t error_led_sequence_values;
static nrf_pwm_values_individual_t error_led_sequence_values[] = 
{
	{error_led_sequence_top - error_led_sequence_top/4, error_led_sequence_top, error_led_sequence_top, 0},
	{error_led_sequence_top - error_led_sequence_top/64, error_led_sequence_top, error_led_sequence_top, 0}
};
nrf_pwm_sequence_t const    error_led_sequence =
{
    .values.p_individual = error_led_sequence_values,
    .length              = NRF_PWM_VALUES_LENGTH(error_led_sequence_values),
    .repeats             = 100,
    .end_delay           = 0
};




//**********************************************************************************************************************//
// PWM handlers
//**********************************************************************************************************************//

static void led_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED) {
        uint8_t channel    = led_handler_pwm_sequence_phase >> 1;
        bool    down       = led_handler_pwm_sequence_phase & 1;
        bool    next_phase = false;

        uint16_t * p_channels = (uint16_t *)&led_handler_pwm_sequence_values;
        uint16_t   value      = p_channels[channel];
        if (down) {
            value -= led_handler_pwm_sequence_step;
            if (value == 0) {
                next_phase = true;
            }
        } else {
            value += led_handler_pwm_sequence_step;
            if (value >= led_handler_pwm_sequence_top) {
                next_phase = true;
            }
        }
        p_channels[channel] = value;

        if (next_phase) {
            if (++led_handler_pwm_sequence_phase >= 2 * 3) {
                led_handler_pwm_sequence_phase = 0;
            }
        }
    }
}



static void motor_handler(nrf_drv_pwm_evt_type_t event_type)
{
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




//**********************************************************************************************************************//
// Init Functions
//**********************************************************************************************************************//
void set_up_led_pwm(nrf_drv_pwm_t *pwm_module, nrf_pwm_sequence_t const *led_sequence) {
	NRF_LOG_INFO("Setting up led pwm\r\n");
	
	nrf_gpio_cfg_output(LED_BANK_A);
	nrf_gpio_cfg_output(LED_BANK_B);
	nrf_gpio_cfg_output(LED_BANK_C);

	
	// Turn on all LEDs to 100%.
	nrf_gpio_pin_set(LED_BANK_A);
	nrf_gpio_pin_set(LED_BANK_B);
	nrf_gpio_pin_set(LED_BANK_C);
    /*
     * This demo plays back a sequence with different values for individual
     * channels (LED 1 - LED 4). Only four values are used (one per channel).
     * Every time the values are loaded into the compare registers, they are
     * updated in the provided event handler. The values are updated in such
     * a way that increase and decrease of the light intensity can be observed
     * continuously on succeeding channels (one second per channel).
     */

    uint32_t                   err_code;
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            LED_PWM_RED_PIN, // channel 0
            LED_PWM_GREEN_PIN, // channel 1
            LED_PWM_BLUE_PIN, // channel 2
            NRF_DRV_PWM_PIN_NOT_USED  // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = led_handler_pwm_sequence_top,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    err_code = nrf_drv_pwm_init(pwm_module, &config0, led_handler);
//	err_code = nrf_drv_pwm_init(pwm_module, &config0, NULL);
    APP_ERROR_CHECK(err_code);
	// TODO do this better I guess.
    m_used |= USED_PWM(0);

    led_handler_pwm_sequence_values.channel_0 = 0;
    led_handler_pwm_sequence_values.channel_1 = 0;
    led_handler_pwm_sequence_values.channel_2 = 0;
    led_handler_pwm_sequence_values.channel_3 = 0;
    led_handler_pwm_sequence_phase                = 0;

    nrf_drv_pwm_simple_playback(pwm_module, led_sequence, 1, NRF_DRV_PWM_FLAG_LOOP);
	
//	nrf_drv_pwm_simple_playback(pwm_module, &fun_led_sequence, 1, NRF_DRV_PWM_FLAG_LOOP);
}


#define M1_PWM_A	24
#define M1_PWM_B	23
#define M2_PWM_A	11
#define M2_PWM_B	12
#define M3_PWM_A	14
#define M3_PWM_B	13


void set_up_motor_pwm(nrf_drv_pwm_t *pwm_module, nrf_pwm_sequence_t const *pwm_sequence) {
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

    err_code = nrf_drv_pwm_init(pwm_module, &config0, motor_handler);
    CTR_APP_ERROR_CHECK(err_code, "nrf_drv_pwm_init failed!");
	// TODO do this better I guess.
    m_used |= USED_PWM(0);

    motor_handler_pwm_sequence_values.channel_0 = 0;
    motor_handler_pwm_sequence_values.channel_1 = 0;
    motor_handler_pwm_sequence_values.channel_2 = 0;
    motor_handler_pwm_sequence_values.channel_3 = 0;
    motor_handler_pwm_sequence_phase                = 0;

    nrf_drv_pwm_simple_playback(pwm_module, pwm_sequence, 1, NRF_DRV_PWM_FLAG_LOOP);
	
	
//	nrf_drv_pwm_simple_playback(pwm_module, &fun_led_sequence, 1, NRF_DRV_PWM_FLAG_LOOP);
}