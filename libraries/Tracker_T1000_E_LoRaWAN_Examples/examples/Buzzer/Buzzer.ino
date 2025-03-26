#include <Tracker_T1000_E_LoRaWAN_Examples.h>
#include <nrfx_timer.h>

////////////////////////////////////////////////////////////////////////////////
// Constants

const nrfx_timer_t pwm_timer = NRFX_TIMER_INSTANCE(2);

////////////////////////////////////////////////////////////////////////////////
// Private functions

static void pwm_toggle(nrf_timer_event_t, void *)
{
    static bool pwm_state = LOW;
    pwm_state = !pwm_state;
    digitalWrite(PIN_BUZZER_PWM, pwm_state);
}


static void pwm_set_frequency(uint32_t frequency)
{
    uint32_t period_us = 1e6 / frequency;

    nrfx_timer_disable(&pwm_timer);
    nrfx_timer_clear(&pwm_timer);
    // Divide by 2 for 50% duty cycle
    nrfx_timer_extended_compare(&pwm_timer, NRF_TIMER_CC_CHANNEL0, period_us / 2, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrfx_timer_enable(&pwm_timer);
}

static void beep_enable(bool enable)
{
    digitalWrite(PIN_BUZZER_EN, enable);
}

////////////////////////////////////////////////////////////////////////////////
// Setup and loop

void setup()
{
    pinMode(PIN_BUZZER_PWM, OUTPUT);
    pinMode(PIN_BUZZER_EN, OUTPUT);
    digitalWrite(PIN_BUZZER_PWM, LOW);
    digitalWrite(PIN_BUZZER_EN, LOW);

    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
    nrfx_timer_init(&pwm_timer, &timer_cfg, pwm_toggle);
}

void loop()
{
    pwm_set_frequency(600);
    beep_enable(true);
    delay(500);
    beep_enable(false);
    delay(500);

    pwm_set_frequency(800);
    beep_enable(true);
    delay(500);
    beep_enable(false);
    delay(500);

    pwm_set_frequency(1000);
    beep_enable(true);
    delay(500);
    beep_enable(false);
    delay(500);
}

////////////////////////////////////////////////////////////////////////////////
