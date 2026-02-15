/*
 * motor_control.c — MCPWM motor driver for 4 mecanum wheels via TB6612FNG
 *
 * Wiring per motor channel on the TB6612FNG:
 *   PWM_A (INx_1) and PWM_B (INx_2) set the direction and duty cycle.
 *   Direction encoding:
 *     Forward:  PWM_A = duty,  PWM_B = 0
 *     Reverse:  PWM_A = 0,     PWM_B = duty
 *     Brake:    PWM_A = HIGH,  PWM_B = HIGH
 *     Coast:    PWM_A = 0,     PWM_B = 0
 *
 * MCPWM mapping:
 *   Group 0, Operator 0 → FL  (GPIO 15 / 16)
 *   Group 0, Operator 1 → FR  (GPIO 18 / 19)
 *   Group 1, Operator 0 → RL  (GPIO 35 / 36)
 *   Group 1, Operator 1 → RR  (GPIO 38 / 39)
 *
 * ESP-IDF v5.x new MCPWM driver API is used throughout.
 */

#include "motor_control.h"

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include <math.h>
#include <string.h>

static const char *TAG = "motor_ctrl";

/* ---------- Per-motor internal state ---------- */
typedef struct {
    mcpwm_cmpr_handle_t  cmp_a;    /* comparator for PWM_A */
    mcpwm_cmpr_handle_t  cmp_b;    /* comparator for PWM_B */
} motor_handle_t;

static motor_handle_t s_motors[WHEEL_COUNT];

/* ------------------------------------------------------------------ *
 * Helper: configure one motor channel on a given MCPWM group/operator.
 * ------------------------------------------------------------------ */
static esp_err_t motor_channel_init(int group_id, int gpio_a, int gpio_b,
                                    motor_handle_t *mh)
{
    /* --- Timer --- */
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = group_id,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MOTOR_PWM_FREQ_HZ * MOTOR_PWM_RESOLUTION,
        .period_ticks  = MOTOR_PWM_RESOLUTION,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    /* --- Operator --- */
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_cfg = {
        .group_id = group_id,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    /* --- Comparator A (PWM_A) --- */
    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &mh->cmp_a));

    /* --- Comparator B (PWM_B) --- */
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &mh->cmp_b));

    /* --- Generator A (drives PWM_A GPIO) --- */
    mcpwm_gen_handle_t gen_a = NULL;
    mcpwm_generator_config_t gen_a_cfg = {
        .gen_gpio_num = gpio_a,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_a_cfg, &gen_a));

    /* Generator A: HIGH on timer empty, LOW on comparator A match */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_a,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       mh->cmp_a,
                                       MCPWM_GEN_ACTION_LOW)));

    /* --- Generator B (drives PWM_B GPIO) --- */
    mcpwm_gen_handle_t gen_b = NULL;
    mcpwm_generator_config_t gen_b_cfg = {
        .gen_gpio_num = gpio_b,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_b_cfg, &gen_b));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_b,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       mh->cmp_b,
                                       MCPWM_GEN_ACTION_LOW)));

    /* Start with zero duty (coast) */
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mh->cmp_a, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mh->cmp_b, 0));

    /* Enable and start the timer */
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
esp_err_t motor_control_init(void)
{
    ESP_LOGI(TAG, "Initialising motor control (4x MCPWM channels)");

    /* Configure TB6612FNG standby pins — drive HIGH to enable drivers */
    const gpio_config_t stby_cfg = {
        .pin_bit_mask = (1ULL << TB6612_STBY1_GPIO) |
                        (1ULL << TB6612_STBY2_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&stby_cfg));
    gpio_set_level(TB6612_STBY1_GPIO, 1);
    gpio_set_level(TB6612_STBY2_GPIO, 1);

    /* Configure DIR GPIOs (active-low or active-high depending on wiring;
     * we initialise them as outputs set LOW — unused in PWM-A/B mode). */
    const gpio_config_t dir_cfg = {
        .pin_bit_mask = (1ULL << MOTOR_FL_DIR_GPIO) |
                        (1ULL << MOTOR_FR_DIR_GPIO) |
                        (1ULL << MOTOR_RL_DIR_GPIO) |
                        (1ULL << MOTOR_RR_DIR_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dir_cfg));

    /* Initialise each motor channel */
    ESP_ERROR_CHECK(motor_channel_init(0, MOTOR_FL_PWM_A_GPIO,
                                          MOTOR_FL_PWM_B_GPIO,
                                          &s_motors[WHEEL_FL]));
    ESP_ERROR_CHECK(motor_channel_init(0, MOTOR_FR_PWM_A_GPIO,
                                          MOTOR_FR_PWM_B_GPIO,
                                          &s_motors[WHEEL_FR]));
    ESP_ERROR_CHECK(motor_channel_init(1, MOTOR_RL_PWM_A_GPIO,
                                          MOTOR_RL_PWM_B_GPIO,
                                          &s_motors[WHEEL_RL]));
    ESP_ERROR_CHECK(motor_channel_init(1, MOTOR_RR_PWM_A_GPIO,
                                          MOTOR_RR_PWM_B_GPIO,
                                          &s_motors[WHEEL_RR]));

    ESP_LOGI(TAG, "Motor control initialised successfully");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
void motor_set_speed(wheel_id_t wheel, float speed)
{
    if (wheel >= WHEEL_COUNT) {
        return;
    }

    /* Clamp speed to [-1.0, +1.0] */
    if (speed >  1.0f) speed =  1.0f;
    if (speed < -1.0f) speed = -1.0f;

    uint32_t duty = (uint32_t)(fabsf(speed) * MOTOR_PWM_RESOLUTION);

    motor_handle_t *mh = &s_motors[wheel];

    if (speed > 0.001f) {
        /* Forward: PWM_A = duty, PWM_B = 0 */
        mcpwm_comparator_set_compare_value(mh->cmp_a, duty);
        mcpwm_comparator_set_compare_value(mh->cmp_b, 0);
    } else if (speed < -0.001f) {
        /* Reverse: PWM_A = 0, PWM_B = duty */
        mcpwm_comparator_set_compare_value(mh->cmp_a, 0);
        mcpwm_comparator_set_compare_value(mh->cmp_b, duty);
    } else {
        /* Stop (coast): both LOW */
        mcpwm_comparator_set_compare_value(mh->cmp_a, 0);
        mcpwm_comparator_set_compare_value(mh->cmp_b, 0);
    }
}

/* ------------------------------------------------------------------ */
void motor_set_all_speeds(const float speeds[WHEEL_COUNT])
{
    for (int i = 0; i < WHEEL_COUNT; i++) {
        motor_set_speed((wheel_id_t)i, speeds[i]);
    }
}

/* ------------------------------------------------------------------ */
void motor_emergency_stop(void)
{
    /* Set all motors to zero */
    for (int i = 0; i < WHEEL_COUNT; i++) {
        motor_set_speed((wheel_id_t)i, 0.0f);
    }

    /* Pull STBY pins LOW to electrically disable TB6612FNG outputs */
    gpio_set_level(TB6612_STBY1_GPIO, 0);
    gpio_set_level(TB6612_STBY2_GPIO, 0);

    ESP_LOGW(TAG, "EMERGENCY STOP activated — motors disabled");
}

/* ------------------------------------------------------------------ */
void motor_enable(void)
{
    gpio_set_level(TB6612_STBY1_GPIO, 1);
    gpio_set_level(TB6612_STBY2_GPIO, 1);
    ESP_LOGI(TAG, "Motors re-enabled (STBY HIGH)");
}
