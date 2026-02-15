/*
 * servo_control.c — LEDC-based servo PWM driver for 4-DOF arm + gripper
 *
 * Uses the ESP32-S3 LEDC peripheral in low-speed mode with 14-bit resolution
 * at 50 Hz (20 ms period).  Each servo angle [0, 180] maps linearly to a
 * pulse width [500, 2500] us.
 *
 * Smooth interpolation uses cosine easing (raised cosine) for natural
 * acceleration and deceleration profiles.
 */

#include "servo_control.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/ledc.h"

static const char *TAG = "servo_ctrl";

/* ---------- Per-joint configuration ---------- */

/* GPIO pin for each joint */
static const int servo_gpio[SERVO_JOINT_COUNT] = {
    SERVO_J1_GPIO,  /* J1 Base yaw       */
    SERVO_J2_GPIO,  /* J2 Shoulder pitch  */
    SERVO_J3_GPIO,  /* J3 Elbow pitch     */
    SERVO_J4_GPIO,  /* J4 Wrist pitch     */
    SERVO_J5_GPIO,  /* J5 Gripper         */
};

/* LEDC channel for each joint */
static const ledc_channel_t servo_channel[SERVO_JOINT_COUNT] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_4,
};

/* Mechanical angle limits (degrees) per joint */
static const servo_limits_t joint_limits[SERVO_JOINT_COUNT] = {
    { .min_angle =   0.0f, .max_angle = 180.0f },  /* J1 Base yaw      */
    { .min_angle =   0.0f, .max_angle = 180.0f },  /* J2 Shoulder       */
    { .min_angle =   0.0f, .max_angle = 180.0f },  /* J3 Elbow          */
    { .min_angle =   0.0f, .max_angle = 180.0f },  /* J4 Wrist          */
    { .min_angle =  10.0f, .max_angle = 170.0f },  /* J5 Gripper        */
};

/* Default (home) angles — arm straight up, gripper open */
static const float home_angles[SERVO_JOINT_COUNT] = {
    90.0f,   /* J1 centre          */
    90.0f,   /* J2 vertical        */
    90.0f,   /* J3 straight        */
    90.0f,   /* J4 neutral         */
    120.0f,  /* J5 gripper open    */
};

/* ---------- Internal state ---------- */

/* Interpolation state for each joint */
static servo_interp_t interp[SERVO_JOINT_COUNT];

/* ---------- Helper functions ---------- */

/**
 * @brief Clamp a float to [lo, hi].
 */
static inline float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
 * @brief Convert angle in degrees [0, 180] to LEDC duty (14-bit ticks).
 *
 * Mapping:
 *   angle  ->  pulse_us  = 500 + (angle / 180) * 2000
 *   pulse_us ->  ticks   = pulse_us * SERVO_PWM_TICKS / SERVO_PWM_PERIOD_US
 */
static uint32_t angle_to_duty(float angle_deg)
{
    float clamped = clampf(angle_deg, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);

    /* Linear mapping: 0 deg -> 500 us, 180 deg -> 2500 us */
    float pulse_us = (float)SERVO_PULSE_MIN_US
                   + (clamped / SERVO_ANGLE_MAX)
                     * (float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);

    /* Convert microseconds to 14-bit timer ticks */
    uint32_t duty = (uint32_t)(pulse_us * (float)SERVO_PWM_TICKS
                               / (float)SERVO_PWM_PERIOD_US);
    return duty;
}

/**
 * @brief Apply a duty value to a servo channel via LEDC.
 */
static esp_err_t set_duty(joint_id_t joint, uint32_t duty)
{
    esp_err_t ret;

    ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, servo_channel[joint], duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_duty failed for J%d: %s",
                 joint + 1, esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, servo_channel[joint]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_update_duty failed for J%d: %s",
                 joint + 1, esp_err_to_name(ret));
    }
    return ret;
}

/* ---------- Public API ---------- */

esp_err_t servo_control_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initialising servo control (%d channels, %d Hz, %d-bit)",
             SERVO_JOINT_COUNT, SERVO_PWM_FREQ_HZ, SERVO_PWM_RESOLUTION);

    /* --- Configure LEDC timer --- */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = SERVO_PWM_RESOLUTION,
        .freq_hz         = SERVO_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* --- Configure each LEDC channel and set home position --- */
    for (int j = 0; j < SERVO_JOINT_COUNT; j++) {
        uint32_t home_duty = angle_to_duty(home_angles[j]);

        ledc_channel_config_t ch_cfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = servo_channel[j],
            .timer_sel  = LEDC_TIMER_0,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = servo_gpio[j],
            .duty       = home_duty,
            .hpoint     = 0,
        };
        ret = ledc_channel_config(&ch_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "LEDC channel config failed for J%d: %s",
                     j + 1, esp_err_to_name(ret));
            return ret;
        }

        /* Initialise interpolation state to home */
        interp[j].start_angle   = home_angles[j];
        interp[j].target_angle  = home_angles[j];
        interp[j].current_angle = home_angles[j];
        interp[j].start_time_us = 0;
        interp[j].duration_us   = 0;
        interp[j].moving        = false;

        ESP_LOGI(TAG, "  J%d: GPIO %d, CH %d, home %.1f deg (duty %lu)",
                 j + 1, servo_gpio[j], servo_channel[j],
                 home_angles[j], (unsigned long)home_duty);
    }

    ESP_LOGI(TAG, "Servo control initialised OK");
    return ESP_OK;
}

esp_err_t servo_set_angle(joint_id_t joint, float angle)
{
    if (joint >= SERVO_JOINT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Clamp to joint limits */
    float clamped = clampf(angle, joint_limits[joint].min_angle,
                                  joint_limits[joint].max_angle);

    uint32_t duty = angle_to_duty(clamped);
    esp_err_t ret = set_duty(joint, duty);
    if (ret == ESP_OK) {
        /* Update state immediately (no interpolation) */
        interp[joint].current_angle = clamped;
        interp[joint].target_angle  = clamped;
        interp[joint].moving        = false;
    }
    return ret;
}

void servo_move_to(joint_id_t joint, float target_deg, uint32_t duration_ms)
{
    if (joint >= SERVO_JOINT_COUNT) {
        return;
    }

    float clamped = clampf(target_deg, joint_limits[joint].min_angle,
                                       joint_limits[joint].max_angle);

    if (duration_ms == 0) {
        /* Instant move */
        servo_set_angle(joint, clamped);
        return;
    }

    interp[joint].start_angle   = interp[joint].current_angle;
    interp[joint].target_angle  = clamped;
    interp[joint].start_time_us = esp_timer_get_time();
    interp[joint].duration_us   = (int64_t)duration_ms * 1000LL;
    interp[joint].moving        = true;
}

void servo_move_joints(const float angles[4], uint32_t duration_ms)
{
    for (int j = 0; j < 4; j++) {
        servo_move_to((joint_id_t)j, angles[j], duration_ms);
    }
}

void servo_update(void)
{
    int64_t now = esp_timer_get_time();

    for (int j = 0; j < SERVO_JOINT_COUNT; j++) {
        if (!interp[j].moving) {
            continue;
        }

        int64_t elapsed = now - interp[j].start_time_us;

        if (elapsed >= interp[j].duration_us) {
            /* Move complete — snap to target */
            interp[j].current_angle = interp[j].target_angle;
            interp[j].moving = false;
        } else {
            /* Cosine easing: smooth start and stop
             * t goes from 0 to 1 linearly with time.
             * eased = 0.5 * (1 - cos(pi * t))  gives S-curve.
             */
            float t = (float)elapsed / (float)interp[j].duration_us;
            float eased = 0.5f * (1.0f - cosf(M_PI * t));

            interp[j].current_angle = interp[j].start_angle
                + eased * (interp[j].target_angle - interp[j].start_angle);
        }

        /* Apply the interpolated angle to hardware */
        uint32_t duty = angle_to_duty(interp[j].current_angle);
        set_duty((joint_id_t)j, duty);
    }
}

bool servo_is_moving(void)
{
    for (int j = 0; j < SERVO_JOINT_COUNT; j++) {
        if (interp[j].moving) {
            return true;
        }
    }
    return false;
}

float servo_get_angle(joint_id_t joint)
{
    if (joint >= SERVO_JOINT_COUNT) {
        return 0.0f;
    }
    return interp[joint].current_angle;
}

void servo_get_all_angles(float angles[SERVO_JOINT_COUNT])
{
    for (int j = 0; j < SERVO_JOINT_COUNT; j++) {
        angles[j] = interp[j].current_angle;
    }
}
