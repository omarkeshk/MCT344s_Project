/*
 * odometry.c — Encoder reading via PCNT and dead-reckoning odometry
 *
 * Each of the four JGA25-371 motors has a hall-effect quadrature encoder.
 * The ESP32-S3 PCNT (Pulse Counter) peripheral decodes quadrature signals
 * in hardware, giving signed counts that increase/decrease with rotation.
 *
 * At each odometry tick (50 Hz):
 *   1. Read delta counts from all four PCNT channels.
 *   2. Convert counts to wheel angular velocities.
 *   3. Apply forward kinematics to get body-frame twist.
 *   4. Fuse IMU yaw with encoder-derived heading via complementary filter.
 *   5. Integrate pose (x, y, theta) in the world frame.
 */

#include "odometry.h"
#include "kinematics.h"
#include "motor_control.h"   /* for WHEEL_COUNT */

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <math.h>
#include <string.h>

static const char *TAG = "odom";

/* ---------- PCNT handles ---------- */
static pcnt_unit_handle_t    s_pcnt_units[WHEEL_COUNT];
static pcnt_channel_handle_t s_pcnt_ch_a[WHEEL_COUNT];
static pcnt_channel_handle_t s_pcnt_ch_b[WHEEL_COUNT];

/* ---------- Encoder accumulation ---------- */
static int32_t s_accum_counts[WHEEL_COUNT];  /* total accumulated counts */

/* ---------- Odometry state ---------- */
static odom_state_t    s_odom;
static SemaphoreHandle_t s_odom_mutex;

/* ---------- GPIO tables for the four encoders ---------- */
static const int s_enc_gpio_a[WHEEL_COUNT] = {
    ENCODER_FL_A_GPIO, ENCODER_FR_A_GPIO,
    ENCODER_RL_A_GPIO, ENCODER_RR_A_GPIO
};
static const int s_enc_gpio_b[WHEEL_COUNT] = {
    ENCODER_FL_B_GPIO, ENCODER_FR_B_GPIO,
    ENCODER_RL_B_GPIO, ENCODER_RR_B_GPIO
};

/* ------------------------------------------------------------------ *
 * Helper: initialise one PCNT unit for quadrature decoding.
 *
 * Channel A: counts on edges of signal A, direction from level of B.
 * Channel B: counts on edges of signal B, direction from level of A
 *            (inverted to double the resolution — full quadrature x4).
 * ------------------------------------------------------------------ */
static esp_err_t encoder_unit_init(int idx)
{
    /* --- Unit --- */
    pcnt_unit_config_t unit_cfg = {
        .high_limit = 32767,
        .low_limit  = -32768,
        .flags.accum_count = true,   /* enable accumulation on overflow */
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_pcnt_units[idx]));

    /* --- Glitch filter (1 us @ 80 MHz = 80 ticks) --- */
    pcnt_glitch_filter_config_t filt_cfg = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_pcnt_units[idx], &filt_cfg));

    /* --- Channel A: edge on A, level of B --- */
    pcnt_chan_config_t ch_a_cfg = {
        .edge_gpio_num  = s_enc_gpio_a[idx],
        .level_gpio_num = s_enc_gpio_b[idx],
    };
    ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_units[idx], &ch_a_cfg,
                                      &s_pcnt_ch_a[idx]));

    /* Rising edge of A: if B is low → +1, if B is high → -1 */
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(s_pcnt_ch_a[idx],
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,   /* pos edge */
        PCNT_CHANNEL_EDGE_ACTION_INCREASE)); /* neg edge */
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(s_pcnt_ch_a[idx],
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      /* B low   */
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE)); /* B high  */

    /* --- Channel B: edge on B, level of A --- */
    pcnt_chan_config_t ch_b_cfg = {
        .edge_gpio_num  = s_enc_gpio_b[idx],
        .level_gpio_num = s_enc_gpio_a[idx],
    };
    ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_units[idx], &ch_b_cfg,
                                      &s_pcnt_ch_b[idx]));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(s_pcnt_ch_b[idx],
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(s_pcnt_ch_b[idx],
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* --- Enable, clear, and start --- */
    ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt_units[idx]));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt_units[idx]));
    ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt_units[idx]));

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
esp_err_t odometry_init(void)
{
    ESP_LOGI(TAG, "Initialising PCNT quadrature encoders (x4)");

    s_odom_mutex = xSemaphoreCreateMutex();
    if (s_odom_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create odometry mutex");
        return ESP_ERR_NO_MEM;
    }

    memset(s_accum_counts, 0, sizeof(s_accum_counts));
    memset(&s_odom, 0, sizeof(s_odom));

    for (int i = 0; i < WHEEL_COUNT; i++) {
        ESP_ERROR_CHECK(encoder_unit_init(i));
    }

    ESP_LOGI(TAG, "Encoders initialised (CPR = %d)", ENCODER_CPR);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
void odometry_read_encoders(int32_t counts[4])
{
    for (int i = 0; i < WHEEL_COUNT; i++) {
        int cnt = 0;
        pcnt_unit_get_count(s_pcnt_units[i], &cnt);
        pcnt_unit_clear_count(s_pcnt_units[i]);
        counts[i] = (int32_t)cnt;
        s_accum_counts[i] += counts[i];
    }
}

/* ------------------------------------------------------------------ */
void odometry_get_raw_counts(int32_t counts[4])
{
    for (int i = 0; i < WHEEL_COUNT; i++) {
        counts[i] = s_accum_counts[i];
    }
}

/* ------------------------------------------------------------------ */
void odometry_update(float imu_yaw_rad, float dt)
{
    if (dt <= 0.0f) return;

    /* 1. Read encoder deltas */
    int32_t delta_counts[WHEEL_COUNT];
    odometry_read_encoders(delta_counts);

    /* 2. Convert counts to angular displacement (radians) and velocity */
    wheel_speeds_t wheel_vel;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        float delta_rad = (float)delta_counts[i] * (2.0f * (float)M_PI)
                          / (float)ENCODER_CPR;
        wheel_vel.w[i] = delta_rad / dt;   /* angular velocity (rad/s) */
    }

    /* 3. Forward kinematics → body twist */
    body_twist_t twist;
    kinematics_forward(&wheel_vel, &twist);

    /* 4. Heading fusion:
     *    If IMU yaw is available (not NAN), apply complementary filter.
     *    Otherwise, use encoder-derived omega integrated. */
    xSemaphoreTake(s_odom_mutex, portMAX_DELAY);

    float theta_enc = s_odom.theta + twist.omega * dt;

    if (!isnanf(imu_yaw_rad)) {
        /* Complementary filter: trust IMU more for heading */
        s_odom.theta = HEADING_ALPHA * imu_yaw_rad
                     + (1.0f - HEADING_ALPHA) * theta_enc;
    } else {
        s_odom.theta = theta_enc;
    }

    /* Wrap to [-pi, +pi] */
    while (s_odom.theta >  (float)M_PI) s_odom.theta -= 2.0f * (float)M_PI;
    while (s_odom.theta < -(float)M_PI) s_odom.theta += 2.0f * (float)M_PI;

    /* 5. Integrate position in world frame using mid-point heading */
    float cos_t = cosf(s_odom.theta);
    float sin_t = sinf(s_odom.theta);

    s_odom.x += (twist.vx * cos_t - twist.vy * sin_t) * dt;
    s_odom.y += (twist.vx * sin_t + twist.vy * cos_t) * dt;

    /* 6. Store body-frame velocities */
    s_odom.vx    = twist.vx;
    s_odom.vy    = twist.vy;
    s_odom.omega = twist.omega;

    xSemaphoreGive(s_odom_mutex);
}

/* ------------------------------------------------------------------ */
void odometry_get_state(odom_state_t *out)
{
    if (out == NULL) return;
    xSemaphoreTake(s_odom_mutex, portMAX_DELAY);
    *out = s_odom;
    xSemaphoreGive(s_odom_mutex);
}

/* ------------------------------------------------------------------ */
void odometry_reset(void)
{
    xSemaphoreTake(s_odom_mutex, portMAX_DELAY);
    memset(&s_odom, 0, sizeof(s_odom));
    for (int i = 0; i < WHEEL_COUNT; i++) {
        pcnt_unit_clear_count(s_pcnt_units[i]);
        s_accum_counts[i] = 0;
    }
    xSemaphoreGive(s_odom_mutex);
    ESP_LOGI(TAG, "Odometry reset to origin");
}
