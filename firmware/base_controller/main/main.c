/*
 * main.c — Base Controller Firmware Entry Point
 *
 * Omni-wheel (mecanum) mobile robot base controller running on ESP32-S3.
 *
 * FreeRTOS task architecture:
 *
 *   Task                Core  Priority  Rate     Purpose
 *   ─────────────────── ────  ────────  ───────  ────────────────────────
 *   control_task         1      20      1 kHz    PID speed control + IMU
 *   odometry_task        0      15      50 Hz    Odometry + TX to RPi
 *   heartbeat_task       0       5      2 Hz     Heartbeat + watchdog
 *
 * The UART RX task is spawned inside comm_interface.c (core 0, prio 10).
 *
 * Safety features:
 *   - Heartbeat watchdog: if no CMD_VELOCITY received for >500 ms, motors
 *     are stopped.
 *   - Emergency stop: triggered by E-STOP flag or communication timeout.
 *   - Task watchdog timer (hardware) configured via sdkconfig.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "motor_control.h"
#include "pid_controller.h"
#include "kinematics.h"
#include "odometry.h"
#include "imu_driver.h"
#include "comm_interface.h"

static const char *TAG = "main";

/* ================================================================== *
 * Constants and Configuration
 * ================================================================== */

/* Maximum wheel angular velocity (rad/s).
 * JGA25-371 at 12V with 21.3:1 gearbox: ~330 RPM = ~34.6 rad/s */
#define MAX_WHEEL_OMEGA         34.0f

/* PID gains (speed controller per wheel, output = duty [-1..+1]) */
#define PID_KP                  0.4f
#define PID_KI                  2.0f
#define PID_KD                  0.005f

/* Heartbeat / safety intervals */
#define HEARTBEAT_PERIOD_MS     500
#define CMD_TIMEOUT_MS          500     /* stop motors if no cmd received */
#define CONTROL_PERIOD_US       1000    /* 1 kHz */
#define ODOM_PERIOD_MS          20      /* 50 Hz */

/* Operating modes */
typedef enum {
    MODE_IDLE       = 0,
    MODE_TELEOP     = 1,
    MODE_AUTONOMOUS = 2,
    MODE_ESTOP      = 0xFF,
} robot_mode_t;

/* ================================================================== *
 * Shared State (protected by mutex / atomic access)
 * ================================================================== */

/* Velocity command from RPi */
static body_twist_t     s_cmd_twist;
static SemaphoreHandle_t s_cmd_mutex;
static int64_t          s_last_cmd_time_us;     /* esp_timer_get_time() */

/* Current mode */
static volatile robot_mode_t s_mode = MODE_IDLE;

/* PID controllers — one per wheel */
static pid_ctrl_t s_pid[WHEEL_COUNT];

/* ================================================================== *
 * UART RX Message Handler
 * ================================================================== */

static void on_uart_message(const comm_msg_t *msg)
{
    switch (msg->msg_id) {

    case MSG_ID_CMD_VELOCITY: {
        if (msg->length < sizeof(msg_cmd_velocity_t)) break;
        const msg_cmd_velocity_t *cmd =
            (const msg_cmd_velocity_t *)msg->payload;

        xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);
        s_cmd_twist.vx    = cmd->vx;
        s_cmd_twist.vy    = cmd->vy;
        s_cmd_twist.omega = cmd->omega;
        s_last_cmd_time_us = esp_timer_get_time();
        xSemaphoreGive(s_cmd_mutex);

        ESP_LOGD(TAG, "CMD_VEL vx=%.2f vy=%.2f w=%.2f",
                 cmd->vx, cmd->vy, cmd->omega);
        break;
    }

    case MSG_ID_MODE_SWITCH: {
        if (msg->length < sizeof(msg_mode_switch_t)) break;
        const msg_mode_switch_t *m = (const msg_mode_switch_t *)msg->payload;
        robot_mode_t new_mode = (robot_mode_t)m->mode;

        if (new_mode == MODE_ESTOP) {
            s_mode = MODE_ESTOP;
            motor_emergency_stop();
            ESP_LOGW(TAG, "E-STOP requested by RPi");
        } else {
            if (s_mode == MODE_ESTOP) {
                motor_enable();
                ESP_LOGI(TAG, "Clearing E-STOP, entering mode %d", new_mode);
            }
            s_mode = new_mode;
            ESP_LOGI(TAG, "Mode changed to %d", new_mode);
        }
        break;
    }

    case MSG_ID_HEARTBEAT: {
        /* RPi heartbeat — we just log it; the command timeout handles safety */
        ESP_LOGD(TAG, "Heartbeat from RPi");
        break;
    }

    default:
        ESP_LOGW(TAG, "Unknown message ID 0x%02X", msg->msg_id);
        break;
    }
}

/* ================================================================== *
 * Task: PID Speed Control Loop — 1 kHz, Core 1
 *
 * 1. Read the latest velocity command.
 * 2. Apply inverse kinematics to get per-wheel target angular velocities.
 * 3. Read current wheel velocities from encoder deltas (sampled at 1 kHz,
 *    but odometry integration happens at 50 Hz separately).
 * 4. Run PID for each wheel.
 * 5. Apply motor duty cycles.
 * 6. Update IMU.
 * ================================================================== */

static void control_task(void *arg)
{
    const float dt = (float)CONTROL_PERIOD_US / 1.0e6f;   /* 0.001 s */
    TickType_t xLastWake = xTaskGetTickCount();

    /* Subscribe this task to the task watchdog */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    ESP_LOGI(TAG, "Control task started (1 kHz, core 1)");

    while (1) {
        /* Feed the watchdog */
        esp_task_wdt_reset();

        /* --- 1. Get velocity command (thread-safe) --- */
        body_twist_t cmd;
        xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);
        cmd = s_cmd_twist;
        int64_t last_cmd = s_last_cmd_time_us;
        xSemaphoreGive(s_cmd_mutex);

        /* Safety: if no command received recently, zero the command */
        int64_t now = esp_timer_get_time();
        if ((now - last_cmd) > (CMD_TIMEOUT_MS * 1000LL)) {
            cmd.vx    = 0.0f;
            cmd.vy    = 0.0f;
            cmd.omega = 0.0f;
        }

        /* If E-STOP, force zero */
        if (s_mode == MODE_ESTOP) {
            cmd.vx = cmd.vy = cmd.omega = 0.0f;
        }

        /* --- 2. Inverse kinematics: desired body twist -> wheel targets --- */
        wheel_speeds_t target;
        kinematics_inverse(&cmd, &target);
        kinematics_normalise(&target, MAX_WHEEL_OMEGA);

        /* --- 3. Measure current wheel velocities ---
         * The odometry task updates body-frame velocities at 50 Hz.
         * We reconstruct per-wheel angular velocities via inverse
         * kinematics so each PID channel has its own setpoint/measurement
         * pair.  The 50 Hz update rate is adequate because the mechanical
         * time constant of the geared DC motors (~100 ms) is much slower
         * than the 20 ms odometry period. */
        odom_state_t odom;
        odometry_get_state(&odom);

        body_twist_t meas_twist = {
            .vx    = odom.vx,
            .vy    = odom.vy,
            .omega = odom.omega,
        };
        wheel_speeds_t measured;
        kinematics_inverse(&meas_twist, &measured);

        /* --- 4. PID for each wheel --- */
        float duties[WHEEL_COUNT];
        for (int i = 0; i < WHEEL_COUNT; i++) {
            duties[i] = pid_update(&s_pid[i], target.w[i],
                                   measured.w[i], dt);
        }

        /* --- 5. Apply to motors --- */
        if (s_mode != MODE_ESTOP) {
            motor_set_all_speeds(duties);
        }

        /* --- 6. Update IMU --- */
        imu_update(dt);

        /* --- Sleep until next 1 ms tick --- */
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(1));
    }
}

/* ================================================================== *
 * Task: Odometry — 50 Hz, Core 0
 *
 * Reads encoders, runs forward kinematics, fuses IMU heading,
 * integrates pose, and transmits odometry + sensor data to the RPi.
 * ================================================================== */

static void odometry_task(void *arg)
{
    TickType_t xLastWake = xTaskGetTickCount();
    const float dt = (float)ODOM_PERIOD_MS / 1000.0f;
    uint32_t odom_counter = 0;

    ESP_LOGI(TAG, "Odometry task started (50 Hz, core 0)");

    while (1) {
        /* Get IMU yaw for heading fusion */
        float imu_yaw = imu_get_yaw();

        /* Update odometry (reads encoders, integrates) */
        odometry_update(imu_yaw, dt);

        /* Get the latest state for transmission */
        odom_state_t state;
        odometry_get_state(&state);

        /* Send odometry to RPi every cycle (50 Hz) */
        comm_send_odometry(state.x, state.y, state.theta,
                           state.vx, state.vy);

        /* Send sensor data at 10 Hz (every 5th cycle) */
        odom_counter++;
        if (odom_counter % 5 == 0) {
            int32_t raw_counts[4];
            odometry_get_raw_counts(raw_counts);
            comm_send_sensor_data(imu_yaw, raw_counts);
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(ODOM_PERIOD_MS));
    }
}

/* ================================================================== *
 * Task: Heartbeat & Watchdog — 2 Hz, Core 0
 * ================================================================== */

static void heartbeat_task(void *arg)
{
    TickType_t xLastWake = xTaskGetTickCount();

    ESP_LOGI(TAG, "Heartbeat task started (2 Hz, core 0)");

    while (1) {
        uint32_t timestamp = (uint32_t)(esp_timer_get_time() / 1000ULL);
        uint8_t status = (uint8_t)s_mode;

        comm_send_heartbeat(timestamp, status);

        /* Check for command timeout and log a warning */
        int64_t now = esp_timer_get_time();
        int64_t last;
        xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);
        last = s_last_cmd_time_us;
        xSemaphoreGive(s_cmd_mutex);

        if (s_mode != MODE_IDLE && s_mode != MODE_ESTOP) {
            if ((now - last) > (CMD_TIMEOUT_MS * 1000LL)) {
                ESP_LOGW(TAG, "No CMD_VELOCITY for >%d ms — motors zeroed",
                         CMD_TIMEOUT_MS);
            }
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
    }
}

/* ================================================================== *
 * App Main
 * ================================================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Base Controller — Mecanum Robot");
    ESP_LOGI(TAG, "  ESP-IDF %s", esp_get_idf_version());
    ESP_LOGI(TAG, "========================================");

    /* --- Create shared-state mutex --- */
    s_cmd_mutex = xSemaphoreCreateMutex();
    assert(s_cmd_mutex != NULL);

    memset(&s_cmd_twist, 0, sizeof(s_cmd_twist));
    s_last_cmd_time_us = esp_timer_get_time();

    /* --- Initialise PID controllers --- */
    for (int i = 0; i < WHEEL_COUNT; i++) {
        pid_init(&s_pid[i], PID_KP, PID_KI, PID_KD, -1.0f, 1.0f);
    }

    /* --- Initialise hardware peripherals --- */
    ESP_LOGI(TAG, "Initialising peripherals...");

    ESP_ERROR_CHECK(motor_control_init());
    ESP_ERROR_CHECK(odometry_init());
    ESP_ERROR_CHECK(imu_init());
    ESP_ERROR_CHECK(comm_uart_init(on_uart_message));
    ESP_ERROR_CHECK(comm_can_init());

    ESP_LOGI(TAG, "All peripherals initialised");

    /* --- Create FreeRTOS tasks --- */

    /* Control loop: 1 kHz, highest priority, pinned to core 1 */
    BaseType_t ret;
    ret = xTaskCreatePinnedToCore(
        control_task, "ctrl", 4096, NULL, 20, NULL, 1);
    assert(ret == pdPASS);

    /* Odometry: 50 Hz, medium priority, core 0 */
    ret = xTaskCreatePinnedToCore(
        odometry_task, "odom", 4096, NULL, 15, NULL, 0);
    assert(ret == pdPASS);

    /* Heartbeat: 2 Hz, low priority, core 0 */
    ret = xTaskCreatePinnedToCore(
        heartbeat_task, "hbeat", 2048, NULL, 5, NULL, 0);
    assert(ret == pdPASS);

    ESP_LOGI(TAG, "All tasks launched — entering IDLE mode");
    s_mode = MODE_IDLE;

    /* app_main returns; FreeRTOS scheduler keeps running the tasks */
}
