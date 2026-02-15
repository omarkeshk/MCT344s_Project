/*
 * main.c — Arm Controller firmware entry point
 *
 * Target: ESP32-S3
 * Framework: ESP-IDF v5.x
 *
 * This module initialises all subsystems and creates the FreeRTOS tasks:
 *
 *   Task              Rate     Priority  Core  Stack
 *   ----              ----     --------  ----  -----
 *   servo_task        50 Hz    5         1     4096
 *   ik_task           20 Hz    4         0     4096
 *   comm_task        100 Hz    6         0     4096
 *   gripper_task      10 Hz    3         1     2048
 *   status_task        5 Hz    2         0     2048
 *
 * Servo control runs on Core 1 for deterministic PWM timing.
 * Communication and IK run on Core 0.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"

#include "servo_control.h"
#include "arm_kinematics.h"
#include "gripper.h"
#include "comm_interface.h"

static const char *TAG = "arm_main";

/* ---------- Shared state (protected by mutex) ---------- */

static SemaphoreHandle_t s_state_mutex;

/* Current operating mode */
static arm_mode_t s_mode = MODE_IDLE;

/* Target joint angles from latest command (degrees) */
static float s_target_joints[4] = { 90.0f, 90.0f, 90.0f, 90.0f };
static bool  s_joints_updated = false;

/* Target Cartesian position for IK mode (mm) */
static arm_cartesian_t s_target_cart = { .x = 200.0f, .y = 0.0f, .z = 150.0f };
static bool s_cart_updated = false;

/* Gripper command */
static uint8_t s_gripper_cmd = 0;  /* 0 = open, 1 = close */
static bool    s_gripper_updated = false;

/* Duration for joint moves (ms) */
#define DEFAULT_MOVE_DURATION_MS 800

/* ---------- Communication callbacks ---------- */

/**
 * @brief Called when a joint angle command (0x03) is received from the RPi.
 */
static void on_joint_cmd(float j1, float j2, float j3, float j4)
{
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        s_target_joints[0] = j1;
        s_target_joints[1] = j2;
        s_target_joints[2] = j3;
        s_target_joints[3] = j4;
        s_joints_updated = true;
        xSemaphoreGive(s_state_mutex);
    }
}

/**
 * @brief Called when a gripper command (0x04) is received from the RPi.
 */
static void on_gripper_cmd(uint8_t state)
{
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        s_gripper_cmd = state;
        s_gripper_updated = true;
        xSemaphoreGive(s_state_mutex);
    }
}

/**
 * @brief Called when a Cartesian target command (0x09) is received.
 */
static void on_cart_cmd(float x, float y, float z)
{
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        s_target_cart.x = x;
        s_target_cart.y = y;
        s_target_cart.z = z;
        s_cart_updated = true;
        xSemaphoreGive(s_state_mutex);
    }
}

/**
 * @brief Called when a mode switch command (0x06) is received.
 */
static void on_mode_switch(uint8_t mode)
{
    if (mode <= MODE_SEQUENCE) {
        if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            s_mode = (arm_mode_t)mode;
            ESP_LOGI(TAG, "Mode changed to %u", mode);
            xSemaphoreGive(s_state_mutex);
        }
    } else {
        ESP_LOGW(TAG, "Invalid mode: %u", mode);
    }
}

/* ---------- FreeRTOS tasks ---------- */

/**
 * @brief Servo control task — runs at 50 Hz on Core 1.
 *
 * Advances the smooth interpolation for all servos and writes
 * updated PWM duty cycles.
 */
static void servo_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(20);  /* 50 Hz */
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "servo_task started (50 Hz, Core 1)");

    while (1) {
        /* Advance all servo interpolations */
        servo_update();

        vTaskDelayUntil(&last_wake, period);
    }
}

/**
 * @brief Inverse kinematics task — runs at 20 Hz on Core 0.
 *
 * In MANUAL mode: applies target joint angles directly.
 * In CARTESIAN mode: computes IK and applies resulting joint angles.
 */
static void ik_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(50);  /* 20 Hz */
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "ik_task started (20 Hz, Core 0)");

    while (1) {
        arm_mode_t mode;
        float joints[4];
        bool do_joints = false;
        arm_cartesian_t cart;
        bool do_cart = false;

        /* Read shared state */
        if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            mode = s_mode;

            if (s_joints_updated) {
                memcpy(joints, s_target_joints, sizeof(joints));
                s_joints_updated = false;
                do_joints = true;
            }
            if (s_cart_updated) {
                cart = s_target_cart;
                s_cart_updated = false;
                do_cart = true;
            }
            xSemaphoreGive(s_state_mutex);
        } else {
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        /* Process based on mode */
        switch (mode) {

        case MODE_MANUAL:
            if (do_joints) {
                ESP_LOGD(TAG, "Manual: J1=%.1f J2=%.1f J3=%.1f J4=%.1f",
                         joints[0], joints[1], joints[2], joints[3]);
                servo_move_joints(joints, DEFAULT_MOVE_DURATION_MS);
            }
            break;

        case MODE_CARTESIAN:
            if (do_cart) {
                arm_joints_t ik_result;
                bool reachable = arm_ik(&cart, &ik_result);

                if (reachable) {
                    float ik_angles[4] = {
                        ik_result.j1, ik_result.j2,
                        ik_result.j3, ik_result.j4
                    };
                    ESP_LOGD(TAG, "IK: (%.0f,%.0f,%.0f) -> J1=%.1f J2=%.1f "
                             "J3=%.1f J4=%.1f",
                             cart.x, cart.y, cart.z,
                             ik_angles[0], ik_angles[1],
                             ik_angles[2], ik_angles[3]);
                    servo_move_joints(ik_angles, DEFAULT_MOVE_DURATION_MS);
                } else {
                    ESP_LOGW(TAG, "IK: target (%.0f, %.0f, %.0f) unreachable",
                             cart.x, cart.y, cart.z);
                }
            }
            break;

        case MODE_IDLE:
        case MODE_SEQUENCE:
            /* IDLE: no action. SEQUENCE: future expansion. */
            break;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/**
 * @brief Communication task — runs at 100 Hz on Core 0.
 *
 * Processes incoming UART and CAN frames, dispatching to callbacks.
 */
static void comm_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(10);  /* 100 Hz */
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "comm_task started (100 Hz, Core 0)");

    while (1) {
        comm_uart_process();
        comm_can_process();

        vTaskDelayUntil(&last_wake, period);
    }
}

/**
 * @brief Gripper task — runs at 10 Hz on Core 1.
 *
 * Checks for gripper commands and updates the gripper state machine.
 */
static void gripper_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(100);  /* 10 Hz */
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "gripper_task started (10 Hz, Core 1)");

    while (1) {
        /* Check for new gripper commands */
        uint8_t cmd = 0;
        bool updated = false;

        if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (s_gripper_updated) {
                cmd = s_gripper_cmd;
                updated = true;
                s_gripper_updated = false;
            }
            xSemaphoreGive(s_state_mutex);
        }

        if (updated) {
            if (cmd == 0) {
                gripper_open();
            } else {
                gripper_close();
            }
        }

        /* Advance the gripper state machine */
        gripper_update();

        vTaskDelayUntil(&last_wake, period);
    }
}

/**
 * @brief Status reporting task — runs at 5 Hz on Core 0.
 *
 * Periodically sends arm status over UART and CAN, and heartbeats.
 */
static void status_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(200);  /* 5 Hz */
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t heartbeat_counter = 0;

    ESP_LOGI(TAG, "status_task started (5 Hz, Core 0)");

    while (1) {
        /* Gather current state */
        float angles[SERVO_JOINT_COUNT];
        servo_get_all_angles(angles);
        uint8_t grip_state = (uint8_t)gripper_get_state();

        /* Send status over UART to RPi */
        comm_send_arm_status(angles, grip_state);

        /* Send condensed status over CAN to base controller */
        comm_can_send_arm_status(angles, grip_state);

        /* Send heartbeat every 1 second (every 5th iteration at 5 Hz) */
        heartbeat_counter++;
        if (heartbeat_counter >= 5) {
            heartbeat_counter = 0;
            uint8_t sys_status = 0x01;  /* 0x01 = healthy */
            comm_send_heartbeat(sys_status);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/* ---------- Application entry point ---------- */

void app_main(void)
{
    ESP_LOGI(TAG, "=== Arm Controller Firmware ===");
    ESP_LOGI(TAG, "ESP-IDF %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %lu bytes",
             (unsigned long)esp_get_free_heap_size());

    /* --- Create mutex for shared state --- */
    s_state_mutex = xSemaphoreCreateMutex();
    if (s_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return;
    }

    /* --- Initialise subsystems --- */
    esp_err_t ret;

    /* Servo PWM */
    ret = servo_control_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "servo_control_init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Gripper (depends on servo_control) */
    ret = gripper_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gripper_init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Communication (UART + CAN) */
    comm_callbacks_t cbs = {
        .on_joint_cmd   = on_joint_cmd,
        .on_gripper_cmd = on_gripper_cmd,
        .on_cart_cmd    = on_cart_cmd,
        .on_mode_switch = on_mode_switch,
    };
    ret = comm_init(&cbs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "comm_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "All subsystems initialised OK");

    /* --- Create FreeRTOS tasks --- */

    /* Servo control: 50 Hz, priority 5, Core 1 */
    BaseType_t xret;
    xret = xTaskCreatePinnedToCore(servo_task, "servo_task",
                                   4096, NULL, 5, NULL, 1);
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create servo_task");
        return;
    }

    /* IK solver: 20 Hz, priority 4, Core 0 */
    xret = xTaskCreatePinnedToCore(ik_task, "ik_task",
                                   4096, NULL, 4, NULL, 0);
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ik_task");
        return;
    }

    /* Communication: 100 Hz, priority 6, Core 0 */
    xret = xTaskCreatePinnedToCore(comm_task, "comm_task",
                                   4096, NULL, 6, NULL, 0);
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create comm_task");
        return;
    }

    /* Gripper state machine: 10 Hz, priority 3, Core 1 */
    xret = xTaskCreatePinnedToCore(gripper_task, "gripper_task",
                                   2048, NULL, 3, NULL, 1);
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create gripper_task");
        return;
    }

    /* Status reporting: 5 Hz, priority 2, Core 0 */
    xret = xTaskCreatePinnedToCore(status_task, "status_task",
                                   2048, NULL, 2, NULL, 0);
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create status_task");
        return;
    }

    ESP_LOGI(TAG, "All tasks created. System running.");
    ESP_LOGI(TAG, "Free heap after init: %lu bytes",
             (unsigned long)esp_get_free_heap_size());

    /* app_main returns — the idle task takes over on Core 0 */
}
