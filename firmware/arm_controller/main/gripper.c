/*
 * gripper.c — Gripper control for SG90 servo (J5)
 *
 * State machine:
 *   OPEN     — gripper is at GRIPPER_OPEN_ANGLE
 *   CLOSING  — gripper is moving incrementally toward GRIPPER_CLOSE_ANGLE
 *   CLOSED   — gripper reached GRIPPER_CLOSE_ANGLE (no object detected)
 *   GRASPED  — gripper did not reach full close within timeout (object held)
 *   OPENING  — gripper is moving toward GRIPPER_OPEN_ANGLE
 *
 * Grasp detection:
 *   When closing, the gripper moves in small steps (GRIPPER_CLOSE_STEP_DEG)
 *   per update cycle.  If the full-close angle is not reached within
 *   GRIPPER_CLOSE_TIMEOUT_MS, we assume the gripper has stalled against
 *   an object.  The actual servo has no feedback, so this is a simple
 *   timeout/position heuristic.
 */

#include "gripper.h"
#include "servo_control.h"

#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "gripper";

/* ---------- Internal state ---------- */

static gripper_state_t s_state = GRIPPER_STATE_OPEN;
static float           s_current_angle = GRIPPER_OPEN_ANGLE;
static int64_t         s_close_start_us = 0;

/* ---------- Public API ---------- */

esp_err_t gripper_init(void)
{
    ESP_LOGI(TAG, "Initialising gripper (open angle=%.1f, close angle=%.1f)",
             GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE);

    /* Set gripper to fully open */
    esp_err_t ret = servo_set_angle(JOINT_GRIPPER, GRIPPER_OPEN_ANGLE);
    if (ret == ESP_OK) {
        s_state = GRIPPER_STATE_OPEN;
        s_current_angle = GRIPPER_OPEN_ANGLE;
    }
    return ret;
}

void gripper_open(void)
{
    ESP_LOGI(TAG, "Gripper OPEN commanded");
    s_state = GRIPPER_STATE_OPENING;

    /* Use smooth interpolation (500 ms move) */
    servo_move_to(JOINT_GRIPPER, GRIPPER_OPEN_ANGLE, 500);
}

void gripper_close(void)
{
    ESP_LOGI(TAG, "Gripper CLOSE commanded");
    s_state = GRIPPER_STATE_CLOSING;
    s_close_start_us = esp_timer_get_time();

    /* Start from the current angle */
    s_current_angle = servo_get_angle(JOINT_GRIPPER);
}

void gripper_set_angle(float angle_deg)
{
    servo_set_angle(JOINT_GRIPPER, angle_deg);
    s_current_angle = servo_get_angle(JOINT_GRIPPER);
}

void gripper_update(void)
{
    switch (s_state) {

    case GRIPPER_STATE_CLOSING: {
        int64_t now = esp_timer_get_time();
        int64_t elapsed_ms = (now - s_close_start_us) / 1000;

        /* Check timeout — if we haven't reached full close, object grasped */
        if (elapsed_ms > GRIPPER_CLOSE_TIMEOUT_MS) {
            if (s_current_angle > GRIPPER_CLOSE_ANGLE + GRIPPER_STALL_THRESH_DEG) {
                /* Gripper did not fully close: object detected */
                s_state = GRIPPER_STATE_GRASPED;
                ESP_LOGI(TAG, "Grasp detected at angle %.1f deg", s_current_angle);
            } else {
                /* Reached close angle within tolerance */
                s_state = GRIPPER_STATE_CLOSED;
                ESP_LOGI(TAG, "Gripper fully closed (no object)");
            }
            break;
        }

        /* Incrementally close the gripper */
        if (s_current_angle > GRIPPER_CLOSE_ANGLE) {
            s_current_angle -= GRIPPER_CLOSE_STEP_DEG;
            if (s_current_angle < GRIPPER_CLOSE_ANGLE) {
                s_current_angle = GRIPPER_CLOSE_ANGLE;
            }
            servo_set_angle(JOINT_GRIPPER, s_current_angle);
        } else {
            /* Already at close angle */
            s_state = GRIPPER_STATE_CLOSED;
            ESP_LOGI(TAG, "Gripper fully closed (no object)");
        }
        break;
    }

    case GRIPPER_STATE_OPENING:
        /* Check if interpolation is done */
        if (!servo_is_moving()) {
            s_state = GRIPPER_STATE_OPEN;
            s_current_angle = GRIPPER_OPEN_ANGLE;
            ESP_LOGI(TAG, "Gripper fully open");
        }
        break;

    case GRIPPER_STATE_OPEN:
    case GRIPPER_STATE_CLOSED:
    case GRIPPER_STATE_GRASPED:
        /* Steady state — nothing to do */
        break;
    }
}

gripper_state_t gripper_get_state(void)
{
    return s_state;
}

bool gripper_is_grasped(void)
{
    return (s_state == GRIPPER_STATE_GRASPED);
}

float gripper_get_angle(void)
{
    return servo_get_angle(JOINT_GRIPPER);
}
