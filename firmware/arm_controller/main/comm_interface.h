/*
 * comm_interface.h — UART + CAN (TWAI) communication interface
 *
 * UART protocol (to/from Raspberry Pi supervisory controller):
 *   Frame: [START=0xAA] [MSG_ID] [LENGTH] [PAYLOAD...] [CRC8] [END=0x55]
 *
 * CAN (TWAI) protocol (to/from Base Controller ESP32):
 *   Standard 11-bit IDs, 8-byte data frames.
 *
 * Message IDs (UART):
 *   0x03  CMD_ARM_JOINTS   RPi -> Arm   j1(f32) j2(f32) j3(f32) j4(f32)
 *   0x04  CMD_GRIPPER      RPi -> Arm   state(u8): 0=open, 1=close
 *   0x05  ARM_STATUS       Arm -> RPi   j1-j5(5xf32) gripper_state(u8)
 *   0x06  MODE_SWITCH      RPi -> Arm   mode(u8)
 *   0x07  HEARTBEAT        bidirectional timestamp(u32) status(u8)
 *   0x09  CMD_ARM_CART     RPi -> Arm   x(f32) y(f32) z(f32)
 */

#ifndef COMM_INTERFACE_H
#define COMM_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- UART configuration ---------- */
#define COMM_UART_NUM           UART_NUM_0
#define COMM_UART_BAUD          115200
#define COMM_UART_TX_BUF_SIZE   256
#define COMM_UART_RX_BUF_SIZE   512

/* ---------- CAN (TWAI) GPIO ---------- */
#define COMM_CAN_TX_GPIO        43
#define COMM_CAN_RX_GPIO        44

/* ---------- Protocol framing ---------- */
#define PROTO_START_BYTE        0xAA
#define PROTO_END_BYTE          0x55
#define PROTO_MAX_PAYLOAD       64

/* ---------- Message IDs ---------- */
#define MSG_CMD_ARM_JOINTS      0x03
#define MSG_CMD_GRIPPER         0x04
#define MSG_ARM_STATUS          0x05
#define MSG_MODE_SWITCH         0x06
#define MSG_HEARTBEAT           0x07
#define MSG_CMD_ARM_CART        0x09

/* ---------- CAN arbitration IDs ---------- */
#define CAN_ID_ARM_STATUS       0x200   /* Arm -> Base: arm status    */
#define CAN_ID_BASE_CMD         0x100   /* Base -> Arm: base commands */
#define CAN_ID_HEARTBEAT        0x700   /* Heartbeat (either dir)     */

/* ---------- Operating modes ---------- */
typedef enum {
    MODE_IDLE       = 0,
    MODE_MANUAL     = 1,    /* Direct joint angle control       */
    MODE_CARTESIAN  = 2,    /* IK-based Cartesian positioning   */
    MODE_SEQUENCE   = 3,    /* Pre-programmed pick-and-place    */
} arm_mode_t;

/* ---------- Received command structure ---------- */
typedef struct {
    uint8_t msg_id;
    uint8_t length;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} comm_message_t;

/* ---------- Callbacks ---------- */

/**
 * @brief Callback type for received joint commands (MSG 0x03).
 * @param j1-j4 Target joint angles in degrees.
 */
typedef void (*comm_joint_cmd_cb_t)(float j1, float j2, float j3, float j4);

/**
 * @brief Callback type for received gripper commands (MSG 0x04).
 * @param state 0 = open, 1 = close.
 */
typedef void (*comm_gripper_cmd_cb_t)(uint8_t state);

/**
 * @brief Callback type for received Cartesian commands (MSG 0x09).
 * @param x, y, z Target position in millimetres.
 */
typedef void (*comm_cart_cmd_cb_t)(float x, float y, float z);

/**
 * @brief Callback type for mode switch commands (MSG 0x06).
 * @param mode New operating mode.
 */
typedef void (*comm_mode_cb_t)(uint8_t mode);

/* ---------- Callback registration structure ---------- */
typedef struct {
    comm_joint_cmd_cb_t     on_joint_cmd;
    comm_gripper_cmd_cb_t   on_gripper_cmd;
    comm_cart_cmd_cb_t      on_cart_cmd;
    comm_mode_cb_t          on_mode_switch;
} comm_callbacks_t;

/* ---------- API ---------- */

/**
 * @brief Initialise UART and CAN (TWAI) peripherals.
 *
 * @param cbs  Pointer to callback structure.  The pointer must remain
 *             valid for the lifetime of the application.
 * @return ESP_OK on success.
 */
esp_err_t comm_init(const comm_callbacks_t *cbs);

/**
 * @brief Process incoming UART data.
 *
 * Call this periodically from the communication task.  Reads available
 * bytes from the UART RX FIFO, parses complete frames, and dispatches
 * to the registered callbacks.
 */
void comm_uart_process(void);

/**
 * @brief Process incoming CAN frames.
 *
 * Reads any pending TWAI frames and handles them.
 */
void comm_can_process(void);

/**
 * @brief Send arm status to the Raspberry Pi (MSG 0x05).
 *
 * Payload: j1(f32) j2(f32) j3(f32) j4(f32) j5(f32) gripper_state(u8)
 *
 * @param joint_angles Array of 5 joint angles in degrees.
 * @param gripper_state Current gripper state (0=open, 1=closed, 3=grasped).
 * @return ESP_OK on success.
 */
esp_err_t comm_send_arm_status(const float joint_angles[5],
                               uint8_t gripper_state);

/**
 * @brief Send a heartbeat message (MSG 0x07).
 *
 * @param status System status byte.
 * @return ESP_OK on success.
 */
esp_err_t comm_send_heartbeat(uint8_t status);

/**
 * @brief Send arm status over CAN bus to the base controller.
 *
 * Packs a condensed 8-byte status frame.
 *
 * @param joint_angles Array of 5 joint angles in degrees.
 * @param gripper_state Current gripper state.
 * @return ESP_OK on success.
 */
esp_err_t comm_can_send_arm_status(const float joint_angles[5],
                                   uint8_t gripper_state);

#ifdef __cplusplus
}
#endif

#endif /* COMM_INTERFACE_H */
