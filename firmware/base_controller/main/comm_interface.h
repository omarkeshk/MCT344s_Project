/*
 * comm_interface.h — UART & CAN communication interface
 *
 * UART protocol (RPi <-> Base):
 *   | START (0xAA) | MSG_ID (1B) | LENGTH (1B) | PAYLOAD (N B) | CRC8 (1B) | END (0x55) |
 *
 * CRC8 is computed over MSG_ID + LENGTH + PAYLOAD using polynomial 0x07 (CRC-8/CCITT).
 *
 * Message IDs:
 *   0x01  CMD_VELOCITY    RPi -> Base    vx(f32), vy(f32), omega(f32)        12 bytes
 *   0x02  ODOMETRY        Base -> RPi    x(f32), y(f32), theta(f32),
 *                                        vx(f32), vy(f32)                    20 bytes
 *   0x06  MODE_SWITCH     RPi -> Base    mode(u8)                             1 byte
 *   0x07  HEARTBEAT       bidirectional  timestamp(u32), status(u8)           5 bytes
 *   0x08  SENSOR_DATA     Base -> RPi    imu_yaw(f32), enc1-4(i32)           20 bytes
 *
 * CAN (TWAI) protocol (Base <-> Arm ESP32):
 *   Standard 11-bit IDs; payload up to 8 bytes per frame.
 */

#ifndef COMM_INTERFACE_H
#define COMM_INTERFACE_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- UART Configuration ---------- */
#define COMM_UART_PORT          UART_NUM_0
#define COMM_UART_BAUD          115200
#define COMM_UART_TX_GPIO       UART_PIN_NO_CHANGE  /* default USB-JTAG pins */
#define COMM_UART_RX_GPIO       UART_PIN_NO_CHANGE
#define COMM_UART_BUF_SIZE      512

/* ---------- CAN (TWAI) Configuration ---------- */
#define CAN_TX_GPIO             43
#define CAN_RX_GPIO             44
#define CAN_BITRATE_KBPS        500

/* ---------- Frame markers ---------- */
#define FRAME_START             0xAA
#define FRAME_END               0x55

/* ---------- Message IDs ---------- */
#define MSG_ID_CMD_VELOCITY     0x01
#define MSG_ID_ODOMETRY         0x02
#define MSG_ID_MODE_SWITCH      0x06
#define MSG_ID_HEARTBEAT        0x07
#define MSG_ID_SENSOR_DATA      0x08

/* Maximum payload size */
#define MAX_PAYLOAD_SIZE        32

/* ---------- Message structures ---------- */

typedef struct {
    float vx;
    float vy;
    float omega;
} __attribute__((packed)) msg_cmd_velocity_t;

typedef struct {
    float x;
    float y;
    float theta;
    float vx;
    float vy;
} __attribute__((packed)) msg_odometry_t;

typedef struct {
    uint8_t mode;
} __attribute__((packed)) msg_mode_switch_t;

typedef struct {
    uint32_t timestamp_ms;
    uint8_t  status;
} __attribute__((packed)) msg_heartbeat_t;

typedef struct {
    float   imu_yaw;
    int32_t enc_counts[4];
} __attribute__((packed)) msg_sensor_data_t;

/* ---------- Generic received message ---------- */

typedef struct {
    uint8_t  msg_id;
    uint8_t  length;
    uint8_t  payload[MAX_PAYLOAD_SIZE];
} comm_msg_t;

/* ---------- Callback type for received messages ---------- */
typedef void (*comm_rx_callback_t)(const comm_msg_t *msg);

/**
 * @brief Initialise UART for RPi communication.
 * @param cb  Callback invoked from the RX task for each valid message.
 * @return ESP_OK on success.
 */
esp_err_t comm_uart_init(comm_rx_callback_t cb);

/**
 * @brief Initialise TWAI (CAN bus) for arm module communication.
 * @return ESP_OK on success.
 */
esp_err_t comm_can_init(void);

/**
 * @brief Send a framed message over UART.
 *
 * Builds the frame (START, MSG_ID, LENGTH, payload, CRC8, END) and transmits.
 *
 * @param msg_id   Message identifier.
 * @param payload  Pointer to payload data.
 * @param length   Payload length in bytes.
 * @return ESP_OK on success.
 */
esp_err_t comm_uart_send(uint8_t msg_id, const void *payload, uint8_t length);

/**
 * @brief Send a CAN frame.
 *
 * @param id       Standard 11-bit CAN ID.
 * @param data     Payload (up to 8 bytes).
 * @param len      Data length (0-8).
 * @return ESP_OK on success.
 */
esp_err_t comm_can_send(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief Compute CRC-8 (poly 0x07) over a buffer.
 */
uint8_t comm_crc8(const uint8_t *data, size_t len);

/**
 * @brief Send odometry message to RPi.
 */
esp_err_t comm_send_odometry(float x, float y, float theta, float vx, float vy);

/**
 * @brief Send heartbeat message.
 */
esp_err_t comm_send_heartbeat(uint32_t timestamp_ms, uint8_t status);

/**
 * @brief Send sensor data message to RPi.
 */
esp_err_t comm_send_sensor_data(float imu_yaw, const int32_t enc_counts[4]);

#ifdef __cplusplus
}
#endif

#endif /* COMM_INTERFACE_H */
