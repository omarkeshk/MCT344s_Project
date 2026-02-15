/*
 * comm_interface.c — UART + CAN (TWAI) communication interface
 *
 * UART frame format:
 *   [0xAA] [MSG_ID] [LENGTH] [PAYLOAD (0..64 bytes)] [CRC8] [0x55]
 *
 * CRC8 is computed over MSG_ID + LENGTH + PAYLOAD using the polynomial
 * x^8 + x^2 + x + 1 (CRC-8/MAXIM, init=0x00).
 *
 * TWAI (CAN 2.0B) is configured at 500 kbit/s with standard 11-bit IDs.
 */

#include "comm_interface.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/twai.h"

static const char *TAG = "comm";

/* ---------- Internal state ---------- */

/* Registered callbacks */
static comm_callbacks_t s_cbs;

/* UART RX state machine */
typedef enum {
    RX_WAIT_START,
    RX_WAIT_ID,
    RX_WAIT_LEN,
    RX_WAIT_PAYLOAD,
    RX_WAIT_CRC,
    RX_WAIT_END,
} rx_state_t;

static rx_state_t  s_rx_state = RX_WAIT_START;
static uint8_t     s_rx_msg_id;
static uint8_t     s_rx_len;
static uint8_t     s_rx_payload[PROTO_MAX_PAYLOAD];
static uint8_t     s_rx_idx;
static uint8_t     s_rx_crc;

/* ---------- CRC-8 (polynomial 0x07, init 0x00) ---------- */

/**
 * @brief Compute CRC-8 over a byte buffer.
 *
 * Uses the standard CRC-8 polynomial: x^8 + x^2 + x + 1 = 0x07.
 */
static uint8_t crc8_compute(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Compute CRC-8 over message header (id, len) + payload.
 */
static uint8_t compute_frame_crc(uint8_t msg_id, uint8_t length,
                                 const uint8_t *payload)
{
    uint8_t buf[2 + PROTO_MAX_PAYLOAD];
    buf[0] = msg_id;
    buf[1] = length;
    if (length > 0 && payload != NULL) {
        memcpy(&buf[2], payload, length);
    }
    return crc8_compute(buf, 2 + length);
}

/* ---------- UART transmit ---------- */

/**
 * @brief Send a framed UART message.
 */
static esp_err_t uart_send_frame(uint8_t msg_id, const uint8_t *payload,
                                 uint8_t length)
{
    /* Build frame: START + ID + LEN + PAYLOAD + CRC + END */
    uint8_t frame[4 + PROTO_MAX_PAYLOAD + 2];  /* worst case */
    int pos = 0;

    frame[pos++] = PROTO_START_BYTE;
    frame[pos++] = msg_id;
    frame[pos++] = length;

    if (length > 0 && payload != NULL) {
        memcpy(&frame[pos], payload, length);
        pos += length;
    }

    frame[pos++] = compute_frame_crc(msg_id, length, payload);
    frame[pos++] = PROTO_END_BYTE;

    int written = uart_write_bytes(COMM_UART_NUM, frame, pos);
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ---------- Dispatch received message ---------- */

/**
 * @brief Handle a fully parsed and CRC-validated UART message.
 */
static void dispatch_message(uint8_t msg_id, const uint8_t *payload,
                             uint8_t length)
{
    switch (msg_id) {

    case MSG_CMD_ARM_JOINTS: {
        /* Expect 4 x float32 = 16 bytes */
        if (length < 16) {
            ESP_LOGW(TAG, "CMD_ARM_JOINTS: payload too short (%d)", length);
            return;
        }
        float j1, j2, j3, j4;
        memcpy(&j1, &payload[0],  sizeof(float));
        memcpy(&j2, &payload[4],  sizeof(float));
        memcpy(&j3, &payload[8],  sizeof(float));
        memcpy(&j4, &payload[12], sizeof(float));
        ESP_LOGD(TAG, "RX CMD_ARM_JOINTS: J1=%.1f J2=%.1f J3=%.1f J4=%.1f",
                 j1, j2, j3, j4);
        if (s_cbs.on_joint_cmd) {
            s_cbs.on_joint_cmd(j1, j2, j3, j4);
        }
        break;
    }

    case MSG_CMD_GRIPPER: {
        /* Expect 1 byte: state */
        if (length < 1) {
            ESP_LOGW(TAG, "CMD_GRIPPER: payload too short (%d)", length);
            return;
        }
        uint8_t state = payload[0];
        ESP_LOGD(TAG, "RX CMD_GRIPPER: state=%u", state);
        if (s_cbs.on_gripper_cmd) {
            s_cbs.on_gripper_cmd(state);
        }
        break;
    }

    case MSG_MODE_SWITCH: {
        /* Expect 1 byte: mode */
        if (length < 1) {
            ESP_LOGW(TAG, "MODE_SWITCH: payload too short (%d)", length);
            return;
        }
        uint8_t mode = payload[0];
        ESP_LOGI(TAG, "RX MODE_SWITCH: mode=%u", mode);
        if (s_cbs.on_mode_switch) {
            s_cbs.on_mode_switch(mode);
        }
        break;
    }

    case MSG_HEARTBEAT: {
        /* Expect 5 bytes: timestamp(u32) + status(u8) */
        if (length >= 5) {
            uint32_t ts;
            memcpy(&ts, &payload[0], sizeof(uint32_t));
            ESP_LOGD(TAG, "RX HEARTBEAT: ts=%lu status=%u",
                     (unsigned long)ts, payload[4]);
        }
        break;
    }

    case MSG_CMD_ARM_CART: {
        /* Expect 3 x float32 = 12 bytes */
        if (length < 12) {
            ESP_LOGW(TAG, "CMD_ARM_CART: payload too short (%d)", length);
            return;
        }
        float x, y, z;
        memcpy(&x, &payload[0], sizeof(float));
        memcpy(&y, &payload[4], sizeof(float));
        memcpy(&z, &payload[8], sizeof(float));
        ESP_LOGD(TAG, "RX CMD_ARM_CART: x=%.1f y=%.1f z=%.1f", x, y, z);
        if (s_cbs.on_cart_cmd) {
            s_cbs.on_cart_cmd(x, y, z);
        }
        break;
    }

    default:
        ESP_LOGW(TAG, "Unknown message ID: 0x%02X", msg_id);
        break;
    }
}

/* ---------- UART RX parser (byte-by-byte state machine) ---------- */

/**
 * @brief Feed one byte into the UART frame parser.
 */
static void rx_feed_byte(uint8_t byte)
{
    switch (s_rx_state) {

    case RX_WAIT_START:
        if (byte == PROTO_START_BYTE) {
            s_rx_state = RX_WAIT_ID;
        }
        break;

    case RX_WAIT_ID:
        s_rx_msg_id = byte;
        s_rx_state = RX_WAIT_LEN;
        break;

    case RX_WAIT_LEN:
        s_rx_len = byte;
        if (s_rx_len > PROTO_MAX_PAYLOAD) {
            ESP_LOGW(TAG, "RX: length %u exceeds max, resetting", s_rx_len);
            s_rx_state = RX_WAIT_START;
        } else if (s_rx_len == 0) {
            /* No payload — go straight to CRC */
            s_rx_state = RX_WAIT_CRC;
        } else {
            s_rx_idx = 0;
            s_rx_state = RX_WAIT_PAYLOAD;
        }
        break;

    case RX_WAIT_PAYLOAD:
        s_rx_payload[s_rx_idx++] = byte;
        if (s_rx_idx >= s_rx_len) {
            s_rx_state = RX_WAIT_CRC;
        }
        break;

    case RX_WAIT_CRC:
        s_rx_crc = byte;
        s_rx_state = RX_WAIT_END;
        break;

    case RX_WAIT_END:
        if (byte == PROTO_END_BYTE) {
            /* Validate CRC */
            uint8_t expected = compute_frame_crc(s_rx_msg_id, s_rx_len,
                                                 s_rx_payload);
            if (s_rx_crc == expected) {
                dispatch_message(s_rx_msg_id, s_rx_payload, s_rx_len);
            } else {
                ESP_LOGW(TAG, "RX: CRC mismatch (got 0x%02X, expected 0x%02X)",
                         s_rx_crc, expected);
            }
        } else {
            ESP_LOGW(TAG, "RX: expected END byte 0x55, got 0x%02X", byte);
        }
        s_rx_state = RX_WAIT_START;
        break;
    }
}

/* ---------- Public API ---------- */

esp_err_t comm_init(const comm_callbacks_t *cbs)
{
    esp_err_t ret;

    /* Store callbacks */
    if (cbs != NULL) {
        memcpy(&s_cbs, cbs, sizeof(comm_callbacks_t));
    } else {
        memset(&s_cbs, 0, sizeof(comm_callbacks_t));
    }

    /* --- UART initialisation --- */
    ESP_LOGI(TAG, "Initialising UART%d at %d baud", COMM_UART_NUM,
             COMM_UART_BAUD);

    uart_config_t uart_cfg = {
        .baud_rate  = COMM_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ret = uart_driver_install(COMM_UART_NUM, COMM_UART_RX_BUF_SIZE,
                              COMM_UART_TX_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(COMM_UART_NUM, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* UART0 uses default pins (USB-JTAG on ESP32-S3), no pin set needed */

    /* --- TWAI (CAN bus) initialisation --- */
    ESP_LOGI(TAG, "Initialising TWAI (CAN) TX=GPIO%d RX=GPIO%d",
             COMM_CAN_TX_GPIO, COMM_CAN_RX_GPIO);

    twai_general_config_t twai_general = TWAI_GENERAL_CONFIG_DEFAULT(
        COMM_CAN_TX_GPIO, COMM_CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_general.rx_queue_len = 16;
    twai_general.tx_queue_len = 8;

    /* 500 kbit/s — standard automotive rate */
    twai_timing_config_t twai_timing = TWAI_TIMING_CONFIG_500KBITS();

    /* Accept all messages (no filtering) */
    twai_filter_config_t twai_filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ret = twai_driver_install(&twai_general, &twai_timing, &twai_filter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Communication interface initialised OK");
    s_rx_state = RX_WAIT_START;
    return ESP_OK;
}

void comm_uart_process(void)
{
    uint8_t buf[64];
    int len = uart_read_bytes(COMM_UART_NUM, buf, sizeof(buf),
                              0);  /* non-blocking: timeout = 0 ticks */
    if (len > 0) {
        for (int i = 0; i < len; i++) {
            rx_feed_byte(buf[i]);
        }
    }
}

void comm_can_process(void)
{
    twai_message_t msg;

    /* Read all pending CAN frames (non-blocking) */
    while (twai_receive(&msg, 0) == ESP_OK) {
        ESP_LOGD(TAG, "CAN RX: ID=0x%03lX DLC=%d",
                 (unsigned long)msg.identifier, msg.data_length_code);

        /* Handle specific CAN IDs as needed */
        if (msg.identifier == CAN_ID_BASE_CMD) {
            /* Base controller sending commands to arm — handle as needed */
            ESP_LOGD(TAG, "CAN: base command received");
        }
    }
}

esp_err_t comm_send_arm_status(const float joint_angles[5],
                               uint8_t gripper_state)
{
    /* Payload: 5 x float32 + 1 x uint8 = 21 bytes */
    uint8_t payload[21];

    memcpy(&payload[0],  &joint_angles[0], sizeof(float));   /* J1 */
    memcpy(&payload[4],  &joint_angles[1], sizeof(float));   /* J2 */
    memcpy(&payload[8],  &joint_angles[2], sizeof(float));   /* J3 */
    memcpy(&payload[12], &joint_angles[3], sizeof(float));   /* J4 */
    memcpy(&payload[16], &joint_angles[4], sizeof(float));   /* J5 */
    payload[20] = gripper_state;

    return uart_send_frame(MSG_ARM_STATUS, payload, sizeof(payload));
}

esp_err_t comm_send_heartbeat(uint8_t status)
{
    /* Payload: timestamp(u32) + status(u8) = 5 bytes */
    uint8_t payload[5];

    uint32_t timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    memcpy(&payload[0], &timestamp_ms, sizeof(uint32_t));
    payload[4] = status;

    return uart_send_frame(MSG_HEARTBEAT, payload, sizeof(payload));
}

esp_err_t comm_can_send_arm_status(const float joint_angles[5],
                                   uint8_t gripper_state)
{
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.identifier = CAN_ID_ARM_STATUS;
    msg.data_length_code = 8;

    /*
     * Pack into 8 bytes:  Each joint angle scaled to [0, 255] from [0, 180].
     *   Bytes 0-4: J1..J5 scaled angles (uint8_t each)
     *   Byte  5:   gripper state
     *   Bytes 6-7: reserved
     */
    for (int i = 0; i < 5; i++) {
        float scaled = (joint_angles[i] / 180.0f) * 255.0f;
        if (scaled < 0.0f)   scaled = 0.0f;
        if (scaled > 255.0f) scaled = 255.0f;
        msg.data[i] = (uint8_t)scaled;
    }
    msg.data[5] = gripper_state;
    msg.data[6] = 0;
    msg.data[7] = 0;

    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CAN TX failed: %s", esp_err_to_name(ret));
    }
    return ret;
}
