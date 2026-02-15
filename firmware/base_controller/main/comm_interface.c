/*
 * comm_interface.c — UART + TWAI (CAN) communication
 *
 * UART framing (RPi <-> Base):
 *   | 0xAA | MSG_ID | LENGTH | PAYLOAD[0..N-1] | CRC8 | 0x55 |
 *
 * A dedicated FreeRTOS task reads incoming bytes from UART, assembles frames
 * via a state machine, validates CRC, and dispatches to a user-provided
 * callback.  Transmit functions build frames and write them atomically.
 *
 * TWAI (CAN bus) is initialised for 500 kbit/s, standard frames only.
 * A receive alert task can be added by the caller if needed.
 */

#include "comm_interface.h"

#include "driver/uart.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "comm";

/* ---------- Module state ---------- */
static comm_rx_callback_t s_rx_callback = NULL;

/* ------------------------------------------------------------------ *
 * CRC-8 (polynomial 0x07, init 0x00, no reflect, no final XOR)
 * ------------------------------------------------------------------ */
uint8_t comm_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ------------------------------------------------------------------ *
 * UART RX task — frame parser state machine
 * ------------------------------------------------------------------ */
typedef enum {
    RX_WAIT_START,
    RX_READ_MSG_ID,
    RX_READ_LENGTH,
    RX_READ_PAYLOAD,
    RX_READ_CRC,
    RX_WAIT_END,
} rx_state_t;

static void uart_rx_task(void *arg)
{
    uint8_t byte;
    rx_state_t state = RX_WAIT_START;
    comm_msg_t msg;
    uint8_t payload_idx = 0;
    uint8_t rx_crc = 0;

    ESP_LOGI(TAG, "UART RX task started");

    while (1) {
        /* Block until at least one byte is available */
        int len = uart_read_bytes(COMM_UART_PORT, &byte, 1, portMAX_DELAY);
        if (len <= 0) {
            continue;
        }

        switch (state) {
        case RX_WAIT_START:
            if (byte == FRAME_START) {
                state = RX_READ_MSG_ID;
                memset(&msg, 0, sizeof(msg));
                payload_idx = 0;
            }
            break;

        case RX_READ_MSG_ID:
            msg.msg_id = byte;
            state = RX_READ_LENGTH;
            break;

        case RX_READ_LENGTH:
            msg.length = byte;
            if (msg.length > MAX_PAYLOAD_SIZE) {
                /* Invalid length — reset */
                ESP_LOGW(TAG, "RX frame length %u exceeds max, dropping",
                         msg.length);
                state = RX_WAIT_START;
            } else if (msg.length == 0) {
                state = RX_READ_CRC;
            } else {
                state = RX_READ_PAYLOAD;
            }
            break;

        case RX_READ_PAYLOAD:
            msg.payload[payload_idx++] = byte;
            if (payload_idx >= msg.length) {
                state = RX_READ_CRC;
            }
            break;

        case RX_READ_CRC:
            rx_crc = byte;
            state = RX_WAIT_END;
            break;

        case RX_WAIT_END:
            if (byte == FRAME_END) {
                /* Verify CRC over MSG_ID + LENGTH + PAYLOAD */
                uint8_t crc_buf[2 + MAX_PAYLOAD_SIZE];
                crc_buf[0] = msg.msg_id;
                crc_buf[1] = msg.length;
                memcpy(&crc_buf[2], msg.payload, msg.length);
                uint8_t calc_crc = comm_crc8(crc_buf, 2 + msg.length);

                if (calc_crc == rx_crc) {
                    if (s_rx_callback) {
                        s_rx_callback(&msg);
                    }
                } else {
                    ESP_LOGW(TAG, "CRC mismatch: got 0x%02X, expected 0x%02X",
                             rx_crc, calc_crc);
                }
            } else {
                ESP_LOGW(TAG, "Missing FRAME_END (got 0x%02X)", byte);
            }
            state = RX_WAIT_START;
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
esp_err_t comm_uart_init(comm_rx_callback_t cb)
{
    ESP_LOGI(TAG, "Initialising UART%d at %d baud", COMM_UART_PORT,
             COMM_UART_BAUD);

    s_rx_callback = cb;

    uart_config_t uart_cfg = {
        .baud_rate  = COMM_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(COMM_UART_PORT, &uart_cfg));

    ESP_ERROR_CHECK(uart_set_pin(COMM_UART_PORT,
                                  COMM_UART_TX_GPIO, COMM_UART_RX_GPIO,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(COMM_UART_PORT,
                                         COMM_UART_BUF_SIZE * 2,
                                         COMM_UART_BUF_SIZE * 2,
                                         0, NULL, 0));

    /* Spawn the RX parser task on core 0, priority 10 */
    BaseType_t ret = xTaskCreatePinnedToCore(
        uart_rx_task, "uart_rx", 4096, NULL, 10, NULL, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART RX task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
esp_err_t comm_uart_send(uint8_t msg_id, const void *payload, uint8_t length)
{
    if (length > MAX_PAYLOAD_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    /* Build the complete frame in a local buffer */
    uint8_t frame[4 + MAX_PAYLOAD_SIZE + 2];  /* START + ID + LEN + payload + CRC + END */
    size_t pos = 0;

    frame[pos++] = FRAME_START;
    frame[pos++] = msg_id;
    frame[pos++] = length;

    if (length > 0 && payload != NULL) {
        memcpy(&frame[pos], payload, length);
        pos += length;
    }

    /* CRC over MSG_ID + LENGTH + PAYLOAD */
    uint8_t crc_buf[2 + MAX_PAYLOAD_SIZE];
    crc_buf[0] = msg_id;
    crc_buf[1] = length;
    if (length > 0 && payload != NULL) {
        memcpy(&crc_buf[2], payload, length);
    }
    frame[pos++] = comm_crc8(crc_buf, 2 + length);

    frame[pos++] = FRAME_END;

    /* Atomic write */
    int written = uart_write_bytes(COMM_UART_PORT, frame, pos);
    if (written < 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
esp_err_t comm_can_init(void)
{
    ESP_LOGI(TAG, "Initialising TWAI (CAN bus) at 500 kbit/s");

    twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    g_cfg.rx_queue_len = 16;
    g_cfg.tx_queue_len = 8;

    twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_cfg, &t_cfg, &f_cfg));
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG, "TWAI started");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
esp_err_t comm_can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (len > 8) {
        return ESP_ERR_INVALID_SIZE;
    }

    twai_message_t msg = {
        .identifier = id,
        .data_length_code = len,
        .flags = 0,   /* standard frame, not RTR, not self-reception */
    };
    if (data && len > 0) {
        memcpy(msg.data, data, len);
    }

    return twai_transmit(&msg, pdMS_TO_TICKS(50));
}

/* ------------------------------------------------------------------ */
esp_err_t comm_send_odometry(float x, float y, float theta,
                             float vx, float vy)
{
    msg_odometry_t payload = {
        .x     = x,
        .y     = y,
        .theta = theta,
        .vx    = vx,
        .vy    = vy,
    };
    return comm_uart_send(MSG_ID_ODOMETRY, &payload, sizeof(payload));
}

/* ------------------------------------------------------------------ */
esp_err_t comm_send_heartbeat(uint32_t timestamp_ms, uint8_t status)
{
    msg_heartbeat_t payload = {
        .timestamp_ms = timestamp_ms,
        .status       = status,
    };
    return comm_uart_send(MSG_ID_HEARTBEAT, &payload, sizeof(payload));
}

/* ------------------------------------------------------------------ */
esp_err_t comm_send_sensor_data(float imu_yaw, const int32_t enc_counts[4])
{
    msg_sensor_data_t payload;
    payload.imu_yaw = imu_yaw;
    memcpy(payload.enc_counts, enc_counts, sizeof(payload.enc_counts));
    return comm_uart_send(MSG_ID_SENSOR_DATA, &payload, sizeof(payload));
}
