/*
 * imu_driver.c — MPU6050 I2C driver with complementary-filter orientation
 *
 * The MPU6050 is configured for:
 *   - Gyroscope:     +/- 500 deg/s  (sensitivity 65.5 LSB/deg/s)
 *   - Accelerometer: +/- 2 g        (sensitivity 16384 LSB/g)
 *   - DLPF:          bandwidth 42 Hz (register 0x1A = 3)
 *   - Sample rate:   1 kHz  (SMPLRT_DIV = 0)
 *
 * A complementary filter fuses accelerometer-derived roll/pitch with gyro
 * integration.  Yaw is integrated from gyro_z only (no magnetometer).
 */

#include "imu_driver.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

#include <math.h>
#include <string.h>

static const char *TAG = "imu";

/* ---------- MPU6050 register addresses ---------- */
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

/* Scale factors */
#define ACCEL_SCALE         (9.80665f / 16384.0f)           /* LSB -> m/s^2 */
#define GYRO_SCALE          ((500.0f / 32768.0f) * (M_PI / 180.0f))  /* LSB -> rad/s */

/* Complementary filter constant for roll/pitch (gyro weight) */
#define COMP_ALPHA          0.98f

/* ---------- Driver state ---------- */
static i2c_master_bus_handle_t  s_bus_handle;
static i2c_master_dev_handle_t  s_dev_handle;
static imu_data_t               s_data;
static bool                     s_initialised = false;

/* ------------------------------------------------------------------ *
 * Low-level I2C helpers
 * ------------------------------------------------------------------ */
static esp_err_t mpu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(s_dev_handle, buf, sizeof(buf), 100);
}

static esp_err_t mpu_read_regs(uint8_t start_reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev_handle, &start_reg, 1,
                                       data, len, 100);
}

/* ------------------------------------------------------------------ */
esp_err_t imu_init(void)
{
    ESP_LOGI(TAG, "Initialising I2C bus and MPU6050");

    /* --- I2C bus configuration --- */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port   = IMU_I2C_PORT,
        .sda_io_num = IMU_I2C_SDA_GPIO,
        .scl_io_num = IMU_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus_handle));

    /* --- Add MPU6050 device on the bus --- */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_ADDR,
        .scl_speed_hz    = IMU_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus_handle, &dev_cfg,
                                               &s_dev_handle));

    /* --- Verify WHO_AM_I --- */
    uint8_t who_am_i = 0;
    esp_err_t err = mpu_read_regs(REG_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return err;
    }
    if (who_am_i != 0x68) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I: 0x%02X (expected 0x68)", who_am_i);
        /* Continue anyway — some clones report different values */
    }

    /* --- Wake up the MPU6050 (clear sleep bit, use PLL with gyro-X ref) --- */
    ESP_ERROR_CHECK(mpu_write_reg(REG_PWR_MGMT_1, 0x01));
    vTaskDelay(pdMS_TO_TICKS(100));     /* wait for PLL to stabilise */

    /* --- Configure sample rate: 1 kHz / (1 + 0) = 1 kHz --- */
    ESP_ERROR_CHECK(mpu_write_reg(REG_SMPLRT_DIV, 0x00));

    /* --- DLPF: bandwidth 42 Hz (register value 3) --- */
    ESP_ERROR_CHECK(mpu_write_reg(REG_CONFIG, 0x03));

    /* --- Gyroscope: +/- 500 deg/s (FS_SEL = 1 → bits 4:3 = 01) --- */
    ESP_ERROR_CHECK(mpu_write_reg(REG_GYRO_CONFIG, 0x08));

    /* --- Accelerometer: +/- 2g (AFS_SEL = 0 → bits 4:3 = 00) --- */
    ESP_ERROR_CHECK(mpu_write_reg(REG_ACCEL_CONFIG, 0x00));

    memset(&s_data, 0, sizeof(s_data));
    s_initialised = true;

    ESP_LOGI(TAG, "MPU6050 initialised (gyro +-500 dps, accel +-2g, DLPF 42 Hz)");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
esp_err_t imu_update(float dt)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Read 14 bytes: ACCEL (6) + TEMP (2) + GYRO (6) */
    uint8_t raw[14];
    esp_err_t err = mpu_read_regs(REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    /* Parse raw big-endian 16-bit signed values */
    int16_t ax_raw = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4]  << 8) | raw[5]);
    /* raw[6..7] = temperature — ignored */
    int16_t gx_raw = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

    /* Convert to physical units */
    s_data.accel_x = ax_raw * ACCEL_SCALE;
    s_data.accel_y = ay_raw * ACCEL_SCALE;
    s_data.accel_z = az_raw * ACCEL_SCALE;
    s_data.gyro_x  = gx_raw * GYRO_SCALE;
    s_data.gyro_y  = gy_raw * GYRO_SCALE;
    s_data.gyro_z  = gz_raw * GYRO_SCALE;

    /* --- Complementary filter for roll and pitch --- */

    /* Accelerometer-based angles (valid when robot is not accelerating) */
    float accel_roll  = atan2f(s_data.accel_y, s_data.accel_z);
    float accel_pitch = atan2f(-s_data.accel_x,
                               sqrtf(s_data.accel_y * s_data.accel_y +
                                     s_data.accel_z * s_data.accel_z));

    /* Fuse with gyroscope integration */
    s_data.roll  = COMP_ALPHA * (s_data.roll  + s_data.gyro_x * dt)
                 + (1.0f - COMP_ALPHA) * accel_roll;
    s_data.pitch = COMP_ALPHA * (s_data.pitch + s_data.gyro_y * dt)
                 + (1.0f - COMP_ALPHA) * accel_pitch;

    /* Yaw: pure gyroscope integration (no magnetometer on MPU6050) */
    s_data.yaw += s_data.gyro_z * dt;

    /* Wrap yaw to [-pi, +pi] */
    while (s_data.yaw >  (float)M_PI) s_data.yaw -= 2.0f * (float)M_PI;
    while (s_data.yaw < -(float)M_PI) s_data.yaw += 2.0f * (float)M_PI;

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
void imu_get_data(imu_data_t *out)
{
    if (out) {
        *out = s_data;
    }
}

/* ------------------------------------------------------------------ */
float imu_get_yaw(void)
{
    return s_data.yaw;
}

/* ------------------------------------------------------------------ */
void imu_reset_yaw(void)
{
    s_data.yaw = 0.0f;
}
