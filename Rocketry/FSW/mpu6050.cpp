#include "mpu6050.h"

// Register map
static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
static constexpr uint8_t REG_CONFIG       = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;   // 14 bytes: accel(6) temp(2) gyro(6)
static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t REG_WHO_AM_I     = 0x75;

static constexpr uint32_t I2C_TIMEOUT_MS = 10;

// ------------------------------------------------------------------------------
bool Mpu6050::writeReg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c_, addr_, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

bool Mpu6050::readRegs(uint8_t reg, uint8_t* buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c_, addr_, reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT_MS) == HAL_OK;
}

// ------------------------------------------------------------------------------
bool Mpu6050::begin()
{
    uint8_t who = 0;
    if (!readRegs(REG_WHO_AM_I, &who, 1)) return false;
    if (who != 0x68) return false;          // note: some clone boards report 0x70/0x72 - relax this if so

    if (!writeReg(REG_PWR_MGMT_1, 0x80)) return false;   // device reset
    HAL_Delay(100);
    if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false;   // wake, clock = gyro X PLL
    HAL_Delay(10);

    if (!writeReg(REG_CONFIG,       0x03)) return false; // DLPF ~44 Hz accel / 42 Hz gyro, 1 kHz internal rate
    if (!writeReg(REG_SMPLRT_DIV,   9))    return false; // 1000 / (1 + 9) = 100 Hz
    if (!writeReg(REG_GYRO_CONFIG,  0x18)) return false; // +-2000 dps
    if (!writeReg(REG_ACCEL_CONFIG, 0x18)) return false; // +-16 g
    return true;
}

// ------------------------------------------------------------------------------
bool Mpu6050::calibrateGyro(uint16_t samples)
{
    float sum[3] = {0.f, 0.f, 0.f};
    gyro_bias_dps_[0] = gyro_bias_dps_[1] = gyro_bias_dps_[2] = 0.f;

    uint16_t good = 0;
    for (uint16_t i = 0; i < samples; i++) {
        ImuData d;
        if (read(d)) {
            sum[0] += d.gx_dps;
            sum[1] += d.gy_dps;
            sum[2] += d.gz_dps;
            good++;
        }
        HAL_Delay(5);       // matches the 100 Hz output rate roughly
    }
    if (good < samples / 2) return false;

    for (int i = 0; i < 3; i++) gyro_bias_dps_[i] = sum[i] / good;
    return true;
}

// ------------------------------------------------------------------------------
bool Mpu6050::read(ImuData& out)
{
    uint8_t b[14];
    if (!readRegs(REG_ACCEL_XOUT_H, b, 14)) return false;

    auto s16 = [&](int i) -> int16_t { return (int16_t)((b[i] << 8) | b[i + 1]); };

    out.ax_g = s16(0) / ACCEL_LSB_PER_G;
    out.ay_g = s16(2) / ACCEL_LSB_PER_G;
    out.az_g = s16(4) / ACCEL_LSB_PER_G;

    out.die_temp_c = s16(6) / 340.0f + 36.53f;

    out.gx_dps = s16(8)  / GYRO_LSB_PER_DPS - gyro_bias_dps_[0];
    out.gy_dps = s16(10) / GYRO_LSB_PER_DPS - gyro_bias_dps_[1];
    out.gz_dps = s16(12) / GYRO_LSB_PER_DPS - gyro_bias_dps_[2];

    switch (spin_axis_) {
        case SpinAxis::X: out.spin_rate_dps = out.gx_dps; break;
        case SpinAxis::Y: out.spin_rate_dps = out.gy_dps; break;
        default:          out.spin_rate_dps = out.gz_dps; break;
    }

    out.timestamp_ms = HAL_GetTick();
    return true;
}
