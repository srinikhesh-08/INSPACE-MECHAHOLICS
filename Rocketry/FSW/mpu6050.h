#ifndef MPU6050_H
#define MPU6050_H
#include "main.h"      // CubeMX main.h pulls in the right stm32xxxx_hal.h for your family
#include <cstdint>

struct ImuData {
    float ax_g, ay_g, az_g;             // -> accel_x/y/z  (in g)
    float gx_dps, gy_dps, gz_dps;       // bias-corrected angular rates (deg/s)
    float spin_rate_dps;                // -> gyro_spin_rate (rate about the chosen spin axis)
    float die_temp_c;                   // MPU6050 die temperature (NOT ambient - use BMP for that)
    uint32_t timestamp_ms;              // HAL_GetTick() at read
};

class Mpu6050 {
public:
    enum class SpinAxis : uint8_t { X, Y, Z };

    // addr7: 0x68 when AD0 = GND, 0x69 when AD0 = VCC
    Mpu6050(I2C_HandleTypeDef* hi2c, uint8_t addr7 = 0x68)
        : hi2c_(hi2c), addr_(addr7 << 1) {}

    // Resets the chip and configures: +-16 g, +-2000 dps, DLPF ~44 Hz, 100 Hz output rate.
    bool begin();

    // Keep the rocket perfectly still (on the pad). Averages gyro samples to remove bias.
    bool calibrateGyro(uint16_t samples = 500);

    // Body axis along the rocket's long axis (the one it spins around). Default Z.
    void setSpinAxis(SpinAxis axis) { spin_axis_ = axis; }

    // Reads accel + gyro + temp in one burst. Returns false on I2C error.
    bool read(ImuData& out);

private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t reg, uint8_t* buf, uint16_t len);

    I2C_HandleTypeDef* hi2c_;
    uint8_t  addr_;
    SpinAxis spin_axis_ = SpinAxis::Z;
    float    gyro_bias_dps_[3] = {0.f, 0.f, 0.f};

    // Sensitivities for +-16 g and +-2000 dps
    static constexpr float ACCEL_LSB_PER_G   = 2048.0f;
    static constexpr float GYRO_LSB_PER_DPS  = 16.4f;
};

#endif // MPU6050_H
