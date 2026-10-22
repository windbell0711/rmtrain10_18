
#ifndef RM10_18_IMU_H
#define RM10_18_IMU_H

#include "accel_gyro.h"


class IMU {
public:
    IMU() = default;
    void acc_calculate();
    void gyro_calculate();
    void sensorDataTransform();
    void update();

private:
    const float R_imu_[3][3] = {{-1.0f, 0.0f, 0.0f},
                             {0.0f, 1.0f, 0.0f},
                             {0.0f, 0.0f, -1.0f}};

    struct RawData_t {
        float gyro[3];
        float accel[3];
    } raw_data_;

    float gyro_sensor_[3]  = { 0.0f, 0.0f, 0.0f };
    float accel_sensor_[3] = { 0.0f, 0.0f, 0.0f };

    float gyro_bias_[3]    = { 0.0f, 0.0f, 0.0f };
    float accel_bias_[3]   = { 0.0f, 0.0f, 0.0f };

    float accel_sen_ = 1.0f;
};


#endif //RM10_18_IMU_H
