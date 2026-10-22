
#ifndef RM10_18_IMU_H
#define RM10_18_IMU_H

#include "main.h"
#include "tim.h"
#include "accel_gyro.h"
#include "pid.hpp"

class IMU {
public:
    IMU() = default;
    void temperature_handle();
    void acc_calculate();
    void gyro_calculate();
    void sensorDataTransform();
    void update();

private:
    PID tpid = PID(15.0f, 0.0f, 1.0f, 0.0f, 99.0f, 0.02f);

    const float R_imu_[3][3] = {{-1.0f, 0.0f, 0.0f},
                             {0.0f, 1.0f, 0.0f},
                             {0.0f, 0.0f, -1.0f}};

    struct RawData_t {
        float gyro[3];
        float accel[3];
    } raw_data_;

    float temperature = 30;
    float temperature_target = 35;

    float gyro_sensor_[3]  = { 0.0f, 0.0f, 0.0f };
    float accel_sensor_[3] = { 0.0f, 0.0f, 0.0f };

    float gyro_bias_[3]    = { -0.08f,  0.23f,  0.05f };
    float accel_bias_[3]   = {  0.20f, -0.12f, -0.18f };

    float accel_sen_ = 1.0f;
};


#endif //RM10_18_IMU_H
