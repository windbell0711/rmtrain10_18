
#ifndef RM10_18_IMU_H
#define RM10_18_IMU_H

#include "main.h"
#include "tim.h"
#include "accel_gyro.h"
#include "pid.hpp"
#include "quat.h"
#include "vector.h"

class IMU {
public:
    IMU() = default;
    void temperature_handle();
    void acc_calculate();
    void gyro_calculate();
    void sensorDataTransform();
    void sensorRecord();
    void quat_update();
    void update();

    Quat quat_ = Quat();
    float roll_=0, pitch_=0, yaw_=0;

private:
    PID tpid = PID(15.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.02f);
    PID qpid_x = PID(0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f);
    PID qpid_y = PID(0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f);
    PID qpid_z = PID(0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f);

    const float R_imu_[3][3] = {{-1.0f, 0.0f, 0.0f},
                             {0.0f, 1.0f, 0.0f},
                             {0.0f, 0.0f, -1.0f}};

    struct RawData_t {
        float gyro[3];
        float accel[3];
    } raw_data_;

    float temperature = 30;
    float temperature_target = 35;

    float gyro_sensor_sum_[3]  = { 0.0f, 0.0f, 0.0f };
    float accel_sensor_sum_[3] = { 0.0f, 0.0f, 0.0f };
    int16_t sensor_record_cnt_ = 0;
    constexpr static int16_t sensor_record_cnt_max_ = 2000;
    int16_t sensor_record_start_cd_ = 3000;

    float gyro_sensor_[3]  = { 0.0f, 0.0f, 0.0f };
    float accel_sensor_[3] = { 0.0f, 0.0f, 0.0f };

    float filter_k_ = 0.05f;

    float gyro_bias_[3]  = { 0.0f, 0.0f, 0.0f };
    float accel_bias_[3] = { 0.0f, 0.0f, 0.0f };

    float accel_sen_ = 0.0098f;
    float gyro_sen_  = 0.01745329252f;  // pi/180
};


#endif //RM10_18_IMU_H
