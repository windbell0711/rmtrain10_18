
#include "imu.h"

void IMU::acc_calculate() {
    // 1. 设置/读取acc0x41寄存器中的量程range参数，并换算为量程系数
    uint8_t raw_range;
    bmi088_accel_read_reg(0x41, &raw_range, 1);
    uint16_t acc_range;
    switch (raw_range & 0b11) {
        case 0:   acc_range = 3000;   break;
        case 1:   acc_range = 6000;   break;
        case 2:   acc_range = 12000;  break;
        case 3:   acc_range = 24000;  break;
        default:  acc_range = 0;      break;
    }

    // 2. 读取acc0x12寄存器中的6位acc数据
    uint8_t raw_acc_data[6];
    bmi088_accel_read_reg(0x12, raw_acc_data, 6);
    auto raw_acc_x = static_cast<int16_t>((raw_acc_data[1] << 8) | raw_acc_data[0]);
    auto raw_acc_y = static_cast<int16_t>((raw_acc_data[3] << 8) | raw_acc_data[2]);
    auto raw_acc_z = static_cast<int16_t>((raw_acc_data[5] << 8) | raw_acc_data[4]);

    // 3. 用量程系数将原始数据转换为常用单位
    // LSB -> acc (m/s2)
    raw_data_.accel[0] = raw_acc_x / 32768.0f * acc_range / 100.0f;
    raw_data_.accel[1] = raw_acc_y / 32768.0f * acc_range / 100.0f;
    raw_data_.accel[2] = raw_acc_z / 32768.0f * acc_range / 100.0f;
}

void IMU::gyro_calculate(){
    // 1. 设置/读取gyro0x0F寄存器中的量程range参数，并换算为量程系数
    uint8_t raw_range;
    bmi088_gyro_read_reg(0x0F, &raw_range, 1);
    uint16_t gyro_range;
    switch (raw_range) {
        case 0:   gyro_range = 2000;  break;
        case 1:   gyro_range = 1000;  break;
        case 2:   gyro_range = 500;   break;
        case 3:   gyro_range = 250;   break;
        case 4:   gyro_range = 125;   break;
        default:  gyro_range = 0;     break;
    }

    // 2. 读取gyro0x02寄存器中的6位gyro数据
    uint8_t raw_gyro_data[6];
    bmi088_gyro_read_reg(0x02, raw_gyro_data, 6);
    auto raw_gyro_x = static_cast<int16_t>((raw_gyro_data[1] << 8) | raw_gyro_data[0]);
    auto raw_gyro_y = static_cast<int16_t>((raw_gyro_data[3] << 8) | raw_gyro_data[2]);
    auto raw_gyro_z = static_cast<int16_t>((raw_gyro_data[5] << 8) | raw_gyro_data[4]);

    // 3. 用量程系数将原始数据转换为常用单位
    // LSB -> angular vel (degree per sec)
    raw_data_.gyro[0] = raw_gyro_x / 32768.0f * gyro_range;
    raw_data_.gyro[1] = raw_gyro_y / 32768.0f * gyro_range;
    raw_data_.gyro[2] = raw_gyro_z / 32768.0f * gyro_range;
}

void IMU::sensorDataTransform() {
    for (uint8_t i = 0; i < 3; i++) {
        gyro_sensor_[i] = (
            raw_data_.gyro[0] * R_imu_[i][0] +
            raw_data_.gyro[1] * R_imu_[i][1] +
            raw_data_.gyro[2] * R_imu_[i][2]
        ) - gyro_bias_[i];
        accel_sensor_[i] = accel_sen_ * (
            raw_data_.accel[0] * R_imu_[i][0] +
            raw_data_.accel[1] * R_imu_[i][1] +
            raw_data_.accel[2] * R_imu_[i][2]
        ) - accel_bias_[i];
    }
}

void IMU::update() {
    acc_calculate();
    gyro_calculate();
    sensorDataTransform();
}
