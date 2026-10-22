
#include "callback.h"
#include "accel_gyro.h"
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

IMU imu = IMU();

void MainInit() {
    if (bmi088_init() == 0) {
        Error_Handler();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM6)
        return;
    imu.update();
}

#ifdef __cplusplus
}
#endif