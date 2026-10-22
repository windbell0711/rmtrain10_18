
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == KEY_Pin) {
        
    }
}

#ifdef __cplusplus
}
#endif