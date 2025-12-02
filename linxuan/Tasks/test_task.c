#include "test_task.h"
#include "logger.h"
#include "cmsis_os.h"
#include "stm32l475e_iot01.h"

extern osMutexId_t sensor_buffer_mutexHandle;

void StartTestTask(void *argument) {
    LOG_INFO("starting test task");

    while(1) {
        BSP_LED_Toggle(LED1);
        BSP_LED_Toggle(LED2);
        osDelay(500);
    }
}