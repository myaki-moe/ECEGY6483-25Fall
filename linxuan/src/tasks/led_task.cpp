#include "tasks/led_task.hpp"
#include "mbed.h"
#include "bsp/led.hpp"
#include "logger.hpp"
#include "tasks/analysis_task.hpp"

void led_task() {
    LOG_INFO("LED Task Started");

    float led_value = 0.0f;
    bool led_direction = true;

    while (true) {
        if (get_fog_status()) {
            led_blue_yellow_on();
        } else if (get_dyskinesia_status()) {
            led_yellow_on();
        } else if (get_tremor_status()) {
            led_blue_on();
        } else {
            led_blue_yellow_off();
        }
        
        if (led_direction) {
            led_value += 0.025f;
            if (led_value > 1.0f) {
                led_direction = false;
            }
        } else {
            led_value -= 0.025f;
            if (led_value < 0.0f) {
                led_direction = true;
            }
        }
        led_green_1_set(led_value);
        led_green_2_set(led_value);
        ThisThread::sleep_for(50ms);

    }
}