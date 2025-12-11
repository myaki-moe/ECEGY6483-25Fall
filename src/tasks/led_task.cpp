#include "tasks/led_task.hpp"
#include "mbed.h"
#include "bsp/led.hpp"
#include "logger.hpp"
#include "tasks/analysis_task.hpp"
#include "tasks/ble_task.hpp"

void led_task() {
    LOG_INFO("LED Task Started");

    float led_value = 0.0f;
    bool led_direction = true;

    while (true) {
        if (get_fog_status()) {
            for (int i = 0; i < 5; i++) {
                led_blue_yellow_on();
                ThisThread::sleep_for(500ms);
                led_blue_yellow_off();
                ThisThread::sleep_for(500ms);
            }
        } else if (get_dyskinesia_status()) {
            led_yellow_on();
        } else if (get_tremor_status()) {
            led_blue_on();
        } else {
            led_blue_yellow_off();
        }
        

        if (ble_is_connected()) {
            led_green_2_set(1);
        } else {
            led_green_2_set(0);
        }

        if (led_direction) {
            led_value += 0.025f;
            if (led_value > 0.75f) {
                led_direction = false;
            }
        } else {
            led_value -= 0.025f;
            if (led_value < 0.0f) {
                led_direction = true;
            }
        }
        led_green_1_set(led_value);
        ThisThread::sleep_for(50ms);

    }
}