#include "bsp/led.hpp"
#include "mbed.h"

PwmOut *led_green_1_out = nullptr;
PwmOut *led_green_2_out = nullptr;
PwmOut *led_blue_yellow_out = nullptr;
DigitalIn *led_blue_yellow_in = nullptr;

bool led_init() {
    led_green_1_out = new PwmOut(LED1);
    led_green_1_out->period_us(100);
    led_green_2_out = new PwmOut(LED2);
    led_green_2_out->period_us(100);
    led_blue_yellow_in = new DigitalIn(LED3, PullNone);
    return true;
}

void led_green_1_set(float value) {
    led_green_1_out->write(value);
}

void led_green_2_set(float value) {
    led_green_2_out->write(value);
}

void led_blue_on() {
    if (led_blue_yellow_in != nullptr) {
        delete led_blue_yellow_in;
        led_blue_yellow_in = nullptr;
    }
    if (led_blue_yellow_out == nullptr) {
        led_blue_yellow_out = new PwmOut(LED3);
        led_blue_yellow_out->period_us(100);
    }
    led_blue_yellow_out->write(0);
}

void led_yellow_on() {
    if (led_blue_yellow_in != nullptr) {
        delete led_blue_yellow_in;
        led_blue_yellow_in = nullptr;
    }
    if (led_blue_yellow_out == nullptr) {
        led_blue_yellow_out = new PwmOut(LED3);
        led_blue_yellow_out->period_us(100);
    }
    led_blue_yellow_out->write(1);
}

void led_blue_yellow_on() {
    if (led_blue_yellow_in != nullptr) {
        delete led_blue_yellow_in;
        led_blue_yellow_in = nullptr;
    }
    if (led_blue_yellow_out == nullptr) {
        led_blue_yellow_out = new PwmOut(LED3);
        led_blue_yellow_out->period_us(100);
    }
    led_blue_yellow_out->write(0.5);
}

void led_blue_yellow_off() {
    if (led_blue_yellow_in != nullptr) {return;}
    if (led_blue_yellow_out != nullptr) {
        delete led_blue_yellow_out;
        led_blue_yellow_out = nullptr;
        led_blue_yellow_in = new DigitalIn(LED3, PullNone);
    }
}