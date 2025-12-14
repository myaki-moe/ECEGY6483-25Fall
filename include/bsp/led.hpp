#pragma once

/**
 * @file led.hpp
 * @brief Board support for the on-board LEDs.
 *
 * This module abstracts simple LED behaviors (green PWM LEDs and a shared
 * blue/yellow LED pin).
 */

/**
 * @brief Initialize LED peripherals.
 * @return true on success, false on failure.
 */
bool led_init();

/**
 * @brief Set green LED 1 brightness (PWM duty cycle).
 * @param value Duty cycle in range [0.0, 1.0].
 */
void led_green_1_set(float value);

/**
 * @brief Set green LED 2 brightness (PWM duty cycle).
 * @param value Duty cycle in range [0.0, 1.0].
 */
void led_green_2_set(float value);

/**
 * @brief Turn the shared LED to blue state (board-specific).
 */
void led_blue_on();

/**
 * @brief Turn the shared LED to yellow state (board-specific).
 */
void led_yellow_on();

/**
 * @brief Turn the shared LED to a mixed blue/yellow state (board-specific).
 */
void led_blue_yellow_on();

/**
 * @brief Turn the shared blue/yellow LED off.
 */
void led_blue_yellow_off();
