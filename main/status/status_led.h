/**
 * @file status_led.h
 * @brief Заголовочный файл модуля управления RGB LED
 * 
 * Определяет API для управления WS2812 RGB LED индикацией состояния системы.
 */

#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Состояния индикации
typedef enum {
    STATUS_LED_OFF = 0,
    STATUS_LED_SCANNING,      // Мигание синим
    STATUS_LED_CONNECTING,     // Мигание желтым
    STATUS_LED_CONNECTED,      // Зеленый
    STATUS_LED_ERROR,          // Красный
    STATUS_LED_READY           // Постоянный синий
} status_led_state_t;

/**
 * @brief Инициализация RGB LED
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t status_led_init(void);

/**
 * @brief Деинициализация RGB LED
 * 
 * Освобождает все выделенные ресурсы (RMT канал, encoder)
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t status_led_deinit(void);

/**
 * @brief Установить состояние индикации
 * 
 * @param state Состояние
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t status_led_set_state(status_led_state_t state);

/**
 * @brief Установить цвет напрямую (RGB)
 * 
 * @param r Красный (0-255)
 * @param g Зеленый (0-255)
 * @param b Синий (0-255)
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t status_led_set_color(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Выключить LED
 */
void status_led_off(void);

/**
 * @brief Обновить индикацию (вызывать периодически для мигания)
 */
void status_led_update(void);

#endif // STATUS_LED_H
