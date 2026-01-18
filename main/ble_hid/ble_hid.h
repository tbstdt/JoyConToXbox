/**
 * @file ble_hid.h
 * @brief Заголовочный файл модуля BLE HID устройства
 * 
 * Определяет API для работы BLE HID устройства, эмулирующего Xbox контроллер.
 */

#ifndef BLE_HID_H
#define BLE_HID_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Размер HID отчета Xbox контроллера
#define XBOX_HID_REPORT_SIZE 20

// Минимальный интервал между отправками HID отчетов (мс)
// Ограничивает частоту отправки до 100 Гц для предотвращения перегрузки BLE канала
#define MIN_REPORT_INTERVAL_MS 10  // 10 мс = 100 Гц максимум

// Callback для получения команд вибрации
typedef void (*ble_hid_vibration_callback_t)(uint8_t left_motor, uint8_t right_motor);

/**
 * @brief Инициализация BLE HID устройства
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t ble_hid_init(void);

/**
 * @brief Отправка HID отчета контроллера
 * 
 * @param report Указатель на буфер с отчетом
 * @param len Размер буфера в байтах (должен быть >= XBOX_HID_REPORT_SIZE)
 * @return esp_err_t ESP_OK при успехе, ESP_ERR_INVALID_SIZE если len < XBOX_HID_REPORT_SIZE
 */
esp_err_t ble_hid_send_report(const uint8_t *report, size_t len);

/**
 * @brief Установка callback для получения команд вибрации
 * 
 * @param callback Функция обратного вызова
 */
void ble_hid_set_vibration_callback(ble_hid_vibration_callback_t callback);

/**
 * @brief Проверка готовности BLE HID устройства
 * 
 * @return true если устройство готово
 */
bool ble_hid_is_ready(void);

/**
 * @brief Запуск рекламы BLE HID устройства
 * 
 * Должна вызываться после синхронизации BLE стека
 */
void ble_hid_start_advertising(void);

/**
 * @brief Обработчик BLE GAP событий для HID (заглушка для совместимости)
 * 
 * GAP события обрабатываются через GATTS callback
 */
void ble_hid_gap_event_handler(void *event);

#endif // BLE_HID_H
