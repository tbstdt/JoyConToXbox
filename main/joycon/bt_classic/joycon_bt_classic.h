/**
 * @file joycon_bt_classic.h
 * @brief Заголовочный файл модуля Bluetooth Classic подключения к Joy-Con
 * 
 * Определяет API для работы с Joy-Con контроллерами через Bluetooth Classic:
 * - Инициализация и управление Bluedroid подключением
 * - GAP inquiry для сканирования устройств
 * - SDP для проверки HID сервиса
 * - Получение данных и отправка команд через Classic HID протокол
 * - Управление состоянием подключения
 */

#ifndef JOYCON_BT_CLASSIC_H
#define JOYCON_BT_CLASSIC_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "../joycon_constants.h"
#include "../joycon_types.h"

// Максимальный размер HID отчета от Joy-Con
// Joy-Con отправляет отчеты размером 50 байт (49 байт данных + 1 байт заголовка)
#define JOYCON_HID_REPORT_SIZE 50

// Callback для получения HID данных
typedef void (*joycon_bt_classic_data_callback_t)(joycon_type_t type, const uint8_t *data, size_t len);

// Callback для изменения состояния
typedef void (*joycon_bt_classic_state_callback_t)(joycon_type_t type, joycon_state_t state);

/**
 * @brief Инициализация Bluedroid стека
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_init(void);

/**
 * @brief Деинициализация Bluedroid стека и освобождение ресурсов
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_deinit(void);

/**
 * @brief Начать GAP inquiry для сканирования Joy-Con устройств
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_start_scan(void);

/**
 * @brief Подключиться к Joy-Con через Bluetooth Classic
 * 
 * @param type Тип Joy-Con (левый или правый)
 * @param addr MAC адрес устройства (NULL для автоматического поиска)
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_connect(joycon_type_t type, uint8_t *addr);

/**
 * @brief Отключиться от Joy-Con
 * 
 * @param type Тип Joy-Con
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_disconnect(joycon_type_t type);

/**
 * @brief Отправить команду вибрации Joy-Con
 * 
 * @param type Тип Joy-Con
 * @param data Данные вибрации
 * @param len Длина данных
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_send_vibration(joycon_type_t type, const uint8_t *data, size_t len);

/**
 * @brief Получить состояние подключения
 * 
 * @param type Тип Joy-Con
 * @return joycon_state_t Текущее состояние
 */
joycon_state_t joycon_bt_classic_get_state(joycon_type_t type);

/**
 * @brief Установить callback для получения данных
 * 
 * @param callback Функция обратного вызова
 */
void joycon_bt_classic_set_data_callback(joycon_bt_classic_data_callback_t callback);

/**
 * @brief Установить callback для изменения состояния
 * 
 * @param callback Функция обратного вызова
 */
void joycon_bt_classic_set_state_callback(joycon_bt_classic_state_callback_t callback);

/**
 * @brief Получить MAC адрес подключенного Joy-Con
 * 
 * @param type Тип Joy-Con
 * @param addr Буфер для MAC адреса (6 байт)
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_get_address(joycon_type_t type, uint8_t *addr);

/**
 * @brief Очистить сохраненные MAC адреса Joy-Con
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_clear_saved_addresses(void);

/**
 * @brief Ожидать инициализации Bluedroid стека
 * 
 * @param timeout_ms Таймаут ожидания в миллисекундах (0 = ждать бесконечно)
 * @return esp_err_t ESP_OK при успехе, ESP_ERR_TIMEOUT при таймауте
 */
esp_err_t joycon_bt_classic_wait_for_init(uint32_t timeout_ms);

#endif // JOYCON_BT_CLASSIC_H
