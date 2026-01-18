/**
 * @file joycon_bt_classic_connection.h
 * @brief Модуль управления подключениями Joy-Con через Bluetooth Classic
 * 
 * Ответственность модуля:
 * - Подключение и отключение от Joy-Con через Classic HID
 * - Обработка ACL соединений
 * - Управление состоянием подключений
 * - Проверка условий для подключения
 */

#ifndef JOYCON_BT_CLASSIC_CONNECTION_H
#define JOYCON_BT_CLASSIC_CONNECTION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "joycon_bt_classic_internal.h"

// discovered_bt_classic_device_t определен в joycon_bt_classic_internal.h

/**
 * @brief Подключиться к Joy-Con через Bluetooth Classic
 * 
 * @param type Тип Joy-Con (LEFT или RIGHT)
 * @param addr MAC-адрес устройства (6 байт, может быть NULL для использования сохраненного адреса)
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_connect(joycon_type_t type, uint8_t *addr);

/**
 * @brief Обработка события подключения ACL
 * 
 * @param bda MAC-адрес устройства
 */
void joycon_bt_classic_handle_acl_connected(uint8_t *bda);

/**
 * @brief Обработка события отключения ACL
 * 
 * @param bda MAC-адрес устройства
 */
void joycon_bt_classic_handle_acl_disconnected(uint8_t *bda);

/**
 * @brief Проверка таймаутов подключения (периодическая проверка)
 */
void joycon_bt_classic_check_connection_timeouts(void);

/**
 * @brief Проверка условий для попытки подключения к Joy-Con
 * 
 * Определяет, является ли обнаруженное устройство Joy-Con и нужно ли к нему подключаться
 * 
 * @param cached_device Обнаруженное устройство из кэша discovery
 * @param is_left Указатель на флаг определения как левый Joy-Con
 * @param is_right Указатель на флаг определения как правый Joy-Con
 * @return true если нужно попытаться подключиться
 */
bool joycon_bt_classic_connection_should_attempt(const discovered_bt_classic_device_t *cached_device,
                                                  bool is_left, bool is_right);

/**
 * @brief Попытка подключения к обнаруженному Joy-Con
 * 
 * Эта функция вызывается из discovery модуля при обнаружении Joy-Con устройства
 * 
 * @param addr MAC-адрес устройства
 * @param cached_device Обнаруженное устройство из кэша discovery
 * @param is_left true если устройство определено как левый Joy-Con
 * @param is_right true если устройство определено как правый Joy-Con
 * @return 0 при успехе, -1 при ошибке
 */
int joycon_bt_classic_connection_attempt(const uint8_t *addr,
                                         const discovered_bt_classic_device_t *cached_device,
                                         bool is_left, bool is_right);

#endif // JOYCON_BT_CLASSIC_CONNECTION_H
