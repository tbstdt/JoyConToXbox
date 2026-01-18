/**
 * @file joycon_bt_classic_discovery.h
 * @brief Модуль обнаружения Bluetooth Classic устройств
 * 
 * Ответственность модуля:
 * - GAP inquiry (сканирование устройств)
 * - Кэширование обнаруженных устройств
 * - Определение типа Joy-Con по имени и MAC-адресу
 * - Уведомления о найденных устройствах через callback
 * 
 * ВНИМАНИЕ: Этот модуль НЕ отвечает за подключение к устройствам.
 * Он только находит и идентифицирует устройства.
 */

#ifndef JOYCON_BT_CLASSIC_DISCOVERY_H
#define JOYCON_BT_CLASSIC_DISCOVERY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "joycon_bt_classic_internal.h"

// discovered_bt_classic_device_t определен в joycon_bt_classic_internal.h

// Callback для обнаружения устройств
typedef struct {
    void (*on_device_found)(const discovered_bt_classic_device_t *device, bool is_left, bool is_right);
    void (*on_discovery_complete)(void);
    void (*on_discovery_started)(void);
} joycon_bt_classic_discovery_callbacks_t;

/**
 * @brief Инициализация модуля discovery
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_discovery_init(void);

/**
 * @brief Деинициализация модуля discovery
 */
void joycon_bt_classic_discovery_deinit(void);

/**
 * @brief Начать сканирование устройств
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_discovery_start(void);

/**
 * @brief Остановить сканирование устройств
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_discovery_stop(void);

/**
 * @brief Получить устройство из кэша по MAC-адресу
 * 
 * @param addr MAC-адрес устройства (6 байт)
 * @return discovered_bt_classic_device_t* Указатель на устройство в кэше или NULL
 */
discovered_bt_classic_device_t* joycon_bt_classic_discovery_get_device(const uint8_t *addr);

/**
 * @brief Определить тип Joy-Con по данным устройства
 * 
 * @param device Устройство из кэша
 * @param mac_addr MAC-адрес устройства
 * @param is_left Указатель для записи результата (левый Joy-Con)
 * @param is_right Указатель для записи результата (правый Joy-Con)
 */
void joycon_bt_classic_discovery_identify_joycon_type(const discovered_bt_classic_device_t *device,
                                                       const uint8_t *mac_addr,
                                                       bool *is_left, bool *is_right);

/**
 * @brief Подписка на события discovery
 * 
 * @param callbacks Структура с callback функциями (может быть NULL)
 */
void joycon_bt_classic_discovery_subscribe(const joycon_bt_classic_discovery_callbacks_t *callbacks);

/**
 * @brief Очистить кэш обнаруженных устройств
 */
void joycon_bt_classic_discovery_clear_cache(void);

/**
 * @brief Обработать GAP событие discovery
 * 
 * Эта функция вызывается из joycon_bt_classic_scan.c для маршрутизации
 * событий discovery в модуль discovery.
 * 
 * @param event Тип события GAP
 * @param param Параметры события
 */
void joycon_bt_classic_discovery_handle_gap_event(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

#endif // JOYCON_BT_CLASSIC_DISCOVERY_H
